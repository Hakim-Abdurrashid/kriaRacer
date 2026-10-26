"""Quantize the car detector for the DPUCZDX8G (Vitis AI 3.5) and compile it for the KV260/KR260 B4096.

Runs inside race-vitis:3.5. The DPU part is everything up to the Detect head's convolutions. Outputs, per scale (40/20/10):
box DFL logits (1, 64, H, W) and class logit (1, 1, H, W) as separate tensors, so each gets its own
fixed-point scale (box logits span ~[-50, 15], the class logit a much smaller range).
DFL softmax, sigmoid, box decoding and NMS run on the board CPU (dpu_detector).

usage: python quantize_dpu.py <best.pt> <calib_image_dir> <out_dir> [n_calib] [n_eval] [eval_image_dir]
"""
import glob, os, random, re, subprocess, sys

import cv2
import numpy as np
import torch
import torch.nn as nn
from pytorch_nndct.apis import torch_quantizer
from ultralytics import YOLO
try:
    from ultralytics.utils.nms import non_max_suppression as nms
except ImportError:
    from ultralytics.utils.ops import non_max_suppression as nms

IMGSZ = 320
ARCH = '/opt/vitis_ai/compiler/arch/DPUCZDX8G/KV260/arch.json'


class DPUModel(nn.Module):
    """Backbone + neck + Detect convolutions, without the DFL/decoding tail."""

    def __init__(self, det_model):
        super().__init__()
        self.layers = det_model.model
        self.save = det_model.save
        self.head = self.layers[-1]

    def forward(self, x):
        y = []
        for m in self.layers[:-1]:
            if m.f != -1:
                x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]
            x = m(x)
            y.append(x if m.i in self.save else None)
        feats = [y[j] for j in self.head.f]
        out = []
        for i, f in enumerate(feats):
            out += [self.head.cv2[i](f), self.head.cv3[i](f)]
        return out


def decode(head, outs, conf=0.25, iou=0.6):
    """DPU outputs -> list of (n, 6) [x1, y1, x2, y2, conf, cls] in letterbox pixels, via ultralytics' head."""
    boxes, scores = outs[0::2], outs[1::2]
    bs = boxes[0].shape[0]
    preds = {"boxes": torch.cat([b.view(bs, 4 * head.reg_max, -1) for b in boxes], -1),
             "scores": torch.cat([c.view(bs, head.nc, -1) for c in scores], -1), "feats": boxes}
    y = head._inference(preds)
    return nms(y, conf_thres=conf, iou_thres=iou)


def box_iou(a, b):
    lt = torch.max(a[:, None, :2], b[None, :, :2]); rb = torch.min(a[:, None, 2:], b[None, :, 2:])
    inter = (rb - lt).clamp(min=0).prod(2)
    area = lambda x: (x[:, 2] - x[:, 0]) * (x[:, 3] - x[:, 1])
    return inter / (area(a)[:, None] + area(b)[None] - inter + 1e-9)


def ap50(model, head, samples):
    """All-point-interpolated AP at IoU 0.5 over (tensor, gt_xyxy) samples."""
    recs, n_gt = [], 0
    with torch.no_grad():
        for x, gt in samples:
            det = decode(head, model(x), conf=0.001)[0]
            n_gt += len(gt)
            used = torch.zeros(len(gt), dtype=torch.bool)
            for d in det[det[:, 4].argsort(descending=True)]:
                tp = False
                if len(gt):
                    ious = box_iou(d[None, :4], gt)[0]; ious[used] = 0
                    j = int(ious.argmax())
                    if ious[j] >= 0.5: used[j] = True; tp = True
                recs.append((float(d[4]), tp))
    recs.sort(key=lambda r: -r[0])
    tp = np.cumsum([r[1] for r in recs]); fp = np.cumsum([not r[1] for r in recs])
    rec = tp / max(n_gt, 1); prec = tp / np.maximum(tp + fp, 1)
    mrec = np.concatenate([[0], rec, [1]]); mpre = np.concatenate([[1], prec, [0]])
    mpre = np.flip(np.maximum.accumulate(np.flip(mpre)))
    i = np.where(mrec[1:] != mrec[:-1])[0]
    return float(np.sum((mrec[i + 1] - mrec[i]) * mpre[i + 1]))


def relu_everywhere(model):
    for parent in model.modules():
        for name, child in parent.named_children():
            if isinstance(child, (nn.SiLU, nn.Hardswish)):
                setattr(parent, name, nn.ReLU(inplace=True))


def letterbox(path, with_meta=False):
    img = cv2.imread(path)[:, :, ::-1]
    h, w = img.shape[:2]
    s = IMGSZ / max(h, w)
    img = cv2.resize(img, (round(w * s), round(h * s)), interpolation=cv2.INTER_LINEAR)
    out = np.full((IMGSZ, IMGSZ, 3), 114, np.uint8)
    top, left = (IMGSZ - img.shape[0]) // 2, (IMGSZ - img.shape[1]) // 2
    out[top:top + img.shape[0], left:left + img.shape[1]] = img
    t = torch.from_numpy(out.transpose(2, 0, 1).copy()).float().unsqueeze(0) / 255.0
    return (t, (s, left, top, w, h)) if with_meta else t


def labelled_sample(path):
    x, (s, left, top, w, h) = letterbox(path, with_meta=True)
    lbl = os.path.splitext(path.replace("/images/", "/labels/"))[0] + ".txt"
    gt = []
    for line in open(lbl) if os.path.exists(lbl) else []:
        c, cx, cy, bw, bh = map(float, line.split()[:5])
        gt.append([(cx - bw / 2) * w * s + left, (cy - bh / 2) * h * s + top,
                   (cx + bw / 2) * w * s + left, (cy + bh / 2) * h * s + top])
    return x, torch.tensor(gt, dtype=torch.float32).reshape(-1, 4)


def main():
    weights, calib_dir, out_dir = sys.argv[1:4]
    n_calib = int(sys.argv[4]) if len(sys.argv) > 4 else 300
    n_eval = int(sys.argv[5]) if len(sys.argv) > 5 else 300
    os.makedirs(out_dir, exist_ok=True)

    det = YOLO(weights).model.float().eval()
    relu_everywhere(det)
    for m in det.modules():  # fuse BN into conv like ultralytics export does
        if hasattr(m, 'bn') and hasattr(m, 'conv') and hasattr(m, 'forward_fuse'):
            from ultralytics.utils.torch_utils import fuse_conv_and_bn
            m.conv = fuse_conv_and_bn(m.conv, m.bn); delattr(m, 'bn'); m.forward = m.forward_fuse
    model = DPUModel(det).eval()

    imgs = sorted(glob.glob(os.path.join(calib_dir, '*')))
    random.Random(0).shuffle(imgs)
    imgs = imgs[:n_calib]
    dummy = torch.zeros(1, 3, IMGSZ, IMGSZ)
    with torch.no_grad():
        ref = model(dummy)
    print('float outputs:', [tuple(o.shape) for o in ref])

    q = torch_quantizer('calib', model, (dummy,), output_dir=out_dir, device=torch.device('cpu'))
    qm = q.quant_model
    with torch.no_grad():
        for i, p in enumerate(imgs):
            qm(letterbox(p))
    q.export_quant_config()

    q = torch_quantizer('test', model, (dummy,), output_dir=out_dir, device=torch.device('cpu'))
    qm = q.quant_model
    eval_dir = sys.argv[6] if len(sys.argv) > 6 else '/home/claude/race/datasets/f1tenth_car/val/images'
    evals = sorted(glob.glob(os.path.join(eval_dir, '*')))
    random.Random(1).shuffle(evals)
    samples = [labelled_sample(p) for p in evals[:n_eval]]
    head = det.model[-1]
    f_ap, q_ap = ap50(model, head, samples), ap50(qm, head, samples)
    print(f'[accuracy] AP50 on {len(samples)} val images: float {f_ap:.3f}  int8 {q_ap:.3f}  drop {100 * (f_ap - q_ap):.1f} pts')
    q.export_xmodel(output_dir=out_dir, deploy_check=False)

    xmodel = glob.glob(os.path.join(out_dir, '*_int.xmodel'))[0]
    log = subprocess.run(['vai_c_xir', '-x', xmodel, '-a', ARCH, '-o', os.path.join(out_dir, 'compiled'),
                          '-n', 'yolov8n_car'], capture_output=True, text=True)
    print(log.stdout[-3000:], log.stderr[-2000:])
    sub = [l for l in (log.stdout + log.stderr).splitlines() if 'subgraph number' in l.lower()]
    print('\n'.join(sub))
    ok = any(re.search(r'DPU subgraph number 1\b', l) for l in sub)
    print('ACCEPTANCE (exactly one DPU subgraph):', 'PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
