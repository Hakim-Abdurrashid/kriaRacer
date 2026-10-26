"""On-board check of the DPU car detector: real images -> DPU -> numpy decode -> compare with labels.

Also the reference for dpu_detector's C++ post-processing (DFL decode + sigmoid + NMS).
Run as root inside the PYNQ venv:
  sudo bash -c 'source /etc/pynq_venv.sh && python3 /home/ubuntu/2x1_w/models/dpu_check.py'
"""
import glob
import os
import time

import cv2
import numpy as np
from pynq_dpu import DpuOverlay

HERE = os.path.dirname(os.path.abspath(__file__))
XMODEL = os.path.join(HERE, 'yolov8n_car_v0.xmodel')
IMGSZ, CONF, IOU = 320, 0.25, 0.6


def letterbox(img):
    h, w = img.shape[:2]
    s = IMGSZ / max(h, w)
    r = cv2.resize(img, (round(w * s), round(h * s)), interpolation=cv2.INTER_LINEAR)
    out = np.full((IMGSZ, IMGSZ, 3), 114, np.uint8)
    top, left = (IMGSZ - r.shape[0]) // 2, (IMGSZ - r.shape[1]) // 2
    out[top:top + r.shape[0], left:left + r.shape[1]] = r
    return out, s, left, top


def decode(box_maps, cls_maps):
    """box_maps/cls_maps: float arrays (H, W, 64) / (H, W, 1) per stride 8/16/32 -> (n, 5) x1y1x2y2,conf."""
    dets = []
    for bm, cm, stride in zip(box_maps, cls_maps, (8, 16, 32)):
        conf = 1.0 / (1.0 + np.exp(-cm[..., 0]))
        ys, xs = np.nonzero(conf > CONF)
        if len(ys) == 0:
            continue
        logits = bm[ys, xs].reshape(-1, 4, 16)
        p = np.exp(logits - logits.max(-1, keepdims=True))
        p /= p.sum(-1, keepdims=True)
        ltrb = (p * np.arange(16)).sum(-1)
        cx, cy = xs + 0.5, ys + 0.5
        boxes = np.stack([cx - ltrb[:, 0], cy - ltrb[:, 1], cx + ltrb[:, 2], cy + ltrb[:, 3]], 1) * stride
        dets.append(np.concatenate([boxes, conf[ys, xs, None]], 1))
    if not dets:
        return np.zeros((0, 5))
    d = np.concatenate(dets)
    keep = cv2.dnn.NMSBoxes(
        [[float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])] for b in d],
        d[:, 4].tolist(), CONF, IOU)
    return d[np.array(keep, dtype=int).reshape(-1)]


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter + 1e-9)


def main():
    ov = DpuOverlay('dpu.bit')
    ov.load_model(XMODEL)
    r = ov.runner
    ins, outs = r.get_input_tensors(), r.get_output_tensors()
    in_scale = 2 ** ins[0].get_attr('fix_point')
    out_scale = [2.0 ** -o.get_attr('fix_point') for o in outs]
    boxes_idx = [i for i, o in enumerate(outs) if o.dims[-1] == 64]
    cls_idx = [i for i, o in enumerate(outs) if o.dims[-1] == 1]
    boxes_idx.sort(key=lambda i: -outs[i].dims[1])  # 40x40, 20x20, 10x10
    cls_idx.sort(key=lambda i: -outs[i].dims[1])

    inp = [np.empty(tuple(ins[0].dims), dtype=np.int8)]
    out = [np.empty(tuple(o.dims), dtype=np.int8) for o in outs]
    tp = fp = n_gt = 0
    t_pre = t_dpu = t_post = 0.0
    images = sorted(glob.glob(os.path.join(HERE, 'val_sample/images/*')))
    for path in images:
        img = cv2.imread(path)[:, :, ::-1]
        h, w = img.shape[:2]
        t0 = time.perf_counter()
        lb, s, left, top = letterbox(img)
        inp[0][0] = np.clip(np.round(lb.astype(np.float32) * (in_scale / 255.0)), -128, 127).astype(np.int8)
        t1 = time.perf_counter()
        r.wait(r.execute_async(inp, out))
        t2 = time.perf_counter()
        f = [out[i][0].astype(np.float32) * out_scale[i] for i in range(len(out))]
        det = decode([f[i] for i in boxes_idx], [f[i] for i in cls_idx])
        t3 = time.perf_counter()
        t_pre += t1 - t0; t_dpu += t2 - t1; t_post += t3 - t2

        lbl = os.path.splitext(path.replace('/images/', '/labels/'))[0] + '.txt'
        gt = []
        for line in open(lbl) if os.path.exists(lbl) else []:
            _, cx, cy, bw, bh = map(float, line.split()[:5])
            gt.append([(cx - bw / 2) * w * s + left, (cy - bh / 2) * h * s + top,
                       (cx + bw / 2) * w * s + left, (cy + bh / 2) * h * s + top])
        n_gt += len(gt)
        used = set()
        for d in det:
            m = max(((iou(d, g), j) for j, g in enumerate(gt) if j not in used), default=(0, -1))
            if m[0] >= 0.5:
                tp += 1; used.add(m[1])
            else:
                fp += 1
    n = len(images)
    print(f'{n} images, {n_gt} labelled cars: TP {tp}, FP {fp}, missed {n_gt - tp} '
          f'-> precision {tp / max(tp + fp, 1):.3f}, recall {tp / max(n_gt, 1):.3f} (conf {CONF}, IoU 0.5)')
    print(f'per frame: preprocess {1e3 * t_pre / n:.2f} ms, DPU {1e3 * t_dpu / n:.2f} ms, '
          f'decode+NMS {1e3 * t_post / n:.2f} ms (Python/numpy)')


if __name__ == '__main__':
    main()
