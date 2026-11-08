"""Train the DPU-friendly single-class car detector (YOLOv8n, ReLU only, 320 px).

usage: python train_car.py <run_name> [epochs] [scale n|s] [imgsz]
"""
import sys

import torch.nn as nn
from ultralytics import YOLO

DATA = '/home/claude/race/datasets/f1tenth_car/data.yaml'


def relu_everywhere(model):
    """Swap every SiLU/Hardswish for ReLU (the DPUCZDX8G only accelerates ReLU-family activations)."""
    swapped = 0
    for parent in model.modules():
        for name, child in parent.named_children():
            if isinstance(child, (nn.SiLU, nn.Hardswish)):
                setattr(parent, name, nn.ReLU(inplace=True))
                swapped += 1
    return swapped


def force_relu(trainer):
    n = relu_everywhere(trainer.model)
    if trainer.ema is not None:
        n += relu_everywhere(trainer.ema.ema)
    print(f'[relu] swapped {n} activations to ReLU')
    for m in (trainer.model, trainer.ema.ema if trainer.ema else trainer.model):
        assert not any(isinstance(x, (nn.SiLU, nn.Hardswish)) for x in m.modules())


def main():
    name = sys.argv[1]
    epochs = int(sys.argv[2]) if len(sys.argv) > 2 else 150
    scale = sys.argv[3] if len(sys.argv) > 3 else 'n'
    imgsz = int(sys.argv[4]) if len(sys.argv) > 4 else 320
    # yolov8{scale}_relu.yaml is a copy of yolov8n_relu.yaml; ultralytics picks the scale from the file name
    model = YOLO(f'yolov8{scale}_relu.yaml').load(f'yolov8{scale}.pt')
    model.add_callback('on_train_start', force_relu)
    model.train(data=DATA, imgsz=imgsz, epochs=epochs, batch=64, workers=8, project='runs',
                name=name, cos_lr=True, close_mosaic=15, patience=50, plots=True, exist_ok=True)
    best = YOLO(str(model.trainer.save_dir / 'weights' / 'best.pt'))
    left = [n for n, x in best.model.named_modules() if isinstance(x, (nn.SiLU, nn.Hardswish))]
    print('[relu] non-ReLU activations in best.pt:', left or 'none')
    metrics = best.val(data=DATA, split='test', imgsz=imgsz)
    print(f'[test] mAP50={metrics.box.map50:.3f} mAP50-95={metrics.box.map:.3f}')


if __name__ == '__main__':
    main()
