"""Fine-tune a ReLU car detector with our own background frames (hard negatives).

usage: python finetune_neg.py <start_weights.pt> <run_name> [epochs]
"""
import sys

from ultralytics import YOLO

from train_car import force_relu

DATA = "/home/claude/race/datasets/f1tenth_car_neg.yaml"


def main():
    start, name = sys.argv[1], sys.argv[2]
    epochs = int(sys.argv[3]) if len(sys.argv) > 3 else 40
    model = YOLO(start)
    model.add_callback("on_train_start", force_relu)
    model.train(data=DATA, imgsz=320, epochs=epochs, batch=64, workers=8, project="runs", name=name,
                lr0=0.002, lrf=0.1, warmup_epochs=1, cos_lr=True, close_mosaic=10, patience=20,
                exist_ok=True, plots=True)
    best = YOLO(str(model.trainer.save_dir / "weights" / "best.pt"))
    m = best.val(data=DATA, split="test", imgsz=320)
    print(f"[test] mAP50={m.box.map50:.3f} mAP50-95={m.box.map:.3f}")
    # false positives on held-out July frames (no cars in them)
    res = best.predict("/home/claude/race/datasets/july_neg/heldout/images", imgsz=320, conf=0.25,
                       verbose=False, stream=True)
    n = hits = hits35 = 0
    for r in res:
        n += 1
        s = r.boxes.conf.cpu().numpy() if r.boxes is not None else []
        hits += int(len(s) > 0)
        hits35 += int((s >= 0.35).any()) if len(s) else 0
    print(f"[heldout July FP] frames with a detection: conf>=0.25 {hits}/{n}, conf>=0.35 {hits35}/{n}")


if __name__ == "__main__":
    main()
