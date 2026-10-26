"""Merge the Roboflow F1Tenth datasets into one single-class ('car') YOLO detection dataset.

- class names meaning "an F1Tenth-style car" -> 0, everything else dropped (kiwi-cart stays as an
  unlabelled negative)
- polygon (segmentation) labels -> bounding boxes
- val/test keep one image per original Roboflow photo (augmented variants share a file stem);
  train keeps augmented variants but drops anything whose original is in val/test (no leakage);
  exact duplicate files (same hash) are always dropped
- images are symlinked, not copied
"""
import glob, hashlib, os, re, yaml

RAW = os.path.expanduser('~/race/datasets/raw')
OUT = os.path.expanduser('~/race/datasets/f1tenth_car')
CAR = {'car', 'cars', 'racecar', 'f1tenth', 'car from the front'}
SPLITS = [('test', 'test'), ('valid', 'val'), ('train', 'train')]

def stem_key(path):
    base = os.path.basename(path)
    return re.split(r'_(?:jpg|jpeg|png)\.rf\.|\.rf\.', base, flags=re.I)[0].lower()

def to_box(parts):
    v = list(map(float, parts[1:]))
    if len(v) == 4:
        return v
    xs, ys = v[0::2], v[1::2]
    x0, x1, y0, y1 = max(0, min(xs)), min(1, max(xs)), max(0, min(ys)), min(1, max(ys))
    return [(x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0]

seen_stem, seen_hash = set(), set()
stats = {s: [0, 0, 0] for _, s in SPLITS}  # images, boxes, duplicates skipped
for _, out_split in SPLITS:
    os.makedirs(f'{OUT}/{out_split}/images', exist_ok=True)
    os.makedirs(f'{OUT}/{out_split}/labels', exist_ok=True)

for src_split, out_split in SPLITS:
    for ds in sorted(os.listdir(RAW)):
        y = yaml.safe_load(open(f'{RAW}/{ds}/data.yaml'))
        names = y['names'] if isinstance(y['names'], list) else [y['names'][k] for k in sorted(y['names'])]
        keep = {i for i, n in enumerate(names) if n.strip().lower() in CAR}
        for img in sorted(glob.glob(f'{RAW}/{ds}/{src_split}/images/*')):
            key = stem_key(img)
            h = hashlib.md5(open(img, 'rb').read()).hexdigest()
            if h in seen_hash or (key in seen_stem):
                stats[out_split][2] += 1
                continue
            seen_hash.add(h)
            if out_split != "train":
                seen_stem.add(key)
            lbl = os.path.splitext(img.replace('/images/', '/labels/'))[0] + '.txt'
            boxes = []
            if os.path.exists(lbl):
                for line in open(lbl):
                    p = line.split()
                    if len(p) >= 5 and int(p[0]) in keep:
                        b = to_box(p)
                        if b[2] > 0.002 and b[3] > 0.002:
                            boxes.append('0 ' + ' '.join(f'{x:.6f}' for x in b))
            name = f'{ds}__{os.path.basename(img)}'
            dst = f'{OUT}/{out_split}/images/{name}'
            if not os.path.lexists(dst):
                os.symlink(img, dst)
            with open(f'{OUT}/{out_split}/labels/{os.path.splitext(name)[0]}.txt', 'w') as f:
                f.write('\n'.join(boxes) + ('\n' if boxes else ''))
            stats[out_split][0] += 1; stats[out_split][1] += len(boxes)

yaml.safe_dump({'path': OUT, 'train': 'train/images', 'val': 'val/images', 'test': 'test/images',
                'nc': 1, 'names': ['car']}, open(f'{OUT}/data.yaml', 'w'))
for s, (n, b, d) in stats.items():
    print(f'{s}: {n} images, {b} car boxes, {d} duplicates skipped')
