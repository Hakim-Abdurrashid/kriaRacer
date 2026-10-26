"""Download the F1Tenth car datasets from Roboflow Universe (latest version, YOLOv8 format)."""
import io, json, os, sys, urllib.request, zipfile

KEY = open(os.path.expanduser('~/race/.roboflow_key')).read().strip()
PROJECTS = [
    'cmu-f1tenth/f1tenth-segmentation',
    'unibo-dgtln/f1tenth-cars-detector-bologna',
    'ksae/f1tenth-car-detection-4ywif',
    'cu-robotics-qtbgu/detect-f1tenth',
    'f1tenth-cars/f1-tenth',
]
OUT = os.path.expanduser('~/race/datasets/raw')

def get(url):
    with urllib.request.urlopen(url, timeout=300) as r:
        return r.read()

for proj in PROJECTS:
    dest = os.path.join(OUT, proj.replace('/', '__'))
    if os.path.isdir(dest):
        print('skip', proj); continue
    info = json.loads(get(f'https://api.roboflow.com/{proj}?api_key={KEY}'))
    versions = sorted(int(v['id'].rsplit('/', 1)[1]) for v in info.get('versions', []))
    if not versions:
        print('no versions for', proj); continue
    ver = versions[-1]
    exp = json.loads(get(f'https://api.roboflow.com/{proj}/{ver}/yolov8?api_key={KEY}'))
    link = exp.get('export', {}).get('link')
    if not link:
        print('export not ready / failed for', proj, list(exp.keys())); continue
    data = get(link)
    zipfile.ZipFile(io.BytesIO(data)).extractall(dest)
    n = sum(len(f) for _, _, f in os.walk(dest))
    print(f'{proj} v{ver}: {len(data)/1e6:.1f} MB, {n} files -> {dest}', flush=True)
