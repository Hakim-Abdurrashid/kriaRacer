"""Exactly where iv2026 FTG puts its bubble (numpy semantics: NaN survives clip/convolve, argmin returns the first NaN),
and what invalid values the scans contain. usage: ftg_bubble.py BAG [step]"""
import glob, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
bag = sys.argv[1]; step = int(sys.argv[2]) if len(sys.argv) > 2 else 10
db = sqlite3.connect(glob.glob(bag + '/*.db3')[0])
tid = db.execute("select id from topics where name='/scan'").fetchone()[0]
rows = db.execute('select data from messages where topic_id=? order by timestamp', (tid,)).fetchall()[::step]
nan_c = inf_c = zero_c = small_c = 0; kinds = {'nan': 0, 'self-rear': 0, 'real': 0}; real_d = []; real_a = []
for (d,) in rows:
    s = deserialize_message(d, LaserScan)
    raw = np.asarray(s.ranges, dtype=np.float64)
    nan_c += np.isnan(raw).sum(); inf_c += np.isinf(raw).sum(); zero_c += (raw == 0).sum(); small_c += ((raw > 0) & (raw < 0.06)).sum()
    r = np.clip(raw, 0, 10)
    half = np.radians(135)
    i0 = max(int(np.ceil((-half - s.angle_min) / s.angle_increment)), 0)
    i1 = min(int(np.floor((half - s.angle_min) / s.angle_increment)) + 1, r.size)
    c = np.convolve(r[i0:i1], np.ones(3), 'same')
    k = int(np.argmin(c)); ang = np.degrees(s.angle_min + (i0 + k) * s.angle_increment)
    if np.isnan(c[k]):
        kinds['nan'] += 1
    elif abs(ang) > 110 and c[k] / 3 < 0.2:
        kinds['self-rear'] += 1
    else:
        kinds['real'] += 1; real_d.append(c[k] / 3); real_a.append(ang)
n = len(rows); beams = n * r.size
print(f'{bag.split("/")[-1]}: {n} scans, range_min {s.range_min:.3f} | per scan: NaN {nan_c/n:.1f}, inf {inf_c/n:.1f}, 0.0 {zero_c/n:.1f}, (0,0.06) {small_c/n:.1f} beams')
print(f'  bubble centre: first NaN {100*kinds["nan"]/n:.0f}%, car self-return rear |a|>110 deg {100*kinds["self-rear"]/n:.0f}%, a real wall {100*kinds["real"]/n:.0f}%'
      + (f' (dist p50 {np.median(real_d):.2f} m, angles p5..p95 {np.percentile(real_a,5):+.0f}..{np.percentile(real_a,95):+.0f})' if real_d else ''))
