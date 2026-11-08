"""Where is iv2026 FTG's 'closest point' (bubble centre)? Same preprocessing: clip 0..10, crop 270 deg, 3-beam box."""
import glob, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
bag = sys.argv[1]; step = int(sys.argv[2]) if len(sys.argv) > 2 else 10
a = float(sys.argv[3]) if len(sys.argv) > 3 else 0; b = float(sys.argv[4]) if len(sys.argv) > 4 else 9e18
db = sqlite3.connect(glob.glob(bag + '/*.db3')[0])
tid = db.execute("select id from topics where name='/scan'").fetchone()[0]
rows = db.execute('select timestamp, data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                  (tid, int(a * 1e9), int(min(b, 9e9) * 1e9))).fetchall()[::step]
angs, dists = [], []
for t, d in rows:
    s = deserialize_message(d, LaserScan)
    r = np.clip(np.nan_to_num(np.asarray(s.ranges, dtype=np.float64), nan=0.0, posinf=10.0), 0, 10)
    half = np.radians(135)
    i0 = int(np.ceil((-half - s.angle_min) / s.angle_increment)); i1 = int(np.floor((half - s.angle_min) / s.angle_increment)) + 1
    i0 = max(i0, 0); i1 = min(i1, r.size)
    c = np.convolve(r[i0:i1], np.ones(3), 'same') / 3
    k = int(np.argmin(c))
    angs.append(np.degrees(s.angle_min + (i0 + k) * s.angle_increment)); dists.append(c[k])
angs = np.array(angs); dists = np.array(dists)
self_ = (angs < -120) & (dists < 0.2)
print(f'{bag.split("/")[-1]}: {len(rows)} scans; closest point at -128..-120 deg & < 0.2 m (self-return) in {100*self_.mean():.0f}% ; '
      f'angle_min {np.degrees(s.angle_min):.1f} deg; median closest {np.median(dists):.3f} m at {np.median(angs):+.1f} deg')
other = ~self_
if other.any():
    print(f'  other closest points: {other.sum()} scans, dist p50 {np.median(dists[other]):.2f} m, angles p5..p95 {np.percentile(angs[other],5):+.0f}..{np.percentile(angs[other],95):+.0f}')
