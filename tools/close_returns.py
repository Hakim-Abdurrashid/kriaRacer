"""Scans with lidar returns closer than THR m (default 0.15) between T_FROM and T_TO (bag time, s), with the
/drive command at that moment and an angle histogram of the close beams. Run on the KR260 against a CLOSED bag.
usage: close_returns.py BAG T_FROM T_TO [THR]"""
import glob, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
bag = sys.argv[1]; a = float(sys.argv[2]); b = float(sys.argv[3]); thr = float(sys.argv[4]) if len(sys.argv) > 4 else 0.15
db = sqlite3.connect(glob.glob(bag + '/*.db3')[0])
tids = {n: i for i, n in db.execute('select id, name from topics')}
def rows(name):
    return db.execute('select timestamp, data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                      (tids[name], int(a * 1e9), int(b * 1e9))).fetchall()
drv = [(t * 1e-9, deserialize_message(d, AckermannDriveStamped).drive) for t, d in rows('/drive')]
dt = np.array([x[0] for x in drv]); st = np.array([x[1].steering_angle for x in drv]); sp = np.array([x[1].speed for x in drv])
n = hits = 0
angs = []
for t, d in rows('/scan'):
    s = deserialize_message(d, LaserScan)
    r = np.asarray(s.ranges, dtype=np.float32)
    ang = np.degrees(s.angle_min + np.arange(r.size) * s.angle_increment)
    close = np.isfinite(r) & (r > 0.0) & (r < thr)
    n += 1
    if close.any():
        hits += 1
        k = np.argmin(np.where(close, r, np.inf))
        i = min(np.searchsorted(dt, t * 1e-9), len(dt) - 1) if len(dt) else 0
        angs.append(ang[close])
        if hits <= 25 or hits % 10 == 0:
            print(f'{t*1e-9 - a:6.2f} s  {close.sum():3d} beams < {thr} m, closest {r[k]:.3f} m at {ang[k]:+.1f} deg '
                  f'(span {ang[close].min():+.1f}..{ang[close].max():+.1f})  steer {st[i] if len(dt) else float("nan"):+.3f} speed {sp[i] if len(dt) else float("nan"):.2f}')
print(f'{hits}/{n} scans have returns < {thr} m')
if angs:
    allang = np.concatenate(angs)
    print('angle histogram of close beams (deg):', np.histogram(allang, bins=[-136,-120,-105,-90,-75,-60,-45,0,45,90,136]))
