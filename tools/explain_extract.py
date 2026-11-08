"""Data for the explainer page from a CLOSED bag (never a live one): one lap of scans + what follow-the-gap
computed on each (numpy, same steps as race_core's iv2026 port) + the car's actual commands, EKF path and
accumulated wall points. usage: explain_extract.py BAG T_FROM T_TO OUT.json"""
import glob, json, math, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray

bag, a, b, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
P = dict(fov=270.0, bubble=175, conv=3, best=200, maxd=10.0, gain=0.5, max_steer=0.50,
         fw=0.1745, dmin=1.0, dmax=4.0, vmin=2.0, vmax=5.0)
db = sqlite3.connect(glob.glob(bag + '/*.db3')[0])
T = {n: i for i, n in db.execute('select id, name from topics')}
def rows(name, lo=a, hi=b):
    return db.execute('select timestamp, data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                      (T[name], int(lo * 1e9), int(hi * 1e9))).fetchall()

# static lidar mount
laser = None
for _, d in db.execute('select timestamp, data from messages where topic_id=?', (T['/tf_static'],)).fetchall():
    for tr in deserialize_message(d, TFMessage).transforms:
        if 'laser' in tr.child_frame_id:
            q = tr.transform.rotation
            laser = dict(parent=tr.header.frame_id, child=tr.child_frame_id, x=tr.transform.translation.x,
                         y=tr.transform.translation.y, yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
print('laser mount', laser)
lx, ly, lyaw = (laser['x'], laser['y'], laser['yaw']) if laser else (0.0, 0.0, 0.0)

odo = []
for t, d in rows('/odometry/filtered'):
    m = deserialize_message(d, Odometry); p = m.pose.pose; q = p.orientation
    odo.append((t * 1e-9, p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)),
                m.twist.twist.linear.x))
odo = np.array(odo)
drv = np.array([(t * 1e-9, m.drive.steering_angle, m.drive.speed) for t, m in
                ((t, deserialize_message(d, AckermannDriveStamped)) for t, d in rows('/drive'))])
tel = [deserialize_message(d, Float32MultiArray).data for _, d in rows('/race_core/telemetry')]
pipe = np.array([x[18] for x in tel if len(x) > 18])

def ftg(r, amin, inc):
    half = math.radians(P['fov']) / 2
    s = max(int(math.ceil((-half - amin) / inc)), 0); e = min(int(math.floor((half - amin) / inc)) + 1, r.size)
    a0 = amin + s * inc; x = np.clip(r[s:e].astype(np.float64), 0, P['maxd']); n = x.size
    sm = np.clip(np.convolve(x, np.ones(P['conv']), 'same') / P['conv'], 0, P['maxd'])
    closest = int(np.argmin(sm)); lo = max(closest - P['bubble'], 0); hi = min(closest + P['bubble'], n - 1)
    ic = int(round(-a0 / inc)); ih = max(1, int(round(P['fw'] / 2 / inc)))
    front = float(sm[max(0, ic - ih):min(n, ic + ih + 1)].mean())
    z = sm.copy(); z[lo:hi] = 0.0
    g0 = g1 = -1; bl = 0; i = 0
    while i < n:
        if z[i] == 0: i += 1; continue
        j = i
        while j < n and z[j] != 0: j += 1
        if j - i > bl: bl, g0, g1 = j - i, i, j
        i = j
    if g0 < 0: best = n // 2
    elif g1 - g0 < P['best']: best = g0 + (g1 - g0) // 2
    else: best = int(np.argmax(np.convolve(z[g0:g1], np.ones(P['best']), 'same') / P['best'])) + g0
    bearing = a0 + best * inc
    steer = float(np.clip(bearing * P['gain'], -P['max_steer'], P['max_steer']))
    t_ = min(max((front - P['dmin']) / (P['dmax'] - P['dmin']), 0.0), 1.0)
    return dict(s=s, closest=closest, lo=lo, hi=hi, g0=g0, g1=g1, best=best, front=front, steer=steer,
                speed=P['vmin'] + t_ * (P['vmax'] - P['vmin']))

scans, walls, errs_s, errs_v = [], set(), [], []
srows = rows('/scan')
for k, (t, d) in enumerate(srows):
    m = deserialize_message(d, LaserScan); r = np.asarray(m.ranges, dtype=np.float32); ts = t * 1e-9
    j = int(np.searchsorted(odo[:, 0], ts)); j = min(max(j, 0), len(odo) - 1); ox, oy, oyaw, ov = odo[j, 1:5]
    f = ftg(r, m.angle_min, m.angle_increment)
    jd = int(np.searchsorted(drv[:, 0], ts))
    cmd = drv[jd] if jd < len(drv) and drv[jd, 0] - ts < 0.05 else None
    if cmd is not None:
        errs_s.append(abs(cmd[1] - f['steer'])); errs_v.append(abs(cmd[2] - f['speed']))
    ang = m.angle_min + np.arange(r.size) * m.angle_increment
    keep = np.isfinite(r) & (r > 0.15) & (r < 3.0) & (np.abs(ang) < math.radians(120))
    xl = r[keep] * np.cos(ang[keep] + lyaw) + lx; yl = r[keep] * np.sin(ang[keep] + lyaw) + ly
    xw = ox + math.cos(oyaw) * xl - math.sin(oyaw) * yl; yw = oy + math.sin(oyaw) * xl + math.cos(oyaw) * yl
    walls.update(zip(np.round(xw / 0.04).astype(int).tolist(), np.round(yw / 0.04).astype(int).tolist()))
    if k % 4 == 0:
        scans.append(dict(t=round(ts - a, 3), x=round(ox, 3), y=round(oy, 3), yaw=round(oyaw, 4), v=round(ov, 2),
                          r=np.round(np.clip(np.nan_to_num(r, nan=0, posinf=30), 0, 30) * 1000).astype(int).tolist(),
                          f=f, cmd=None if cmd is None else [round(float(cmd[1]), 4), round(float(cmd[2]), 3)]))
path = [[round(float(o[0] - a), 2), round(float(o[1]), 3), round(float(o[2]), 3), round(float(o[4]), 2)] for o in odo[::5]]
dpath = [[round(float(x[0] - a), 2), round(float(x[1]), 3), round(float(x[2]), 2)] for x in drv[::4]]
res = dict(params=P, laser=laser, angle_min=m.angle_min, inc=m.angle_increment, n=int(r.size), scans=scans,
           walls=sorted(walls), wall_cell=0.04, path=path, drive=dpath,
           pipeline_ms=dict(p50=float(np.median(pipe)) if pipe.size else None, p99=float(np.percentile(pipe, 99)) if pipe.size else None,
                            max=float(pipe.max()) if pipe.size else None, n=int(pipe.size)),
           check=dict(n=len(errs_s), max_steer_err=float(max(errs_s)) if errs_s else None,
                      p99_steer_err=float(np.percentile(errs_s, 99)) if errs_s else None,
                      max_speed_err=float(max(errs_v)) if errs_v else None,
                      frac_steer_match=float(np.mean(np.array(errs_s) < 1e-3)) if errs_s else None))
json.dump(res, open(out, 'w'), separators=(',', ':'))
print('scans', len(srows), 'kept', len(scans), 'walls', len(walls), 'path', len(path), 'pipeline', res['pipeline_ms'])
print('check vs car commands', res['check'])
