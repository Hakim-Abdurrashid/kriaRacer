"""Share of scans whose closest smoothed return (where follow-the-gap centres its bubble) is the car itself
(|angle| > 110 deg and < 0.2 m), and where the rest are. Run on the KR260 against CLOSED bags.
usage: bubble_rest.py BAG [BAG ...]"""
import glob, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
for bag in sys.argv[1:]:
    db = sqlite3.connect(glob.glob(bag + '/*.db3')[0])
    tid = db.execute("select id from topics where name='/scan'").fetchone()[0]
    rows = db.execute('select data from messages where topic_id=? order by timestamp', (tid,)).fetchall()[::5]
    rest = []; n = 0; selfn = 0
    for (d,) in rows:
        s = deserialize_message(d, LaserScan); r = np.clip(np.asarray(s.ranges, dtype=np.float64), 0, 10)
        c = np.convolve(r, np.ones(3), 'same') / 3; k = int(np.argmin(c)); a = np.degrees(s.angle_min + k * s.angle_increment)
        n += 1
        if abs(a) > 110 and c[k] < 0.2: selfn += 1
        else: rest.append((c[k], a))
    rest = np.array(rest) if rest else np.zeros((0, 2))
    msg = f'{bag.split("/")[-1]}: {n} scans, self {100*selfn/n:.1f}%'
    if len(rest):
        msg += f'; rest {len(rest)}: dist max {rest[:,0].max():.3f} p95 {np.percentile(rest[:,0],95):.3f}, share < 0.06 m {100*np.mean(rest[:,0] < 0.06):.0f}%, angles {rest[:,1].min():+.0f}..{rest[:,1].max():+.0f}'
    print(msg)
