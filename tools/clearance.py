#!/usr/bin/env python3
"""Wall clearance per autonomous stint, from a bag's /scan and the race_core log (sqlite direct, fast).

For every scan while RB was held: the closest lidar return ahead (-45..+45 deg), to the left (+45..+110)
and to the right (-110..-45). Returns beyond +-110 deg and below 0.06 m (the UST-10LX minimum) are
skipped: on this car they are its own parts (11/07: a return at -127 deg, 0.11 m, in every scan; 11/06:
one at +115..+125 deg). Raw lidar ranges: the lidar sits ~0.1 m inside the car's sides. Reports the minimum and the 1st percentile per stint, and where
(seconds into the stint) the closest approach happened.

usage: clearance.py BAG CORE_LOG [last_n_stints]
"""
import glob
import re
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

bag, log = sys.argv[1], sys.argv[2]
last_n = int(sys.argv[3]) if len(sys.argv) > 3 else 99

ts = re.compile(r'\[(\d+)\.(\d+)\] \[race_core\]: (.*)')
stints, t0 = [], None
for line in open(log, errors='ignore'):
    m = ts.search(re.sub(r'\x1b\[[0-9;]*m', '', line))
    if not m:
        continue
    t, msg = int(m.group(1)) + int(m.group(2)) * 1e-9, m.group(3)
    if msg.startswith('deadman HELD'):
        t0 = t
    elif msg.startswith('deadman released') and t0 is not None:
        if t - t0 > 5.0:
            stints.append((t0, t))
        t0 = None
stints = stints[-last_n:]

db = sqlite3.connect(glob.glob(bag + '/*.db3')[0])
tid = db.execute("select id from topics where name='/scan'").fetchone()[0]
for a, b in stints:
    rows = db.execute('select timestamp, data from messages where topic_id=? and timestamp between ? and ? '
                      'order by timestamp', (tid, int(a * 1e9), int(b * 1e9))).fetchall()
    if not rows:
        print(f'stint at {a:.0f}: no scans in bag')
        continue
    mins = {'front': [], 'left': [], 'right': []}
    when = []
    for stamp, data in rows:
        s = deserialize_message(data, LaserScan)
        r = np.asarray(s.ranges, dtype=np.float32)
        ang = s.angle_min + np.arange(r.size) * s.angle_increment
        ok = np.isfinite(r) & (r > 0.06)
        r = np.where(ok, r, np.inf)
        mins['front'].append(r[np.abs(ang) <= np.radians(45)].min())
        mins['left'].append(r[(ang > np.radians(45)) & (ang < np.radians(110))].min())
        mins['right'].append(r[(ang < np.radians(-45)) & (ang > np.radians(-110))].min())
        when.append(stamp * 1e-9 - a)
    print(f'stint {a:.0f} ({b - a:.1f} s, {len(rows)} scans):', end='')
    for k in ('front', 'left', 'right'):
        v = np.asarray(mins[k])
        i = int(np.argmin(v))
        print(f'  {k} min {v[i]:.3f} m @ {when[i]:.1f} s, p1 {np.percentile(v, 1):.3f}', end='')
    print()
