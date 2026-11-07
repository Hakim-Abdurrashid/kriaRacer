#!/usr/bin/env python3
"""Per-run driving stats straight from a bag's sqlite file (fast: only the needed topics).

Battery voltage (VESC /sensors/core), commanded steering/speed while engaged (/drive), actual speed
(/odometry/filtered), steering saturation per side, and speed tracking. Optional time window.

usage: run_stats.py BAG [t_from_s t_to_s]   (seconds from bag start)
"""
import glob
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

bag = sys.argv[1]
t_from = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
t_to = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
db = sqlite3.connect(glob.glob(bag + '/*.db3')[0])
topics = {name: (tid, typ) for tid, name, typ in db.execute('select id, name, type from topics')}
t0 = db.execute('select min(timestamp) from messages').fetchone()[0]


def rows(name, every=1):
    tid = topics[name][0]
    q = 'select timestamp, data from messages where topic_id=? and timestamp between ? and ? order by timestamp'
    out = db.execute(q, (tid, t0 + int(t_from * 1e9), t0 + int(t_to * 1e9))).fetchall()
    return out[::every]


try:
    from vesc_msgs.msg import VescStateStamped
    vs = [(ts, deserialize_message(d, VescStateStamped).state) for ts, d in rows('/sensors/core', 50)]
    volt = np.array([[(ts - t0) * 1e-9, s.voltage_input] for ts, s in vs])
except Exception as e:  # noqa: BLE001
    volt = None
    print('no VESC voltage:', e)
dr = np.array([[(ts - t0) * 1e-9, m.drive.steering_angle, m.drive.speed] for ts, m in
               ((ts, deserialize_message(d, AckermannDriveStamped)) for ts, d in rows('/drive'))])
od = np.array([[(ts - t0) * 1e-9, m.twist.twist.linear.x, m.twist.twist.angular.z] for ts, m in
               ((ts, deserialize_message(d, Odometry)) for ts, d in rows('/odometry/filtered', 2))])
eng = dr[dr[:, 2] > 0.05]
print(f'{bag} [{t_from:.0f}, {min(t_to, dr[-1,0]):.0f}] s')
if volt is not None and len(volt):
    mv = volt[(volt[:, 0] >= eng[0, 0]) & (volt[:, 0] <= eng[-1, 0])] if len(eng) else volt
    print(f'  battery: {volt[0,1]:.2f} V at start, {volt[-1,1]:.2f} V at end; while driving min {mv[:,1].min():.2f} '
          f'mean {mv[:,1].mean():.2f} V')
if len(eng):
    st = eng[:, 1]
    print(f'  engaged {len(eng)/40:.0f} s | steer: left {np.mean(st > 0.02)*100:.0f}% right {np.mean(st < -0.02)*100:.0f}% '
          f'| at +0.49+ (full left) {np.mean(st >= 0.49)*100:.1f}%  at -0.49- (full right) {np.mean(st <= -0.49)*100:.1f}%')
    print(f'  mean |steer| left {st[st > 0.02].mean():.3f} right {-st[st < -0.02].mean():.3f} rad | '
          f'cmd speed mean {eng[:,2].mean():.2f} p95 {np.percentile(eng[:,2],95):.2f} max {eng[:,2].max():.2f}')
    mo = od[(od[:, 0] >= eng[0, 0]) & (od[:, 0] <= eng[-1, 0])]
    cmd_at = np.interp(mo[:, 0], eng[:, 0], eng[:, 2])
    fast = cmd_at > 3.0
    print(f'  actual speed mean {mo[:,1].mean():.2f} max {mo[:,1].max():.2f} | when cmd > 3 m/s: actual/cmd '
          f'{(mo[fast,1]/cmd_at[fast]).mean() if fast.any() else float("nan"):.2f} ({fast.mean()*100:.0f}% of time)')
    print(f'  yaw rate: max left {mo[:,2].max():.2f} max right {-mo[:,2].min():.2f} rad/s')
