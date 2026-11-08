#!/usr/bin/env python3
"""Lap times without a map: on a closed loop every lap adds 2*pi of heading. Integrates the EKF yaw
rate (D435i gyro) from /odometry/filtered and reports the time of each full turn, plus speed stats
per lap and moving time. Laps that include a stop (speed < 0.3 m/s for > 1 s) are flagged.

usage: yaw_laps.py BAG [cw|ccw|auto]
"""
import sys

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

bag = sys.argv[1]
direction = sys.argv[2] if len(sys.argv) > 2 else 'auto'
r = rosbag2_py.SequentialReader()
uri = bag
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/odometry/filtered']))
t, v, w = [], [], []
while r.has_next():
    _, d, ts = r.read_next()
    m = deserialize_message(d, Odometry)
    t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    v.append(m.twist.twist.linear.x)
    w.append(m.twist.twist.angular.z)
t, v, w = map(np.array, (t, v, w))
moving = v > 0.3
if not moving.any():
    print('car never moved')
    sys.exit(0)
i0 = np.argmax(moving)
t, v, w = t[i0:], v[i0:], w[i0:]
yaw = np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(t))])
sgn = np.sign(yaw[-1]) if direction == 'auto' else (1.0 if direction == 'ccw' else -1.0)
turns = sgn * yaw / (2 * np.pi)
dist = np.concatenate([[0.0], np.cumsum(0.5 * (np.abs(v[1:]) + np.abs(v[:-1])) * np.diff(t))])
print(f'{bag}: {t[-1]-t[0]:.1f} s from first motion, {turns[-1]:.2f} turns ({"ccw" if sgn > 0 else "cw"}), '
      f'{dist[-1]:.1f} m driven, mean speed while moving {v[v > 0.3].mean():.2f} m/s, max {v.max():.2f}')
prev_t, prev_i = t[0], 0
for k in range(1, int(turns[-1]) + 1):
    i = int(np.argmax(turns >= k))
    seg_v = v[prev_i:i]
    seg_t = t[prev_i:i]
    stopped = 0.0
    if len(seg_v) > 1:
        dt = np.diff(seg_t)
        stopped = dt[seg_v[1:] < 0.3].sum()
    print(f'  lap {k}: {t[i]-prev_t:6.2f} s  {dist[i]-dist[prev_i]:5.1f} m  mean {seg_v.mean():.2f} m/s  '
          f'max {seg_v.max():.2f} m/s  stopped {stopped:.1f} s' + ('  (includes a stop)' if stopped > 1.0 else ''))
    prev_t, prev_i = t[i], i
