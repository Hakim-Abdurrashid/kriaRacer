#!/usr/bin/env python3
"""Compare race_core's C++ follow-the-gap against blaze_racer's Python follow_the_gap_2.

Feeds the same recorded /scan messages to both, in order (both are stateful),
and reports the largest steering/speed differences.

usage: ftg_parity.py <bag_dir> <ftg_replay_binary> [max_scans]
"""
import os
import subprocess
import sys

import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

SRC = os.path.expanduser('~/airacer_basecode/blaze_racer/autonomy_ws/src')
sys.path[:0] = [os.path.join(SRC, 'autonomy_core'), os.path.join(SRC, 'follow_the_gap_2')]
from follow_the_gap_2.follow_the_gap_2_node import ReactiveFollowGap  # noqa: E402


def read_scans(bag, limit):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    scans = []
    while reader.has_next() and len(scans) < limit:
        topic, data, _ = reader.read_next()
        if topic == '/scan':
            scans.append(deserialize_message(data, LaserScan))
    return scans


def main():
    bag, binary = sys.argv[1], sys.argv[2]
    limit = int(sys.argv[3]) if len(sys.argv) > 3 else 2000
    scans = read_scans(bag, limit)

    # Same values as follow_the_gap_2/config/params.yaml (the launch defaults).
    rclpy.init(args=['--ros-args', '--log-level', 'error', '-p', 'speed_max:=2.8',
                     '-p', 'steering_gain:=0.7', '-p', 'max_steer:=0.6'])
    py = ReactiveFollowGap()
    py_out = [py.compute_drive(s) for s in scans]

    lines = '\n'.join(f'{s.angle_min!r},{s.angle_increment!r},' + ','.join(repr(float(r)) for r in s.ranges)
                      for s in scans)
    cpp_out = subprocess.run([binary], input=lines, capture_output=True, text=True,
                             check=True).stdout.split()

    max_ds = max_dv = 0.0
    mismatches = 0
    for p, c in zip(py_out, cpp_out):
        if p is None or c == 'stop':
            mismatches += (p is None) != (c == 'stop')
            continue
        cs, cv = map(float, c.split(','))
        ds, dv = abs(p[0] - cs), abs(p[1] - cv)
        max_ds, max_dv = max(max_ds, ds), max(max_dv, dv)
        mismatches += ds > 1e-6 or dv > 1e-6
    print(f'{len(scans)} scans: max |d steering| = {max_ds:.2e} rad, '
          f'max |d speed| = {max_dv:.2e} m/s, commands differing > 1e-6: {mismatches}')
    rclpy.shutdown()
    return 1 if mismatches else 0


if __name__ == '__main__':
    sys.exit(main())
