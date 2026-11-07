#!/usr/bin/env python3
"""Compare race_core's C++ port with blaze_racer iv2026's Python follow_the_gap planner.

Feeds the same recorded /scan messages to both and reports the largest steering/speed
differences. Uses the planner's deployed params.yaml (follow_the_gap share directory).

usage: ftg_iv_parity.py <bag_dir> <ftg_replay_binary> [max_scans] [blaze_racer_dir]
"""
import array
import os
import sys
from argparse import Namespace
import subprocess

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


def read_scans(bag, limit):
    reader = rosbag2_py.SequentialReader()
    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(bag)) else 'sqlite3'
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    scans = []
    while reader.has_next() and len(scans) < limit:
        topic, data, _ = reader.read_next()
        if topic == '/scan':
            scans.append(deserialize_message(data, LaserScan))
    return scans


def main():
    bag, binary = sys.argv[1], sys.argv[2]
    limit = int(sys.argv[3]) if len(sys.argv) > 3 else 4000
    root = os.path.expanduser(sys.argv[4] if len(sys.argv) > 4 else '~/blaze_racer')
    src = os.path.join(root, 'autonomy_ws', 'src')
    sys.path[:0] = [os.path.join(src, 'autonomy_core'), os.path.join(src, 'follow_the_gap')]
    from follow_the_gap.follow_the_gap_node import DEFAULT_PARAMS  # noqa: E402
    from follow_the_gap.planner import FollowTheGapPlanner  # noqa: E402

    with open(os.path.join(src, 'follow_the_gap', 'config', 'params.yaml')) as f:
        deployed = yaml.safe_load(f)['follow_the_gap_node']['ros__parameters']
    params = dict(DEFAULT_PARAMS)
    params.update(deployed)
    planner = FollowTheGapPlanner(Namespace(**params))

    scans = read_scans(bag, limit)
    py_out = []
    for s in scans:
        r = s.ranges if isinstance(s.ranges, array.array) else array.array('f', s.ranges)
        finite = any(x == x and abs(x) != float('inf') for x in r)
        py_out.append(planner.plan(r, s.angle_min, s.angle_increment) if finite else None)

    lines = '\n'.join(f'{s.angle_min!r},{s.angle_increment!r},' +
                      ','.join(repr(float(x)) for x in s.ranges) for s in scans)
    cpp_out = subprocess.run([binary, 'iv'], input=lines, capture_output=True, text=True,
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
    print(f'{len(scans)} scans, params {deployed}')
    print(f'max |d steering| = {max_ds:.2e} rad, max |d speed| = {max_dv:.2e} m/s, '
          f'commands differing > 1e-6: {mismatches}')
    return 1 if mismatches else 0


if __name__ == '__main__':
    sys.exit(main())
