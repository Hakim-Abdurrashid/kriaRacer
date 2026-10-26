#!/usr/bin/env python3
"""Lap analyser (RACE_PLAN.md §9): race_core telemetry from a bag -> lap table + trajectory plot.

usage: lap_report.py BAG_DIR --map MAP.yaml --raceline RACELINE.csv [--out report.png]

Reads /race_core/telemetry (and /sim/pose in simulation) straight from the bag's sqlite file, so it
runs on the car without ROS sourced. Prints per-lap time, lateral error (RMS / max), mean speed,
emergency-brake activations and localization health; draws the trajectory over the map coloured by
speed, with emergency-brake points in red and REACTIVE stretches in magenta.
"""
import argparse
import glob
import math
import os
import sqlite3
import struct
import sys

import cv2
import numpy as np
import yaml

FIELDS = ['t', 'state', 'engaged', 'x', 'y', 'yaw', 'v', 's', 'd', 'v_ref', 'steer', 'speed_cmd',
          'match', 'neff', 'spread', 'ttc', 'laps', 'last_lap', 'ms', 'opp_valid', 'opp_gap', 'opp_vs',
          'localized', 'aeb', 'opp_cam', 'cam_alive']
STATES = {0: 'STOP', 1: 'REACTIVE', 2: 'GB_TRACK', 3: 'TRAILING', 4: 'OVERTAKE'}


def read_topic(db, topic):
    con = sqlite3.connect(db)
    row = con.execute('SELECT id FROM topics WHERE name=?', (topic,)).fetchone()
    if row is None:
        return []
    return con.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                       row).fetchall()


def parse_f32_array(data):
    """CDR std_msgs/Float32MultiArray: layout (dim sequence + offset) then float32 sequence."""
    off = 4                                        # encapsulation header
    ndim = struct.unpack_from('<I', data, off)[0]
    off += 4
    for _ in range(ndim):
        slen = struct.unpack_from('<I', data, off)[0]
        off += 4 + slen
        off = (off + 3) & ~3
        off += 8                                   # size, stride
    off += 4                                       # data_offset
    n = struct.unpack_from('<I', data, off)[0]
    off += 4
    return np.frombuffer(data, '<f4', n, off)


def parse_pose_stamped(data):
    off = 4 + 8                                    # header stamp
    slen = struct.unpack_from('<I', data, off)[0]
    off += 4 + slen
    off = 4 + ((off - 4 + 7) & ~7)                 # CDR aligns relative to the payload start
    x, y, z, qx, qy, qz, qw = struct.unpack_from('<7d', data, off)
    return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--map', required=True)
    ap.add_argument('--raceline', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    dbs = sorted(glob.glob(os.path.join(args.bag, '*.db3')))
    if not dbs:
        sys.exit('no .db3 in ' + args.bag + ' (record with --storage sqlite3)')
    tel = np.array([parse_f32_array(d) for db in dbs for _, d in read_topic(db, '/race_core/telemetry')])
    if len(tel) == 0:
        sys.exit('no /race_core/telemetry in the bag')
    T = {k: tel[:, i] for i, k in enumerate(FIELDS) if i < tel.shape[1]}
    truth = np.array([parse_pose_stamped(d) for db in dbs for _, d in read_topic(db, '/sim/pose')])

    # laps: telemetry lap counter increments
    lap_idx = np.flatnonzero(np.diff(T['laps']) > 0) + 1
    print(f'{len(tel)} telemetry samples over {T["t"][-1] - T["t"][0]:.1f} s; '
          f'race_core pipeline p50 {np.median(T["ms"]):.2f} ms p99 {np.percentile(T["ms"], 99):.2f} ms')
    print('lap  time_s  rms_d  max_d  mean_v  max_v  aeb  min_match  reactive_%')
    bounds = [0] + list(lap_idx) + [len(tel)]
    for k in range(1, len(bounds) - 1):
        a, b = bounds[k], bounds[k + 1]
        if b <= a:
            continue
        seg = slice(a, b)
        track = T['state'][seg] >= 2
        d = T['d'][seg][track] if track.any() else np.array([0.0])
        aeb_on = np.flatnonzero(np.diff(np.concatenate([[0], T['aeb'][seg]])) > 0).size
        full = k < len(bounds) - 2
        lap_t = T['last_lap'][bounds[k + 1]] if full else float('nan')
        print(f'{k:3d}  {lap_t:6.2f}  {np.sqrt(np.mean(d ** 2)):5.2f}  {np.max(np.abs(d)):5.2f}  '
              f'{np.mean(T["v"][seg]):6.2f}  {np.max(T["v"][seg]):5.2f}  {aeb_on:3d}  '
              f'{np.min(T["match"][seg]):9.2f}  {100 * np.mean(T["state"][seg] == 1):10.1f}'
              + ('' if full else '   (partial)'))

    # where on the lap the car struggles: largest lateral errors and emergency brakes, by arc length s
    track = T['state'] >= 2
    if track.any():
        bins = {}
        for s_, d_ in zip(T['s'][track], np.abs(T['d'][track])):
            k = int(s_ // 1.0)
            bins[k] = max(bins.get(k, 0.0), d_)
        worst = sorted(bins.items(), key=lambda kv: -kv[1])[:5]
        print('largest lateral error by 1 m sector (s_m: max |d|): ' +
              ', '.join(f'{k:.0f}-{k + 1:.0f}: {v:.2f}' for k, v in worst))
    aeb_s = T['s'][np.flatnonzero(np.diff(np.concatenate([[0], T['aeb']])) > 0)]
    if len(aeb_s):
        print('emergency brakes at s = ' + ', '.join(f'{v:.1f}' for v in aeb_s[:10]) + ' m')
    print('slow a corner: ros2 param set /race_core sectors "[s_start, s_end, 0.85]"')

    # plot
    meta = yaml.safe_load(open(args.map))
    img_path = meta['image'] if os.path.isabs(meta['image']) else os.path.join(os.path.dirname(args.map), meta['image'])
    if not os.path.exists(img_path):
        img_path = os.path.join(os.path.dirname(args.map), os.path.basename(meta['image']))
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    S = 3
    vis = cv2.resize(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), (img.shape[1] * S, img.shape[0] * S),
                     interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        return np.stack([((x - ox) / res) * S, (img.shape[0] - (y - oy) / res) * S], 1).astype(np.int32)

    rl = np.loadtxt(args.raceline, delimiter=',')
    cv2.polylines(vis, [px(rl[:, 1], rl[:, 2])], True, (255, 160, 0), 1)
    P = px(T['x'], T['y'])
    vmax = max(1e-3, T['v'].max())
    for i in range(1, len(P)):
        if T['state'][i] == 1:
            col = (255, 0, 255)
        else:
            f = T['v'][i] / vmax
            col = (0, int(255 * f), int(255 * (1 - f)))
        cv2.line(vis, tuple(P[i - 1]), tuple(P[i]), col, 1)
    for i in np.flatnonzero(T['aeb'] > 0):
        cv2.drawMarker(vis, tuple(P[i]), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 6, 1)
    if len(truth):
        cv2.polylines(vis, [px(truth[:, 0], truth[:, 1])], False, (80, 80, 80), 1)
    out = args.out or os.path.join(args.bag, 'lap_report.png')
    cv2.imwrite(out, vis)
    print('plot:', out)


if __name__ == '__main__':
    main()
