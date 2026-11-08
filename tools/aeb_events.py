#!/usr/bin/env python3
"""Plot the LiDAR points behind race_core's emergency brakes on the map.

For each AEB onset in /race_core/telemetry (field 23), take the scan at that time, put its points in
the map frame with race_core's pose (telemetry x, y, yaw), and mark points farther than 0.15 m from
mapped walls ("foreign"). Writes <out>.png and prints one line per event.

usage: aeb_events.py BAG MAP_YAML OUT_PNG [max_events]
"""
import sys

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

bag, map_yaml, out = sys.argv[1:4]
maxev = int(sys.argv[4]) if len(sys.argv) > 4 else 40
cfg = yaml.safe_load(open(map_yaml))
img = cv2.imread(cfg['image'] if cfg['image'].startswith('/') else
                 map_yaml.rsplit('/', 1)[0] + '/' + cfg['image'], 0)
res, ox, oy = cfg['resolution'], cfg['origin'][0], cfg['origin'][1]
H, W = img.shape
occ = (img < 100).astype(np.uint8)
dist = cv2.distanceTransform(1 - occ, cv2.DIST_L2, 5) * res

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/race_core/telemetry']))
last_scan = None
events = []
prev_aeb = 0.0
traj = []
while r.has_next():
    t, d, ts = r.read_next()
    if t == '/scan':
        last_scan = deserialize_message(d, LaserScan)
        continue
    m = deserialize_message(d, Float32MultiArray).data
    traj.append((m[3], m[4], m[1], m[2]))
    if m[23] > 0.5 and prev_aeb < 0.5 and last_scan is not None and len(events) < maxev:
        events.append((ts * 1e-9, list(m), last_scan))
    prev_aeb = m[23]

vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
S = 5
vis = cv2.resize(vis, (W * S, H * S), interpolation=cv2.INTER_NEAREST)


def px(x, y):
    return int((x - ox) / res * S), int((H - (y - oy) / res) * S)


for x, y, st, eng in traj:
    if eng > 0.5:
        cv2.circle(vis, px(x, y), 1, (0, 160, 0) if st >= 2 else (0, 160, 255), -1)
t0 = events[0][0] if events else 0
for i, (ts, m, scan) in enumerate(events):
    x, y, yaw, v, steer = m[3], m[4], m[5], m[6], m[10]
    a = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    rr = np.array(scan.ranges)
    ok = np.isfinite(rr) & (rr > 0.05)
    lx, ly = 0.27 + rr[ok] * np.cos(a[ok]), rr[ok] * np.sin(a[ok])
    mx, my = x + np.cos(yaw) * lx - np.sin(yaw) * ly, y + np.sin(yaw) * lx + np.cos(yaw) * ly
    ci = np.clip(((mx - ox) / res).astype(int), 0, W - 1)
    cj = np.clip((H - 1 - (my - oy) / res).astype(int), 0, H - 1)
    dd = dist[cj, ci]
    # points in the corridor ahead (0.45..1.5 m ahead, |lateral| < 0.25 in base_link, straight approx)
    ahead = (lx > 0.4) & (lx < 1.6) & (np.abs(ly) < 0.35)
    foreign = dd > 0.15
    col = (0, 0, 255)
    for X, Y, f, ah in zip(mx, my, foreign, ahead):
        if ah:
            cv2.circle(vis, px(X, Y), 2, (255, 0, 255) if f else (255, 160, 0), -1)
    cv2.arrowedLine(vis, px(x, y), px(x + 0.3 * np.cos(yaw), y + 0.3 * np.sin(yaw)), col, 2)
    cv2.putText(vis, str(i + 1), px(x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    fa = foreign & ahead
    near = np.where(fa)[0]
    desc = ''
    if len(near):
        k = near[np.argmin(lx[near])]
        desc = f'nearest foreign ahead: base_link ({lx[k]:.2f}, {ly[k]:+.2f}) m, {dd[k]:.2f} m from mapped wall'
    print(f'#{i+1} t+{ts-t0:5.1f}s pose ({x:5.2f},{y:5.2f}) yaw {np.degrees(yaw):5.0f} v {v:.2f} steer {steer:+.2f} '
          f'state {int(m[1])} match {m[12]:.2f} d {m[8]:+.2f} ttc {m[15]:.2f} | foreign ahead {fa.sum()} | {desc}')
cv2.imwrite(out, vis)
print('wrote', out, len(events), 'events')
