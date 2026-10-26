#!/usr/bin/env python3
"""Synthetic test track (map_server yaml + png) for simulator tests, e.g. overtaking on a wide track.

usage: make_test_track.py OUT_PREFIX [--width 2.0] [--straight 8.0] [--radius 2.5]
Stadium shape: two straights joined by semicircles; the start line (map 0,0, heading +x) is in the
middle of the bottom straight. Walls are 0.1 m thick.
"""
import argparse
import os

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('out')
ap.add_argument('--width', type=float, default=2.0)
ap.add_argument('--straight', type=float, default=8.0)
ap.add_argument('--radius', type=float, default=2.5, help='centerline radius of the turns')
a = ap.parse_args()
res = 0.05
half = a.straight / 2
# centerline: bottom straight y=0 from x=-half..half, turns centred at (+-half, R)
pad = a.radius + a.width / 2 + 1.0
xmin, xmax, ymin, ymax = -half - pad, half + pad, -a.width / 2 - 1.0, 2 * a.radius + a.width / 2 + 1.0
W, H = int((xmax - xmin) / res), int((ymax - ymin) / res)
img = np.full((H, W), 205, np.uint8)                       # unknown
ys, xs = np.mgrid[0:H, 0:W]
X = xmin + (xs + 0.5) * res
Y = ymin + (H - 1 - ys + 0.5) * res                        # row 0 = top
# distance from the centerline loop
cx = np.clip(X, -half, half)
d_straight = np.where(np.abs(X) <= half, np.minimum(np.abs(Y), np.abs(Y - 2 * a.radius)), np.inf)
d_turn = np.where(np.abs(X) > half, np.abs(np.hypot(X - np.sign(X) * half, Y - a.radius) - a.radius), np.inf)
d = np.minimum(d_straight, d_turn)
img[d <= a.width / 2] = 254                                # free
img[(d > a.width / 2) & (d <= a.width / 2 + 0.1)] = 0      # walls
cv2.imwrite(a.out + '.png', img)
with open(a.out + '.yaml', 'w') as f:
    f.write(f'image: {os.path.basename(a.out)}.png\nmode: trinary\nresolution: {res}\n'
            f'origin: [{xmin:.3f}, {ymin:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print('wrote', a.out + '.yaml', f'({W}x{H} px, lap ~{2 * a.straight + 2 * np.pi * a.radius:.1f} m)')
