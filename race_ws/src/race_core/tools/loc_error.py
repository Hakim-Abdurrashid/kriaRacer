#!/usr/bin/env python3
"""Localization error of race_core against simulator truth: loc_error.py BAG_DIR (needs /sim/pose)."""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lap_report as L  # noqa: E402

db = glob.glob(os.path.join(sys.argv[1], '*.db3'))[0]
tel = [(ts, L.parse_f32_array(d)) for ts, d in L.read_topic(db, '/race_core/telemetry')]
tru = [(ts, L.parse_pose_stamped(d)) for ts, d in L.read_topic(db, '/sim/pose')]
tt = np.array([t for t, _ in tru])
tx, ty = np.array([p[0] for _, p in tru]), np.array([p[1] for _, p in tru])
tyaw = np.unwrap([p[2] for _, p in tru])
e, ey = [], []
for ts, a in tel:
    if a[22] < 0.5 or a[6] < 0.5:      # localized and moving
        continue
    e.append(np.hypot(a[3] - np.interp(ts, tt, tx), a[4] - np.interp(ts, tt, ty)))
    ey.append(abs((a[5] - np.interp(ts, tt, tyaw) + np.pi) % (2 * np.pi) - np.pi))
e, ey = np.array(e), np.array(ey)
print(f'position error: mean {e.mean():.3f} m, p95 {np.percentile(e, 95):.3f} m, max {e.max():.3f} m; '
      f'heading p95 {np.percentile(ey, 95):.3f} rad (n={len(e)}, bag-time aligned)')
