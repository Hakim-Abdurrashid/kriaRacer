#!/usr/bin/env python3
"""Per-stint summary from a race_core log (any track shape: works on figure 8s where the gyro lap
counter cannot). A stint = from 'deadman HELD' to 'deadman released'. Reports duration, mean and max
speed from the 1 Hz status lines, emergency brakes, and the speed scale in force.

usage: stints.py CORE_LOG [since HH:MM]
"""
import re
import sys
import time

log = open(sys.argv[1], errors='ignore').read().splitlines()
since = sys.argv[2] if len(sys.argv) > 2 else None
ts_re = re.compile(r'\[(\d+)\.(\d+)\] \[race_core\]: (.*)')
scale = '1.0'
stints, cur = [], None
for line in log:
    line = re.sub(r'\x1b\[[0-9;]*m', '', line)
    m = ts_re.search(line)
    if not m:
        continue
    t = int(m.group(1)) + int(m.group(2)) * 1e-9
    msg = m.group(3)
    if 'reactive_speed_scale' in msg and 'up:' in msg:
        scale = re.search(r'reactive_speed_scale=([\d.]+)', msg).group(1)
    if msg.startswith('deadman HELD'):
        cur = {'t0': t, 'v': [], 'aeb': 0, 'scale': scale}
    elif msg.startswith('deadman released') and cur:
        cur['t1'] = t
        stints.append(cur)
        cur = None
    elif cur is not None:
        if msg.startswith('EMERGENCY'):
            cur['aeb'] += 1
        sm = re.match(r'\w+ \| pose .*\| v ([-\d.]+) ref', msg)
        if sm:
            cur['v'].append(float(sm.group(1)))
for s in stints:
    hm = time.strftime('%H:%M:%S', time.localtime(s['t0']))
    if since and hm < since:
        continue
    v = [x for x in s['v'] if x > 0.3]
    dur = s['t1'] - s['t0']
    if dur < 3:
        continue
    print(f"{hm}  {dur:5.1f} s  mean {sum(v)/len(v) if v else 0:.2f} m/s  max {max(v) if v else 0:.2f}  "
          f"~{(sum(v)/len(v) if v else 0)*dur:5.1f} m  AEB {s['aeb']}")
