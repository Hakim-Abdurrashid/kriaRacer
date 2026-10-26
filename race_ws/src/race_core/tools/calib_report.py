#!/usr/bin/env python3
"""Vehicle calibration from a bag (RACE_PLAN.md §4.3): speed gain (ERPM) and steering gain/offset.

usage (ROS sourced):  calib_report.py BAG_DIR [--erpm-gain 4214] [--servo-gain -0.8552]
                                       [--servo-offset 0.5396] [--wheelbase 0.33]

Record (sqlite3) while driving:  ros2 bag record -s sqlite3 -o calib /scan /odom /ackermann_cmd \
                                     /odometry/filtered /camera/camera/imu
  speed:    3-4 straight runs towards a flat wall (start ~6 m away, 1-3 m/s, stop before 1 m)
  steering: steady circles both ways at ~1.5 m/s with 3-4 different stick positions incl. full lock

Speed: v_lidar = -d(front range)/dt on straight runs vs the VESC /odom speed -> ERPM gain correction.
Steering: effective angle atan(wheelbase * yaw_rate / v) vs the commanded angle (/ackermann_cmd is
what ackermann_mux actually sends to the VESC) -> servo gain/offset correction, per-side saturation.
Yaw rate from /odometry/filtered (EKF, base_link) if present, else the D435i gyro (-y of the optical frame).
"""
import argparse
import glob
import math
import os
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan


def read(dbs, topic, cls):
    out = []
    for db in dbs:
        con = sqlite3.connect(db)
        row = con.execute('SELECT id FROM topics WHERE name=?', (topic,)).fetchone()
        if row is None:
            continue
        for ts, data in con.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', row):
            out.append((ts * 1e-9, deserialize_message(data, cls)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--erpm-gain', type=float, default=4214.0)
    ap.add_argument('--servo-gain', type=float, default=-0.8552)
    ap.add_argument('--servo-offset', type=float, default=0.5396)
    ap.add_argument('--wheelbase', type=float, default=0.33)
    a = ap.parse_args()
    dbs = sorted(glob.glob(os.path.join(a.bag, '*.db3')))
    if not dbs:
        sys.exit('no .db3 files (record with -s sqlite3)')
    cmd = read(dbs, '/ackermann_cmd', AckermannDriveStamped) or read(dbs, '/drive', AckermannDriveStamped)
    odom = read(dbs, '/odom', Odometry)
    ekf = read(dbs, '/odometry/filtered', Odometry)
    imu = read(dbs, '/camera/camera/imu', Imu)
    scans = read(dbs, '/scan', LaserScan)
    print(f'messages: cmd {len(cmd)}, /odom {len(odom)}, EKF {len(ekf)}, imu {len(imu)}, scans {len(scans)}')
    if not cmd or not odom:
        sys.exit('need /ackermann_cmd (or /drive) and /odom')
    tc = np.array([t for t, _ in cmd])
    steer = np.array([m.drive.steering_angle for _, m in cmd])
    to = np.array([t for t, _ in odom])
    vo = np.array([m.twist.twist.linear.x for _, m in odom])

    # ---- speed: straight runs towards a wall
    k = 1.0
    if scans:
        ts = np.array([t for t, _ in scans])
        front, square = [], []
        for _, s in scans:
            r = np.asarray(s.ranges, dtype=np.float32)
            ang = s.angle_min + np.arange(len(r)) * s.angle_increment

            def at(deg, half=1.5):
                sel = (np.abs(ang - math.radians(deg)) < math.radians(half)) & np.isfinite(r) & (r > s.range_min)
                return float(np.median(r[sel])) if sel.any() else np.nan
            r0, rl, rr = at(0.0, 3.0), at(8.0), at(-8.0)
            front.append(r0)
            # a wall square to the car: both side beams see it at r0 / cos(8 deg), within 3%
            expect = r0 / math.cos(math.radians(8.0))
            square.append(abs(rl - expect) < 0.03 * expect and abs(rr - expect) < 0.03 * expect)
        front = np.array(front)
        square = np.array(square)
        st = np.interp(ts, tc, steer)
        v_od = np.interp(ts, to, vo)
        n = 4
        v_li = np.full(len(ts), np.nan)
        v_li[n:-n] = -(front[2 * n:] - front[:-2 * n]) / (ts[2 * n:] - ts[:-2 * n])
        ok = ((np.abs(st) < 0.05) & (v_od > 0.5) & (front > 0.8) & (front < 8.0) & (v_li > 0.3) &
              np.isfinite(v_li) & square)
        if ok.sum() >= 20:
            ratio = v_li[ok] / v_od[ok]
            k = float(np.median(ratio))
            print(f'\nSPEED ({ok.sum()} samples): true speed = {k:.3f} x VESC speed '
                  f'(IQR {np.percentile(ratio, 25):.3f}-{np.percentile(ratio, 75):.3f})')
            print(f'  -> vesc.yaml speed_to_erpm_gain: {a.erpm_gain / k:.0f}   (now {a.erpm_gain:.0f})')
        else:
            print(f'\nSPEED: only {ok.sum()} usable samples - drive straight at a wall from ~6 m, 1-3 m/s '
                  '(speed assumed correct for the steering fit below)')

    # ---- steering: effective angle from yaw rate
    if ekf:
        tw = np.array([t for t, _ in ekf])
        wz = np.array([m.twist.twist.angular.z for _, m in ekf])
        src = '/odometry/filtered'
    elif imu:
        tw = np.array([t for t, _ in imu])
        wz = -np.array([m.angular_velocity.y for _, m in imu])
        src = 'D435i gyro (-y)'
    else:
        print('\nSTEERING: no yaw-rate source (/odometry/filtered or /camera/camera/imu)')
        return
    # smooth the yaw rate over 0.2 s
    if len(tw) > 10:
        dt = np.median(np.diff(tw))
        w = max(1, int(0.2 / max(dt, 1e-3)))
        wz = np.convolve(wz, np.ones(w) / w, 'same')
    st = np.interp(tw, tc, steer)
    st_prev = np.interp(tw - 0.3, tc, steer)
    v = k * np.interp(tw, to, vo)
    ok = (v > 0.8) & (np.abs(st - st_prev) < 0.02) & (np.abs(st) > 0.03)
    if ok.sum() < 30:
        print(f'\nSTEERING: only {ok.sum()} usable samples - drive steady circles at >= 1 m/s, both ways')
        return
    eff = np.arctan(a.wheelbase * wz[ok] / v[ok])
    A = np.stack([st[ok], np.ones(ok.sum())], 1)
    (gain, off), *_ = np.linalg.lstsq(A, eff, rcond=None)
    print(f'\nSTEERING ({ok.sum()} samples, yaw rate from {src}): effective = {gain:.3f} x commanded {off:+.3f} rad')
    for name, side in (('left ', st[ok] > 0), ('right', st[ok] < 0)):
        if side.sum() >= 10:
            g, o = np.linalg.lstsq(A[side], eff[side], rcond=None)[0]
            print(f'  {name}: effective = {g:.3f} x cmd {o:+.3f}; max achieved {np.max(np.abs(eff[side])):.3f} rad '
                  f'at cmd {np.max(np.abs(st[ok][side])):.3f}')
    # actuation delay: lag that best aligns the yaw rate the commands ask for with the measured one
    t_u = np.arange(tw[0] + 1.0, tw[-1] - 1.0, 0.01)
    if len(t_u) > 500:
        v_u = k * np.interp(t_u, to, vo)
        moving = v_u > 0.5
        want = v_u * np.tan(np.interp(t_u, tc, steer)) / a.wheelbase
        best, lag_best = -1.0, 0.0
        for lag in np.arange(0.0, 0.31, 0.01):
            got = np.interp(t_u + lag, tw, wz)
            if moving.sum() > 300:
                c = np.corrcoef(want[moving], got[moving])[0, 1]
                if c > best:
                    best, lag_best = c, lag
        print(f'  actuation delay (command -> yaw rate) ~{lag_best:.2f} s (correlation {best:.2f}) '
              f'-> race_core pp.delay ~{lag_best:.2f}')
    print(f'  -> vesc.yaml steering_angle_to_servo_gain: {a.servo_gain / gain:.4f}   (now {a.servo_gain})')
    print(f'  -> vesc.yaml steering_angle_to_servo_offset: {a.servo_offset - a.servo_gain * off / gain:.4f}   '
          f'(now {a.servo_offset})')
    print('  Use the smaller per-side max achieved angle as max_steer for make_raceline.py and pp.max_steer_*.')


if __name__ == '__main__':
    main()
