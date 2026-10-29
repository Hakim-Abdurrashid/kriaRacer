#!/usr/bin/env python3
"""Sweep the steering servo (motor untouched): centre -> full right -> centre -> full left -> centre.

Run with only vesc_driver_node up (nothing else may publish /commands/servo/position):
  python3 servo_sweep.py [right_end] [left_end]     (defaults: vesc.yaml servo_max 0.77 / servo_min 0.05)
vesc_driver clips to servo_min/max anyway; higher servo value = steer right on this car.
"""
import sys
import time

import rclpy
from std_msgs.msg import Float64

CENTRE, STEP, STEP_TIME, HOLD = 0.50, 0.01, 0.15, 2.0


def main():
    right = float(sys.argv[1]) if len(sys.argv) > 1 else 0.77
    left = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
    rclpy.init()
    node = rclpy.create_node('servo_sweep')
    pub = node.create_publisher(Float64, '/commands/servo/position', 10)
    value = CENTRE

    def send(v, seconds):
        end = time.time() + seconds
        while time.time() < end:
            pub.publish(Float64(data=v))
            time.sleep(0.05)

    def ramp(target, label):
        nonlocal value
        print(f'-> {label} ({target:.2f})', flush=True)
        while abs(target - value) > 1e-6:
            value = round(value + max(-STEP, min(STEP, target - value)), 3)
            send(value, STEP_TIME)
        print(f'   at {value:.2f}, holding {HOLD:.0f} s', flush=True)
        send(value, HOLD)

    send(CENTRE, 1.0)
    for target, label in ((right, 'FULL RIGHT'), (CENTRE, 'centre'), (left, 'FULL LEFT'), (CENTRE, 'centre')):
        ramp(target, label)
    print('done, servo left at 0.50', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
