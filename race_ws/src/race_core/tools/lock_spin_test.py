#!/usr/bin/env python3
"""ON THE STAND ONLY: spin the wheels at full right lock, then full left lock (bypasses deadman and mux).

Run with only vesc_driver_node up (it must allow the servo ends: -p servo_max:=1.0):
  python3 lock_spin_test.py SPEED_M_S [both|right|left] [right_servo] [left_servo]
Per side: steer to lock, ramp 2 s to speed, hold 3 s, ramp 2 s down, centre. Always ends at speed 0.
"""
import sys
import time

import rclpy
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

ERPM_PER_M_S, CENTRE, RATE = 4214.0, 0.50, 50.0


def main():
    speed = float(sys.argv[1])
    sides = sys.argv[2] if len(sys.argv) > 2 else 'both'
    right = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    left = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05
    rclpy.init()
    node = rclpy.create_node('lock_spin_test')
    motor = node.create_publisher(Float64, '/commands/motor/speed', 10)
    servo = node.create_publisher(Float64, '/commands/servo/position', 10)
    peak = {'rpm': 0.0, 'current': 0.0}

    def on_core(msg):
        peak['rpm'] = max(peak['rpm'], abs(msg.state.speed))
        peak['current'] = max(peak['current'], abs(msg.state.current_motor))
    node.create_subscription(VescStateStamped, '/sensors/core', on_core, 10)

    def run(seconds, steer, erpm_from, erpm_to):
        start = time.time()
        while (t := time.time() - start) < seconds:
            servo.publish(Float64(data=steer))
            motor.publish(Float64(data=erpm_from + (erpm_to - erpm_from) * t / seconds))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / RATE)

    erpm = speed * ERPM_PER_M_S
    try:
        run(1.0, CENTRE, 0.0, 0.0)
        for steer, label in ((right, 'FULL RIGHT'), (left, 'FULL LEFT')):
            if sides != 'both' and not label.endswith(sides.upper()):
                continue
            peak.update(rpm=0.0, current=0.0)
            print(f'{label} servo {steer:.2f}, ramping to {speed:.1f} m/s ({erpm:.0f} erpm)', flush=True)
            run(1.0, steer, 0.0, 0.0)
            run(2.0, steer, 0.0, erpm)
            run(3.0, steer, erpm, erpm)
            run(2.0, steer, erpm, 0.0)
            run(1.0, CENTRE, 0.0, 0.0)
            print(f'   peak {peak["rpm"]:.0f} erpm = {peak["rpm"] / ERPM_PER_M_S:.2f} m/s, '
                  f'peak motor current {peak["current"]:.1f} A', flush=True)
    finally:
        for _ in range(20):
            motor.publish(Float64(data=0.0))
            servo.publish(Float64(data=CENTRE))
            time.sleep(0.02)
        print('done, speed 0, servo 0.50', flush=True)


if __name__ == '__main__':
    main()
