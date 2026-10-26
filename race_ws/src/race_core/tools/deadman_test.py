#!/usr/bin/env python3
"""Sim check of the deadman: with require_deadman on, race_core must stop the car when /joy stops.

Run against a running simulation (sim.launch.py) on the same ROS domain.
"""
import subprocess
import time

import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

rclpy.init()
node = rclpy.create_node('deadman_test')
state = {'v': 0.0}
node.create_subscription(Float32MultiArray, '/sim/status', lambda m: state.update(v=m.data[5]), 10)
joy = node.create_publisher(Joy, '/joy', 10)


def spin_for(sec, rb=None):
    t0 = time.time()
    while time.time() - t0 < sec:
        if rb is not None:
            j = Joy()
            j.header.stamp = node.get_clock().now().to_msg()
            j.buttons = [0] * 8
            j.buttons[5] = 1 if rb else 0
            joy.publish(j)
        rclpy.spin_once(node, timeout_sec=0.05)


def time_until(cond, limit, rb=None):
    t0 = time.time()
    while time.time() - t0 < limit:
        spin_for(0.05, rb)
        if cond():
            return time.time() - t0
    return None


spin_for(6.0)
print(f'driving without deadman requirement: v = {state["v"]:.2f} m/s')
subprocess.run(['ros2', 'param', 'set', '/race_core', 'require_deadman', 'true'], capture_output=True)
t = time_until(lambda: state['v'] < 0.1, 5.0)
print(f'deadman required, no /joy: stopped after {t} s')
t = time_until(lambda: state['v'] > 1.0, 6.0, rb=True)
print(f'RB held: moving (> 1 m/s) after {t} s')
spin_for(2.0, rb=True)
t = time_until(lambda: state['v'] < 0.1, 5.0)
print(f'/joy stopped (dongle lost): stopped after {t} s')
t = time_until(lambda: state['v'] > 1.0, 6.0, rb=True)
spin_for(1.0, rb=True)
t = time_until(lambda: state['v'] < 0.1, 5.0, rb=False)
print(f'RB released: stopped after {t} s')
