"""End-to-end rclpy cost: 40 Hz LaserScan publisher -> Python 'planner' node -> Ackermann publisher. Reports CPU of each process."""
import math, os, sys, time, resource
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

role = sys.argv[1]
rclpy.init()
if role == 'pub':
    n = Node('scan_pub_bench'); pub = n.create_publisher(LaserScan, '/bench_scan', 10)
    msg = LaserScan(); msg.angle_min, msg.angle_max, msg.angle_increment = -2.356, 2.356, 0.004363
    msg.ranges = [float(2.0 + math.sin(i * 0.01)) for i in range(1081)]
    n.create_timer(0.025, lambda: pub.publish(msg))
else:
    n = Node('planner_bench'); out = n.create_publisher(AckermannDriveStamped, '/bench_drive', 10); cnt = [0]
    def cb(m):
        r = np.clip(np.asarray(m.ranges, dtype=np.float32), 0, 5.0)
        r = np.convolve(r, np.ones(3) / 3, mode='same'); i = int(np.argmin(r)); r[max(0, i - 40):i + 40] = 0
        c = AckermannDriveStamped(); c.drive.speed = 2.0; c.drive.steering_angle = float((np.argmax(r) - 540) * 0.0044); out.publish(c); cnt[0] += 1
    n.create_subscription(LaserScan, '/bench_scan', cb, 10)
t_end = time.time() + 15.0; c0 = time.process_time(); w0 = time.time()
while time.time() < t_end: rclpy.spin_once(n, timeout_sec=0.1)
cpu = time.process_time() - c0; wall = time.time() - w0
extra = f", callbacks={cnt[0]} ({cnt[0]/wall:.1f} Hz)" if role != 'pub' else ""
print(f"[{role}] CPU {100*cpu/wall:.1f}% of one core over {wall:.1f}s{extra}")
