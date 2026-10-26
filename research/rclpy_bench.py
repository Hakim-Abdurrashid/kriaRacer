"""Per-scan cost of the Python path on the K26: deserialize LaserScan + typical numpy processing."""
import time, math
import numpy as np
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

scan = LaserScan()
scan.angle_min, scan.angle_max, scan.angle_increment = -2.356, 2.356, 0.004363
scan.ranges = [float(2.0 + math.sin(i * 0.01)) for i in range(1081)]
raw = serialize_message(scan)

def bench(fn, n=400):
    t0 = time.perf_counter()
    for _ in range(n): fn()
    return (time.perf_counter() - t0) / n * 1e3

d_ms = bench(lambda: deserialize_message(raw, LaserScan))
m = deserialize_message(raw, LaserScan)
np_ms = bench(lambda: np.asarray(m.ranges, dtype=np.float32))
def ftg_like():
    r = np.asarray(m.ranges, dtype=np.float32)
    r = np.clip(r, 0, 5.0)
    r = np.convolve(r, np.ones(3) / 3, mode='same')
    i = int(np.argmin(r)); r[max(0, i - 40):i + 40] = 0
    gaps = np.flatnonzero(r > 1.0)
    return int(gaps[len(gaps) // 2]) if len(gaps) else 540
ftg_ms = bench(ftg_like)
cmd = AckermannDriveStamped(); cmd.drive.speed = 3.0; cmd.drive.steering_angle = 0.1
s_ms = bench(lambda: serialize_message(cmd))
total = d_ms + ftg_ms + s_ms
print(f"deserialize LaserScan(1081): {d_ms:6.3f} ms")
print(f"np.asarray(ranges):          {np_ms:6.3f} ms")
print(f"FTG-like numpy processing:   {ftg_ms:6.3f} ms")
print(f"serialize Ackermann cmd:     {s_ms:6.3f} ms")
print(f"=> ~{total:.2f} ms/scan of pure Python work before any executor/DDS overhead = {total*40/10:.1f}% of one core at 40 Hz per node that touches /scan")
