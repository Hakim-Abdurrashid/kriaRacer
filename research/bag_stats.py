import sys, math, statistics as st
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def open_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    return r, types

for path in sys.argv[1:]:
    r, types = open_bag(path)
    drive_v, drive_s, odom_v, odom_w, scan_t, odom_frames = [], [], [], [], [], set()
    scan_info = None
    front_min = []
    while r.has_next():
        topic, data, t = r.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == '/drive':
            drive_v.append(msg.drive.speed); drive_s.append(msg.drive.steering_angle)
        elif topic == '/odom':
            odom_v.append(msg.twist.twist.linear.x); odom_w.append(msg.twist.twist.angular.z)
            odom_frames.add((msg.header.frame_id, msg.child_frame_id))
        elif topic == '/scan':
            scan_t.append(t * 1e-9)
            if scan_info is None:
                scan_info = (msg.angle_min, msg.angle_max, msg.angle_increment, len(msg.ranges), msg.range_min, msg.range_max, msg.scan_time, msg.time_increment)
            n = len(msg.ranges); c = n // 2
            left = [x for x in msg.ranges[c + int(n*0.30):c + int(n*0.36)] if 0.05 < x < 30]
            right = [x for x in msg.ranges[c - int(n*0.36):c - int(n*0.30)] if 0.05 < x < 30]
            if left and right:
                front_min.append(min(left) + min(right))
    print(f'===== {path}')
    if drive_v:
        nz = [v for v in drive_v if abs(v) > 0.05]
        print(f'  /drive: n={len(drive_v)} speed mean(nonzero)={st.mean(nz) if nz else 0:.2f} max={max(drive_v):.2f}  steer |max|={max(abs(s) for s in drive_s):.3f} rad')
    if odom_v:
        nz = [v for v in odom_v if abs(v) > 0.05]
        print(f'  /odom: n={len(odom_v)} frames={odom_frames} v mean(moving)={st.mean(nz) if nz else 0:.2f} max={max(odom_v):.2f} m/s  |yaw rate| max={max(abs(w) for w in odom_w):.2f}')
    if scan_t:
        dts = [b - a for a, b in zip(scan_t, scan_t[1:])]
        print(f'  /scan: n={len(scan_t)} rate~{1/st.median(dts):.1f} Hz  max gap={max(dts)*1000:.0f} ms  info(angle_min,max,inc,N,rmin,rmax,scan_time,time_inc)={tuple(round(x,5) for x in scan_info)}')
    if front_min:
        print(f'  approx corridor width (min left+right at ~+-90deg): median={st.median(front_min):.2f} m  p10={sorted(front_min)[len(front_min)//10]:.2f}  p90={sorted(front_min)[9*len(front_min)//10]:.2f}')
