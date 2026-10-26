"""Extract camera frames from ROS 2 sqlite3 bags as JPEGs (for detector validation / labelling).

usage: python3 bag_frames.py OUT_DIR STRIDE BAG_DIR [BAG_DIR ...]
Writes OUT_DIR/<bagname>_<index>.jpg for every STRIDE-th frame of /camera/camera/color/image_raw.
"""
import os
import sqlite3
import sys

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

TOPIC = '/camera/camera/color/image_raw'


def main():
    out, stride, bags = sys.argv[1], int(sys.argv[2]), sys.argv[3:]
    os.makedirs(out, exist_ok=True)
    total = 0
    for bag in bags:
        dbs = sorted(f for f in os.listdir(bag) if f.endswith('.db3'))
        name = os.path.basename(os.path.normpath(bag))
        for db in dbs:
            con = sqlite3.connect(os.path.join(bag, db))
            tid = con.execute('SELECT id FROM topics WHERE name=?', (TOPIC,)).fetchone()
            if tid is None:
                continue
            try:
                ids = [r[0] for r in con.execute('SELECT id FROM messages WHERE topic_id=? ORDER BY timestamp', tid)]
            except sqlite3.DatabaseError as e:          # truncated recordings
                print(name, 'skipped:', e)
                continue
            for i in range(0, len(ids), stride):    # fetch only the frames we keep (blobs are big)
                ts, data = con.execute('SELECT timestamp, data FROM messages WHERE id=?', (ids[i],)).fetchone()
                msg = deserialize_message(data, Image)
                img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                cv2.imwrite(os.path.join(out, f'{name}_{i:05d}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
                total += 1
        print(name, 'done', flush=True)
    print('frames written:', total)


if __name__ == '__main__':
    main()
