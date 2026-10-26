#!/usr/bin/env python3
"""Rebroadcast an Odometry topic as the odom -> base_link TF (bag replays recorded without the EKF)."""
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

rclpy.init()
node = rclpy.create_node('odom_to_tf')
br = TransformBroadcaster(node)


def cb(m):
    t = TransformStamped()
    t.header = m.header
    t.child_frame_id = m.child_frame_id or 'base_link'
    t.transform.translation.x = m.pose.pose.position.x
    t.transform.translation.y = m.pose.pose.position.y
    t.transform.rotation = m.pose.pose.orientation
    br.sendTransform(t)


node.create_subscription(Odometry, sys.argv[1] if len(sys.argv) > 1 else '/odom', cb, 50)
rclpy.spin(node)
