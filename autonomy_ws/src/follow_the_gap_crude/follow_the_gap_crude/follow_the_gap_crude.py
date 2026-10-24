#!/usr/bin/env python3
"""
ROS 2 node wrapping the Follow-the-Gap planner.

All ROS plumbing (scan/odom subscriptions, drive publishing, safety stop,
status logging, parameter loading) lives in :class:`BaseDriveNode`. This file
only declares the algorithm's tunables and hands each scan to the planner —
the pattern every new algorithm in this workspace should follow.
"""

import rclpy

from autonomy_core.base_drive_node import BaseDriveNode
from cv_bridge import CvBridge
from .planner import FollowTheGapPlanner

# Algorithm tunables. Merged on top of BaseDriveNode's defaults (topics,
# frame_id, status_interval). See config/params.yaml for the deployed values.
#
# NOTE: bubble_radius, safe_threshold and best_point_conv_size are measured in
# *scan indices*, evaluated after the FOV crop. They are resolution-dependent,
# so re-tune them if the LiDAR or fov_degrees changes.
DEFAULT_PARAMS = {
    'fov_degrees': 180.0,          # forward field of view actually used
    'bubble_radius': 30,           # half-width of the safety bubble, in beams
    'preprocess_conv_size': 3,     # moving-average window for noise smoothing
    'max_lidar_dist': 10.0,        # ranges are clamped to this (metres)
    'safe_threshold': 15,          # min gap width to be preferred, in beams
    'best_point_conv_size': 200,   # aim-point smoothing window, in beams
    'max_steer': 0.349066,         # 20 deg steering limit
    'steering_gain': 0.5,          # bearing-to-steering damping factor
    'straights_steering_angle': 0.1396,  # 8 deg
    'fast_steering_angle': 0.0698,       # 4 deg
    'corners_speed': 1.5,
    'straights_speed': 2.0,
    'fast_speed': 3.5,
    'min_clearance': 10.0
}


class FollowTheGapCrudeNode(BaseDriveNode):
    """Reactive gap-following node: subscribes to /scan, publishes /drive."""

    def __init__(self):
        """Initialise the base node and construct the planner."""
        super().__init__('follow_the_gap_crude_node', DEFAULT_PARAMS, uses_odom=False, uses_camera=True, uses_lidar=False) #added this
        self.planner = FollowTheGapPlanner(self.params)
        self.bridge=CvBridge()

    def compute_drive(self, image_msg):
        """Delegate to the planner; the base class handles publishing/stops."""
        cv_mask = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='mono8')
        return self.planner.plan(cv_mask)


def main(args=None):
    """Spin the Follow-the-Gap node until interrupted."""
    rclpy.init(args=args)
    node = FollowTheGapCrudeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
