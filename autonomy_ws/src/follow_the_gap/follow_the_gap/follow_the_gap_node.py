#!/usr/bin/env python3
"""
ROS 2 node wrapping the Follow-the-Gap planner.

All ROS plumbing (scan/odom subscriptions, drive publishing, safety stop,
status logging, parameter loading) lives in :class:`BaseDriveNode`. This file
only declares the algorithm's tunables and hands each scan to the planner —
the pattern every new algorithm in this workspace should follow.
"""

import time

import rclpy

from autonomy_core.base_drive_node import BaseDriveNode

from .planner import FollowTheGapPlanner

# Algorithm tunables. Merged on top of BaseDriveNode's defaults (topics,
# frame_id, status_interval). See config/params.yaml for the deployed values.
#
# NOTE: safe_threshold and best_point_conv_size are measured in
# *scan indices*, evaluated after the FOV crop. They are resolution-dependent,
# so re-tune them if the LiDAR or fov_degrees changes.
DEFAULT_PARAMS = {
    'min_drive_speed': 0.6,  # provisional breakaway speed; verify on the track
    'emergency_distance': 0.3,
    'steering_response': 0.5,  # scan-matching estimate; verify wheel calibration
    'vehicle_length': 0.568,
    'lidar_offset': 0.27,
    'disparity_threshold': 0.3,
    'lookahead_distance': 1.0,
    'path_horizon': 2.0,
    'max_acceleration': 0.5,
    'max_lateral_acceleration': 1.0,
    'vehicle_width': 0.296,
    'safety_margin': 0.05,
    'wheelbase': 0.324,
    'min_gap_distance': 0.6,
    'stop_distance': 0.15,
    'reaction_time': 0.3,
    'braking_deceleration': 2.0,
    'fov_degrees': 180.0,          # forward field of view actually used
    'preprocess_conv_size': 3,     # moving-average window for noise smoothing
    'max_lidar_dist': 10.0,        # ranges are clamped to this (metres)
    'safe_threshold': 30,          # min gap width to be preferred, in beams
    'best_point_conv_size': 40,   # aim-point smoothing window, in beams
    'max_steer': 0.349066,         # 20 deg steering limit
    'straights_steering_angle': 0.10472,  # 6 deg
    'fast_steering_angle': 0.0542,       # 3.1 deg
    'corners_speed': 0.8,
    'straights_speed': 1.0,
    'fast_speed': 1.2,
}


class FollowTheGapNode(BaseDriveNode):
    """Reactive gap-following node: subscribes to /scan, publishes /drive."""

    def __init__(self):
        """Initialise the base node and construct the planner."""
        super().__init__('follow_the_gap_node', DEFAULT_PARAMS, uses_odom=False)
        self.planner = FollowTheGapPlanner(self.params)
        self._last_plan_time = time.monotonic()

    def publish_stop(self):
        """Reset acceleration history when the base node rejects a scan."""
        self.planner.reset()
        super().publish_stop()

    def compute_drive(self, scan_msg):
        """Delegate to the planner; the base class handles publishing/stops."""
        now = time.monotonic()
        dt = now - self._last_plan_time
        self._last_plan_time = now
        return self.planner.plan(
            scan_msg.ranges, scan_msg.angle_min, scan_msg.angle_increment, dt=dt)


def main(args=None):
    """Spin the Follow-the-Gap node until interrupted."""
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
