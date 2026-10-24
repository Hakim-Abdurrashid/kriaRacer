"""
Shared base class for reactive autonomy nodes.

``BaseDriveNode`` owns every piece of ROS plumbing that an autonomy algorithm
on this car needs but should not have to re-implement:

* declaring parameters from a single dict and exposing them as ``self.params``,
* subscribing to ``/scan`` (and optionally ``/odom``),
* publishing ``ackermann_msgs/AckermannDriveStamped`` to ``/drive``,
* a safety-stop path that fires on missing/invalid LiDAR data,
* rate-limited status logging.

A concrete algorithm subclasses this, declares its tunables in a
``DEFAULT_PARAMS`` dict, and implements a single method, :meth:`compute_drive`,
which turns one scan into a ``(steering_angle, speed)`` pair. Returning
``None`` from that method requests a stop. The subclass never touches a ROS
message, which keeps the algorithm code small and testable.
"""

from argparse import Namespace

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

# Parameters every drive node gets for free. A subclass' DEFAULT_PARAMS is
# merged on top of these and may override them.
BASE_DEFAULTS = {
    'scan_topic': '/scan',
    'drive_topic': '/drive',
    'odom_topic': '/odom',
    'image_topic': '/camera/camera/color/image_raw',
    'drive_mask' : '/yolo/drivable_mask',
    'frame_id': 'base_link',
    'status_interval': 0.5,  # seconds between terminal status messages
}


class BaseDriveNode(Node):
    """Base ROS node wiring sensors to drive output for a reactive planner."""

    def __init__(self, node_name, default_params=None, uses_odom=False, uses_camera=False, uses_lidar=True):
        """
        Set up parameters, publishers, and subscribers.

        :param node_name: name to register this node under.
        :param default_params: dict of ``{param_name: default_value}`` specific
            to the algorithm. Merged on top of :data:`BASE_DEFAULTS`.
        :param uses_odom: when True, also subscribe to the odometry topic and
            cache the latest message in ``self.latest_odom``.
        """
        super().__init__(node_name)

        merged = dict(BASE_DEFAULTS)
        if default_params:
            merged.update(default_params)
        self.params = self._load_params(merged)

        # Statistics and rate-limiting state for status logging.
        self.min_speed = float('inf')
        self.max_speed = float('-inf')
        self.min_angle = float('inf')
        self.max_angle = float('-inf')
        self.last_status_time_ns = 0

        # Standard interface: subscribe to the LiDAR, publish drive commands.
        if uses_lidar:
            self.scan_sub = self.create_subscription(
                LaserScan, self.params.scan_topic, self.scan_callback, 10)

        # Odometry is optional: only algorithms that need pose/velocity ask for
        # it (e.g. pure pursuit). Reactive gap-followers leave it off.
        self.latest_odom = None
        if uses_odom:
            self.odom_sub = self.create_subscription(
                Odometry, self.params.odom_topic, self.odom_callback, 10)

        self.get_logger().info(f'{node_name} has been initialized')

        if uses_camera:
            self.image_sub = self.create_subscription(
               Image, self.params.image_topic, self.image_callback, 10)
            
            self.image_sub= self.create_subscription(
                Image, self.params.drive_mask, self.mask_callback, 10
            )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.params.drive_topic, 10)
        self.get_logger().info(f'{node_name} has been initialized')

    def _load_params(self, defaults):
        """Declare each parameter in ``defaults`` and return them as a Namespace."""
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        loaded = {name: self.get_parameter(name).value for name in defaults}
        return Namespace(**loaded)

    def odom_callback(self, odom_msg):
        """Cache the most recent odometry message for subclasses to read."""
        self.latest_odom = odom_msg

    def image_callback(self, image_msg):
        self.latest_image=image_msg


    def mask_callback(self, mask_msg):
        self.latest_mask=mask_msg
        result = self.compute_drive(mask_msg)

        if result is None:
            self.publish_stop()
            return

        steering_angle, speed = result
        self.publish_drive(steering_angle, speed)
        self._log_status(speed, steering_angle)
        
    def scan_callback(self, scan_msg):
        """Validate the scan, dispatch to :meth:`compute_drive`, and publish."""
        ranges = np.array(scan_msg.ranges)

        # Guard against a dead or empty LiDAR feed: stop the car rather than
        # act on garbage.
        if ranges.size == 0 or np.count_nonzero(np.isfinite(ranges)) == 0:
            self._warn_rate_limited(
                'No valid LiDAR input received. Publishing stop command.')
            self.publish_stop()
            return

        result = self.compute_drive(scan_msg)

        # The algorithm may decline to produce a command (e.g. no gap found);
        # treat that as a request to stop.
        if result is None:
            self.publish_stop()
            return

        steering_angle, speed = result
        self.publish_drive(steering_angle, speed)
        self._log_status(speed, steering_angle)

    def compute_drive(self, scan_msg):
        """
        Compute a drive command from one scan. Must be overridden.

        :param scan_msg: the incoming ``sensor_msgs/LaserScan``.
        :returns: a ``(steering_angle_rad, speed_mps)`` tuple, or ``None`` to
            request a safety stop.
        """
        raise NotImplementedError(
            'Subclasses of BaseDriveNode must implement compute_drive()')

    def publish_drive(self, steering_angle, speed):
        """Publish an Ackermann command with the given steering and speed."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.params.frame_id
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def publish_stop(self):
        """Publish a zero-speed, zero-steering command (the safety stop)."""
        self.publish_drive(0.0, 0.0)

    def _warn_rate_limited(self, message):
        """Emit a warning at most once per ``status_interval`` seconds."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_status_time_ns > self.params.status_interval * 1e9:
            self.get_logger().warning(message)
            self.last_status_time_ns = now_ns

    def _log_status(self, speed, steering_angle):
        """Track speed/angle extremes and log a summary, rate-limited."""
        self.min_speed = min(self.min_speed, speed)
        self.max_speed = max(self.max_speed, speed)
        self.min_angle = min(self.min_angle, steering_angle)
        self.max_angle = max(self.max_angle, steering_angle)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_status_time_ns <= self.params.status_interval * 1e9:
            return

        # Angles are reported in degrees for human readability.
        deg_current = np.degrees(steering_angle)
        deg_min = np.degrees(self.min_angle)
        deg_max = np.degrees(self.max_angle)
        self.get_logger().info(
            f'Status - current_speed={speed:.2f}, current_angle_deg={deg_current:.2f}, '
            f'min_speed={self.min_speed:.2f}, max_speed={self.max_speed:.2f}, '
            f'min_angle_deg={deg_min:.2f}, max_angle_deg={deg_max:.2f}')
        self.last_status_time_ns = now_ns
