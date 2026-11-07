"""Race-day mapping: slam_toolbox (online async) on EKF odometry. Needs bringup.launch.py (scan,
base_link -> laser) and state_estimation.launch.py (odom -> base_link) running.

Start with the car on the start line facing the race direction: that pose becomes the map origin,
which is where race_core starts localizing. Drive 1-2 smooth laps, then ./race.sh savemap NAME.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('race_core'), 'config', 'slam_toolbox.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
