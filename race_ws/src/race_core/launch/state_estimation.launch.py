"""Odometry: D435i IMU (explicit rates) + robot_localization EKF (RACE_PLAN.md §4.1, §5.2).

Run after subsystem_stack bringup (which provides /odom from the VESC and base_link->laser).

The camera mount pose is a placeholder until it is measured on the car: pass
cam_x/cam_y/cam_z/cam_yaw/cam_pitch (metres, radians) for base_link -> camera_link.
Only the gyro's yaw rate is used, so for the EKF the mount position barely matters,
but its orientation does (a tilted camera mixes axes).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('race_core')
    args = {k: DeclareLaunchArgument(k, default_value=v) for k, v in {
        'cam_x': '0.20', 'cam_y': '0.0', 'cam_z': '0.18', 'cam_yaw': '0.0', 'cam_pitch': '0.0',
        'color': 'true'}.items()}
    lc = LaunchConfiguration
    camera = Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='camera', name='camera', output='screen',
        parameters=[{
            'enable_gyro': True, 'enable_accel': True,
            'gyro_fps': 200, 'accel_fps': 100,     # set explicitly; a July bag showed ~81/41 Hz
            'unite_imu_method': 2,                 # linear interpolation -> /camera/camera/imu
            'enable_depth': False, 'enable_infra1': False, 'enable_infra2': False,
            'enable_color': lc('color'),           # dpu_detector will own color later
            'rgb_camera.color_profile': '640x480x30',
            'publish_tf': True,
        }])
    cam_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_link_to_camera',
        arguments=['--x', lc('cam_x'), '--y', lc('cam_y'), '--z', lc('cam_z'),
                   '--yaw', lc('cam_yaw'), '--pitch', lc('cam_pitch'), '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'])
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[os.path.join(share, 'config', 'ekf.yaml')])
    return LaunchDescription([*args.values(), camera, cam_tf, ekf])
