"""DPU car detector on the RealSense color stream (RACE_PLAN.md §5.4).

  ros2 launch dpu_detector dpu_detector.launch.py model:=/home/ubuntu/2x1_w/models/yolov8n_car_v2neg.xmodel

Needs the DPU overlay loaded (setup/dpu-overlay.service) and the camera running with color enabled
(race_core state_estimation.launch.py color:=true). Pinned to core 1 (RACE_PLAN.md §5.3).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='/home/ubuntu/2x1_w/models/yolov8n_car_v2neg.xmodel'),
        DeclareLaunchArgument('conf', default_value='0.35'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        Node(package='dpu_detector', executable='dpu_detector_node', name='dpu_detector',
             output='screen', prefix='taskset -c 1',
             parameters=[{'model': lc('model'),
                          'conf': ParameterValue(lc('conf'), value_type=float),
                          'camera_yaw': ParameterValue(lc('camera_yaw'), value_type=float)}]),
    ])
