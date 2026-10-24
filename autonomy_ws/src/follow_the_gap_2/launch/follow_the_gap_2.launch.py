import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('follow_the_gap_2'),
        'config',
        'params.yaml'
    )

    follow_the_gap_2_node = Node(
        package='follow_the_gap_2',
        executable='follow_the_gap_2_node',
        name='follow_the_gap_2_node',
        parameters=[LaunchConfiguration('params_file'), {
            'scan_topic': LaunchConfiguration('scan_topic'),
            'drive_topic': LaunchConfiguration('drive_topic'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config,
                              description='Controller parameter YAML file'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='Vehicle LaserScan input'),
        DeclareLaunchArgument('drive_topic', default_value='/drive',
                              description='Ackermann mux navigation input'),
        follow_the_gap_2_node,
    ])
