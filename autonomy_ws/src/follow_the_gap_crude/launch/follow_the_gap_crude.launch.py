import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('follow_the_gap_crude'),
        'config',
        'params.yaml'
    )

    follow_the_gap_crude_node = Node(
        package='follow_the_gap_crude',
        executable='follow_the_gap_crude_node',
        name='follow_the_gap_crude_node',
        parameters=[config],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([follow_the_gap_crude_node])
