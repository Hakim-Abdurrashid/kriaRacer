"""Launch race_core pinned to cores 2-3 (RACE_PLAN.md §5.3).

  ros2 launch race_core race_core.launch.py                       # REACTIVE (follow-the-gap)
  ros2 launch race_core race_core.launch.py mode:=race map:=/path/track.yaml \
      raceline:=/path/raceline.csv speed_scale:=0.5

Real-time priority (chrt -f) needs CAP_SYS_NICE; pass rt:=true only after
setting that up, otherwise the node runs at normal priority on cores 2-3.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('race_core'), 'config', 'race_core.yaml')
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('rt', default_value='false'),
        DeclareLaunchArgument('mode', default_value='reactive'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('raceline', default_value=''),
        DeclareLaunchArgument('speed_scale', default_value='0.5'),
        DeclareLaunchArgument('reactive_speed_scale', default_value='1.0'),
        DeclareLaunchArgument('deadman', default_value='true'),
    ]
    overrides = {
        'mode': lc('mode'), 'map_yaml': lc('map'), 'raceline_csv': lc('raceline'),
        'speed_scale': ParameterValue(lc('speed_scale'), value_type=float),
        'reactive_speed_scale': ParameterValue(lc('reactive_speed_scale'), value_type=float),
        'require_deadman': ParameterValue(lc('deadman'), value_type=bool),
    }
    # OpenMP threads (particle filter) sleep instead of spinning between scans
    common = dict(package='race_core', executable='race_core_node', name='race_core',
                  output='screen', emulate_tty=True, additional_env={'OMP_WAIT_POLICY': 'PASSIVE'},
                  parameters=[lc('params_file'), overrides])
    return LaunchDescription([
        *args,
        Node(**common, prefix='taskset -c 2,3', condition=UnlessCondition(lc('rt'))),
        Node(**common, prefix='taskset -c 2,3 chrt -f 80', condition=IfCondition(lc('rt'))),
    ])
