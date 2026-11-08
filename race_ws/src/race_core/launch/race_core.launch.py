"""Launch race_core pinned to cores 2-3 (RACE_PLAN.md §5.3).

  ros2 launch race_core race_core.launch.py                       # REACTIVE (follow-the-gap)
  ros2 launch race_core race_core.launch.py mode:=race map:=/path/track.yaml \
      raceline:=/path/raceline.csv speed_scale:=0.5

Follow-the-gap is blaze_racer iv2026's planner (the team's tested node) ported to C++. Its tunables
are read from the installed follow_the_gap/config/params.yaml when that package is on the path, so
tuning that file tunes race_core's REACTIVE mode too (pass ftg_params:=none to use race_core.yaml).

Real-time priority (chrt -f) needs CAP_SYS_NICE; pass rt:=true only after
setting that up, otherwise the node runs at normal priority on cores 2-3.
"""
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

FTG_KEYS = ('fov_degrees', 'bubble_radius', 'preprocess_conv_size', 'safe_threshold',
            'best_point_conv_size', 'max_lidar_dist', 'steering_gain', 'max_steer',
            'straights_steering_angle', 'fast_steering_angle', 'corners_speed', 'straights_speed',
            'fast_speed', 'use_distance_speed', 'front_angle_width', 'dist_min_speed',
            'dist_max_speed', 'distance_min_speed', 'distance_max_speed')
INT_KEYS = ('bubble_radius', 'preprocess_conv_size', 'safe_threshold', 'best_point_conv_size')


def team_ftg_params(path):
    """ftg_iv.* overrides from blaze_racer's follow_the_gap params.yaml ({} if unavailable)."""
    if path == 'none':
        return {}, 'race_core.yaml ftg_iv values (ftg_params:=none)'
    if not path:
        try:
            path = os.path.join(get_package_share_directory('follow_the_gap'), 'config', 'params.yaml')
        except PackageNotFoundError:
            return {}, 'race_core.yaml ftg_iv values (follow_the_gap package not found)'
    try:
        with open(path) as f:
            p = yaml.safe_load(f)['follow_the_gap_node']['ros__parameters']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as e:
        return {}, f'race_core.yaml ftg_iv values ({path} unreadable: {e})'
    out = {}
    for k in FTG_KEYS:
        if k in p:
            v = p[k]
            out['ftg_iv.' + k] = int(v) if k in INT_KEYS else (bool(v) if k == 'use_distance_speed'
                                                               else float(v))
    return out, path


def nodes(context):
    lc = LaunchConfiguration
    params = lc('params_file').perform(context)
    ftg, src = team_ftg_params(lc('ftg_params').perform(context))
    # race-day deltas on top of the team's file, e.g. ftg_set:=max_steer=0.5,distance_min_speed=1.5
    for item in filter(None, lc('ftg_set').perform(context).split(',')):
        k, v = item.split('=')
        k = k.strip()
        if k not in FTG_KEYS:
            raise ValueError(f'ftg_set: unknown follow-the-gap parameter {k}')
        ftg['ftg_iv.' + k] = int(v) if k in INT_KEYS else (
            v.strip().lower() == 'true' if k == 'use_distance_speed' else float(v))
        src += f' + {k}={v.strip()}' 
    overrides = {
        'mode': lc('mode'), 'map_yaml': lc('map'), 'raceline_csv': lc('raceline'),
        'drive_mode': lc('drive_mode'), 'reactive_planner': lc('reactive_planner'),
        'speed_scale': ParameterValue(lc('speed_scale'), value_type=float),
        'reactive_speed_scale': ParameterValue(lc('reactive_speed_scale'), value_type=float),
        'require_deadman': ParameterValue(lc('deadman'), value_type=bool),
        'trail.enable': ParameterValue(lc('trail'), value_type=bool),
        'overtake.mode': lc('overtake'),
        **ftg,
    }
    # OpenMP threads (particle filter) sleep instead of spinning between scans
    common = dict(package='race_core', executable='race_core_node', name='race_core',
                  output='screen', emulate_tty=True, additional_env={'OMP_WAIT_POLICY': 'PASSIVE'},
                  parameters=[params, overrides])
    return [
        LogInfo(msg=f'race_core follow-the-gap params from {src}'),
        Node(**common, prefix='taskset -c 2,3', condition=UnlessCondition(lc('rt'))),
        Node(**common, prefix='taskset -c 2,3 chrt -f 80', condition=IfCondition(lc('rt'))),
    ]


def generate_launch_description():
    params = os.path.join(get_package_share_directory('race_core'), 'config', 'race_core.yaml')
    args = [
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('rt', default_value='false'),
        DeclareLaunchArgument('mode', default_value='reactive'),
        DeclareLaunchArgument('drive_mode', default_value='raceline'),
        DeclareLaunchArgument('reactive_planner', default_value='iv2026'),
        DeclareLaunchArgument('ftg_params', default_value='',
                              description="'' = installed follow_the_gap params.yaml, 'none' = race_core.yaml"),
        DeclareLaunchArgument('ftg_set', default_value='',
                              description='overrides on top of ftg_params: key=value,key=value'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('raceline', default_value=''),
        DeclareLaunchArgument('speed_scale', default_value='0.5'),
        DeclareLaunchArgument('reactive_speed_scale', default_value='1.0'),
        DeclareLaunchArgument('deadman', default_value='true'),
        DeclareLaunchArgument('trail', default_value='true'),
        DeclareLaunchArgument('overtake', default_value='none'),
    ]
    return LaunchDescription([*args, OpaqueFunction(function=nodes)])
