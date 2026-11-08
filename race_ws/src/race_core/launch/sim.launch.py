"""Closed-loop simulation: race_sim (car + LiDAR + odometry + optional opponent) + race_core.

  ros2 launch race_core sim.launch.py map:=/path/track.yaml raceline:=/path/raceline.csv \
      speed_scale:=0.7 opponent:=true opponent_speed:=0.5

race_core runs with the deadman disabled; everything else is the car configuration.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('race_core'), 'config', 'race_core.yaml')
    lc = LaunchConfiguration
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in {
        'map': '', 'raceline': '', 'mode': 'race', 'speed_scale': '0.5', 'mu': '0.6',
        'opponent': 'false', 'opponent_speed': '0.5', 'opponent_gap': '3.0',
        'overtake': 'none', 'start_x': '0.0', 'start_y': '0.0', 'start_yaw': '0.0',
        'start_on_raceline': 'false', 'odom_noise': '0.05', 'particles': '1000',
        'pf_threads': '2', 'max_steer': '0.42', 'core_params': params,
        'core_map': '', 'core_raceline': ''}.items()]
    sim = Node(package='race_core', executable='race_sim', name='race_sim', output='screen',
               parameters=[{
                   'map_yaml': lc('map'), 'raceline_csv': lc('raceline'),
                   'mu': ParameterValue(lc('mu'), value_type=float),
                   'opponent.enable': ParameterValue(lc('opponent'), value_type=bool),
                   'opponent.speed_fraction': ParameterValue(lc('opponent_speed'), value_type=float),
                   'opponent.start_gap': ParameterValue(lc('opponent_gap'), value_type=float),
                   'start_on_raceline': ParameterValue(lc('start_on_raceline'), value_type=bool),
                   'start_x': ParameterValue(lc('start_x'), value_type=float),
                   'start_y': ParameterValue(lc('start_y'), value_type=float),
                   'start_yaw': ParameterValue(lc('start_yaw'), value_type=float),
                   'odom.speed_noise': ParameterValue(lc('odom_noise'), value_type=float),
                   'max_steer': ParameterValue(lc('max_steer'), value_type=float)}])
    core = Node(package='race_core', executable='race_core_node', name='race_core', output='screen',
                additional_env={'OMP_WAIT_POLICY': 'PASSIVE'},
                parameters=[params, lc('core_params'), {
                    'mode': lc('mode'),
                    # race_core may use a different map than the simulated world (e.g. one made by
                    # slam_toolbox in the simulator): core_map / core_raceline, default = the world's
                    'map_yaml': PythonExpression(["'", lc('core_map'), "' or '", lc('map'), "'"]),
                    'raceline_csv': PythonExpression(["'", lc('core_raceline'), "' or '", lc('raceline'), "'"]),
                    'speed_scale': ParameterValue(lc('speed_scale'), value_type=float),
                    'require_deadman': False,
                    'overtake.mode': lc('overtake'),
                    'pf.particles': ParameterValue(lc('particles'), value_type=int),
                    'pf.threads': ParameterValue(lc('pf_threads'), value_type=int),
                    'pp.max_steer_left': ParameterValue(lc('max_steer'), value_type=float),
                    'pp.max_steer_right': ParameterValue(lc('max_steer'), value_type=float),
                    'init_x': ParameterValue(lc('start_x'), value_type=float),
                    'init_y': ParameterValue(lc('start_y'), value_type=float),
                    'init_yaw': ParameterValue(lc('start_yaw'), value_type=float)}])
    laser_tf = Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
                    arguments=['--x', '0.27', '--z', '0.11', '--frame-id', 'base_link',
                               '--child-frame-id', 'laser'])
    return LaunchDescription([*args, sim, core, laser_tf])
