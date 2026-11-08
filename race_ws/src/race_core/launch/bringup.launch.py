"""Car drivers for race_core: blaze_racer's own subsystem_stack bringup_launch.py, unchanged except
for one flag.

The team's vesc.yaml (servo calibration, ERPM gain, limits) is used as is; only
vesc_to_odom_node.publish_tf is set to false, because the EKF (state_estimation.launch.py) publishes
odom -> base_link. Two publishers of one transform make slam_toolbox maps and RViz jump.

The team's sensors.yaml is used as is except urg_node calibrate_time: on 2026-11-06 its experimental
time calibration computed a latency of -361984 s, stamping every scan ~4 days in the past (the
team's follow-the-gap ignores stamps; slam_toolbox and race_core's localization cannot). With
calibrate_time false the stamp is the host receive time, about one scan late; scan_latency (default
-0.02 s) moves it back towards the first beam. 10/26 ran with calibrate_time false.

  ros2 launch race_core bringup.launch.py                 # drivers for race_core + EKF
  ros2 launch race_core bringup.launch.py vesc_tf:=true   # drivers exactly as the team runs them
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(context):
    share = get_package_share_directory('subsystem_stack')
    lc = LaunchConfiguration
    ssrc = lc('sensors_config').perform(context) or os.path.join(share, 'config', 'sensors.yaml')
    with open(ssrc) as f:
        scfg = yaml.safe_load(f)
    urg = scfg.setdefault('urg_node', {}).setdefault('ros__parameters', {})
    urg['calibrate_time'] = lc('scan_calibrate_time').perform(context).lower() == 'true'
    urg['default_user_latency'] = float(lc('scan_latency').perform(context))
    sout = os.path.join(tempfile.gettempdir(), f'race_sensors_{os.getuid()}.yaml')
    with open(sout, 'w') as f:
        yaml.safe_dump(scfg, f, default_flow_style=False)
    src = LaunchConfiguration('vesc_config').perform(context) or \
        os.path.join(share, 'config', 'vesc.yaml')
    with open(src) as f:
        cfg = yaml.safe_load(f)
    vesc_tf = LaunchConfiguration('vesc_tf').perform(context).lower() == 'true'
    cfg.setdefault('vesc_to_odom_node', {}).setdefault('ros__parameters', {})['publish_tf'] = vesc_tf
    out = os.path.join(tempfile.gettempdir(), f'race_vesc_{os.getuid()}.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False)
    shared = cfg.get('/**', {}).get('ros__parameters', {})
    msg = (f'bringup: {src} with vesc_to_odom publish_tf={vesc_tf} | servo = '
           f"{shared.get('steering_angle_to_servo_gain')} * steer + "
           f"{shared.get('steering_angle_to_servo_offset')}, servo "
           f"{shared.get('servo_min')}..{shared.get('servo_max')}, "
           f"erpm gain {shared.get('speed_to_erpm_gain')} | urg_node calibrate_time="
           f"{urg['calibrate_time']} latency {urg['default_user_latency']:+.3f} s")
    return [LogInfo(msg=msg),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup_launch.py')),
                launch_arguments={'vesc_config': out, 'sensors_config': sout}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vesc_config', default_value='',
                              description="team vesc.yaml ('' = subsystem_stack's installed one)"),
        DeclareLaunchArgument('vesc_tf', default_value='false',
                              description='true only when the EKF is NOT running'),
        DeclareLaunchArgument('sensors_config', default_value='',
                              description="team sensors.yaml ('' = subsystem_stack's installed one)"),
        DeclareLaunchArgument('scan_calibrate_time', default_value='false'),
        DeclareLaunchArgument('scan_latency', default_value='-0.02',
                              description='s added to the scan stamp (receive time -> first beam)'),
        OpaqueFunction(function=include),
    ])
