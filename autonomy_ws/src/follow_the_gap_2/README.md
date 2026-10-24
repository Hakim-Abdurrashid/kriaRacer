# Follow the Gap 2 — F1TENTH ROS 2 controller

`follow_the_gap_2_node` is a ROS 2 node using the vehicle's existing
`autonomy_core.BaseDriveNode`. The controller uses a fixed 5 m LiDAR cap; corridor-based range switching is disabled.

## Build in this repository

This package lives in `autonomy_ws/src/follow_the_gap_2` and is discovered by
the standard autonomy workspace build:

```bash
cd /home/ubuntu/airacer_basecode
source /opt/ros/humble/setup.bash
source install/setup.bash
cd blaze_racer/autonomy_ws
colcon build --packages-select autonomy_core follow_the_gap_2 --symlink-install
source install/setup.bash
```

The first `install/setup.bash` supplies this checkout's existing vehicle message
packages, including `ackermann_msgs`. On another vehicle, source its subsystem
workspace instead. Build outputs go into the autonomy workspace's normal
`build`, `install`, and `log` directories; no source relocation is required.

## Run on the vehicle

Start the existing hardware stack in one terminal, using its installed overlay:

```bash
cd /home/ubuntu/airacer_basecode
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch subsystem_stack bringup_launch.py
```

In the terminal where the new autonomy overlay was sourced:

```bash
ros2 launch follow_the_gap_2 follow_the_gap_2.launch.py
```

Stop any other autonomous controller publishing `/drive` before starting this
one. Hold the configured navigation deadman button (button 5: RB/R1) to let
navigation commands through the existing mux. Releasing it restores the
joystick's default stop command; manual control has higher mux priority.

The vehicle connection is:

```text
Hokuyo /scan (sensor_msgs/msg/LaserScan)
  -> follow_the_gap_2_node
  -> /drive (ackermann_msgs/msg/AckermannDriveStamped)
  -> existing ackermann_mux -> /ackermann_drive -> VESC
```

The command frame is `base_link`; no odometry or camera input is required.
The existing launch file, console entry point, package manifest, and resource
marker register this as an `ament_python` ROS 2 package.

## Launch options

```bash
ros2 launch follow_the_gap_2 follow_the_gap_2.launch.py --show-args
ros2 launch follow_the_gap_2 follow_the_gap_2.launch.py \
  scan_topic:=/scan drive_topic:=/drive
```

`params_file:=/absolute/path/to/params.yaml` selects a different parameter file.
The launch topic arguments override the topic values in that file. Other tuning
comes from `config/params.yaml` and the supplied algorithm defaults.

The executable can also be started directly:

```bash
ros2 run follow_the_gap_2 follow_the_gap_2_node --ros-args \
  -p scan_topic:=/scan -p drive_topic:=/drive
```

## Verification and existing behavior

Use `ros2 node info /follow_the_gap_2_node` to inspect the scan subscription and
drive publisher. `ros2 topic info /drive --verbose` shows the controller and mux
endpoints. These commands require the nodes to be running.

The algorithm retains its central 60% scan crop, disparity extension, fixed
5 m range cap, metre-based bubble, 80/20 target blend, steering history,
and speed bands. The launch configuration and Python defaults use a 1.0 m/s
maximum commanded speed and a 0.24 rad steering limit. Every speed-selection
branch is capped by `speed_max`, including narrow-gap overrides. The vehicle's
actuator configuration still applies. Invalid scans or a `None` algorithm result
publish a zero-speed, zero-steering stop instead of continuing to creep. This controller
has no scan-loss watchdog. Physical driving has not been validated by this
integration.
