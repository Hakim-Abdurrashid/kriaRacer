# Blaze Racer

## TODO

1. Update KR260 firmware from version 1.02 to 1.07
2. Solder broken lidar wire
3. Supervise charging of the battery in the lipo fire bag

---

A ROS 2 software stack for a small-scale autonomous racing platform. The repository is split into two independent ROS 2 workspaces: a hardware-facing **subsystem workspace** that brings up all drivers and teleoperation, and an **autonomy workspace** that houses the autonomous driving algorithms.

>**Note that the stack was tailored to run on Ubuntu 20.04**

---

## Main Hardware

| Component | Model |
|---|---|
| Motor controller | VESC (connected via `/dev/sensors/vesc`) |
| LiDAR | Hokuyo (connected via `urg_node`) |
| Camera | Intel RealSense D435i (RGB stream only) |
| Gamepad Controller | Logitech F-710 or PS4 DualShock |

---

## Setup

### 1. Clone the repository

```bash
git clone https://github.com/airacer-ucf/blaze_racer.git
```

### 2. Initialize rosdep (first-time only)

```bash
sudo rosdep init
rosdep update
```

Might need to run `rosdep update --include-eol-distros` instead.

---

## Running the Stack

Open two terminals:

**Terminal 1 -- subsystem stack**
```bash
cd ~/blaze_racer/subsystem_ws
source install/setup.bash
ros2 launch subsystem_stack bringup_launch.py
```

**Terminal 2 -- autonomy**
```bash
cd ~/blaze_racer/autonomy_ws
source install/setup.bash
ros2 launch follow_the_gap follow_the_gap.launch.py
```

Hold the navigation deadman button (RB on Logitech F-710, R1 on PS4) to allow autonomous commands through the mux. Release it or hold the teleop deadman button to take manual control.

---

## Autonomy Workspace

The autonomy workspace contains the reactive planner that drives the car autonomously. It is independent of the subsystem workspace and can be built and run separately.

### Build and launch

```bash
cd ~/blaze_racer/autonomy_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch follow_the_gap follow_the_gap.launch.py
```

The subsystem workspace must already be running (bringup launched) before starting the autonomy node, since it depends on `/scan` being published and `/drive` being consumed.

### Autonomy Packages

#### autonomy_core (shared library)

`autonomy_core` holds the reusable scaffolding every algorithm builds on: a
`BaseDriveNode` that handles the `/scan`→`/drive` ROS plumbing and safety stop,
plus ROS-free `lidar_utils`. New algorithms subclass `BaseDriveNode` and
implement a single `compute_drive` method — see `autonomy_ws/README.md` for the
pattern. It ships no algorithm of its own.

#### Follow the Gap algorithm

The `follow_the_gap` package implements the Disparity Extender / Follow-the-Gap reactive planner. It processes raw LiDAR scans and publishes Ackermann drive commands in real time with no map or localization required. The gap-following logic lives in a pure, unit-tested `planner.py`; the node is a thin `BaseDriveNode` subclass.

---

## Subsystem Workspace

The subsystem workspace manages all hardware: the VESC drive-by-wire system, Hokuyo LiDAR, RealSense camera, joystick input, and command multiplexing.

### Build and launch

```bash
cd ~/blaze_racer/subsystem_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch subsystem_stack bringup_launch.py
```

### Packages

**subsystem_stack** -- Master package. Contains the single `bringup_launch.py` that starts the full driver stack and all parameter files consumed by every other node.

**realsense_rgb** -- Publishes the D435i's color stream as `sensor_msgs/Image` via V4L2, with no depth or point-cloud overhead. Can be launched standalone or is included automatically by the bringup.

```bash
ros2 launch realsense_rgb realsense_rgb.launch.py
```

**camera_capture** -- Listens to the `/joy` topic and saves timestamped photos and videos on button press. Intended button mapping: X to take a photo, Triangle to start recording, Square to stop recording. Captures go to `~/captures` by default. Launch standalone with:

```bash
ros2 launch camera_capture camera_capture.launch.py
```

Both `realsense_rgb` and `camera_capture` are present in the bringup launch file but commented out. Uncomment the relevant lines in `bringup_launch.py` to include them automatically.

**ackermann_mux** -- A priority-based multiplexer that arbitrates between teleop and autonomous drive commands. The autonomous `/drive` topic has lower priority than the human joystick input, so the operator can take control at any time.

**teleop_tools** -- Provides `joy_teleop`, `key_teleop`, and `mouse_teleop` nodes. The joystick teleop is the primary interface and is configured via `subsystem_stack/config/joy_teleop.yaml`.

**vesc** -- Low-level driver suite for the VESC motor controller. Consists of three sub-packages: `vesc_driver` (serial communication), `vesc_ackermann` (Ackermann-to-VESC and VESC-to-odometry conversion), and `vesc_msgs` (custom message definitions).

### Nodes launched by bringup

The `bringup_launch.py` file starts the following nodes in order:

1. `joy` -- joystick driver
2. `joy_teleop` -- maps joystick axes and buttons to `AckermannDriveStamped` commands
3. `ackermann_to_vesc_node` -- converts Ackermann commands to VESC motor and servo commands
4. `vesc_to_odom_node` -- integrates VESC telemetry into wheel odometry
5. `vesc_driver_node` -- low-level VESC serial interface
6. `urg_node` -- Hokuyo LiDAR driver
7. `ackermann_mux` -- priority multiplexer for command sources
8. `static_transform_publisher` -- publishes the fixed TF from `base_link` to `laser` (x=0.27 m, z=0.11 m)

The `throttle_interpolator` node (acceleration smoother) is present in the launch file but commented out. Enable it if smoother throttle ramps are needed (might need to troubleshoot it as well).

### Deadman switch

A deadman button must be held at all times for control inputs to pass through. Defaults:

| Controller | Teleop | Navigation (autonomous) |
|---|---|---|
| Logitech F-710 | LB | RB |
| PS4 DualShock | L1 | R1 |

Button bindings are configurable in `subsystem_stack/config/joy_teleop.yaml`.

### Joystick axis configuration

Steering axis indices differ by controller. Set the correct index under `human_control -> drive-steering_angle` in `joy_teleop.yaml`:

| Controller | Axis index |
|---|---|
| PS4 DualShock | 2 |
| Logitech F-710 | 3 |

To identify unknown axis mappings, run `ros2 topic echo /joy` and move the stick while watching which index in the `axes` array changes.

### Topics

**Subscribed by the driver stack**

| Topic | Type | Description |
|---|---|---|
| `/drive` | `AckermannDriveStamped` | Autonomous navigation commands |

**Published by the driver stack**

| Topic | Type | Description |
|---|---|---|
| `/scan` | `LaserScan` | Hokuyo LiDAR scans |
| `/odom` | `Odometry` | Wheel odometry from VESC |
| `/sensors/imu/raw` | `Imu` | Raw IMU data from VESC |
| `/sensors/core` | `VescStateStamped` | VESC telemetry |

**Published by the camera stack**

| Topic | Type | Description |
|---|---|---|
| `/camera/color/image_raw` | `Image` | RealSense D435i RGB stream |

### Key configuration files

| File | Purpose |
|---|---|
| `subsystem_stack/config/vesc.yaml` | VESC speed/servo gains, port, safety limits, odometry frame |
| `subsystem_stack/config/joy_teleop.yaml` | Joystick axis/button mappings and deadman configuration |
| `subsystem_stack/config/sensors.yaml` | Hokuyo LiDAR connection parameters |
| `subsystem_stack/config/mux.yaml` | Ackermann mux topic priorities |
| `subsystem_stack/config/realsense.yaml` | RealSense node parameters |
| `camera_capture/config/camera_capture.yaml` | Save directory, button indices, codec, FPS |

### VESC calibration parameters

The VESC config uses linear mappings to convert between SI units and hardware values:

```
erpm = speed_to_erpm_gain * speed_m_s + speed_to_erpm_offset
servo = steering_angle_to_servo_gain * angle_rad + steering_angle_to_servo_offset
```

Current defaults (tune for your specific vehicle):

| Parameter | Value |
|---|---|
| `speed_to_erpm_gain` | 4614.0 |
| `speed_to_erpm_offset` | 0.0 |
| `steering_angle_to_servo_gain` | -1.2135 |
| `steering_angle_to_servo_offset` | 0.5304 |
| `servo_min` / `servo_max` | 0.15 / 0.85 |
| `wheelbase` | 0.25 m |

---

## External Dependencies

| Package | Purpose | Source |
|---|---|---|
| `ackermann_msgs` | Ackermann command message type | https://index.ros.org/r/ackermann_msgs |
| `urg_node` | Hokuyo LiDAR driver | https://index.ros.org/p/urg_node |
| `joy` | Joystick driver | https://index.ros.org/p/joy |
| `teleop_tools` | Joystick/key/mouse teleop | https://index.ros.org/p/teleop_tools |
| `vesc` | VESC motor controller driver | https://github.com/f1tenth/vesc/tree/ros2 |
| `ackermann_mux` | Priority command multiplexer | https://github.com/f1tenth/ackermann_mux |
| `cv_bridge` | OpenCV / ROS image bridge | https://index.ros.org/p/cv_bridge |
| `python3-opencv` | OpenCV Python bindings | `sudo apt install python3-opencv` |

Install all ROS dependencies at once from within each workspace's root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---