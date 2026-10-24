# Follow the Gap

A ROS 2 package implementing a reactive gap-following planner for the blaze_racer platform. The node reads raw LiDAR scans and publishes Ackermann drive commands in real time with no map, localization, or prior environment knowledge required. The implementation inflates obstacles for vehicle width, retains a closest-obstacle bubble, and caps speed using available braking clearance.

---

## Algorithm Overview

The planner runs entirely inside a single LiDAR callback. Each scan goes through the following steps:

**1. Preprocessing**
Range values above `max_lidar_dist` are clipped and `inf` readings are replaced with `max_lidar_dist`. Invalid forward readings (NaN, negative infinity, zero, or negative ranges) request a stop. A moving-average filter uses edge padding and never increases a raw obstacle distance. Every obstacle is expanded angularly by the vehicle half-width plus `safety_margin`.

**2. Safety bubble**
The index of the closest obstacle in the processed scan is found. Scans whose readings are all at `max_lidar_dist` are treated as clear and do not get a bubble. Otherwise, indices within `bubble_radius_near` are zeroed when the closest reading is at or below `near_obstacle_threshold`; `bubble_radius_far` is used for farther obstacles.

**3. Gap finding**
Returns at or below `min_gap_distance` are marked occupied. The longest remaining segment whose width exceeds `safe_threshold` indices is selected. If none qualifies, the node stops; there is no narrow-gap fallback.

**4. Best point selection**
A second sliding-window average of width `best_point_conv_size` is applied over the selected gap. The index with the highest averaged range value is chosen as the heading target. If the gap is narrower than the window, the midpoint of the gap is used instead.

**5. Steering and speed**
The target index is converted to a LiDAR angle relative to straight ahead, scaled by `steering_gain`, and clamped to `max_steer`. A speed ceiling is selected based on the magnitude of the steering angle:

| Steering angle | Speed |
|---|---|
| Greater than `straights_steering_angle` | `corners_speed` |
| Greater than `fast_steering_angle` | `straights_speed` |
| At or below `fast_steering_angle` | `fast_speed` |

Clearance along both the straight path and the requested Ackermann arc further
limits this ceiling. Obstacle checks include vehicle half-width and margin.
Speed satisfies `v * reaction_time + v² / (2 * braking_deceleration) <= clearance - stop_distance`;
no available stopping distance yields zero speed. The path horizon is limited
by `max_lidar_dist`. These checks use the current scan and commanded steering;
they are not a closed-loop emergency brake using measured vehicle speed.

Geometry/braking parameters (also identical in direct-node defaults):

| Parameter | Default | Meaning |
|---|---|---|
| `vehicle_width` | 0.296 m | Nominal Slash 4x4 width; verify added hardware fits within it |
| `safety_margin` | 0.10 m | Additional clearance around the vehicle |
| `wheelbase` | 0.324 m | Wheelbase used for the commanded turn arc |
| `min_gap_distance` | 0.60 m | Minimum inflated range accepted as free space |
| `stop_distance` | 0.15 m | Additional stand-off for front-mounted LiDAR with no bumper |
| `reaction_time` | 0.30 s | Sensing and actuation allowance |
| `braking_deceleration` | 2.0 m/s² | Must not exceed measured braking ability |

The 0.15 m stand-off assumes the LiDAR is at the vehicle front with no bumper.
It is additional to the swept-path radius and safety margin, not the total
wall clearance. Increase it if hardware projects ahead of the LiDAR.

Verify geometry, sensor placement, latency, and braking ability before physical
operation. Offline scan replay cannot establish collision-free driving.

---

## Topics

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/scan` | Subscribed | `sensor_msgs/LaserScan` | Raw LiDAR input |
| `/drive` | Published | `ackermann_msgs/AckermannDriveStamped` | Steering and speed commands |

---

## Parameters

All parameters are declared in `config/params.yaml` and loaded at launch.

| Parameter | Default | Description |
|---|---|---|
| `bubble_radius_near` | 100 | Indices zeroed on each side of an obstacle at or nearer than `near_obstacle_threshold` |
| `bubble_radius_far` | 60 | Indices zeroed on each side of a farther obstacle |
| `near_obstacle_threshold` | 0.45 m | Boundary between near and far safety bubbles |
| `preprocess_conv_size` | 3 | Moving-average window width for scan preprocessing |
| `max_lidar_dist` | 10.0 m | Ranges above this value are clipped |
| `safe_threshold` | 30 | Minimum gap width in indices to be preferred |
| `best_point_conv_size` | 200 | Sliding-window width for best-point selection inside the gap |
| `steering_gain` | 0.8 | Scale applied to target bearing |
| `max_steer` | 0.349066 rad (20 deg) | Hard clamp on output steering angle |
| `straights_steering_angle` | 0.10472 rad (6 deg) | Boundary between corner speed and straight speed |
| `fast_steering_angle` | 0.0542 rad (3.1 deg) | Boundary between straight speed and fast speed |
| `corners_speed` | 1.5 m/s | Speed used in tight corners |
| `straights_speed` | 2.5 m/s | Speed used on mild curves |
| `fast_speed` | 3.0 m/s | Speed used on open straights |

---

## Build and Launch

```bash
cd ~/blaze_racer/autonomy_ws
colcon build --packages-select follow_the_gap
source install/setup.bash
ros2 launch follow_the_gap follow_the_gap.launch.py
```

The subsystem workspace bringup must already be running before launching this node.

## Recording a diagnostic bag

With the subsystem and autonomy nodes running, open another terminal in the
autonomy workspace, source its ROS environment, and start the recorder:

```bash
cd ~/blaze_racer/autonomy_ws
source install/setup.bash
bash src/follow_the_gap/scripts/record_follow_the_gap.sh
```

The script records `/scan`, `/drive`, and `/odom` to a timestamped directory
under `~/rosbags`. Press Ctrl-C to stop and finalize the bag. To use a different
output directory, pass it as the first argument:

```bash
bash src/follow_the_gap/scripts/record_follow_the_gap.sh /path/to/bags
```

Inspect a recording with `ros2 bag info ~/rosbags/follow_the_gap_<timestamp>`.

---

## Tuning Guide

**The car clips obstacles on corners**
Increase `bubble_radius_near` or `bubble_radius_far` to widen the safety bubble, or lower `corners_speed`. The bubble radii are scan-beam counts after the FOV crop and vary in angular width with LiDAR resolution.

**The car hesitates or stops unnecessarily in open space**
Lower `safe_threshold`. A high threshold causes the planner to reject valid gaps that are technically wide enough to drive through.

**Steering is jerky or oscillates**
Increase `preprocess_conv_size` to smooth the scan more aggressively. Also check that `best_point_conv_size` is large relative to the gap width so the best-point selection averages over a broad region.

**The car is too slow on straights**
Raise `fast_speed` only after validating stopping distance. Raising `fast_steering_angle` makes the fast tier apply at wider steering angles; the clearance-based cap still applies.

**The car cuts corners too aggressively**
Lower `straights_steering_angle` to keep the car in the corner speed tier for a wider range of turn magnitudes.

---

## Package Layout

```
follow_the_gap/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── params.yaml
├── launch/
│   └── follow_the_gap.launch.py
└── follow_the_gap/
    ├── __init__.py
    └── follow_the_gap_node.py
```