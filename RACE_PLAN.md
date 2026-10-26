# Race Plan: Winning the F1Tenth Head-to-Head with the Kria KR260

- **Version:** 1.1, updated Sunday 2026-10-25 (Day 1); v1.0 written Saturday 2026-10-24 (Day 0)
- **Race day:** Saturday 2026-11-07 (14 days out)
- **Team:** airacer-ucf / blaze_racer
- **Basis:** web research, a survey of the existing code and bags, and measurements taken on the KV260 dev board (same K26 SOM as the race KR260)

This file is the goal for all work until race day. When something changes (rules, a missed gate, a new measurement), record it in the Decision Log (Appendix B) instead of letting the plan drift.

**For future work sessions:** read §0, today's row in §8, the gates in §8.2, and the Decision Log first.

---

## 0. The plan on one page

**Why we can win.** The score is start-line crossings in 60 seconds, so pace dominates. Every crash costs a hand reset, which is several seconds and probably a whole lap.

Today the car averages about 0.7 m/s on follow-the-gap (measured in the Oct 9 bags). That is roughly 30 s per lap on our 21 m test loop, or 1–2 laps per heat. A friction-limited racing line on the same loop models at **5–7.5 s per lap, or 7–11 laps per heat** (§2.5). That 4–5× gain is the prize, and everything else in this plan exists to protect it.

**What the KR260 does and does not give us.**

- **It does not give us more compute.** The Jetson Orin Nano has roughly 5× our general-purpose CPU throughput, and an order of magnitude more nominal AI TOPS. A Python-heavy race stack that runs comfortably on an Orin will not fit on four Cortex-A53 cores (measured, §2.4).
- **It does give us three things the others lack:**
  1. A DPU in the programmable logic (PL) that runs YOLO with almost no CPU cost.
  2. Cores we can pin and prioritize for a deterministic C++ control loop.
  3. The only camera in the field, and that camera has an IMU (gyro) that our sensorless motor badly needs.

**The solution.**

1. **Race-day pipeline:** map the track, generate a minimum-curvature raceline with a friction-limited speed profile, then race. This uses the TUM optimizer through ForzaETH's global planner, and must run in under 15 minutes.
2. **State estimation:** an EKF fusing VESC ERPM with the D435i gyro, plus a C++ particle filter. With 2000 particles at 40 Hz the filter takes 8.5 ms per update (measured on the K26).
3. **One C++ `race_core` process on two isolated cores.** It does localization, Frenet-frame tracking, opponent detection and tracking, the state machine (GB_TRACK / TRAILING / OVERTAKE / REACTIVE), spline overtaking, pure pursuit (later the MAP controller), and emergency braking. The architecture is copied from ForzaETH's open-source stack (several-time F1Tenth winners) and rebuilt lean for the K26.
4. **DPU vision:** a DPU-friendly YOLOv8n (ReLU activations, no attention), retrained on F1Tenth cars and fused with LiDAR clusters to confirm the opponent. For comparison, the team's current YOLO26n-seg takes **1.08 s per frame on the CPU** using all four cores.
5. **Follow-the-gap stays** as the REACTIVE fallback (after hand resets or localization loss) and as the Plan B race mode.

**Hardware decisions.** One RealSense D435i (RGB for YOLO, IMU for the EKF). No T265 and no second camera. The KR260 runs from a regulated 12 V supply. The Hokuyo connects to a PS-side Ethernet port.

**Gates.**

| Gate | Date | Criterion |
|---|---|---|
| M1 | Tue 10/27 | Autonomous raceline laps on the KV260 |
| M2 | Sun 11/1 | KR260 in the car; 10 consecutive laps at ≥75% speed scaler; race-day pipeline ≤20 min |
| M3 | Thu 11/5 | Head-to-head features validated; code frozen |
| — | Fri 11/6 | Tuning only |
| — | Sat 11/7 | Race |

---

## 1. Mission, scoring, and success criteria

### 1.1 Rules as known

From `race_rules.txt` and your answers on 10/24:

- 7 contestants race two at a time, starting side by side at the start line.
- Each heat lasts 60 s. The car with the most start-line crossings after the start wins.
- **Practice and mapping time is on race day.** The track layout is not known in advance.
- **Hand resets are allowed.** A crashed or stalled car can be put back on track.
- Contact rules, tie-breaks and tournament format are unknown (§12).

### 1.2 What the scoring implies

- **Crossings ≈ (1 if the start crossing counts) + floor((60 s − time to first crossing) / lap time).** The score is discrete, so a small pace edge only matters near a lap-count boundary. A large edge means lapping the opponent.
- **A crash costs about 5–10 s** (estimate: stop, operator walks over, places the car, relocalizes). At race pace that is about one lap, so avoiding one crash is worth as much as being 10–15% faster.
- **We will lap slow cars several times per heat.** With 7.5 s laps against 15 s laps, we catch the other car about every 15 s, which is 3–4 passes per heat. Safe lapping is a core feature.
- **Against an equally fast car, three things decide the heat:** the launch, the first corner, and not crashing.

### 1.3 Success criteria

| ID | Metric | Target | How measured |
|---|---|---|---|
| G1 | Lap time on the blaze_track test loop at race scaler | ≤ 8.5 s (≥7 laps/60 s); stretch ≤ 7.5 s | lap analyser |
| G2 | Reliability | ≥ 20 consecutive laps at race scaler with no crash; no computer resets or brownouts | test log |
| G3 | Hand-reset recovery | Back at raceline pace ≤ 3 s after the car is placed | drill (§6.4) |
| G4 | Race-day pipeline | New track to race-ready in ≤ 15 min, rehearsed twice | stopwatch |
| G5 | Opponent perception | ≥ 95% recall within 6 m ahead, ≤ 2% false positives; DPU inference ≤ 10 ms | labelled bags |
| G6 | Compute | `race_core` loop p99 ≤ 5 ms; total CPU ≤ 60% of 4 cores | /proc, tracing |
| G7 | Racecraft | ≥ 80% clean overtakes against an opponent at ≤ 80% of our pace; 0 contacts in 10 trailing runs | test log |

The race-day track will differ from blaze_track. There, aim for ≥ 75% of the lap time the model in §2.5 predicts for that track.

---

## 2. Ground truth (verified 2026-10-24)

### 2.1 Hardware

**Development board (in hand): a KV260, not a KR260.** The device tree reports `ZynqMP KV260 revB` and only Ethernet controller GEM3 is enabled. The **race board is a factory-new KR260.**

**Both boards carry the same K26 SOM:**

- 4× Cortex-A53 at 1.333 GHz (measured)
- 2× Cortex-R5F real-time cores
- PL with 256K logic cells and 1,248 DSP slices
- 4 GB DDR4

AMD's own KR260 Vitis AI 3.5 flow compiles models with the **KV260 `arch.json`** (`files/cifar10_tf2/vitisai_compile.sh` in `amd/Kria-RoboticsAI`). Every model and benchmark built on the KV260 therefore transfers to the KR260 unchanged.

**Sensors and drivetrain:**

- **Camera actually attached:** RealSense **D435i** (USB ID 8086:0b3a). `hardware.txt` says D415 or T265. You confirmed the camera configuration is our choice.
- **Hokuyo UST-10LX (from the Oct 9 bags):** 1081 beams over ±135°, about 40 Hz.
- **Motor:** sensorless BLDC (per the blaze_racer README). The team found VESC odometry unreliable and dead-reckons from commanded values (`slam_mapping/cmd_odometry`).
- **ESC "autoro aesc v6.7":** I could not identify this model online. The stack treats it as VESC-6-compatible (f1tenth `vesc` driver on `/dev/ttyACM0`). It was not connected during the survey.

### 2.2 KR260 vs. Jetson Orin Nano

| | Jetson Orin Nano 8 GB (competitors) | Kria K26 SOM (us) |
|---|---|---|
| CPU | 6× Cortex-A78AE | 4× Cortex-A53 @ 1.333 GHz, roughly 5× less general-purpose throughput |
| NN accelerator | 1024-core Ampere GPU + 32 Tensor Cores; up to 67 TOPS (sparse INT8, Super mode) | DPU B4096 @ 300 MHz ≈ 1.2 TOPS peak INT8; deterministic, near-zero CPU cost |
| Memory | 8 GB LPDDR5 | 4 GB DDR4 |
| Real-time cores | not a user-facing feature | 2× Cortex-R5F (OpenAMP) |
| Programmable logic | none | 256K logic cells, 1,248 DSPs |
| Extra sensor | none | D435i camera + IMU |

**What this means for the design:**

- CPU is our scarcest resource. No Python in the control loop, and no extra image streams.
- Every neural network goes on the DPU.
- Our edge has to come from the camera and IMU, determinism, and execution: raceline quality, tuning, reliability, and racecraft.

### 2.3 Current software state

Survey of `~/airacer_basecode`, `~/Visual_InertialSLAM` and this board:

- **blaze_racer, subsystem_ws:** drivers for the VESC, `urg_node`, joystick, `ackermann_mux` (with deadman button) and RealSense RGB.
- **blaze_racer, autonomy_ws:** three follow-the-gap variants, `slam_mapping` (slam_toolbox with `cmd_odometry`), and `yolo_detection`.
- **Map:** `maps/blaze_track` at 0.05 m resolution, built on another machine (`/home/f1tenth-open`).
- **Oct 9 follow-the-gap bags:** mean speed 0.6–0.7 m/s, odometry maximum 1.5 m/s, maximum commanded speed 3.0 m/s.
- **Existing YOLO model:** YOLO26n-seg (Ultralytics 8.4.129), classes `Car` and `Wall`, 640 px input. It was trained on the dataset *"CMU F1 Tenth – Cars and Walls"* stored in the team's Google Drive. Its graph contains 100 SiLU sigmoids, attention (MatMul ×4, Softmax ×2) and an in-graph TopK head. **None of those run on the DPU.**
- **Not installed system-wide on this board:** slam_toolbox, nav2, robot_localization, or any Vitis AI runtime. The VESC packages exist only in `~/airacer_basecode/install`. Only the default `k26_starter_kits` bitstream is present, so there is no DPU.
- **blaze_racer has uncommitted changes** (planner rewrites, VESC parameters, a new `yolo_detection` package). Commit them before we branch.

### 2.4 Measurements on the K26 (A53 @ 1.333 GHz)

| Workload | Result | Meaning |
|---|---|---|
| C++ likelihood-field particle filter, 1000 particles × 108 beams | 4.3 ms/update = 17% of one core at 40 Hz | cheap |
| Same, 2000 × 108 | 8.5 ms = 34% of one core (4.6 ms with 2 threads) | **our default** |
| Same, 1000 × 1081 beams (full resolution) | 39 ms = 157% of one core | full-resolution scans are unaffordable on the CPU; downsample |
| Python per-scan compute (deserialize LaserScan + numpy, follow-the-gap style) | 0.57 ms | Python arithmetic itself is fine |
| Trivial Python ROS 2 node on `/scan` at 40 Hz | **13.5% of one core** (a Python publisher adds 11%) | overhead dominates, so **no Python in the loop** |
| Team's YOLO26n-seg on the CPU (onnxruntime, 4 threads) | **1076 ms at 640 px, 361 ms at 320 px, using all 4 cores** | camera AI must go on the DPU |

**ForzaETH comparison.** Their full head-to-head stack used **245–296% CPU on a laptop-class Intel i5-10210U**, and those are the ROS 1 numbers (ROS 2 Python is generally slower). Scaled to A53 cores, that is on the order of 10 or more cores. **Their Python stack cannot run as-is on the K26. We port its logic and parameters instead.**

### 2.5 Lap-time model on blaze_track

Script: `research/laptime_model.py`. Figure: `research/raceline_debug.png` (blue is the centerline, red the minimum-curvature line).

Track geometry: **21.0 m centerline**, 19.6 m raceline, **1.0–1.5 m wide**, tightest centerline radius about 0.6 m.

| Friction μ | Centerline, 100% speed | Raceline, 100% speed | Raceline, 75% speed (head-to-head) |
|---|---|---|---|
| 0.5 | 7.05 s (8 laps) | 6.26 s (9 laps) | 7.55 s (7 laps) |
| 0.7 | 6.19 s (9 laps) | 5.50 s (10 laps) | 6.51 s (9 laps) |
| 0.9 | 5.63 s (10 laps) | 5.00 s (11 laps) | 5.89 s (10 laps) |
| **Today's follow-the-gap at 0.7 m/s** | **≈30 s per lap → 1–2 laps** | | |

**Model assumptions:** ideal tracking, acceleration 4 m/s², braking 5 m/s², top speed 7 m/s, friction ellipse, car center kept ≥0.30 m from the walls. Real laps will be slower. Use the table for relative comparisons and targets.

**Takeaways:**

- A planned, friction-limited speed profile is worth about 4–5×.
- On this narrow track, the raceline's shape adds about another 11%.

### 2.6 Lessons from ForzaETH

From their *Journal of Field Robotics* paper, arXiv 2403.11784:

- **"Upstream affects downstream."** Errors in state estimation propagate into control regardless of how good the controller is.
- **Odometry:** an EKF fuses ERPM with IMU yaw rate at 50 Hz. They excluded the accelerometer because it was too noisy.
- **Localization trade-off:** Cartographer achieved 5 cm RMSE but **failed under wheel spin**. Their particle filter (SynPF) had about 20 cm RMSE but was robust to slip and used 30–50% less CPU.
- **Controller:** MAP was about **4.5% faster per lap with about 50% less lateral error** than pure pursuit.
- **Speed scaling:** lap time improved up to about 79% of the optimal profile. At 80% and above, the car hit the boundaries. Head-to-head ran at **75%**.
- **Overtaking:** passes succeeded against opponents at 66–80% of their own pace and **failed (collision) at 86%**. So do not attempt to pass anything faster than about 80% of our pace; trail it instead.
- **Opponent detection:** adaptive-breakpoint clustering with a track-boundary filter reached about 97% true-positive rate and 2% false-detection rate. A Frenet-frame EKF tracked the opponent with 0.17 m and 0.49 m/s RMSE.
- **Development style:** prototype in Python, then move only the latency-critical parts to C++.

---

## 3. Strategy: where the laps come from

Ranked by expected effect:

| # | Lever | Expected effect | KR260 element used |
|---|---|---|---|
| 1 | Friction-limited speed profile on a planned line, instead of reactive follow-the-gap | about 4–5× more laps than today | APU (C++), offline planner |
| 2 | Zero crashes and fast resets (CPU headroom, fallbacks, emergency braking, drills) | each avoided crash ≈ +1 lap | APU headroom; the DPU keeps vision off the CPU |
| 3 | Localization at speed (EKF with D435i gyro + particle filter) | enables #1 and allows a higher speed scaler | camera IMU, APU |
| 4 | Clean lapping and overtaking of slower cars | saves 1–2 s (or a crash) per pass, about 3 passes per heat | DPU YOLO + LiDAR fusion |
| 5 | Raceline shape (minimum curvature instead of centerline) | about 11% on the test loop | offline |
| 6 | Controller quality (delay compensation, then MAP) | about 5%, plus less lateral error, so a higher safe scaler | APU |
| 7 | Launch and first corner | wins the first corner against an equal opponent | APU, VESC tuning |
| 8 | Vehicle setup (steering and ERPM calibration, VESC FOC tuning, tires) | prerequisite for everything above | — |

**Custom FPGA accelerators are deliberately not on this list.** The measurements in §2.4 show the CPU can afford the particle filter. Within 14 days, with nobody on the team experienced in FPGA design, custom PL work would put the race at risk for no lap-time gain. It is in the stretch roadmap (§11).

---

## 4. Hardware configuration

### 4.1 Camera decision: one D435i, front-mounted

**Roles:**

- RGB 640×480 at 30 fps (60 fps if latency matters) feeds YOLO on the DPU.
- The IMU gyro (200 Hz) feeds yaw rate to the EKF; the accelerometer is enabled but ignored.
- The depth stream is **off**. The LiDAR provides range.

**Configure the IMU rates explicitly and check them with `ros2 topic hz`.** A July bag recorded the gyro at only about 81 Hz and the accelerometer at about 41 Hz while the board was loaded. Use `unite_imu_method: 2` (linear interpolation).

**Mounting:**

- Rigid, to keep vibration out of the IMU.
- Above or behind the LiDAR so it never blocks the scan plane. The LiDAR sits at x = 0.27 m, z = 0.11 m in `base_link`.
- Slightly tilted down.
- Measure the camera-to-LiDAR extrinsics with a ruler, then refine with one target test.

**Why not the alternatives:**

- **T265:** end-of-life. librealsense dropped it after 2.53.1, and this board has 2.57.7, so adding it would force a downgrade plus a separate wrapper. It also adds failure modes (pose jumps, tracking loss under vibration or motion blur). LiDAR particle filter + IMU + ERPM is the proven formula. Revisit only if Week 1 data shows odometry is what limits pace.
- **D415:** the same RGB sensor as the D435i, but no IMU and rolling-shutter depth.
- **Two or three cameras:** every stream costs scarce A53 CPU (USB handling, librealsense conversion, ROS). The LiDAR already covers 270°, and a rear camera adds little because blocking is not our strategy. The KR260's **four USB ports are already taken** by the D435i, VESC, Wi-Fi dongle and joystick receiver.

### 4.2 KR260 bring-up (Day 0–1, critical path)

1. **Boot firmware.** Ubuntu 22.04 needs K26 boot firmware **2022.1 or later**. A factory board may be older.
   - Check with `sudo xmutil bootfw_status`.
   - Update with `sudo xmutil bootfw_update -i <BOOT.BIN>`, reboot, then validate.
   - **Do this before any `apt upgrade`.** The Xilinx wiki warns the boot firmware utilities stop working after an upgrade.
   - If Ubuntu will not boot at all, use the KR260 Boot Image Recovery Tool (UG1092).
2. **Operating system.** Install the same Ubuntu 22.04 Kria image as the KV260 fresh, then run a setup script (Claude will write `setup_kr260.sh`): ROS 2 Humble, `librealsense`/`realsense2_camera`, `urg_node`, `slam_toolbox`, `robot_localization`, `nav2_map_server`, then the workspaces. Fallback: clone the KV260's SD card, then change the hostname, regenerate `/etc/machine-id` and re-authenticate Tailscale.
3. **Network.**
   - Connect the Hokuyo to a **PS-side RJ45 port**. Those work without PL firmware; the PL-side ports need TSN firmware, which the DPU overlay replaces.
   - Give the host a static address on 192.168.0.x (the Hokuyo is 192.168.0.10).
   - Wi-Fi dongle with the same BrosTrend rtl8852bu driver used on the KV260, plus Tailscale.
4. **Power.** The KR260 is designed for a **regulated 12 V** supply. A 3S pack runs from about 12.6 V down to 9.9 V and sags further under launch current, so do not feed the board from the raw pack. Use a 12 V buck-boost converter rated at least 4 A from the power distribution board, then log the supply during 10 hard launches.
5. **Thermal.** Keep the SOM fan running; the DPU adds sustained load.

### 4.3 Vehicle setup checklist (Day 1)

- [ ] **VESC Tool:** run FOC motor detection; set current limits and ramp; set the command timeout to about 300 ms with a timeout brake so the car stops if commands stop; review the sensorless settings (open-loop ERPM; HFI if this ESC supports it) for clean launches.
- [ ] **Steering:** calibrate servo offset and gain using a straight line plus a known-radius circle.
- [ ] **Speed:** calibrate the ERPM gain over a measured distance.
- [ ] **Test whether sensorless ERPM is trustworthy.** Compare ERPM speed against LiDAR scan-matching speed from 1 to 5 m/s. My hypothesis is that it is fine above about 1 m/s, and the team's bad experience came from the ~0.6 m/s driving where sensorless observers are weakest.
- [ ] **Check for an ESC IMU.** Look for real data on `/sensors/imu/raw`. If there is none, the D435i gyro is a sensor no competitor has.
- [ ] **Mechanical:** clean tires, wheel alignment, threadlocker, battery strap, no loose connectors.

---

## 5. Software architecture

### 5.1 Data flow

```
          Hokuyo UST-10LX        RealSense D435i                  VESC (sensorless BLDC + servo)
          /scan 40 Hz            RGB 30 fps      IMU 200 Hz       ERPM / state 50 Hz
               |                     |               |                 |
               |          +----------v---------+     |                 |
               |          | dpu_detector (C++) |     |                 |
               |          |  letterbox -> VART |     |                 |
               |          |  [PL: DPU B4096]   |     |                 |
               |          |  decode + NMS      |     |                 |
               |          +----------+---------+     |                 |
               |                     | car bearings  |                 |
               |                     |        +------v-----------------v------+
               |                     |        | EKF (robot_localization, C++) |
               |                     |        | odom -> base_link, 100 Hz     |
               |                     |        +---------------+---------------+
               v                     v                        v
       +---------------------------------------------------------------------+
       | race_core  (one C++ process, cores 2-3, SCHED_FIFO)                 |
       |  particle filter (map -> odom)     opponent detection + tracking    |
       |  Frenet frame on raceline          state machine                    |
       |  spline overtake / trailing        pure pursuit -> MAP, 100 Hz      |
       |  emergency braking (always on)     follow-the-gap = REACTIVE        |
       +-----------------------------------+---------------------------------+
                                           | /drive (Ackermann)
                     joystick deadman ---> ackermann_mux -> vesc_driver -> motor + servo
```

### 5.2 Modules

| Module | Implementation | Rate | Notes |
|---|---|---|---|
| Drivers | blaze_racer `subsystem_ws`: `urg_node`, `vesc_driver`, `ackermann_to_vesc`, `vesc_to_odom` (`publish_tf: false`), `joy`, `ackermann_mux`; `realsense2_camera` for the IMU | 40–200 Hz | reuse existing |
| Odometry EKF | `robot_localization` ekf_node (C++) | 100 Hz | inputs: VESC forward speed + D435i gyro z; publishes odom→base_link; replaces `cmd_odometry` |
| Mapping (race day) | `slam_toolbox` online-async using EKF odometry | — | existing `slam_mapping` configs; save map + pose graph |
| Raceline | ForzaETH `global_planner` around the TUM minimum-curvature optimizer, plus speed profile from friction/acceleration limits | offline, ~1–3 min | run without ROS on the KR260; laptop as backup; outputs a waypoint file (x, y, s, d_left, d_right, ψ, κ, v, a) |
| **race_core** (new, C++) | see the list below | scan-triggered + 100 Hz timer | one process, modules built as libraries, no Python |
| **dpu_detector** (new, C++) | librealsense color frames in-process (no ROS image transport) → letterbox 320×320 → VART → decode + NMS → `/perception/camera_dets` | 30 fps | debug image at 2 Hz only |
| Tools (Python, offline) | `race_day.sh {check,map,plan,race}`, lap analyser, sector tuner, bag analysis, raceline viewer | — | never inside the 40 Hz loop |

**race_core modules:**

- **Particle filter localization:**
  - likelihood-field model on a distance transform of the race-day map
  - 2000 particles, about 108 beams, 40 Hz, 2 threads
  - motion model from EKF odometry deltas, with noise inflated when slip is suspected (gyro vs. Ackermann yaw-rate mismatch)
  - scan deskewing
  - publishes map→odom plus a health signal (effective sample size, scan-match score)
  - reset modes: local re-initialization around the last pose, and global
- **Frenet projection:** position along and across the raceline (s, d).
- **Opponent detection:**
  - transform points into the map frame
  - keep only points inside the track corridor shrunk by 0.1 m
  - adaptive-breakpoint clustering
  - size filter 0.1–0.7 m
  - fuse YOLO bearings to label clusters as cars
- **Tracking:** Frenet EKF over (s, v_s, d, v_d), with static/dynamic voting and the YOLO label as a prior.
- **State machine:** START, GB_TRACK, TRAILING, OVERTAKE, REACTIVE, RECOVER, STOP. Transitions follow ForzaETH (opponent present, overtake feasible, overtake done, out of control, back in control).
- **Local planner:** ForzaETH "spliner" evasion spline, plus trailing control (PID on a 1.0–1.5 m gap).
- **Controller:**
  - v0: pure pursuit with speed-scaled lookahead and **actuation-delay compensation**; speed is the raceline speed at a delay-compensated lookahead point, times the scaler, reduced for lateral error.
  - v2: MAP (L1 guidance plus a steering lookup table from system identification).
  - runs at 100 Hz.
- **Follow-the-gap (REACTIVE):** a C++ port of `follow_the_gap_2`.
- **Emergency braking:** time-to-collision computed in the car's swept corridor. Always on, tuned so it never fires while trailing at the set gap.
- **Stuck recovery:** if the car has not moved for 1 s with an obstacle within 0.4 m ahead, back up 0.3 m and hand over to follow-the-gap.

### 5.3 Compute budget and core allocation

| Process | Core | CPU (est. unless marked) |
|---|---|---|
| `urg_node`, VESC driver and bridge, `ackermann_mux`, `joy` | 0 | 10–15% |
| `realsense2_camera` (IMU only, if `dpu_detector` owns the color stream) | 0 | 5–10% |
| `dpu_detector` (capture, preprocessing, post-processing; the DPU runs the network) | 1 | 15–25% |
| EKF (`robot_localization`, 100 Hz) | 1 | ~5% |
| `race_core`: particle filter 2000×108 (**34%, measured**), perception, planning, control | 2–3, SCHED_FIFO | 45–60% |
| rosbag2 logging (no images) | 0 | ~10% |
| **Total** | | **about 1.0–1.3 of 4 cores, leaving ≥2.5 cores of headroom** |

**Rules:**

- No Python node subscribes to `/scan` on the car while racing. Each one costs at least 13.5% of a core.
- `race_core` is pinned to cores 2–3 (`taskset` plus `chrt -f`).
- Do not record images during heats.
- Measure CPU and loop latency on every build and fail the build check if it exceeds the budget.

### 5.4 PL, DPU and YOLO pipeline

**Runtime on the board.** Follow AMD's KR260 route from `amd/Kria-RoboticsAI` (`install_update_kr260_to_vitisai35.sh`):

1. Install Kria-PYNQ with `install.sh -b KV260` ("from PYNQ SW stack point of view KV260 and KR260 are the same").
2. Install the Vitis AI 3.5 runtime from `vai3.5_kr260.zip`.
3. Install the DPU-PYNQ `design_contest_3.5` overlay (a B4096 DPU).

Validate this on the KV260 first. Then package the overlay as an `xmutil` app, or load it with a boot-time one-shot, so the C++ VART runner works without Jupyter.

**Fallback:** Kria-PYNQ's stock Vitis AI **2.5** DPU, with models compiled in the Vitis AI 2.5 docker. The compiler version must match the DPU in the bitstream.

**Model:**

- YOLOv8n detector with `activation: nn.ReLU()` in the model YAML. The DPU does not accelerate SiLU, and the attention blocks in YOLO11/YOLO26 fall back to the CPU.
- One class: `car`.
- Input 320×320 (try 416×320 if far detections matter).
- Export the raw head outputs. DFL decoding and NMS run on the CPU, only for candidates above the score threshold.

**Data:**

- The "CMU F1 Tenth – Cars and Walls" dataset: convert segmentation labels to boxes and drop `Wall`.
- Frames from our own D435i: the July bags plus new recordings.
- Auto-label new frames with the existing YOLO26n-seg on the RTX 3060, then review by hand.

**Training (RTX 3060):** 150–200 epochs, longer than usual because the activation changed. Augment with motion blur and brightness changes.

**Quantization (Vitis AI 3.5 PyTorch docker on the x86 box):**

- `vai_q_pytorch` post-training quantization with 300–500 calibration frames from our camera.
- Use fast finetuning or QAT if mAP50 drops by more than 2 points.

**Compilation:**

```
vai_c_xir -x <quantized>.xmodel -a /opt/vitis_ai/compiler/arch/DPUCZDX8G/KV260/arch.json -o out -n yolov8n_car
```

**Acceptance: exactly one DPU subgraph.**

**Expected performance:** about 3–6 ms per frame on the DPU at 320 px. This is an estimate; measure it on Day 2 with a model-zoo model. End-to-end latency will be dominated by the camera itself.

**How racing uses it:** to confirm and classify opponents only. **Racing never depends on the camera.** If the DPU detector is silent or dead, the car falls back to LiDAR-only perception automatically.

**Project deliverable:** this section is also the project's required "quantized YOLO on the DPU in the PL" deliverable.

### 5.5 What we reuse vs. build

| Component | Source | Action |
|---|---|---|
| Drivers, mux, teleop | blaze_racer `subsystem_ws` | reuse |
| Follow-the-gap | blaze_racer `follow_the_gap_2` | port to C++ as REACTIVE |
| Mapping | `slam_toolbox` (apt) + `slam_mapping` configs | reuse; switch odometry source to the EKF |
| Odometry EKF | `robot_localization` (apt) | configure |
| Raceline generation | ForzaETH `global_planner` + TUM optimizer (`ros2-humble` branch; packages declare MIT) | reuse offline |
| MAP steering lookup | ForzaETH `system_identification/steering_lookup` | reuse offline |
| Particle filter, controller, perception, tracking, state machine, spliner | ForzaETH logic and parameters (Python reference) | re-implement in C++ inside `race_core` |
| DPU detector | — | write new |
| Simulator | ForzaETH `f1tenth_gym_ros` (humble) in Docker on the x86 box | use for logic tests |

---

## 6. Race-day operations playbook

### 6.1 Pack list (Thursday night)

- [ ] Charged batteries, including spares
- [ ] Spare servo and spare parts
- [ ] Backup SD card image of the frozen KR260
- [ ] Laptop with RViz and the tools
- [ ] Joystick plus spare batteries
- [ ] Ethernet cable
- [ ] Hand tools
- [ ] Printed parameter card and these checklists

### 6.2 Practice session (target: race-ready in ≤ 15 minutes)

```
practice lap(s) -> slam_toolbox map -> global planner (min-curvature + speed profile) -> raceline
                                                                                           |
race card <- speed ramp 60->70->80% (lap analyser) <- validation lap @ 50% <- localize at start
```

1. `race_day.sh check`: confirm `/scan` at 40 Hz, IMU, VESC and the DPU detector. *(2 min)*
2. `race_day.sh map`: drive 1–2 laps at about 1.5 m/s (teleop or REACTIVE); save the map and pose graph. *(3 min)*
3. `race_day.sh plan`: generate the raceline. In RViz, check the line stays inside the track and has no spikes. *(3 min)*
4. Localize at the start line and drive one validation lap at 50% speed. *(1 min)*
5. Ramp the speed scaler 60 → 70 → 80%, one lap each. *(5 min)*
   - Stop increasing when lateral error exceeds about 0.15 m or there is a near miss.
   - Set the race scaler 5% below the last clean level.
   - Use per-sector reductions if one corner is the limiter.
6. Save the parameter card and the bags.

**If mapping or planning fails, use Plan B:** the REACTIVE race mode (tuned follow-the-gap). Practice it in rehearsals too.

### 6.3 Each heat

- [ ] Check battery voltage
- [ ] Place the car and keep it still for 2 s so the gyro bias can settle
- [ ] Select SAFE or ATTACK mode (§7)
- [ ] Hold the deadman button at the start signal
- [ ] Reset operator ready

### 6.4 Hand-reset drill (practice until it takes ≤ 3 s)

1. Release the deadman button. The car stops.
2. Place the car on the track facing the race direction, on or near the raceline.
3. Press **RESET** on the joystick. The particle filter re-initializes around the last pose, and the car drives REACTIVE until localization health is good.
4. Hold the deadman button again.

### 6.5 Between heats

- Review any crash or near miss in the bags (about 2 min).
- Adjust the scaler or mode.
- Time the next opponent's laps.

---

## 7. Head-to-head tactics

- **Scouting.** Time each opponent's laps during practice and earlier heats. Classify them as **SLOW** (below 80% of our pace) or **EQUAL**.
- **SAFE mode (against SLOW opponents).**
  - Race at the normal scaler.
  - Overtake only when clearance is at least 0.15 m (0.2 m against follow-the-gap cars, which weave), on a straight or wide section, and the opponent is at no more than 0.8× our speed.
  - Otherwise trail at a 1.0–1.5 m gap. Trailing briefly costs about a second; a collision costs a lap.
- **ATTACK mode (against EQUAL opponents).** ForzaETH's data says passing a car above 80% of our pace fails, so the plan is different:
  - Win the launch and the first corner.
  - Then stay fast and predictable.
  - If we end up behind, trail closely; the spliner triggers when the opponent slows or runs wide.
- **Start.** Use traction-limited launch control.
- **First-corner rule.** Claim the corner only if we are ahead by at least half a car length at the braking point. Otherwise tuck in behind and pass later on a wide section.
- **Defending.** Hold the racing line. No weaving: it is unpredictable for both cars and invites contact.
- **End of heat.** Only take overtaking risk if passing changes our final crossing count.
- **After a crash.** Run the reset drill. Do not raise the scaler to "make up time".

---

## 8. 14-day schedule

**Owners:**

- **You:** car, track and hardware.
- **Claude:** code and analysis on the boards.
- **x86 box:** Ubuntu 24.04, RTX 3060, Ryzen 5 5600G; used for training, quantization and simulation.

### 8.1 Day by day

| Day | Date | You (car/track) | Claude (code) | x86 box |
|---|---|---|---|---|
| D0 | Sat 10/24 | Approve plan. KR260: SD card, boot firmware check/update, first boot, put it on the network. Commit blaze_racer changes. | `setup_kr260.sh`; `race_core` scaffold; REACTIVE port | Docker + NVIDIA toolkit; pull Vitis AI 3.5 PyTorch docker; fetch the CMU dataset |
| D1 | Sun 10/25 | Vehicle setup (§4.3); record calibration bags (straight runs 1–5 m/s, circles) | ERPM-vs-LiDAR analysis; D435i IMU config; EKF config | Train YOLOv8n-ReLU v0 |
| D2 | Mon 10/26 | Map the test track with slam_toolbox + EKF. **DPU bring-up on the KV260** (needs sudo). | Raceline pipeline running without ROS on the blaze_track map; particle filter v0 | Quantize and compile v0; check for one subgraph |
| D3 | Tue 10/27 | **M1:** first autonomous raceline laps at 40–50% | Pure pursuit + delay compensation + speed profile; lap analyser | — |
| D4 | Wed 10/28 | Speed ramp 50→70%; tune lookahead; failure log | Particle filter v1 (deskew, latency compensation); emergency braking; GB_TRACK/REACTIVE | YOLO v1 (own frames, auto-labelled) |
| D5 | Thu 10/29 | **Swap the KR260 into the car**; repeat baseline tests on it | `dpu_detector` (VART); camera to detections in ≤ 50 ms | Quantize and compile v1 |
| D6 | Fri 10/30 | Speed ramp 70→80%; system-identification runs (steady circles at 3/4/5 m/s) | LiDAR opponent detection + tracking; YOLO fusion | Simulator set up for head-to-head |
| D7 | Sat 10/31 | **Rehearsal #1:** rearrange the track; time the full map → race pipeline | MAP controller (steering lookup) | Simulator: spliner and trailing scenarios |
| D8 | Sun 11/1 | **M2 gate** | Spliner overtaking + trailing | — |
| D9 | Mon 11/2 | Opponent tests with a second car (a human-driven RC car is fine): trailing, passes at 60–80% pace | Launch control, first-corner rule, reset recovery | — |
| D10 | Tue 11/3 | Hand-reset drills; overtake tuning | Fixes; parameter cards | — |
| D11 | Wed 11/4 | **Rehearsal #2:** mock 60 s heats with an opponent and resets | Fixes | — |
| D12 | Thu 11/5 | **M3 freeze:** tag the release; image the SD card | Docs and checklists | — |
| D13 | Fri 11/6 | Tuning only; spares; batteries; pack | — | — |
| D14 | Sat 11/7 | **RACE** | — | — |

### 8.2 Gates and fallbacks

**M1, Tue 10/27:** autonomous laps on the raceline with localization at 40–50% speed.

- Fallback: if M1 is still missed by Thu 10/29, the race strategy switches to Plan B (tuned REACTIVE racer). Map-based work continues in parallel.

**M2, Sun 11/1:** all of the following.

- KR260 in the car.
- ≥ 10 consecutive laps at ≥ 75% scaler.
- Rehearsal #1 pipeline ≤ 20 min.
- CPU ≤ 60%.

The KR260 is mandatory race hardware. **If its bring-up is not done by Fri 10/30, escalate that day.**

**DPU sub-gates:**

- Model-zoo model running on the KV260 DPU by Tue 10/27.
- Our YOLO on the DPU by Fri 10/30.
- Fused with LiDAR by Mon 11/2.

If the DPU slips past Mon 11/2, race LiDAR-only and finish the DPU work in parallel for the project deliverable.

**M3, Thu 11/5:** head-to-head features (trailing, overtaking, reset) validated in drills. Tag the release and freeze; only parameter changes after this point.

### 8.3 Immediate next actions

Updated Sunday 10/25 (D1). Today's session follows `TEST_DAY.md`.

**You (today):**

1. KR260 bring-up from the bundle (`TEST_DAY.md` §0), then into the car (§1).
2. Follow-the-gap laps on race_core (§2).
3. Calibration bags (§3). The steering limits matter most: `vesc.yaml` implies 0.57 rad left but only 0.27 rad right.
4. Map, raceline, first raceline laps at 40% and up (§4-5).
5. Bags and maps back to Claude (§7).

**Claude (next):**

1. Apply the calibration results, and tune the controller from the first raceline bags.
2. Spline overtaking (ForzaETH spliner), and camera-LiDAR fusion of the detections.
3. Scan deskewing and latency compensation in the particle filter.
4. Detector v3 with frames of a real second car (§6 of the runbook).

---

## 9. Test and validation plan

- **Simulation (x86).** Run ForzaETH's `f1tenth_gym_ros` (humble) in Docker. Test `race_core` logic against scripted opponents (follow-the-gap; raceline at 60/80/100%), including resets. Convert the blaze_track map for the simulator.
- **Hardware-in-the-loop CPU test.** Run the simulator on the x86 box and `race_core` on the KV260 over the network. Measure CPU and latency.
- **Bag replay.** Run particle-filter and perception regression tests on recorded bags after every change.
- **On-track progression.** Calibration → mapping → 40% → … → 80% scaler, with ≥ 3 clean laps before each step. Log lateral error, speed tracking, localization health and CPU.
- **Opponent testing.** We need a second car; any RC car driven by a person works as a moving obstacle. Use static boxes to test the spliner.
- **Metrics to track every session:**
  - lap times
  - lateral error (RMS and maximum)
  - near-miss count
  - CPU %
  - loop latency p99
  - detection precision and recall

---

## 10. Risk register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| KR260 won't boot Ubuntu 22.04 (old boot firmware) | Medium | High | Update firmware first via `xmutil` or the recovery tool, Day 0 |
| A53 CPU overload causes latency and jitter | High if Python | High | C++ `race_core`, budget table, pinning, measure every build |
| Sensorless ERPM odometry is poor | Medium | High | Verify Day 1; EKF with gyro; slip-adaptive particle filter; T265 experiment only if needed |
| Localization lost at speed | Medium | High | Health signal triggers REACTIVE fallback; scaler; scan deskewing |
| Vitis AI / DPU toolchain friction | Medium | Medium for the project, Low for the race | Validate on KV260 early; Vitis AI 2.5 fallback; car races LiDAR-only |
| Unknown track and floor friction | High | Medium | Rehearse the pipeline twice; conservative ramp; per-sector scalers |
| Collisions in heats | Medium | High | 80% overtake rule; trailing; emergency braking; SAFE mode |
| Computer brownout during launch | Medium | High | Regulated 12 V; launch test with logging |
| Only one car, so no head-to-head testing | High | Medium | Simulator, human-driven RC car, static obstacles; ask other teams |
| One-person bandwidth | High | High | Tiered scope; freeze on Nov 5; Claude writes the code |
| Regression right before the race | Medium | High | Freeze, tagged release, SD card backup, rehearsals |
| Venue Wi-Fi unavailable or crowded | Medium | Medium | Car fully self-contained; launch via joystick or systemd |

---

## 11. Stretch roadmap: using more of the KR260

Only after M2/M3, or after the race.

| Idea | What it buys | Why it is not on the critical path |
|---|---|---|
| Particle filter in the PL (HLS) | Full 1081-beam evaluation with 10k+ particles at 40 Hz in about 1 ms | The CPU filter already takes 8.5 ms (34% of a core). A custom bitstream must also contain the DPU; builds take hours; no FPGA specialist on the team |
| Cortex-R5F safety supervisor (OpenAMP) | Watchdog and emergency braking independent of Linux | Needs the VESC UART routed through the PL (custom bitstream); modest benefit |
| Image preprocessing in the PL (Vitis Vision) | About 2 ms of CPU per frame | Negligible at 320 px |
| Hardware video encoder (H.264) race video | Debug video at no CPU cost | Nice to have |
| Basalt VIO or T265 | Odometry independent of wheel slip | CPU cost or end-of-life software; only if Week 1 data shows odometry limits pace |
| Bayesian sector tuning | Per ForzaETH: same lap time with about 1.5× less lateral error | Needs many laps |

---

## 12. Open questions

### For the organizers

1. **Track:** width range, barrier type and height (barriers must be visible to a LiDAR at about 10–15 cm), floor surface, approximate length. Is the same track used for every heat?
2. **Practice:** how long, how many cars on track at once, and can we drive solo mapping laps?
3. **Start:** who gets which grid slot (inside vs. outside), what is the start signal, and are the cars behind the line?
4. **Counting:** does the first crossing at the start count? What is the tie-break?
5. **Tournament format:** round-robin or bracket? How many heats per team, and how much time between heats (for battery swaps)?
6. **Resets:** who resets the car, where is it placed (crash spot or start line), and is there a time penalty?
7. **Contact:** is there any penalty for causing a collision?
8. **Operation:** may we hold a deadman button and press RESET during heats? May we change parameters between heats?
9. **Wireless:** is Wi-Fi allowed and available at the venue?

### For the team

1. Can we get a second car (any RC car) for opponent testing?
2. Where exactly is the "CMU F1 Tenth – Cars and Walls" dataset?
3. How should Claude access the x86 box (SSH over Tailscale)?
4. What regulator powers the computer today?
5. What is the exact ESC model, and does it report IMU data?
6. Is sudo access OK for installing packages on the KV260 and KR260?

---

## 13. References

- ForzaETH race stack, `ros2-humble` branch: https://github.com/ForzaETH/race_stack
- ForzaETH paper (head-to-head racing on commercial off-the-shelf hardware): https://arxiv.org/abs/2403.11784
- MAP controller (Model- and Acceleration-based Pursuit): https://arxiv.org/abs/2209.04346
- TUM global race trajectory optimization: https://github.com/TUMFTM/global_racetrajectory_optimization
- AMD Kria-RoboticsAI (KR260 + Vitis AI 3.5): https://github.com/amd/Kria-RoboticsAI
- Kria-PYNQ: https://github.com/Xilinx/Kria-PYNQ
- DPU-PYNQ (archived; KV260/KR260 B4096): https://github.com/Xilinx/DPU-PYNQ
- Ultralytics AMD Xilinx integration (DPU constraints: SiLU, attention, `arch.json`): https://docs.ultralytics.com/integrations/xilinx
- Vitis AI Library 3.5, YOLOv8 detection (UG1354): https://docs.amd.com/r/en-US/ug1354-xilinx-ai-sdk/YOLOv8-Detection
- Certified Ubuntu 22.04 for Xilinx devices (boot firmware table): https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/2363129857
- KR260 Boot Image Recovery Tool (UG1092): https://docs.amd.com/r/en-US/ug1092-kr260-starter-kit/Boot-Image-Recovery-Tool
- Kria KV260/KR260 firmware update for Ubuntu 22.04 (element14): https://community.element14.com/technologies/fpga-group/b/blog/posts/kria-kv260-kr260-firmware-update-for-booting-ubuntu-22-04
- YOLOv11 on Vitis AI 3.5 and the DPU (LogicTronix): https://www.hackster.io/LogicTronix/making-yolov11-compatible-with-vitis-ai-3-5-and-dpu-aaad80
- KR260 DPU architecture benchmarks (iotengineer22): https://hackster.io/iotengineer22/benchmark-architectures-of-the-dpu-with-kr260-699f19
- Jetson Orin Nano Super specs: https://cnx-software.com/2024/12/18/249-nvidia-jetson-orin-nano-super-developer-kit-targets-generative-ai-applications-at-the-edge/

---

## Appendix A: Reproducing the measurements

Everything is in `research/`. Source ROS first (`source /opt/ros/humble/setup.bash && source ~/airacer_basecode/install/setup.bash`); the ROS scripts need it for `ackermann_msgs`.

| What | Command |
|---|---|
| Lap-time model + raceline figure | `python3 research/laptime_model.py ~/airacer_basecode/blaze_racer/maps/blaze_track.png /tmp/raceline.png` |
| Particle filter cost, 1 core | `g++ -O3 -mcpu=cortex-a53 -ffast-math -o pf_bench research/pf_bench.cpp && ./pf_bench` |
| Particle filter cost, 2 threads | add `-fopenmp` to the build, then run `OMP_NUM_THREADS=2 ./pf_bench` |
| Python per-scan cost | `python3 research/rclpy_bench.py` |
| Python node overhead, end to end | `export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1; python3 research/rclpy_e2e.py pub & sleep 2; python3 research/rclpy_e2e.py sub` |
| Bag statistics (speed, scan rate, corridor width) | `python3 research/bag_stats.py <bag_dir> ...` |

The YOLO CPU timing used onnxruntime with 4 threads on the existing ONNX model in `blaze_racer/autonomy_ws/src/yolo_detection/yolo_detection/`.

---

## Appendix B: Decision log

| Date | Decision | Reason |
|---|---|---|
| 2026-10-24 | Use the KV260 as the development twin; race on the KR260 | Same K26 SOM; the KR260 flow compiles with the KV260 `arch.json` |
| 2026-10-24 | One D435i; no T265, no extra cameras | IMU plus RGB in one device; CPU and USB-port budget; T265 is end-of-life (§4.1) |
| 2026-10-24 | C++ `race_core`; ForzaETH used as the reference design, not run as-is | Measured Python overhead plus ForzaETH's 245–296% i5 CPU (§2.4) |
| 2026-10-24 | YOLOv8n-ReLU on the DPU replaces YOLO26n-seg for racing | DPU operator support; 1.08 s per frame on the CPU (§2.3, §2.4) |
| 2026-10-24 | Particle filter instead of Cartographer | ForzaETH: robust to wheel slip, 30–50% less CPU; our CPU cost measured |
| 2026-10-24 | Automated map → raceline pipeline, plus REACTIVE Plan B | Track only revealed at race-day practice |
| 2026-10-24 | Reset drill plus REACTIVE recovery state | Hand resets are allowed; a crash costs about one lap |
| 2026-10-24 | No custom PL accelerators before M3 | No lap-time gain measured; schedule risk; no FPGA specialist on the team |
| 2026-10-25 | KR260 goes into the car on D1 (Sun 10/25) instead of D5 | User's call: live tests on the race board from day one; the KV260 stays the dev twin |
| 2026-10-25 | Own raceline generator (`race_core/tools/make_raceline.py`, numpy + OpenCV) instead of the TUM optimizer via ForzaETH's global planner | No extra dependencies on the board, ~3 s on the A53; sequential QP with uniform resampling, curvature limit from the measured max steering, Euclidean wall clearance (sharp wall corners). Reconsider TUM only if lap times trail the model |
| 2026-10-25 | Particle filter default 1000 particles x 108 beams, 2 OpenMP threads (passive wait) | Same localization in the simulator (mean error 2.2 cm) at 31% of one core and 3.3 ms per scan; 2000 particles cost 54% |
| 2026-10-25 | Raceline wall margin 0.35 m (centre to wall) by default; pure pursuit lookahead 0.25 + 0.20 v, min 0.5 m | Simulator on blaze_track: 0.30 m margin and longer lookahead clipped the island tips at 70-85%; with these values 0 crashes at 85% (7.3 s laps) |
| 2026-10-25 | Emergency braking: frontal TTC on the steering arc; on the raceline only points the map does not explain, plus a hard 0.12 s limit on everything | Walls next to a narrow raceline caused one false brake per lap; the planner already keeps the wall margin |
| 2026-10-25 | Autonomous deadman enforced inside race_core (hold RB) | blaze_racer's joy_teleop "autonomous_control" deadman publishes to /dev/null, i.e. autonomy was never gated |
| 2026-10-25 | Detector v2neg (v0 fine-tuned with 829 of our own July D435i frames as background) is the default; boxes wider/taller than 60% of the frame are dropped | v0 fired on 14% of held-out July frames (frame-filling boxes on blurred ducts); v2neg 0/269, val AP50 0.972 INT8 |
| 2026-10-25 | Overtaking off by default for the first car tests; spline overtake ready to enable | Reactive (follow-the-gap) overtaking failed on the narrow blaze_track in simulation; the spline overtake passes slow cars with 0 collisions in simulation but has not met a real second car yet |

---

## Appendix C: Progress log

| Date | Done | Evidence |
|---|---|---|
| 2026-10-24 | `race_ws/src/race_core` v0: C++ REACTIVE mode (port of `follow_the_gap_2`) with scan-timeout watchdog, loop-latency logging, and launch pinned to cores 2–3 | 7/7 gtests pass. `tools/ftg_parity.py`: 3,899 recorded scans match the Python planner to 5e-10 rad with 0 differing commands. Bag replay: 2.4% of one core vs 25.2% for Python; p99 0.2 ms per scan |
| 2026-10-24 | `setup/setup_kr260.sh`: fresh Ubuntu 22.04 → race configuration (boot-firmware guard, ROS 2, drivers, slam_toolbox, robot_localization, Wi-Fi, Tailscale, Hokuyo IP, workspaces) | Syntax-checked only; not yet run on a KR260 |
| 2026-10-24 | KV260 packages: slam_toolbox, robot_localization, nav2 map server + AMCL + lifecycle manager, MCAP storage, CycloneDDS | apt install completed |
| 2026-10-24 | EKF config (`race_core/config/ekf.yaml`) + `state_estimation.launch.py` (D435i IMU at explicit rates, camera mount TF placeholder, robot_localization) | Written; untested until the car runs. Camera mount pose must be measured |
| 2026-10-24 | RTX box: SSH key access, Docker + GPU, Vitis AI 3.5 image + ultralytics (`race-vitis:3.5`), training venv (torch 2.6 CUDA) | GPU containers and nndct import verified |
| 2026-10-24 | Dataset `~/race/datasets/f1tenth_car` on the RTX box: 5 Roboflow F1Tenth sets (CMU, UniBo, KSAE, CU, f1-tenth) merged to one class `car` | 8,851 train / 480 val / 184 test images; val/test keep one copy per original photo; no train image shares an original with val/test |
| 2026-10-24 | YOLOv8n-ReLU car detector v0 (320 px, `~/race/yolo/train_car.py`) | Float: val mAP50 0.977, test mAP50 0.994 (P 0.992, R 0.989). **Public data only; still to be checked on our own D435i frames (G5)** |
| 2026-10-24 | Quantize + compile for the DPU (`~/race/vitis/quantize_dpu.py`): 6 raw outputs (box DFL + class logit per scale), DFL/decode/NMS on CPU | INT8 AP50 0.974 vs float 0.975 on 480 val images (−0.1 pt). Exactly 1 DPU subgraph. Model: `models/yolov8n_car_v0.xmodel` |
| 2026-10-24 | DPU runtime on the KV260 (`setup/install_dpu_vai35.sh`: Kria-PYNQ + Vitis AI 3.5 runtime + DPU-PYNQ 3.5 overlay; Jupyter disabled; PYNQ venv moved out of `/etc/profile.d` to `/etc/pynq_venv.sh`; XRT 2.13 backup in `/root/xrt-2.13-backup.tgz`) | `xdputil query`: DPUCZDX8G_ISA1_B4096 @ 300 MHz, IP v4.1.0, fingerprint 0x101000056010407 = model fingerprint |
| 2026-10-24 | **DPU inference measured on the KV260** (`research/dpu_bench.py`) | **p50 6.0 ms, p99 7.4 ms per frame (~167 FPS)** vs 1,076 ms for the old YOLO26n-seg on the CPU (~180×). Harmless log line: "cannot set read range" from the register probe |
| 2026-10-25 | **race_core v1** (C++): occupancy map + distance field, likelihood-field particle filter, raceline/Frenet, pure pursuit with delay compensation, emergency braking, LiDAR opponent detection + tracking + TRAILING, deadman (RB) / RESET (A), auto raceline direction, lap timer, telemetry, global localization service | 18/18 gtests (incl. 5 overtake-planner tests). Simulator, blaze_track, mu 0.6: 85% scaler 7.3 s laps, 0 crashes in 60 s; race_core 31% of one core, scan pipeline p50 3.7 ms / p99 4.4 ms; localization error vs truth mean 2.2 cm, p95 5.8 cm |
| 2026-10-25 | `race_sim` closed-loop simulator (bicycle + friction limit + actuator lag + delay, ray-marched LiDAR, drifting odometry, crashes + hand resets, opponent) and `tools/sim_eval.sh` | Deadman test: stop 0.3 s after the requirement is enabled, 0.8 s after /joy is lost, 0.55 s after RB release. Trailing a 40% opponent: 0 crashes |
| 2026-10-25 | **Race-day pipeline rehearsed in simulation:** follow-the-gap laps -> slam_toolbox on EKF-style odometry (loop-closure saver wrote the map after 1 lap) -> `make_raceline.py` -> race mode on the SLAM map | 70% scaler: 8.66 s laps, spread 0.01 s, 0 crashes, 0 emergency brakes |
| 2026-10-25 | `make_raceline.py`, `lap_report.py` (bag -> lap table + plot), `calib_report.py` (speed gain from wall runs, steering gain/offset + per-side max from yaw rate), `race.sh` (one entry point), `TEST_DAY.md` runbook | Generator 3 s on the KV260 |
| 2026-10-25 | **dpu_detector** (C++ VART RunnerExt, zero-copy buffers; offline CLI + ROS node) and `dpu-overlay.service` (loads the DPU at boot) | Bit-identical to the Python reference (TP 54 / FP 4 / missed 3 on the 60-image sample). Live on the D435i: 9.4 ms per frame (DPU 5.5 ms, preprocessing 3.4 ms), 20% of one core, ~47 ms camera-to-detection. Runs as the normal user (needs glog 0.5 from /usr/lib first: handled with RUNPATH) |
| 2026-10-25 | Detector v2neg (RTX box: fine-tune with July background frames; quantized with 200 public + 100 own calibration frames) | INT8 AP50 0.972 (float 0.978); held-out July frames with a detection: 0/269 (v0: 36/269) |
| 2026-10-25 | YOLOv8s-ReLU trained for comparison | val mAP50-95 0.816 vs 0.814 for v8n: not worth 3x the DPU time; v8n stays |
| 2026-10-25 | Real-bag check of mapping/localization (July 22 bag, blaze_track map) | slam_toolbox with scan matching only, or with VESC odometry, gave smeared maps; the PF did not track reliably on that bag. The bag's odometry (servo-derived yaw) is poor and the July 22 layout may differ from the July 8 map. **Takeaway: map at ~1 m/s with the EKF (D435i gyro) running** |
| 2026-10-25 | **Two EKF bugs fixed, both tested on the KV260 with the real D435i.** (1) `ekf.yaml` mixed `0` and `0.05` in `process_noise_covariance`, so the ekf_node crashed at start-up. (2) robot_localization reads `imu0_config` in the IMU message frame (camera optical frame), so "vyaw" was the car's roll rate and the gyro was never fused. Now all three angular rates are enabled and two_d_mode keeps yaw | Debug output: transformed update vector has vyaw set. EKF yaw rate at rest -0.0013 rad/s = the measured gyro bias (D435i IMU uncalibrated; bias 0.001-0.005 rad/s, noise 0.002 rad/s). IMU 200 Hz, EKF 100 Hz, color 30 Hz |
| 2026-10-25 | Scan deskewing in race_core (each beam moved to mid-scan with the odometry's v and yaw rate); race_sim now simulates the rotating-mirror skew (stamp = first beam, as urg_node) | Simulator at 85% with skew: localization error p95 6.6 cm with deskewing vs 10.3 cm without |
| 2026-10-25 | Controller retuned in the simulator: `pp.delay` 0.07 s, `pp.lookahead_time` 0.15 s. The calibration tool now estimates the real actuation delay from command vs yaw rate | Simulator, blaze_track: **85% 6.85 s laps, RMS lateral error 0.05 m, max 0.11 m, 0 emergency brakes; 95% 6.1 s laps, 0 crashes**. The model predicts 6.9 s at 85% and 5.9 s at 100% |
| 2026-10-25 | Compute budget measured on the KV260 with everything that runs on the car except the drivers: race_core at 70% in the simulator, realsense (color 30 fps + IMU 200 Hz), dpu_detector, EKF | race_core 34%, realsense 26%, dpu_detector 18%, EKF 7% of one core, about 21% of the 4 cores (budget: 60%); race_core p99 7.6 ms under that load |
| 2026-10-25 | Spline overtake (`overtake.mode: spline`): lateral offset from the raceline with cosine ramps, on the side with more room over the whole predicted pass, re-planned each scan on the same side, abort with hysteresis + 2 s cooldown, speed scale +0.15 while passing. A dispatch bug (OVERTAKE was driven by follow-the-gap) found and fixed; trailing now checks the opponent against our planned path | Simulator, 2 m wide stadium, us at 75%: vs a 40% opponent, 3 min: 10 attempts, 5 passes, 5 clean aborts, **0 collisions**, laps 8.0-9.2 s instead of 15.2 s trailing; vs 25%: 6 of 8 passes, 0 collisions; vs 60% (above the 80% rule): no attempts, trails at 10.1 s; on the narrow blaze_track it never starts (no room) and trails safely. **Off by default until tried with a real second car** (`ros2 param set /race_core overtake.mode spline`) |
| 2026-10-25 | Camera-LiDAR fusion v0: race_core marks the LiDAR opponent track "camera-confirmed" when a fresh dpu_detector bearing span covers it (+-4 deg); telemetry fields 24-25, log line. Label only | Simulator with detections synthesized from truth: confirmations at 2.4-5.6 m, no effect on driving |
| 2026-10-25 | Track-side tuning aids: per-sector speed factors (`sectors` parameter, live), `lap_report.py` lists the worst 1 m sectors and emergency-brake positions by arc length, RViz config (`./race.sh rviz`), `./race.sh scale` / `relocalize` | Simulator: slowing s 2-8 m to 60% live changed laps from 6.88 s to 8.30 s as expected |
| 2026-10-25 | Code bundle for the KR260 on the RTX box (`~/kr260_bundle.tgz`: `~/2x1_w` + blaze_racer kria-race snapshot) | `setup_kr260.sh` now builds slam_mapping and race_ws with their dependencies |
| 2026-10-26 | KR260 bring-up from the KV260 (`setup/migrate_to_kr260.sh` rsync over Tailscale, then setup_kr260.sh steps without `apt upgrade`): boot FW K26-BootFW-01.02 (A/B bootable; 1.07 update deferred to a bench day), ROS 2 Humble + drivers mirroring the KV260 package set, rtl8852bu Wi-Fi on "AI Racer", Hokuyo on eth0 = bottom PS port (192.168.0.15), unattended apt disabled, Fast DDS / domain 0 kept | All workspaces built (one compile job: two parallel jobs froze the 4 GB board, no swap); race_core 21 tests, 0 failures. Open: Wi-Fi dongle at power-on stalls boot ~4 min (fake CD-ROM 0bda:1a2b; fix = `usb-storage.quirks=0bda:1a2b:i`), swap file, DPU runtime |
| 2026-10-26 | KR260 overnight (Claude on the RTX box): image kernel 1027 oopsed in `uvcvideo` when the D435i stream stopped, so only the kernel was upgraded to 1080 (same as the KV260); 4 GB swap; Wi-Fi boot quirk; DPU runtime (Kria-PYNQ + VAI 3.5) with two installer fixes (eth0 without IP, `/etc/vart.conf`); `dpu-overlay.service` at boot; `race.sh check` reads the KR260 temperature | D435i color 30 Hz / IMU 200 Hz / EKF 100 Hz with clean stops; DPU fingerprint `0x101000056010407`; check set TP 54 FP 4 missed 3, DPU p50 5.54 ms (= KV260); live detector 30 Hz, 9.2 ms. LiDAR wire broken: `/scan` untested. `race-stack` branch pushed |
