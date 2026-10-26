# Test Day Runbook: KR260 in the car (first written for Sunday 2026-10-25)

> **Status 2026-10-26:** §0 (bench bring-up) is done on the KR260; details and open items are in `KR260_STATUS.md`. Before the track: solder the LiDAR wire, then start at §1. Claude runs on the RTX box (`ssh claude@100.105.87.85`, `tmux attach -t race`) and reaches the car with `ssh kr260`.

This is the step-by-step for the first live session on the KR260. Each step lists the command, what you should see, and what to do if you don't. Do the steps in order; stop and tell Claude at the first failure (paste the terminal output).

The goal for today, in priority order:

1. KR260 boots the race software and drives the car by joystick.
2. Follow-the-gap (REACTIVE) laps on race_core, the Plan B race mode.
3. Calibration bags: speed gain and steering (straight runs and circles).
4. A map of the test track and a raceline.
5. First autonomous laps on the raceline at 40%, then 50–60% if they are clean.
6. The DPU detector running on the car's camera while driving; frames of a second car if one is available.

**Joystick:** 8BitDo Ultimate 2C on its **2.4G dongle** (XInput mode). Before driving, run `ros2 topic echo /joy --field buttons` with bringup up: holding LB, RB and pressing A must set indices 4, 5 and 0.

- **Hold LB:** you drive (teleop always wins).
- **Hold RB:** race_core drives. Release RB and the car stops; this is the deadman.
- **Press A:** RESET. Use it after a hand reset: race_core re-localizes and drives REACTIVE until localization is healthy again.
- **If the indices are different** (controller in Bluetooth or another mode), re-pair it on the 2.4G dongle; as a stopgap, `ros2 param set /race_core reset_button <index of A>` (the LB/RB deadmen are in blaze_racer `subsystem_stack/config/joy_teleop.yaml` and race_core `deadman_button`).

---

## 0. Before the track (at the bench, about 1–1.5 h)

### 0.1 First boot and boot firmware

1. Boot the KR260 from the Ubuntu 22.04 SD card. Log in as `ubuntu`.
2. Check the boot firmware:
   ```
   sudo xmutil bootfw_status
   ```
   - It must be 2022.1 or later (a factory-new board may not be).
   - If it is older, update it **before any apt upgrade**: `sudo xmutil bootfw_update -i <BOOT.BIN>`, reboot, then `sudo xmutil bootfw_update -v`.
   - See RACE_PLAN.md §4.2.

### 0.2 Network

- **The KR260 has two PS-side RJ45 ports** that work without PL firmware.
  - Plug one into a router for internet.
  - The other is for the Hokuyo.
  - `ip -br link` shows which port has link. Note the name of the free port (`eth0` or `eth1`) for the Hokuyo.
- Wi-Fi dongle and Tailscale come in the setup script.

### 0.3 Get the code onto the KR260

Use whichever works:

- **(a) From the RTX box (always on).** It holds a bundle prepared tonight:
  ```
  scp claude@100.105.87.85:~/kr260_bundle.tgz ~ && tar -xzf ~/kr260_bundle.tgz -C ~
  ```
  This creates `~/2x1_w` and `~/airacer_basecode/blaze_racer`. Use the claude password you set.
- **(b) From GitHub,** if you pushed `2x1_w` (see `GIT_PUSH.md`).
- **(c) From the KV260,** if it is on:
  ```
  rsync -a ubuntu@kria:~/2x1_w/ ~/2x1_w/
  ```

### 0.4 Setup script

About 30–60 min, mostly apt and building the Wi-Fi driver.

```
cd ~/2x1_w/setup && HOKUYO_IFACE=eth1 bash setup_kr260.sh     # use the port name from 0.2
sudo tailscale up                                             # log the board into the tailnet
```

Log out and back in (group changes). Then check:

```
cd ~/2x1_w/race_ws && colcon build && colcon test --packages-select race_core && colcon test-result
```

Expected: `Summary: 21 tests, 0 errors, 0 failures` (18 gtests plus 3 CTest entries).

### 0.5 DPU runtime

About 20–30 min. It downloads about 1 GB.

```
cd ~/2x1_w/setup && sudo BOARD=KR260 bash install_dpu_vai35.sh
```

Then:

```
sudo bash -c 'source /etc/pynq_venv.sh && xdputil query' | grep -E 'DPU Arch|fingerprint'
cd ~/2x1_w/race_ws && colcon build --packages-select dpu_detector       # now that VART exists
~/2x1_w/race_ws/install/dpu_detector/lib/dpu_detector/dpu_detect_images \
    ~/2x1_w/models/yolov8n_car_v0.xmodel ~/2x1_w/models/val_sample/images \
    --labels ~/2x1_w/models/val_sample/labels --conf 0.25 --max-frac 1.0
```

**Expected** (identical to the KV260):

- `DPUCZDX8G_ISA1_B4096` with fingerprint `0x101000056010407`.
- `TP 54, FP 4, missed 3`.
- DPU p50 about 5.5 ms.

**If `xdputil` says "Not support this dpu arch" and shows a "cannot set read range" line,** that is harmless (it happens on the KV260 too).

**If the fingerprint is not `0x101000056010407`:**

- DPU-PYNQ installs a KR260-specific bitstream (`kr260_som`). AMD's flow compiles KR260 models with the KV260 arch, so it should match.
- If it does not match, the model files will refuse to load. Send Claude the fingerprint line; recompiling for it takes about 10 minutes on the RTX box.

The overlay now loads at every boot (`systemctl status dpu-overlay`).

---

## 1. Car integration (on a stand, wheels off the ground)

- **Power:** feed the KR260 from a **regulated 12 V** supply (buck-boost rated at least 4 A). Do not feed it from the raw 3S pack.
- **Cabling:**
  - Hokuyo goes to the **bottom PS RJ45 port** (Linux `eth0`, 192.168.0.15; LiDAR at 192.168.0.10).
  - VESC USB, D435i (USB 3, blue port), joystick dongle and Wi-Fi dongle go to the four USB ports. The Wi-Fi dongle can stay plugged in at power-on (boot quirk applied 10/26); first boot after a power-on takes about 1–2 min.
- **Check the devices:**
  ```
  ls /dev/ttyACM0 /dev/input/js0 && lsusb | grep -i realsense && ping -c2 192.168.0.10
  ```

**Terminal 1:**

```
cd ~/2x1_w && ./race.sh bringup
```

**Terminal 2:**

```
cd ~/2x1_w && ./race.sh ekf
```

**Terminal 3:**

```
cd ~/2x1_w && ./race.sh check
```

| Topic | Expected | If not |
|---|---|---|
| /scan | ~40 Hz | Hokuyo IP/cable; `nmcli con up hokuyo` |
| /odom | ~50 Hz | VESC on /dev/ttyACM0? `dialout` group (log out/in) |
| /joy | ~20 Hz while a button is held | 8BitDo on the 2.4G dongle (not Bluetooth); `ls /dev/input/js0` |
| /camera/camera/imu | ~200 Hz | USB 3 port; `rs-enumerate-devices` |
| /odometry/filtered | ~100 Hz | needs /odom and imu |

**Yaw-rate sign check (30 s, important for mapping and localization):**

1. Run `ros2 topic echo /odometry/filtered --field twist.twist.angular.z`.
2. Turn the car by hand **counter-clockwise** (to the left, seen from above).
3. The value must go **positive** (about +1 for a brisk turn), and back to about 0.00 when you stop.
4. **If it stays at 0.00 or goes negative,** stop and tell Claude: the EKF is not using the gyro.

The EKF config had two bugs that were found and fixed tonight. The first crashed it on start-up. The second fused the camera's roll rate instead of the yaw rate.

**Hold LB and drive the wheels.** Steering left on the stick must turn the wheels left, and forward on the stick must spin them forward.

**Sign check of the autonomy path (on the stand, 1 min).** race_core uses the ROS convention: positive steering means left. Run:

```
ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {steering_angle: 0.3, speed: 0.0}}"
```

- The wheels must turn **left**. `Ctrl-C` to stop.
- Then use `speed: 0.5`, `steering_angle: 0.0`: the wheels must spin **forward**.
- **If either is reversed,** flip the sign of `steering_angle_to_servo_gain` or `speed_to_erpm_gain` in `vesc.yaml`. Do not drive autonomously until both are right.

**Steering limits (important).**

- `vesc.yaml` implies **left 0.57 rad but right only 0.27 rad** (servo_min/max with the current offset). That asymmetry would make every right-hander wide.
- Look at the wheels at full left and full right lock. Measure each angle with a protractor, or measure the circle radius in step 3.

---

## 2. Follow-the-gap on race_core (Plan B, about 15 min)

**Terminal 3:**

```
./race.sh reactive 0.8
```

**Terminal 4:**

```
./race.sh record ftg
```

1. Place the car on the track and hold RB. It drives follow-the-gap at 80% of the old Python planner's speeds.
2. Run 3 laps. Then `Ctrl-C` in race_core and relaunch with 1.0, then 1.2 if it stays clean.
3. In the race_core terminal, watch for `scan pipeline: p50 ... p99 ...`. It should be under 1 ms.

**Pass:** 3 clean laps at 1.0.

---

## 3. Calibration bags (about 15 min)

**Terminal 4:**

```
./race.sh record calib
```

Drive by hand (LB):

1. **Speed.** Do 3–4 straight runs **towards a flat wall**: start about 6 m away, use a steady 1–3 m/s, and stop before 1 m.
2. **Steering.** Drive steady circles at about 1.5 m/s, **both directions**, at half stick, three-quarter stick and full lock, about 2 laps each.

Then `Ctrl-C` the recorder and run:

```
./race.sh calib ~/bags/<the calib bag>
```

The report gives:

- The corrected `speed_to_erpm_gain`.
- The corrected `steering_angle_to_servo_gain` and `steering_angle_to_servo_offset`.
- **The maximum steering angle achieved on each side.**
- The actuation delay. Set it with `ros2 param set /race_core pp.delay <value>`, or edit `race_core.yaml`; the default 0.07 s was tuned in the simulator.

**The steering fit uses the speed calibration,** so do the wall runs first.

**Then:**

1. Put the new values into `~/airacer_basecode/blaze_racer/subsystem_ws/src/subsystem_stack/config/vesc.yaml`.
2. Restart bringup.
3. Use the **smaller** max angle in `./race.sh plan NAME <max_steer>`, and in `race_core.yaml` as `pp.max_steer_left` and `pp.max_steer_right`.

---

## 4. Map and raceline (about 10 min)

1. Put the car **on the start line, facing the race direction.** That pose becomes map (0, 0, 0) and the start pose for racing.
2. **Terminal 2:** restart `./race.sh ekf` so odometry starts at zero here.
3. **Terminal 3:**
   ```
   ./race.sh map
   ```
4. Drive 1–2 slow laps by hand (about 1 m/s), back to the start line.
   - slam_toolbox closes the loop.
   - The loop-closure saver writes `~/blaze_racer/maps/blaze_track*.{png,yaml}` automatically.
5. Save it under a name of your choice:
   ```
   ./race.sh savemap test1
   ./race.sh plan test1 0.40          # use the max steering angle from step 3
   ```
6. Look at `~/maps/test1_lines/raceline.png`:
   - Walls are black.
   - The raceline is coloured by speed (red slow, green fast).
   - The blue line is the centerline.
   - The green dot is s=0 (the start); the arrow shows the direction.
7. The `plan` output prints:
   - Lap-time estimates at 100% and 75%.
   - The minimum wall clearance (should be ≥ 0.33 m).
   - A WARNING if a corner is tighter than the car can steer.
8. **Optional, no car needed:**
   ```
   ./race.sh sim test1 0.5
   ```
   This runs the simulator on the new map for a minute. Watch for `LAP` lines and no `CRASH`.

---

## 5. Raceline laps (the main event)

1. Stop the mapping terminal (slam_toolbox must not run while racing).
2. Keep bringup and ekf running.
3. Put the car on the start line, facing the race direction.

**Terminal 3:**

```
./race.sh race test1 0.4
```

**Terminal 4:**

```
./race.sh record race40
```

**Before holding RB, the race_core status line (once per second) must show all of these:**

- `localized (match 0.9x ...)`
- The state `GB_TRACK`.
- `match` ≥ 0.65.
- `spread` < 0.1.

**If it stays `REACTIVE`:**

- Localization has not converged. Push the car a metre along the track by hand.
- If that does not help, run `./race.sh relocalize`. That is global localization, which works anywhere on the track and takes a few seconds.
- Or restart race_core with the car exactly on the start line.

**Then hold RB.**

**Speed ramp:** 0.4 → 0.5 → 0.6 → 0.7. Use `./race.sh scale 0.5` (live, no restart needed). Run at least 3 clean laps before each step.

After each run:

```
./race.sh report ~/bags/<bag> test1
```

This prints per-lap time, RMS and maximum lateral error, AEB count and REACTIVE share, and writes `lap_report.png` into the bag folder.

**Stop raising the speed when any of these happens:**

- Maximum lateral error exceeds about 0.15 m.
- An `EMERGENCY BRAKE` line appears.
- A near miss.

**If the car weaves on straights:** raise `pp.lookahead_time` (0.15 → 0.20) or `pp.delay` (0.07 → 0.10). If it cuts corners into the inside wall: lower `pp.lookahead_base` (0.25 → 0.15). All of these can be changed live, for example:

```
ros2 param set /race_core pp.lookahead_time 0.20
```

**One corner limits the speed?** `./race.sh report` lists the 1 m sectors with the largest lateral error, and the emergency-brake positions, by arc length s. Slow only that stretch:

```
ros2 param set /race_core sectors "[s_start, s_end, 0.85]"
```

You can list several triples in the same array; the change applies live.

**With a screen on the KR260, or a laptop on the same network:** `./race.sh rviz` shows the map, raceline, particles, pose and scan. Its 2D Pose Estimate tool re-seeds localization.

**Hand reset:** release RB, put the car down on the line facing the race direction, press A, then hold RB.

**Plan B on the spot:** `./race.sh race test1 0.4 centerline` follows the centerline instead (more wall clearance).

---

## 6. DPU detector on the car

```
./race.sh ekf color          # restart ekf with the RGB stream
./race.sh detector
```

- **Expected log:** `frame: total p50 ~9.5 ms (DPU p50 5.5 ms)`, latency about 50 ms.
- `ros2 topic echo /perception/camera_dets` gives `[latency_ms, n, x1, y1, x2, y2, score, bearing, ...]`.
- With race_core running in race mode and a second car ahead, race_core logs `opponent confirmed by the camera`. That means the DPU detection's bearing matched the LiDAR track; telemetry fields 24 and 25 carry it. This is a label only, and racing never depends on it.
- **Data for the next model:**
  - Record images with `./race.sh record cars images` while a second car (any RC car) drives in front at 0.5–5 m.
  - Also record some laps with no car in view.
  - Images cost CPU and disk, so keep these recordings short (1–2 min).
- **Overtaking (only with a second car, slow speeds first):** `ros2 param set /race_core overtake.mode spline`. race_core trails a car that is slower than 80% of our pace, then passes it on the side with more room. In simulation it made 0 collisions.
- **Measure the camera mount** (x, y, z from the rear axle; tilt). Pass it to `./race.sh ekf` later with `cam_x`, `cam_z`, `cam_pitch`. Until then the defaults (0.20, 0, 0.18) are used.

---

## 7. What to send back to Claude

- The terminal output of any failed step.
- The bags in `~/bags/`:
  - `calib`
  - `ftg`
  - `race*`
  - `cars` (images)
- The maps in `~/maps/`.
- The `./race.sh report` tables.

A copy can go to the RTX box with `rsync -a ~/bags ~/maps claude@100.105.87.85:~/from_car/`.

**Quick numbers to note:**

- Lap times at each speed scale.
- The highest clean scale.
- CPU from `htop` (race_core about 32% of one core expected).
- SoC temperature from `./race.sh check`.
