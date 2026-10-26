# Engineering Workflow: Requirements to Race Day

- **Version:** 1.0, written Monday 2026-10-26 (Day 2, 12 days to the race)
- **Applies to:** everything in `~/2x1_w` (race_core, dpu_detector, race_sim, tools, setup) running on the KR260 in the car
- **Companion documents:** `RACE_PLAN.md` (the goal, gates, Decision Log), `TEST_DAY.md` (the track runbook)

This document says *how* we work. `RACE_PLAN.md` says *what* we are building and *why*. The loop:

```
 ┌──────────────────────────────────────────────────────────────────────────────┐
 │                                                                              │
 ▼                                                                              │
(1) Requirements ─► (2) ICD ─► (3) Bring-up & drive ─► (4) Control logic         │
                                                         AI-written, human-reviewed
                                                              │                 │
                                                              ▼                 │
(8) Data review ◄── (7) Run procedure & log ◄── (6) HIL on the stand ◄── (5) SIL in CI
 AI-written, human-reviewed                                 AI-written, human-reviewed
 │                                                                              │
 └──────────────────────────► (9) Iterate: back to (1) ─────────────────────────┘
```

**Who does what.** The team is small, has no FPGA specialist, and Claude writes most of the code. So the split is:

| | Claude (AI) | Human (team) |
|---|---|---|
| Writes | requirement drafts, ICD drafts, control code, SIL tests, data-review reports | requirement sign-off, run notes, ICD corrections from the real hardware |
| Runs | SIL, bag replay, offline analysis | the car, the stand, the track, the joystick deadman |
| Decides | nothing safety-critical alone | merges, scaler steps, go/no-go at each gate |

**The one rule.** Nothing goes on the car unless it has passed SIL and a human has reviewed it. Nothing goes faster on the car unless the previous speed passed HIL and the run was reviewed.

---

## Step 1. Requirements

**Purpose:** state what "good" means as numbers we can test, so every later step has a pass/fail line.

**Where they live:** `RACE_PLAN.md` §1.3 (G1–G7 success criteria) is the top level. Derived requirements go in a new table, `REQUIREMENTS.md` (to create), one row each:

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-CTL-01 | Pure pursuit RMS lateral error ≤ 0.08 m at 85% scaler on blaze_track | G1 | SIL `sim_eval.sh` scenario `blaze_85`; track run | SIL pass (0.05 m, 10/25) |
| R-SAF-01 | Car stops within 0.6 s of deadman (RB) release | G2 | `deadman_test.py` in SIL; stand test | SIL pass (0.55 s) |
| R-SAF-02 | Car stops within 1.0 s of `/joy` loss | G2 | `deadman_test.py`; stand test (unplug dongle) | SIL pass (0.8 s) |
| R-CPU-01 | `race_core` loop p99 ≤ 5 ms with full car load | G6 | `cpu_snapshot.sh` on HIL | **Fail (7.6 ms, 10/25)** |
| R-LOC-01 | Localization p95 error ≤ 0.08 m at race speed | G1, G2 | `loc_error.py` in SIL; PF health on track | SIL pass (6.6 cm) |
| R-PER-01 | Opponent recall ≥ 95% within 6 m, FP ≤ 2% | G5 | labelled D435i bags | Open (public data only) |

**Rules:**

- Every requirement has an ID, a number, a parent goal (G1–G7) and a named verification method. "Works well" is not a requirement.
- Keep the *verified by* column honest: "SIL pass" is not "track pass". Use the status values `Open`, `SIL pass`, `HIL pass`, `Track pass`, `Fail`, `Waived` (with a Decision Log entry).
- Claude drafts requirements from the plan and from data reviews (Step 8); a human signs them off. Changing a requirement's number is a Decision Log entry in `RACE_PLAN.md` Appendix B.
- Race rules we do not know yet (contact rules, tie-breaks; `RACE_PLAN.md` §12) are recorded as assumptions with an ID, so we can find what to revisit when the organizers answer.

**Exit criterion:** every requirement touched in this iteration has an ID, a number and a verification method.

---

## Step 2. Interface Control Document (ICD)

**Purpose:** pin down every boundary between parts (topics, frames, units, rates, wires, parameters) before code depends on it. Most of our real bugs so far were interface bugs: the EKF read the D435i gyro in the camera optical frame, so "yaw rate" was actually roll rate (fixed 10/25); `ekf.yaml` mixed int and float in one array.

**Where it lives:** `ICD.md` (to create). Sections and what to put in each:

### 2.1 ROS interfaces

One row per topic: name, type, publisher, subscribers, rate, frame, QoS, units and sign convention. Starting point (from `race_core` today):

| Topic | Type | Publisher → Subscriber | Rate | Notes |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `urg_node` → `race_core` | 40 Hz | stamp = first beam; `race_core` deskews to mid-scan |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF → `race_core` | 100 Hz | VESC speed + D435i gyro; odom→base_link |
| `/joy` | `sensor_msgs/Joy` | `joy` → `race_core` | event | RB = deadman, A = reset; timeout `joy_timeout` |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | `race_core` → `ackermann_mux` | 100 Hz | steering rad, + = left; speed m/s |
| `/perception/camera_dets` | `std_msgs/Float32MultiArray` | `dpu_detector` → `race_core` | 30 Hz | layout must be written down field by field |
| `/race_core/telemetry` | `std_msgs/Float32MultiArray` | `race_core` → bag | 40 Hz | fields 0–25; index table required (24–25 = camera confirmation) |

**Rule:** a `Float32MultiArray` is only allowed if the ICD has its index table. Adding a field means updating the ICD in the same change.

### 2.2 Frames and transforms

`map → odom` (race_core PF), `odom → base_link` (EKF), `base_link → laser` (`laser_x/y/yaw` parameters), `base_link → camera_link → camera_imu_optical_frame` (measured mount pose; still a placeholder). Record who publishes each transform and how it was measured.

### 2.3 Electrical and network

- Battery 3S LiPo (11.1 V nominal) → regulated 12 V → KR260. Record the brownout voltage that we measured, not the one we assume.
- Hokuyo UST-10LX on a PS-side Ethernet port, static IP (as in `setup_kr260.sh`).
- VESC (autoro aesc v6.7) over USB; D435i on USB 3. Record the `/dev` names and udev rules.
- Tailscale/Wi-Fi addresses for the KR260, KV260 and RTX box.

### 2.4 Actuator calibration

The `vesc.yaml` mapping (servo offset 0.5396, gain −0.8552, speed-to-ERPM gain) and the **measured** steering limits per side. The current file implies left 0.574 rad, right 0.268 rad, which is asymmetric and must be measured with full-lock circles (`calib_report.py`). The measured actuation delay (current `pp.delay` 0.07 s comes from the simulator).

### 2.5 Files and models

- Map: `*.yaml` + `*.pgm` (slam_toolbox output).
- Raceline CSV columns: `x, y, s, d_left, d_right, psi, kappa, v, a` (from `make_raceline.py`).
- DPU model: `models/*.xmodel`, input 320×320 letterbox, 6 raw outputs, decode and NMS on the CPU; DPU fingerprint must match `xdputil query`.
- Parameter file `race_core/config/race_core.yaml`: each parameter's unit and safe range.

**Rules:**

- Claude drafts the ICD from the code (`grep create_publisher/create_subscription/declare_parameter`). A human corrects it from the real hardware (cable labels, measured mount poses, measured limits).
- A change that crosses an interface updates the ICD **in the same commit**. Reviewers reject code that changes an interface without it.
- SIL tests check the ICD where they can (message rates, telemetry length, frame IDs).

**Exit criterion:** every interface the iteration touches is in the ICD, with units and sign conventions.

---

## Step 3. Bring-up and drive

**Purpose:** prove that each piece of hardware works on its own, and that a person can drive the car by joystick, before any autonomy runs on it. This is the step the other session is doing now on the KR260.

**Procedure:** `TEST_DAY.md` §0–1 and `setup/setup_kr260.sh` / `setup/migrate_to_kr260.sh`. Bring up one layer at a time, and do not start the next layer until the current one passes:

1. **Power and boot:** boot firmware version, Ubuntu 22.04, stable on the regulated 12 V supply with the motor loaded.
2. **Compute:** ROS 2 Humble, workspaces built (`race_ws`, blaze_racer `subsystem_ws`), DPU overlay loaded at boot (`dpu-overlay.service`), `xdputil query` fingerprint matches the model.
3. **Sensors, one at a time:** `/scan` at 40 Hz with the right IP; D435i color 30 Hz and IMU 200 Hz; gyro bias at rest is in the ICD range (0.001–0.005 rad/s).
4. **Actuators, wheels off the ground:** servo sweeps left/right with the correct sign; motor spins forward for a positive speed command; e-stop and deadman cut the motor.
5. **Manual drive on the floor:** joystick teleop through `ackermann_mux`; record a short bag.

**Record for each item:** pass/fail, the measured value, and any ICD correction. The check is scripted where possible (`./race.sh check`), so it can be run again after every reboot or cable change.

**Exit criterion:** every item passes, the ICD has the measured values, and a teleop bag exists.

---

## Step 4. Control logic (AI-generated, human-reviewed)

**Purpose:** write the code that makes the car race: `race_core` (localization, Frenet tracking, opponent tracking, state machine, overtake, pure pursuit, emergency braking, follow-the-gap), `dpu_detector`, and the offline tools.

**How Claude writes it:**

- Start from a requirement ID and the ICD. The commit or change note names the requirement it serves ("R-CTL-01: add speed preview to pure pursuit").
- Write the test together with the code (Step 5), not after.
- Keep the architecture rules from `RACE_PLAN.md` §5: one C++ process, no Python on `/scan` in the car, `race_core` pinned to cores 2–3 with `SCHED_FIFO`, everything new behind a parameter that defaults to the safe behaviour (example: `overtake.mode` stays off until it has been tried with a real second car).
- Never change a safety path (deadman, scan/odometry timeouts, emergency braking, stuck recovery) without a matching SIL test.

**What the human reviewer checks** (10–15 minutes per change; ask Claude to explain any line):

1. **Safety paths still hold:** on lost scan, lost odometry, lost `/joy` or released deadman, the commanded speed goes to 0. Nothing in the change can skip that.
2. **Units and signs** match the ICD (rad vs deg, + = left, m/s vs ERPM).
3. **New parameters** have a unit, a safe default, and a range in the ICD.
4. **Failure behaviour:** what happens on NaN, an empty scan, a lost localization, or an empty raceline.
5. **Compute:** no new allocation or logging in the 40 Hz path; the CPU number in the SIL report did not jump.
6. **Tests:** the test fails without the change and passes with it.

**Version control.** `~/2x1_w` is not a git repository yet (the code is mirrored to `kriaRacer`, branch `kria-race`). For this workflow to work it should be: one branch per change, review on the diff, merge only after the SIL job is green. Until then, record each change in `RACE_PLAN.md` Appendix C with its evidence.

**Exit criterion:** the code builds, a human reviewed the diff against the checklist above, and the change names its requirement.

---

## Step 5. Simulation tests (SIL) in Continuous Integration (AI-generated, human-reviewed)

**Purpose:** catch logic, regression and performance bugs in seconds on a computer, instead of in crashes on the track.

**What we already have:**

| Test | Tool | Checks |
|---|---|---|
| Unit tests (18 gtests) | `race_core/test/test_*.cpp` | follow-the-gap, localization, overtake planner |
| Follow-the-gap parity | `tools/ftg_parity.py` | C++ matches the old Python planner on 3,899 recorded scans |
| Closed-loop scenarios | `race_sim` + `tools/sim_eval.sh` | lap time, crashes, emergency brakes, lateral error, CPU, loop p99 |
| Deadman | `tools/deadman_test.py` | stop times after RB release and `/joy` loss |
| Localization accuracy | `tools/loc_error.py` | PF error vs simulator truth |
| Detector | `dpu_detector` offline CLI on a fixed image set | bit-identical to the Python reference; TP/FP/missed |

**What CI should run on every change** (to set up; the RTX box is the natural runner, with the KV260 as a nightly runner so timings are on the real K26 CPU):

1. Build `race_ws` with warnings as errors.
2. Run all gtests.
3. Run `ftg_parity.py` and the detector reference check.
4. Run the scenario matrix with `sim_eval.sh`, each scenario with a pass line taken from `REQUIREMENTS.md`:
   - blaze_track at 50%, 70%, 85%, 95% scaler: laps within the model's prediction, 0 crashes, RMS lateral error under the limit
   - scan skew on and off
   - opponent at 25%, 40%, 60% (trail, pass, or no attempt, each with 0 collisions)
   - hand reset mid-lap (the car recovers within G3's 3 s)
   - sensor loss: drop `/scan`, `/odometry/filtered`, `/joy` (the car must stop)
   - race-day pipeline: follow-the-gap lap → slam_toolbox → `make_raceline.py` → race (G4, timed)
5. Produce one summary file per run: pass/fail per requirement ID, plus lap times, CPU and p99 compared with the last green run.

**Rules:**

- A red CI job blocks the change from going on the car. No exceptions on race week; if a test is wrong, fix the test in a reviewed change.
- Claude writes the tests and the scenarios; the human reviews *what the pass line is*, because a test with a loose limit is worse than no test.
- When a bug is found on the stand or the track, the first fix is a SIL test that reproduces it. Then fix the code.
- Know the simulator's limits: `race_sim` models a bicycle, a friction limit, actuator lag and delay, but not tire behaviour on carpet, battery sag, or real LiDAR artifacts. A SIL pass says "the logic is right", not "it is fast enough on the real car".

**Exit criterion:** the CI summary is green and a human has read it.

---

## Step 6. Hardware in the loop (HIL) on the test stand

**Purpose:** run the real code on the real KR260 with the real drivers, with the wheels off the ground, so that timing, wiring, CPU load and failure behaviour are tested without risking the car.

**Two HIL setups:**

1. **Car on the stand (wheels off the ground).** The full race stack runs on the KR260: urg_node, VESC driver, realsense, EKF, `dpu_detector`, `race_core`. The LiDAR sees the room, so it can be run in follow-the-gap mode or with a stand map.
2. **Simulator feeding the real board.** `race_sim` on the RTX box publishes `/scan` and odometry over the network; `race_core` runs on the KR260 and its `/drive` goes back to the simulator (`RACE_PLAN.md` §9). This tests the real CPU and scheduling with closed-loop driving.

**Stand checklist (each one pass/fail, recorded):**

- [ ] Deadman: release RB → wheels stop within R-SAF-01.
- [ ] Unplug the joystick dongle → stop within R-SAF-02.
- [ ] Unplug the LiDAR Ethernet → stop within `scan_timeout`.
- [ ] Kill the EKF → stop within `odom_timeout`.
- [ ] Steering sign and limits match the ICD; servo does not hit the mechanical stop.
- [ ] Wheel speed (ERPM → m/s) matches the commanded speed.
- [ ] CPU and loop p99 with the whole stack running (`tools/cpu_snapshot.sh`): G6, R-CPU-01.
- [ ] No brownout or reboot at full throttle on the stand, measured at a low battery.
- [ ] Reboot test: power-cycle, then `./race.sh check` passes with no manual steps.

**Rule:** the car goes on the track only after the stand checklist for the current build passes. A new build means a new stand check of the safety items (5 minutes).

**Exit criterion:** every checklist item passes for this build, recorded in the run log.

---

## Step 7. Run procedure and log

**Purpose:** run the car on the track the same way every time, so results can be compared and nothing safety-critical is forgotten.

**Procedure:** `TEST_DAY.md` is the runbook (calibration bags → map → raceline → raceline laps → DPU on the car). `RACE_PLAN.md` §6 is the race-day version. Speed goes up in steps (40% → 50% → … → 85%) with ≥ 3 clean laps at each step before the next one (`RACE_PLAN.md` §9).

**Every run gets a log entry** (one row in a run sheet, for example `logs/RUNS.md`):

| Field | Example |
|---|---|
| Run ID | `2026-10-27-r04` |
| Build | git hash / bundle date of `race_ws` |
| Parameters changed | `speed_scale 0.70`, `pp.delay 0.07` |
| Track, map, raceline file | blaze_track, `maps/blaze_track.yaml`, `raceline.csv` |
| Battery voltage at start / end | 12.4 V / 11.6 V |
| Goal of the run | "3 clean laps at 70%" |
| Bag path | `logs/bag_2026_10_27_r04` (MCAP, no images during laps) |
| Result | 5 laps, 8.7 s best, 0 resets |
| Operator notes | "slid at the hairpin exit on lap 3" |

**Rules:**

- **Always record a bag.** A run without a bag did not happen, as far as data review is concerned. Topics: `/scan`, `/odometry/filtered`, `/drive`, `/joy`, `/race_core/telemetry`, `/race_core/pose`, `/perception/camera_dets`, `/tf`. No images during laps (CPU); a separate short image bag for detector checks.
- Write operator notes during or right after the run, not at the end of the day.
- One change per run where possible, so the review can tell what caused the difference.
- The deadman is held by a person at all times. Abort on anything unexpected, and note it.

**Exit criterion:** every run has a log row and a bag.

---

## Step 8. Data review (AI-generated, human-reviewed)

**Purpose:** turn bags into decisions: did we meet the requirements, what broke, and what to change next.

**How Claude produces the review:**

- Run the analysis tools on the bags:
  - `lap_report.py`: lap table, trajectory plot, worst 1 m sectors, emergency-brake positions by arc length
  - `calib_report.py`: speed gain, steering gain/offset, per-side maximum steering, actuation delay
  - `loc_error.py` and PF health from telemetry: effective sample size, scan-match score, relocalizations
  - CPU and loop p99, detection counts
- Write a short report per session (`logs/REVIEW_<date>.md`):
  1. Requirement status table: each requirement touched, measured value, pass/fail, and the change from the last session.
  2. Anomalies: every crash, reset, emergency brake and operator note, with the time in the bag and a likely cause.
  3. **Sim vs real gap:** the same scenario's SIL numbers next to the track numbers (lap time, lateral error, delay). Where they differ, propose a simulator update so SIL stays useful.
  4. Proposed changes, each linked to a requirement and ranked by expected laps gained or crashes avoided.

**What the human checks:**

- The numbers come from the bags, not from memory. Spot-check one or two plots against what was seen on the track.
- Every "likely cause" is labelled as a hypothesis until a test confirms it.
- Agree on the change list and its order. Anything that changes the plan, a gate or a requirement goes into the `RACE_PLAN.md` Decision Log.

**Exit criterion:** the review is written, a human agreed with it, and the change list is in priority order.

---

## Step 9. Iterate (back to Step 1)

The review's change list feeds the next loop:

- **New or changed requirement** (a number turned out to be wrong, or a new failure mode): update `REQUIREMENTS.md` and the Decision Log. → Step 1
- **Interface surprise** (a sign, a unit, a mount pose, a delay): update the ICD. → Step 2
- **Hardware fault** (cable, brownout, sensor dropout): fix and repeat the affected bring-up item. → Step 3
- **Logic bug or tuning:** write a SIL test that reproduces it, then fix. → Steps 4–5
- **Simulator too optimistic:** update `race_sim` from the measured data (delay, friction, steering limits) so the next SIL run is closer to reality. → Step 5

A small fix does not need to repeat every step in full, but it never skips SIL (5) or the stand safety checks (6).

---

## Cadence for the remaining 12 days

The loop has to run fast. Target: **one full loop per track day**, with Steps 4–5 overnight.

| When | Steps | Notes |
|---|---|---|
| Evening, after the track | 8 → 9 → 1–2 | Claude produces the review from the day's bags; human agrees on the change list (30 min) |
| Overnight | 4 → 5 | Claude writes changes and tests; CI runs; a summary is ready in the morning |
| Morning, at the bench | review → 6 | Human reviews the diffs and the CI summary (30–45 min), then the stand checklist |
| Track session | 7 | Runbook, run sheet, bags |

**Gates still decide** (`RACE_PLAN.md` §8.2): M1 Tue 10/27 (autonomous raceline laps), M2 Sun 11/1 (KR260 in the car, 10 consecutive laps at ≥ 75%, race-day pipeline ≤ 20 min), M3 Thu 11/5 (code freeze). After M3 the loop shrinks to Steps 6–8 with parameter-only changes: no new logic goes on the car after the freeze.

---

## Immediate setup work for this workflow

Small tasks that make the loop above real, in order:

1. `git init` `~/2x1_w` (or adopt the `kriaRacer` `kria-race` branch as the single source) so that changes, reviews and CI have a hash to point at.
2. Create `REQUIREMENTS.md` from `RACE_PLAN.md` §1.3 and the current evidence in Appendix C.
3. Create `ICD.md` from the code's topics and parameters, then have the team fill in the measured values from the KR260 bring-up (the other session's results).
4. Turn `sim_eval.sh` into one CI script that prints pass/fail per requirement ID; run it on the RTX box per change and on the KV260 nightly.
5. Start `logs/RUNS.md` (run sheet) and a `logs/REVIEW_<date>.md` template.
