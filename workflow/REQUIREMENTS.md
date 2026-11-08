# Requirements

- **Version:** 0.3 DRAFT, written by Claude on Thursday 2026-10-29 (9 days to the race). **Not signed off.**
- **Process:** `WORKFLOW.md` Step 1. Claude drafts; a human signs off. After sign-off, changing a number needs a Decision Log entry in `RACE_PLAN.md` Appendix B.
- **Parents:** the success criteria G1–G7 in `RACE_PLAN.md` §1.3, and the gates M1–M3 in §8.2.
- **Evidence cut-off:** the last track results are from the first KR260 track session on **2026-10-26**; the servo was re-seated and its limits recorded on the stand on **2026-10-29** (R-ACT-01). Fill in anything run after that before signing off.

## How to read this file

- **IDs:** `R-<area>-<nn>`. Areas: CTL control and lap time, ACT actuators and calibration, SAF safety, LOC localization and state estimation, CPU compute, SEN sensors and rates, PER perception, PWR power, OPS operations and race day, RAC racecraft. Assumptions about unknown race rules are `A-<nn>`.
- **Status values** (`WORKFLOW.md` Step 1): `Open`, `SIL pass`, `HIL pass`, `Track pass`, `Fail`, `Waived`. Each status names its evidence and date. A SIL pass is not a track pass.
- **Where each number comes from:**
  - Plain numbers come from `RACE_PLAN.md`, `TEST_DAY.md` or the code's parameters, so the team has already seen them.
  - **†** marks a number Claude proposes here for the first time. **Check these first.**
- **SIL** means `race_sim` on the RTX box or KV260. **HIL** means the KR260 on the stand. **Track** means real laps.

## Review first (decisions for the reviewer)

1. **Steering failed on 10/26 and was fixed on the stand on 10/29 (R-ACT-01, R-RAC-03).** The 10/26 follow-the-gap lap needed 2 manual takeovers, both on right turns. The cause was the off-centre servo horn: the car steered 0.27 rad right and 0.57 rad left. On 10/29 the horn was re-seated at servo 0.50 and the absolute full-left and full-right servo values were recorded (0.05 and 1.0). The steering angles in radians are not measured yet, so nothing faster should run until full-lock circles confirm R-ACT-01. Please confirm the proposed per-side minimum of 0.40 rad.
2. **Odometry loss: stop or follow-the-gap? (R-SAF-04)** The `WORKFLOW.md` stand checklist says "kill the EKF → stop within `odom_timeout`". The code does something else: after `odom_timeout` (0.2 s), `race_core` leaves GB_TRACK for REACTIVE (follow-the-gap) and keeps driving. Pick one; the test follows the choice.
3. **R-CPU-01 is a known Fail.** It measured 7.6 ms p99 against a 5 ms target (KV260, 10/25), and it has not been measured on the KR260 yet. Options:
   - Fix it, for example by reducing particles or beams under load.
   - Waive it to ≤ 10 ms. The scan period is 25 ms, so 10 ms still leaves margin. A waiver needs a Decision Log entry.
4. **Which loop is "the test track"?** G1 and R-CTL-01/02 name blaze_track. The 10/26 duct-hose track (`test_day_1.png`) may be a different layout. Either name the loop that counts, or re-plan the targets on today's map.
5. **Gate M1 is overdue.** M1 (autonomous raceline laps) was due Tue 10/27. Per `RACE_PLAN.md` §8.2, if it is still missed on Thu 10/29 (today), the race strategy switches to Plan B (tuned follow-the-gap), and map-based work continues in parallel. That makes R-RAC-03 the requirement that matters most this week. Please confirm or overrule in the Decision Log.
6. **Two metric definitions need agreement:**
   - Steering jitter (R-CTL-05).
   - What counts as a "clean overtake" in G7 (R-RAC-01).
7. **Speed target (see the note below).** The proposed target is laps at 80–85% of the measured grip limit with zero crashes, not top speed. This rests on R-ACT-06.

## Note: grip, not top speed, sets our pace

The Traxxas Slash 4x4 is advertised at 22.35 m/s on 3S. That number does not apply to an F1Tenth track. The fast-lap requirements here (R-CTL-02, R-CTL-04, R-RAC-04) aim at the **grip limit** instead, for these reasons:

- **Corners cap the speed.** The fastest a car can take a corner is about √(μ·g·r), where μ is tire grip, g is gravity and r is the corner radius. In blaze_track's tightest corner (r ≈ 0.6 m) that is 1.7 m/s at μ 0.5, 2.0 m/s at μ 0.7 and 2.3 m/s at μ 0.9. No sensor or computer raises this; only grip and the line do.
- **The straights are too short.** At the traction limit (about 7 m/s²), getting from 2 to 22 m/s takes about 36 m, and braking back down takes another 36 m. The whole test loop is 21 m. A 10 m straight tops out at about 7–8.5 m/s. `make_raceline.py` caps speed at 7 m/s (`--v-max 7`), and the lap model hardly ever reaches it.
- **The sensors can't see far enough.** At 22 m/s the car would need about 36 m to stop, but the Hokuyo sees 10 m. The car would travel 0.56 m between scans, and more than 2 m during the ~0.1 s it takes to sense and steer.
- **What the payoff looks like.** Today's follow-the-gap laps take about 30 s (1–2 laps a heat). The raceline model gives 6.5–7.5 s at 75% of the grip limit (7–9 laps) and 5–6 s at 100% (10–11 laps). ForzaETH's laps got faster up to about 79% of the limit, and their car started hitting the walls above 80%.

The whole speed profile follows from μ: `make_raceline.py --mu` sets every corner speed, and `race_core`'s `speed_scale` is a fraction of that profile. `--mu` is still the default 0.6, a guess. That is why grip measurement is a requirement (R-ACT-06). On race day the floor will be different. If there is no time to measure grip there, plan with μ 0.6 or the measured test-floor value, whichever is lower, and let the speed ramp (`RACE_PLAN.md` §6.2) find the margin.

---

## 1. Control and lap time (CTL)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-CTL-01 | Pure pursuit RMS lateral error ≤ 0.08 m at 85% scaler on blaze_track | G1 | SIL `sim_eval.sh` at `speed_scale:=0.85`; track: `lap_report.py` | SIL pass (0.05 m, 10/25). Track: Open |
| R-CTL-02 | Lap time on the test loop at race scaler ≤ 8.5 s (≥ 7 laps in 60 s); stretch ≤ 7.5 s | G1 | SIL `sim_eval.sh`; track: `lap_report.py` | SIL pass (6.85 s at 85%, 10/25). Track: Open |
| R-CTL-03 | Maximum lateral error ≤ 0.15 m on every lap at the race scaler. This is the speed-ramp stop rule, made a requirement | G1, G2 | SIL `sim_eval.sh`; track: `lap_report.py` max column | SIL pass (0.11 m at 85%, 10/25). Track: Open |
| R-CTL-04 | On an unseen (race-day) track, lap time ≤ the lap-time model's prediction for that track ÷ 0.75 | G1, G4 | rehearsal track: `research/laptime_model.py` on the new map vs `lap_report.py` | Open |
| R-CTL-05 | Follow-the-gap on a straight does not weave: ≤ 0.2 steering reversals per second† (a reversal is a change of the steering command from > +0.10 rad to < −0.10 rad, or back†) | G2 | bag analysis of `/drive` while RB is held (add to `lap_report.py`) | **Fail** (≈ 0.6 per second, 10/26 bag `1026_223505_ftg`; that figure came from an ad-hoc script with a looser definition, so recompute it with this one) |
| R-CTL-06 | `pp.delay` within ± 0.02 s† of the measured actuation delay | G1 | `calib_report.py` on the circle bags | Open (0.07 s comes from the simulator) |

## 2. Actuators and calibration (ACT)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-ACT-01 | Measured steering lock ≥ 0.40 rad† on **each** side (turning radius ≤ 0.78 m at 0.33 m wheelbase), the two sides within 0.05 rad† of each other, and the servo does not stall (buzz) at either commanded limit | G1, G2 | stand: servo trim tool + protractor; track: full-lock circles with `calib_report.py` | Open (10/26 **Fail**: right 0.27 rad, left 0.57 rad with the servo stalled at full left; straight = servo 0.54. **10/29 stand, absolute maximums recorded:** horn re-seated with the servo held at 0.50; straight = **0.50**, full left = **0.05**, full right = **1.0** (the servo's absolute maximum: `vesc_driver` and the VESC cap the range at 0–1, and right lock is still short of the steering's hand stop); no buzz at either value. `vesc.yaml` now has offset 0.50, `servo_min` 0.05, `servo_max` 1.0. Full-lock wheel spin on the stand reached 6.97 m/s right and 6.96 m/s left at 7 m/s commanded, and 3.96 m/s right at 4 m/s. Angles with the nominal gain 0.8552: ≈0.58 rad right, ≈0.53 rad left, not measured. Still to do: full-lock circles or a protractor to measure the radians and set the gain) |
| R-ACT-02 | With steering 0, the car drifts ≤ 0.10 m† sideways over a 5 m run | G1 | track: straight run, tape measure or LiDAR | Open |
| R-ACT-03 | Sign conventions: + steering turns left, + speed drives forward | G2 | stand sign check (`TEST_DAY.md` §1) | HIL pass (10/26, KR260: left and forward) |
| R-ACT-04 | Measured speed within ± 5%† of the commanded speed from 1 to 3 m/s (`speed_to_erpm_gain` calibrated) | G1 | `calib_report.py` on wall-run bags (`TEST_DAY.md` §3) | Open |
| R-ACT-05 | From standstill, the car moves off without the stutter seen on 10/26: no more than 0.2 s† between the first non-zero speed command and steady acceleration, forward and reverse | G1 (launch), G7 | bag: `/commands/motor/speed` vs `/odom` speed; VESC sensorless start-up tuning | Open (operator note 10/26: about 0.5 s of jitter at every start. Hypothesis: sensorless BLDC start-up; not yet tested) |
| R-ACT-06 | Tire grip μ on the test floor measured to ± 0.05†, and the raceline is planned with `make_raceline.py --mu` no higher than the measured value. Method: steady circles at increasing speed until the car slides; μ ≈ peak (v × yaw rate) / g, using EKF speed and the gyro | G1, G2 | track: circle bags (`TEST_DAY.md` §3, `RACE_PLAN.md` D6 system-identification runs); add a μ estimate to `calib_report.py` | Open (`--mu` is the default 0.6, not measured; `calib_report.py` has no μ output yet; needs R-ACT-01 first) |

## 3. Safety (SAF)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-SAF-01 | The car stops within 0.6 s of deadman (RB) release | G2 | SIL `deadman_test.py`; stand: release RB, time the wheels on video | SIL pass (0.55 s, 10/25). HIL: Open (10/26 virtual pad: the command went to 0 on the next message; wheel stop not timed) |
| R-SAF-02 | The car stops within 1.0 s of `/joy` loss (`joy_timeout` 0.3 s) | G2 | SIL `deadman_test.py`; stand: unplug the dongle | SIL pass (0.8 s, 10/25). HIL: Open |
| R-SAF-03 | Without `/scan` for `scan_timeout` (0.15 s), the commanded speed goes to 0, and the car is stopped within 1.0 s† | G2 | SIL: sensor-loss scenario (to write); stand: unplug the LiDAR Ethernet | Open (the watchdog is in the code; no SIL scenario yet) |
| R-SAF-04 | Without `/odometry/filtered` for `odom_timeout` (0.2 s): **[reviewer decision 2]**, either (a) stop within 1.0 s, or (b) switch to REACTIVE within 0.25 s and keep driving on LiDAR only | G2 | SIL: sensor-loss scenario (to write); stand: kill the EKF | Open |
| R-SAF-05 | Holding LB (manual teleop) overrides autonomy within 0.1 s† (`ackermann_mux` priority 100 over 10) | G2 | stand: hold RB, then press LB and steer; check `/ackermann_cmd` in a bag | Open (used twice on the track on 10/26 and it worked; latency not measured) |
| R-SAF-06 | Emergency braking: 0 collisions with a static box placed in the path at speeds up to 2 m/s†, and 0 false brakes per clean lap at the race scaler | G2 | SIL `sim_eval.sh` (AEB count); track: box test + `lap_report.py` AEB positions | SIL pass for false brakes (0 at 85%, 10/25). Box test: Open |
| R-SAF-07 | Every safety-path change (deadman, timeouts, emergency braking, stuck recovery) has a SIL test that fails without it | G2 | code review checklist (`WORKFLOW.md` Step 4) | Open (process; checked on each review) |

## 4. Localization and state estimation (LOC)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-LOC-01 | Localization p95 error ≤ 0.08 m at race speed | G1, G2 | SIL `loc_error.py`; track: PF health in telemetry | SIL pass (6.6 cm at 85% with scan skew, 10/25). Track: Open |
| R-LOC-02 | When localization is lost (score < 0.40 for 8 scans, about 0.2 s), race_core switches to REACTIVE in the same scan, and returns to GB_TRACK only after 12 good scans | G2, G3 | SIL: kidnap / hand-reset scenario; telemetry state field | Open (logic in the code; no dedicated test) |
| R-LOC-03 | EKF yaw rate positive for a counter-clockwise turn; at rest, absolute value ≤ 0.005 rad/s | G1 | stand: yaw sign check (`TEST_DAY.md` §1); rest bias from a 10 s bag | HIL pass (sign +0.73 rad/s CCW on the KR260 10/26; bias −0.0013 rad/s on the KV260 10/25, to repeat on the KR260) |
| R-LOC-04 | Camera mount pose in the EKF measured to ± 1 cm† and ± 2°† (currently placeholders 0.20, 0, 0.18 m) | G1 | tape measure; recorded in `ICD.md` §2.2 | Open |

## 5. Compute (CPU)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-CPU-01 | `race_core` loop p99 ≤ 5 ms with the full car load | G6 | `race_core` "scan pipeline" log line on the stand with everything running | **Fail** (7.6 ms, KV260, 10/25). KR260: not measured. **[reviewer decision 3]** |
| R-CPU-02 | Total CPU ≤ 60% of 4 cores with the full race load | G6, M2 | `tools/cpu_snapshot.sh` on the KR260 | Open (KV260 without drivers 21%, 10/25. KR260 10/26: load average ≈ 4 on 4 cores, which is not a CPU % but suggests little headroom: measure) |
| R-CPU-03 | Follow-the-gap scan pipeline p99 ≤ 1 ms | G6 | `race_core` log line in reactive mode | SIL pass (0.2 ms bag replay, 10/24). HIL: Open |
| R-CPU-04 | No Python node subscribes to `/scan` on the car during laps; no image topics recorded during laps | G6 | `ros2 node list` + bag topic list at each run | Open (rule from `RACE_PLAN.md` §5.3; add to `race.sh check`) |

## 6. Sensors and rates (SEN)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-SEN-01 | Under the full race load: `/scan` ≥ 39 Hz, IMU ≥ 195 Hz, `/odometry/filtered` ≥ 95 Hz, `/drive` ≥ 39 Hz, `/odom` ≥ 48 Hz† | G2, G6 | short C++ `ros2 bag record` on the car + bag metadata counts; `./race.sh check` | HIL pass (10/26: IMU 200, EKF 95–100, `/drive` 40 with a fake scan; real `/scan` 39.9 Hz on the car) |
| R-SEN-02 | After a cold power-up, `./race.sh check` passes with no manual steps (joystick, LiDAR link, camera, VESC, DPU) | G2, G4 | stand: power-cycle, run the check | Open (10/26: the pad worked only after it was moved to the fourth USB port; find out why) |

## 7. Perception (PER)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-PER-01 | Opponent recall ≥ 95% within 6 m ahead; false positives ≤ 2% | G5 | labelled D435i bags of a real second car | Open (public data INT8 AP50 0.972; 0 of 269 held-out July background frames fired, 10/25. Own car frames: none yet) |
| R-PER-02 | DPU inference ≤ 10 ms per frame | G5 | `dpu_detector` timing log on the KR260 | HIL pass (5.6 ms, KR260, 10/26) |
| R-PER-03 | Camera to `/perception/camera_dets` latency ≤ 50 ms; ≥ 20 Hz† under the full race load | G5 | `dpu_detector` latency log; bag rate | HIL pass (≈ 50 ms; 22–24 Hz under load, 10/26). Latency is at the limit |
| R-PER-04 | Racing never depends on the camera: with `dpu_detector` stopped, laps continue with no change in lap time beyond ± 2%† | G2, G5 | SIL: run with and without detections; stand: kill the detector | Open (fusion only labels tracks, by design; not tested) |

## 8. Power (PWR)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-PWR-01 | No KR260 brownout or reboot at full throttle with the battery at 11.3 V† or lower | G2 | stand: full-throttle bursts at low battery, check `uptime` and the kernel log | Open |
| R-PWR-02 | Runs start at ≥ 12.2 V† and stop at 11.1 V (3.7 V per cell); start and end voltage logged for every run | G2 | run sheet (`logs/RUNS.md`) | Open (10/26 started at 12.7 V; not yet logged per run) |

## 9. Operations and race day (OPS)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-OPS-01 | New track to race-ready (map → raceline → validation lap) in ≤ 15 min, rehearsed twice; ≤ 20 min at rehearsal #1 (M2) | G4, M2 | stopwatch at rehearsals (`RACE_PLAN.md` §6.2) | SIL pass (pipeline in simulation, 10/25; planner 16 s on the KR260, 10/26). Track: Open |
| R-OPS-02 | Hand-reset recovery: back at raceline pace ≤ 3 s after the car is placed and RESET (A) is pressed | G3 | SIL hand-reset scenario; drill (`RACE_PLAN.md` §6.4) timed on video | Open |
| R-OPS-03 | Every run has a log row and a bag (`WORKFLOW.md` Step 7) | G2 | `logs/RUNS.md` | Open (`logs/RUNS.md` not created yet; the 10/26 bag exists) |

## 10. Racecraft and reliability (RAC)

| ID | Requirement | Parent | Verified by | Status |
|---|---|---|---|---|
| R-RAC-01 | Against an opponent at ≤ 80% of our pace, ≥ 80% of overtake attempts end in a clean pass (definition to agree: no contact, and no emergency brake?) | G7 | SIL opponent scenarios at 25/40/60%; track with a second car | SIL partial (40% opponent: 10 attempts, 5 passes, 5 clean aborts, 0 collisions, 10/25). Track: Open. `overtake.mode` stays `none` until then |
| R-RAC-02 | No overtake attempt against an opponent above 80% of our pace; 0 contacts in 10 trailing runs | G7 | SIL 60% opponent; track with a second car | SIL pass (60% opponent: 0 attempts, trails, 10/25). Track: Open |
| R-RAC-03 | Plan B: 3 clean laps (no takeover, no contact) in follow-the-gap at `reactive 1.0` on the test track | G2, M1 fallback | track, `TEST_DAY.md` §2 | **Fail** (10/26 at `reactive 0.5`: 2 takeovers on right turns; root cause R-ACT-01) |
| R-RAC-04 | ≥ 10 consecutive laps at ≥ 75% scaler with no crash (M2); ≥ 20 at the race scaler (G2) | G2, M2 | run sheet + bags | Open |

---

## Assumptions about race rules we do not know yet

From `RACE_PLAN.md` §12. Each one names the requirements to revisit when the organizers answer.

| ID | Assumption | If wrong, revisit |
|---|---|---|
| A-01 | We can drive solo laps for mapping during practice, for at least 15 min | R-OPS-01, Plan B choice |
| A-02 | Hand resets put the car back where it crashed, with no time penalty | R-OPS-02, R-RAC-01 |
| A-03 | An operator may hold a deadman button and press RESET during a heat | R-SAF-01, R-SAF-02, R-OPS-02 |
| A-04 | Parameters may be changed between heats | race-day tuning (`RACE_PLAN.md` §6.5) |
| A-05 | Barriers are visible to the LiDAR (≥ 10 cm high at the scan plane) and the track is ≥ 1.0 m wide | R-CTL-04, R-RAC-01 |
| A-06 | Contact is not penalized, but we treat any contact as a likely crash and avoid it | R-RAC-01, R-RAC-02 |
| A-07 | Wi-Fi may be missing at the venue; the car runs with no network | R-SEN-02 |
| A-08 | There is time to swap batteries between heats | R-PWR-02 |
| A-09 | The first start-line crossing is not a factor; laps decide the heat | R-ACT-05 (launch) priority |

## Sign-off

| Section | Reviewer | Date | Notes |
|---|---|---|---|
| Review-first decisions 1–6 | | | |
| 1 CTL | | | |
| 2 ACT | | | |
| 3 SAF | | | |
| 4 LOC | | | |
| 5 CPU | | | |
| 6 SEN | | | |
| 7 PER | | | |
| 8 PWR | | | |
| 9 OPS | | | |
| 10 RAC | | | |
| Assumptions | | | |

## Change log

| Date | Change | By |
|---|---|---|
| 2026-10-29 | v0.1 draft from `WORKFLOW.md` Step 1, `RACE_PLAN.md` §1.3/§12/Appendix C, `TEST_DAY.md`, `KR260_STATUS.md`, `race_core.yaml` and the 10/26 track notes | Claude |
| 2026-10-29 | v0.2: added R-ACT-06 (grip μ measured, raceline planned with it), the note "grip, not top speed, sets our pace", and review decision 7 | Claude |
| 2026-10-29 | v0.3: R-ACT-01 Fail → Open. The servo horn was re-seated, and the absolute full-left (0.05) and full-right (1.0) servo values were recorded on the stand on 10/29 and saved in `vesc.yaml`; the steering angles in radians are still to be measured. Updated review decision 1 and the evidence cut-off | Claude |
