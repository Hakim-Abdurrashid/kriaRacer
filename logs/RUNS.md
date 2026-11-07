# Run sheet

One row per run (`WORKFLOW.md` Step 7, R-OPS-03). Bags are on the KR260 in `~/bags`; race_core/driver logs are in
`~/race_logs` (copied to `logs/1106_night/race_logs` on the RTX box). Lap times come from the gyro heading
(`tools/yaw_laps.py`, race_core "TURN LAP" lines) unless noted.

## 2026-11-06 night (test track, orange duct hose, lane 0.58-1.25 m; race on 11/07)

Stack: blaze_racer iv2026 drivers + servo calibration (0.50 / -0.8552 / 0.05-1.0) via `race_core/launch/bringup.launch.py`,
EKF (D435i gyro), race_core. FTG = iv2026 planner (C++ port, team params.yaml) unless noted.

| # | Time (EST) | Mode | Settings | Result | Bag |
|---|---|---|---|---|---|
| 1 | 21:53 | FTG, CCW | iv2026 params as is (max_steer 0.349), x1.0 | laps 5.01 / 4.60 / 4.61 s, 12 m, 2.5 m/s mean, 3.0 max. Hit walls in the 0.81 m turns (5 AEB events at TTC < 0.1 s): 0.349 rad cap = 0.91 m min radius | `1107_025334_ftg_full` |
| 2 | 22:16 | mapping, CCW | FTG x0.5, max_steer 0.50, slam_toolbox on EKF | user: "went narrow on turns and did great"; 0 AEB. Map `~/maps/track1106` (12.1 m centreline) | `1107_031602_map` |
| 3 | 22:34 | race mode, CCW | raceline (margin 0.28, max_steer 0.45), speed_scale 0.5 | followed the line but hit walls; 15+ AEB (foreign points 0.16-0.3 m off the map = localization error), best clean lap 6.88 s; PF match 0.6-0.8 while driving | `1107_032349_race050` |
| 4 | 22:44 | FTG, CCW | iv2026 + max_steer 0.50, x1.0 (PF only for timing; lost at speed) | **14 laps 4.75-4.91 s, 12.0 m, 2.45 m/s, 0 AEB, 0 stops**. User: "by far the best run", 25 in turns at full left | `1107_034100_ftg_race` |
| 5 | 23:06 | FTG, CW | same as 4, plain reactive (no PF) | **13 laps 5.65-6.14 s, 17.0-17.5 m (layout changed?), 2.85-3.00 m/s, 0 AEB**; 3 more laps 6.6 s (18.4 m) at 11.7 V. Full right lock 7% of the time. User: "the most amazing clockwise run", 23 in full-right turn flawless. **Reference run** | `1107_035829_ftg_cw` |
| 6 | 23:45 | FTG, CW then CCW (layout ~18 m) | run 5 + top speed 5.0 m/s at 4.0 m clearance (was 4.5 at 5.0) | CW 4 laps 5.81-6.32 s, **3.08-3.21 m/s**; CCW 6 laps 6.02-6.07 s, 2.97 m/s; 1 AEB (0.11 s). User: "awesome run, great turns on full right and full left". **Race-day default (race.sh FTG_SET)** | `1107_044523_ftg_run6` |
| 7 | 00:21 | FTG x1.1, CW then CCW (oval) | run 6 x1.1 (corners 2.2, top 5.5 m/s) | CW 10 laps 3.30-3.33 m/s; CCW 7 laps 4.96-5.09 s, **3.40 m/s**. User: "clean". AEB about once per lap at one apex (TTC 0-0.04 s, no contact seen) | `1107_050227_ceiling` |
| 8 | 00:40 | FTG x1.2, tight figure 8 | run 6 x1.2, reactive AEB off | stints 3.44-3.80 m/s, top 5.7 m/s, good on straights without slipping, **hit the wall a few times**: ceiling is between x1.1 and x1.2 on narrow tracks | `1107_050227_ceiling` |

Race-day configuration (tested cold-start 11/07 01:5x): `./race.sh up` (add `det` for the DPU detector) =
iv2026 drivers + EKF + joystick watchdog + race_core follow-the-gap with run 6 settings at x1.0; `./race.sh rscale 1.1`
for more speed, `./race.sh ftgset KEY VALUE` for single parameters, `./race.sh down` to stop. Laps print as
"TURN LAP" lines in the core log (not on figure 8s); `tools/stints.py` summarises stints on any layout.

Notes:
- Race mode is not competitive on this narrow track: the raceline keeps 0.28 m to the walls and localization errs by
  0.15-0.3 m at speed (match 0.6-0.8). FTG uses the live scan and needs no map.
- The 8BitDo sleeps after a few idle minutes (dongle shows `2dc8:301c IDLE`); press Home before each run.
  joy_node segfaulted once on the reconnect: it now runs in a restart loop (tmux window `joy`).
