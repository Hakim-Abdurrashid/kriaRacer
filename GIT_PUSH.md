# Putting ~/2x1_w under version control

Optional. Claude did not create a git repository or commit anything without your go-ahead.

`.gitignore` already excludes build output, `data/` (July camera frames) and `logs/`.

**On the KV260, push to a new branch of the private kriaRacer repo.** Git asks for your GitHub username and a personal access token.

```
cd ~/2x1_w
git init -b race-stack
git add -A
git commit -m "Race stack: race_core v1, race_sim, dpu_detector, raceline/calibration/lap tools, plan and runbook"
git remote add kriaracer https://github.com/Hakim-Abdurrashid/kriaRacer.git
git push -u kriaracer race-stack
```

**On the KR260, instead of the bundle:**

```
git clone -b race-stack https://github.com/Hakim-Abdurrashid/kriaRacer.git ~/2x1_w
git clone -b kria-race https://github.com/Hakim-Abdurrashid/kriaRacer.git ~/airacer_basecode/blaze_racer
```

**What is in the tree:**

| Path | Contents |
|---|---|
| `RACE_PLAN.md` | The goal document |
| `TEST_DAY.md` | Track-session runbook |
| `race.sh` | One entry point for track sessions |
| `race_ws/src/race_core` | C++ race stack, simulator and tools |
| `race_ws/src/dpu_detector` | DPU car detector |
| `setup/` | KR260 setup, DPU runtime and the boot service |
| `models/` | DPU models (xmodel) and a small labelled check set |
| `maps/` | blaze_track and the simulator rehearsal maps |
| `ml/` | Training and quantization scripts. They run on the RTX box. |
| `research/` | Benchmarks behind the measurements in the plan |
| `tools/` | Bag frame extractor, odometry-to-TF relay and CPU snapshot |
