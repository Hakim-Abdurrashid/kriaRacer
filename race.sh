#!/usr/bin/env bash
# One entry point for track sessions (RACE_PLAN.md §6.2, TEST_DAY.md). Run each long-lived command in
# its own terminal (or tmux window):
#
# Drivers, servo calibration and follow-the-gap come from blaze_racer branch iv2026 (~/blaze_racer,
# BLAZE=... to override); race_core runs the team's follow-the-gap (C++ port, same params.yaml) as its
# REACTIVE mode.
#
#   ./race.sh up [det]                  start everything for follow-the-gap racing in tmux session "car"
#                                       (bringup, ekf, joywatch, ftg; det = also the DPU car detector)
#   ./race.sh down                      stop the tmux session cleanly
#   ./race.sh check                     sensor rates, DPU, CPU temperature
#   ./race.sh bringup                   iv2026 drivers + joystick + ackermann_mux (VESC odom TF off: EKF owns it)
#   ./race.sh ekf [color]               D435i IMU + robot_localization EKF (color: also the RGB stream)
#   ./race.sh ftg [scale]               race_core follow-the-gap (iv2026 planner x scale), deadman = RB
#   ./race.sh map                       slam_toolbox mapping on EKF odometry (drive 1-2 laps, close the loop)
#   ./race.sh savemap NAME              save the map to ~/maps/NAME.{yaml,png}
#   ./race.sh plan NAME [max_steer]     raceline + centerline into ~/maps/NAME_lines/ (look at raceline.png)
#   ./race.sh race NAME [scale] [solo|h2h] [raceline|centerline]
#                                       race_core on the map. solo = time trial (no trailing/overtaking),
#                                       h2h = 1v1 (trail + spline overtake of slower cars)
#   ./race.sh ftgmap NAME [scale]       follow-the-gap laps on the map (localized, laps timed) for comparison
#   ./race.sh drive raceline|ftg        switch race_core's driving live (same map, same lap counter)
#   ./race.sh scale 0.6                 change race_core's speed scale live (raceline mode)
#   ./race.sh rscale 0.8                change the follow-the-gap speed scale live
#   ./race.sh ftgset key value          change one follow-the-gap parameter live (e.g. distance_max_speed 5.0)
#   ./race.sh joywatch                  restart the joystick driver whenever it dies (part of 'up')
#   ./race.sh relocalize                global re-localization (car somewhere on the track, not at the start)
#   ./race.sh detector                  DPU car detector on the camera (needs: ./race.sh ekf color)
#   ./race.sh record TAG [images]       rosbag of everything race_core needs (+ camera images if 'images')
#   ./race.sh report BAG NAME           lap table + trajectory plot of a race bag
#   ./race.sh calib BAG                 speed / steering calibration from a calibration bag
#   ./race.sh sim NAME [scale]          closed-loop simulator on a saved map (no car needed)
#   ./race.sh rviz                      RViz: map, raceline, particles, pose, lookahead, scan (2D Pose
#                                       Estimate tool = set the start pose)
#
# Joystick (8BitDo Ultimate 2C on its 2.4G dongle, XInput layout): hold LB = manual driving (teleop), hold RB = autonomous (race_core),
# A = RESET (re-localize after a hand reset). Releasing RB stops the car.
set -eo pipefail
W="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
MAPS="${MAPS:-$HOME/maps}"
BAGS="${BAGS:-$HOME/bags}"
set +u
BLAZE="${BLAZE:-$HOME/blaze_racer}"
[ -d "$BLAZE/subsystem_ws/install" ] || BLAZE="$HOME/airacer_basecode/blaze_racer"
source /opt/ros/humble/setup.bash
[ -f "$BLAZE/subsystem_ws/install/setup.bash" ] && source "$BLAZE/subsystem_ws/install/setup.bash"
[ -f "$BLAZE/autonomy_ws/install/setup.bash" ] && source "$BLAZE/autonomy_ws/install/setup.bash"
source "$W/race_ws/install/setup.bash"
set -u
cmd="${1:-help}"; shift || true
# Race-day deltas on top of the team's follow_the_gap params.yaml (which stays untouched), = run 6 of
# 11/06 ("awesome run, great turns on full right and full left"; logs/RUNS.md):
# - 0.50 rad steering instead of 0.349: the 0.349 cap means a 0.91 m minimum turning radius, and the
#   car hit the walls in the 0.81 m wide turns on 11/06 (the iv2026 servo calibration reaches ~0.53 rad)
# - top speed 5.0 m/s (was 4.5) reached at 4.0 m of front clearance (was 5.0): faster short straights
# Speed scale: x1.0 by default; x1.1 was clean on the oval, x1.2 hit walls on a tight figure 8.
FTG_SET="${FTG_SET-max_steer=0.50,dist_max_speed=4.0,distance_max_speed=5.0}"

case "$cmd" in
  check)
    ros2 daemon start >/dev/null 2>&1 || true   # a cold daemon makes the first hz windows read NO DATA
    # All probes at once, 25 s: under the full race load (KR260, 10/26) each ros2 CLI needs 6-15 s just to
    # start, and one 5 s probe after another read NO DATA for every topic. The color stream is probed through
    # its camera_info (one per frame): Python falls behind deserializing the images and under-reads them.
    topics=(/scan /odom /joy /camera/camera/imu /odometry/filtered /camera/camera/color/camera_info)
    tmp=$(mktemp -d)
    for i in "${!topics[@]}"; do timeout 25 ros2 topic hz "${topics[$i]}" > "$tmp/$i" 2>/dev/null & done
    wait
    for i in "${!topics[@]}"; do
      r=$(grep -oE 'average rate: [0-9.]+' "$tmp/$i" | tail -1) || true
      printf '%-36s %s\n' "${topics[$i]}" "${r:-NO DATA}"
    done
    rm -rf "$tmp"
    dpu=$(systemctl is-active dpu-overlay 2>/dev/null) || true
    printf '%-36s %s\n' "DPU overlay service" "${dpu:-n/a}"
    # KV260: thermal_zone0; KR260: no thermal zone, the SoC sensors are the 'ams' hwmon (hottest of them)
    temps=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || \
            for h in /sys/class/hwmon/hwmon*; do [[ "$(cat "$h/name")" != ams ]] || cat "$h"/temp*_input; done)
    printf '%-36s %s C\n' "SoC temperature" "$(echo "$temps" | awk 'NF {if ($1 > m) m = $1} END {printf "%.1f", m/1000}')"
    echo "expected: /scan ~40, /odom ~50, /joy ~20 (pad connected), imu ~200, EKF ~100, color camera_info ~30 Hz"
    ;;
  up)
    tmux has-session -t car 2>/dev/null && { echo "tmux session 'car' already running (./race.sh down first)"; exit 1; }
    mkdir -p "$HOME/race_logs"
    win() { tmux new-window -t car -n "$1" "bash -lc \"cd $W && ./race.sh $2 2>&1 | tee -a $HOME/race_logs/$1_\$(date +%m%d_%H%M).log; exec bash\""; }
    tmux new-session -d -s car -n bringup "bash -lc \"cd $W && ./race.sh bringup 2>&1 | tee -a $HOME/race_logs/bringup_\$(date +%m%d_%H%M).log; exec bash\""
    sleep 10; win ekf "ekf color"; sleep 8; win joy joywatch; win core "ftg ${FTG_SCALE:-1.0}"
    [[ "${1:-}" == "det" ]] && { sleep 6; win det detector; }
    echo "started: tmux attach -t car   (windows: $(tmux list-windows -t car -F '#W' | tr '\n' ' '))" ;;
  down)
    for w in $(tmux list-windows -t car -F '#W' 2>/dev/null); do tmux send-keys -t "car:$w" C-c; done
    sleep 5; tmux kill-session -t car 2>/dev/null; echo "stopped" ;;
  joywatch)
    # SDL2 joy_node segfaulted once when the 8BitDo reconnected after sleeping (11/06): start a new one
    # whenever none is running. pgrep -x matches the process name only, not this loop's command line.
    cfg="$(ros2 pkg prefix subsystem_stack)/share/subsystem_stack/config/joy_teleop.yaml"
    while true; do
      if ! pgrep -x joy_node >/dev/null; then
        echo "$(date +%T) no joy_node running: starting one"
        ros2 run joy joy_node --ros-args -r __node:=joy --params-file "$cfg" || true
      fi
      sleep 1
    done ;;
  ftgset)
    exec ros2 param set /race_core "ftg_iv.${1:?parameter, e.g. distance_max_speed}" "${2:?value}" ;;
  bringup)
    exec ros2 launch race_core bringup.launch.py ;;
  ekf)
    color=false; [[ "${1:-}" == "color" ]] && color=true
    exec ros2 launch race_core state_estimation.launch.py color:=$color ;;
  ftg|reactive)
    exec ros2 launch race_core race_core.launch.py mode:=reactive reactive_speed_scale:="${1:-1.0}" \
      ftg_set:="$FTG_SET" ;;
  map)
    echo "The car must be ON THE START LINE facing the race direction: that pose becomes map 0,0,0."
    echo "Resetting the EKF pose to 0 ..."
    ros2 topic pub --once -w 1 /set_pose geometry_msgs/msg/PoseWithCovarianceStamped \
      "{header: {frame_id: odom}, pose: {pose: {orientation: {w: 1.0}}}}" >/dev/null
    exec ros2 launch race_core mapping.launch.py ;;
  savemap)
    mkdir -p "$MAPS"
    ros2 run nav2_map_server map_saver_cli -f "$MAPS/${1:?name}" --ros-args -p map_subscribe_transient_local:=true
    ls -la "$MAPS/$1".* ;;
  plan)
    n="${1:?name}"
    python3 "$W/race_ws/src/race_core/tools/make_raceline.py" "$MAPS/$n.yaml" --out "$MAPS/${n}_lines" \
      --max-steer "${2:-0.40}"
    echo "check: $MAPS/${n}_lines/raceline.png" ;;
  race)
    n="${1:?name}"; fmt="${3:-solo}"; line="${4:-raceline}"
    case "$fmt" in
      solo) trail=false; ot=none ;;
      h2h)  trail=true;  ot=spline ;;
      *) echo "format is solo or h2h"; exit 1 ;;
    esac
    echo "Put the car on the start line (map origin) facing the race direction, then hold RB. ($fmt)"
    exec ros2 launch race_core race_core.launch.py mode:=race map:="$MAPS/$n.yaml" \
      raceline:="$MAPS/${n}_lines/$line.csv" speed_scale:="${2:-0.4}" trail:=$trail overtake:=$ot \
      ftg_set:="$FTG_SET" ;;
  ftgmap)
    n="${1:?name}"
    exec ros2 launch race_core race_core.launch.py mode:=race drive_mode:=ftg map:="$MAPS/$n.yaml" \
      raceline:="$MAPS/${n}_lines/raceline.csv" reactive_speed_scale:="${2:-1.0}" trail:=false \
      ftg_set:="$FTG_SET" ;;
  drive)
    exec ros2 param set /race_core drive_mode "${1:?raceline or ftg}" ;;
  scale)
    exec ros2 param set /race_core speed_scale "${1:?scale, e.g. 0.6}" ;;
  rscale)
    exec ros2 param set /race_core reactive_speed_scale "${1:?scale, e.g. 0.8}" ;;
  relocalize)
    exec ros2 service call /race_core/global_localization std_srvs/srv/Trigger ;;
  detector)
    exec ros2 launch dpu_detector dpu_detector.launch.py model:="$W/models/yolov8n_car_v2neg.xmodel" ;;
  record)
    mkdir -p "$BAGS"
    topics=(/scan /odom /odometry/filtered /ackermann_cmd /drive /joy /race_core/telemetry
            /camera/camera/imu /perception/camera_dets /tf /tf_static /sensors/core)
    [[ "${2:-}" == "images" ]] && topics+=(/camera/camera/color/image_raw /camera/camera/color/camera_info
                                           /perception/debug_image)
    exec ros2 bag record -s sqlite3 -o "$BAGS/$(date +%m%d_%H%M%S)_${1:-run}" "${topics[@]}" ;;
  report)
    b="${1:?bag}"; n="${2:?name}"
    python3 "$W/race_ws/src/race_core/tools/lap_report.py" "$b" --map "$MAPS/$n.yaml" \
      --raceline "$MAPS/${n}_lines/raceline.csv" ;;
  calib)
    python3 "$W/race_ws/src/race_core/tools/calib_report.py" "${1:?bag}" ;;
  rviz)
    command -v rviz2 >/dev/null || { echo "rviz2 missing: sudo apt install ros-humble-rviz2"; exit 1; }
    exec rviz2 -d "$W/race_ws/src/race_core/rviz/race.rviz" ;;
  sim)
    n="${1:?name}"
    ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 exec ros2 launch race_core sim.launch.py map:="$MAPS/$n.yaml" \
      raceline:="$MAPS/${n}_lines/raceline.csv" speed_scale:="${2:-0.5}" ;;
  *)
    sed -n '2,40p' "$0" ;;
esac
