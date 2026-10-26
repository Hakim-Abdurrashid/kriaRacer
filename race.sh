#!/usr/bin/env bash
# One entry point for track sessions (RACE_PLAN.md §6.2, TEST_DAY.md). Run each long-lived command in
# its own terminal (or tmux window):
#
#   ./race.sh check                     sensor rates, DPU, CPU temperature
#   ./race.sh bringup                   drivers + joystick + ackermann_mux (blaze_racer subsystem_stack)
#   ./race.sh ekf [color]               D435i IMU + robot_localization EKF (color: also the RGB stream)
#   ./race.sh reactive [scale]          race_core follow-the-gap (Plan B), deadman = RB
#   ./race.sh map                       slam_toolbox mapping on EKF odometry (drive 1-2 laps, close the loop)
#   ./race.sh savemap NAME              save the map to ~/maps/NAME.{yaml,png}
#   ./race.sh plan NAME [max_steer]     raceline + centerline into ~/maps/NAME_lines/ (look at raceline.png)
#   ./race.sh race NAME [scale] [line]  race_core on the map; line = raceline (default) or centerline
#   ./race.sh scale 0.6                 change race_core's speed scale live (raceline mode)
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
source /opt/ros/humble/setup.bash
[ -f "$HOME/airacer_basecode/blaze_racer/subsystem_ws/install/setup.bash" ] && \
  source "$HOME/airacer_basecode/blaze_racer/subsystem_ws/install/setup.bash"
[ -f "$HOME/airacer_basecode/blaze_racer/autonomy_ws/install/setup.bash" ] && \
  source "$HOME/airacer_basecode/blaze_racer/autonomy_ws/install/setup.bash"
source "$W/race_ws/install/setup.bash"
set -u
cmd="${1:-help}"; shift || true

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
  bringup)
    exec ros2 launch subsystem_stack bringup_launch.py ;;
  ekf)
    color=false; [[ "${1:-}" == "color" ]] && color=true
    exec ros2 launch race_core state_estimation.launch.py color:=$color ;;
  reactive)
    exec ros2 launch race_core race_core.launch.py mode:=reactive reactive_speed_scale:="${1:-0.8}" ;;
  map)
    echo "Start with the car ON THE START LINE facing the race direction (that pose becomes map 0,0,0)."
    exec ros2 launch slam_mapping slam_mapping.launch.py odom_source:=ekf ;;
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
    n="${1:?name}"; line="${3:-raceline}"
    echo "Put the car on the start line (map origin) facing the race direction, then hold RB."
    exec ros2 launch race_core race_core.launch.py mode:=race map:="$MAPS/$n.yaml" \
      raceline:="$MAPS/${n}_lines/$line.csv" speed_scale:="${2:-0.4}" ;;
  scale)
    exec ros2 param set /race_core speed_scale "${1:?scale, e.g. 0.6}" ;;
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
    sed -n '2,26p' "$0" ;;
esac
