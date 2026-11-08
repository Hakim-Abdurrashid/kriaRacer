#!/usr/bin/env bash
# Run the closed-loop simulator for a fixed time and summarise laps, crashes, AEB events and CPU.
# usage: sim_eval.sh SECONDS LOGFILE [sim.launch.py args...]
#   e.g. sim_eval.sh 90 /tmp/run.log map:=$PWD/maps/t.yaml raceline:=$PWD/maps/t_lines/raceline.csv speed_scale:=0.7
set -u
T=$1; LOG=$2; shift 2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_LOCALHOST_ONLY=1
ros2 launch race_core sim.launch.py "$@" > "$LOG" 2>&1 &
LPID=$!
if [[ -n "${BAG:-}" ]]; then   # BAG=<dir> also records telemetry for lap_report.py
  ros2 bag record -s sqlite3 -o "$BAG" /race_core/telemetry /sim/pose /sim/status /drive > /dev/null 2>&1 &
  BPID=$!
fi
sleep 5
CPID=$(pgrep -f 'lib/race_core/race_core_node' | head -1)
SPID=$(pgrep -f 'lib/race_core/race_sim' | head -1)
tick() { awk '{print $14+$15}' /proc/$1/stat 2>/dev/null || echo 0; }
c0=$(tick "$CPID"); s0=$(tick "$SPID"); t0=$(date +%s.%N)
sleep $((T - 5))
c1=$(tick "$CPID"); s1=$(tick "$SPID"); t1=$(date +%s.%N)
kill -TERM $LPID 2>/dev/null   # (SIGINT is ignored by background jobs of a script)
for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 $LPID 2>/dev/null || break; sleep 0.5; done
pkill -f '[l]ib/race_core/race_sim'; pkill -f '[l]ib/race_core/race_core_node'; wait $LPID 2>/dev/null
if [[ -n "${BPID:-}" ]]; then kill -INT $BPID 2>/dev/null; kill -TERM $BPID 2>/dev/null; sleep 2; fi
HZ=$(getconf CLK_TCK)
echo "== $* ($T s)"
echo "race_core CPU: $(echo "scale=1; 100*($c1-$c0)/$HZ/($t1-$t0)" | bc)% of one core; race_sim: $(echo "scale=1; 100*($s1-$s0)/$HZ/($t1-$t0)" | bc)%"
echo "race_core laps: $(grep 'race_core\]: LAP' "$LOG" | grep -oE 'LAP [0-9]+: [0-9.]+ s' | sed 's/LAP [0-9]*: //' | tr '\n' ' ')"
echo "sim laps:       $(grep 'SIM LAP' "$LOG" | grep -oE 'LAP [0-9]+: [0-9.]+ s' | sed 's/LAP [0-9]*: //' | tr '\n' ' ')"
echo "sim crashes: $(grep -c 'CRASH #' "$LOG")  AEB events: $(grep -c 'EMERGENCY BRAKE' "$LOG")  loc lost: $(grep -c 'LOCALIZATION LOST' "$LOG")"
grep -oE 'state [A-Z_]+ -> [A-Z_]+' "$LOG" | sort | uniq -c | sort -rn | head -8
grep -E 'scan pipeline' "$LOG" | tail -1 | sed 's/.*scan pipeline/scan pipeline/'
