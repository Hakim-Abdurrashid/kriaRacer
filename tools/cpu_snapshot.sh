#!/usr/bin/env bash
# CPU of named processes over N seconds, % of one core (RACE_PLAN.md §5.3 budget check).
# usage: cpu_snapshot.sh SECONDS name1 name2 ...   (names as in `ps -e`, max 15 chars)
T=$1; shift
HZ=$(getconf CLK_TCK)
declare -A P S
for n in "$@"; do
  P[$n]=$(pgrep -x "$n" | head -1)
  [[ -n "${P[$n]}" ]] && S[$n]=$(awk '{print $14+$15}' /proc/${P[$n]}/stat)
done
t0=$(date +%s.%N); sleep "$T"; t1=$(date +%s.%N)
tot=0
for n in "$@"; do
  if [[ -z "${P[$n]}" || ! -e /proc/${P[$n]}/stat ]]; then printf '%-16s not running\n' "$n"; continue; fi
  e=$(awk '{print $14+$15}' /proc/${P[$n]}/stat)
  pct=$(echo "scale=1; 100*($e-${S[$n]})/$HZ/($t1-$t0)" | bc)
  tot=$(echo "$tot+$pct" | bc)
  printf '%-16s %5.1f%% of one core\n' "$n" "$pct"
done
printf '%-16s %5.1f%% of one core (%.0f%% of 4 cores)\n' "total" "$tot" "$(echo "$tot/4" | bc -l)"
