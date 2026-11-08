#!/usr/bin/env bash
# Copy the race code and data from this KV260 to the KR260 over Tailscale/LAN.
# Runs on the KV260. Needs the 'kr260' alias in ~/.ssh/config (key auth).
#
#   bash migrate_to_kr260.sh            # code, maps, models, logs, DPU installer cache
#   WITH_BAGS=1 bash migrate_to_kr260.sh  # also the July rosbags (~17 GB)
#
# Build products (build/ install/ log/) are skipped; setup_kr260.sh rebuilds them on the KR260.
set -euo pipefail
H=kr260
EX=(--exclude build/ --exclude install/ --exclude log/ --exclude __pycache__/ --exclude '*.pyc')
R=(rsync -a --info=stats1 --partial)

ssh "$H" 'mkdir -p ~/airacer_basecode ~/blaze_racer'
echo "== ~/2x1_w";                      "${R[@]}" "${EX[@]}" ~/2x1_w/ "$H":~/2x1_w/
echo "== blaze_racer (kria-race, git)"; "${R[@]}" "${EX[@]}" ~/airacer_basecode/blaze_racer/ "$H":~/airacer_basecode/blaze_racer/
echo "== kria_hand_expo";               "${R[@]}" "${EX[@]}" ~/airacer_basecode/kria_hand_expo/ "$H":~/airacer_basecode/kria_hand_expo/
echo "== ~/blaze_racer/maps";           "${R[@]}" ~/blaze_racer/maps/ "$H":~/blaze_racer/maps/
# DPU installer cache: saves the downloads and git clones in install_dpu_vai35.sh
echo "== /opt/kria-dpu-setup -> ~/kria-dpu-setup"
"${R[@]}" --exclude 'Kria-PYNQ/' --exclude 'DPU-PYNQ/build/' /opt/kria-dpu-setup/ "$H":~/kria-dpu-setup/
if [[ "${WITH_BAGS:-0}" == "1" ]]; then
  echo "== rosbags"
  "${R[@]}" ~/airacer_basecode/rosbags ~/airacer_basecode/rosbag2_2026_07_22-* "$H":~/airacer_basecode/
fi
echo "Done. On the KR260: sudo mkdir -p /opt/kria-dpu-setup && sudo cp -a ~/kria-dpu-setup/. /opt/kria-dpu-setup/"
