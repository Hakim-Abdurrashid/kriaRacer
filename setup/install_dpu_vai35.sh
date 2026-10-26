#!/usr/bin/env bash
# DPU runtime for the Kria (RACE_PLAN.md §5.4): AMD's KR260 Vitis AI 3.5 route
# (amd/Kria-RoboticsAI install_update_kr260_to_vitisai35.sh), plus cleanup of its
# side effects. Run as root:  sudo BOARD=KV260 bash install_dpu_vai35.sh
#                             sudo BOARD=KR260 bash install_dpu_vai35.sh
#
# Steps: Kria-PYNQ (venv + DPU-PYNQ 2.5) -> Vitis AI 3.5 runtime debs + XRT 2.15
# libs -> DPU-PYNQ design_contest_3.5 overlay -> patches -> cleanup -> boot-time overlay service.
# A backup of the replaced XRT 2.13 libraries goes to /root/xrt-2.13-backup.tgz.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }
BOARD="${BOARD:-KV260}"
WORK=/opt/kria-dpu-setup
mkdir -p "$WORK" && cd "$WORK"
log() { printf '\n==== %s  %s\n' "$(date +%T)" "$*"; }

log "Backup XRT 2.13 user-space libraries and /etc/profile.d"
[[ -f /root/xrt-2.13-backup.tgz ]] || tar -czf /root/xrt-2.13-backup.tgz /usr/lib/libxrt* /etc/profile.d

log "1. Kria-PYNQ (BOARD=$BOARD)"
if [[ ! -d /usr/local/share/pynq-venv ]]; then
  [[ -d Kria-PYNQ ]] || git clone https://github.com/Xilinx/Kria-PYNQ.git
  ( cd Kria-PYNQ && bash install.sh -b "$BOARD" )
fi

log "1b. Keep the PYNQ venv out of every login shell"
# Kria-PYNQ's /etc/profile.d/pynq_venv.sh activates its venv (and inserts a dtbo) for ALL login
# shells, which breaks ROS / user Python. Keep it as an opt-in file instead.
if [[ -f /etc/profile.d/pynq_venv.sh ]]; then mv /etc/profile.d/pynq_venv.sh /etc/pynq_venv.sh; fi
PYNQ_ENV=/etc/pynq_venv.sh

log "2. Vitis AI 3.5 runtime + XRT 2.15 libs"
[[ -f vai3.5_kr260.zip ]] || wget -q -O vai3.5_kr260.zip "https://www.xilinx.com/bin/public/openDownload?filename=vai3.5_kr260.zip"
rm -rf vai3.5_kr260 && unzip -q vai3.5_kr260.zip
( cd vai3.5_kr260/target/runtime_deb && bash setup.sh )
( cd vai3.5_kr260/target && tar -xzf lack_lib.tar.gz && cp -r lack_lib/* /usr/lib )
cp vai3.5_kr260/xbutil_tool/xbutil2 /usr/bin/unwrapped/ 2>/dev/null || true
ldconfig

log "3. DPU-PYNQ design_contest_3.5 overlay into the pynq venv"
[[ -d DPU-PYNQ ]] || git clone https://github.com/Xilinx/DPU-PYNQ -b design_contest_3.5
set +u; source $PYNQ_ENV; set -u
( cd DPU-PYNQ && python3 -m pip install . --no-build-isolation )

log "4. Patches from AMD's script"
grep -q "LD_LIBRARY_PATH=/usr/lib" $PYNQ_ENV || echo "export LD_LIBRARY_PATH=/usr/lib" >> $PYNQ_ENV
[[ -f /usr/bin/xdputil ]] && sed -i "s/\/usr\/bin\///g" /usr/bin/xdputil || true
echo /usr/lib/python3.10/site-packages > /usr/lib/python3/dist-packages/vart.pth

log "5. Cleanup of Kria-PYNQ side effects"
# Jupyter listens on :9090 with the default password 'xilinx' - not on a race car.
systemctl disable --now jupyter.service 2>/dev/null || true
# needrestart was switched to auto-restart services after upgrades; put it back to interactive.
sed -i "s/^\$nrconf{restart} = 'a';/#\$nrconf{restart} = 'i';/" /etc/needrestart/needrestart.conf || true

log "6. Load the DPU overlay at every boot (dpu-overlay.service)"
install -m 644 "$(dirname "$(readlink -f "$0")")/load_dpu_overlay.py" /usr/local/bin/load_dpu_overlay.py
install -m 644 "$(dirname "$(readlink -f "$0")")/dpu-overlay.service" /etc/systemd/system/dpu-overlay.service
systemctl daemon-reload
systemctl enable dpu-overlay.service
systemctl restart dpu-overlay.service && journalctl -u dpu-overlay -n 3 --no-pager || true

log "Done. Check with:  sudo bash -c 'source /etc/pynq_venv.sh && xdputil query'"
log "C++ check (as the normal user): dpu_detect_images <model.xmodel> <image_dir>"
