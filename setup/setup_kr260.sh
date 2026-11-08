#!/usr/bin/env bash
# Bring a fresh Ubuntu 22.04 Kria image on the KR260 up to race configuration
# (RACE_PLAN.md §4.2). Safe to re-run; each step skips work already done.
#
#   bash setup_kr260.sh                 # everything
#   HOKUYO_IFACE=eth1 bash setup_kr260.sh
#
# Before running:
#   1. Boot firmware: Ubuntu 22.04 needs K26 boot FW >= 2022.1, and the Xilinx
#      wiki warns xmutil bootfw_* stops working after apt upgrade. This script
#      prints the firmware status and will not upgrade packages until you
#      confirm (or set BOOTFW_OK=1).
#   2. Internet: plug one PS-side RJ45 port into a router (the Wi-Fi driver is
#      installed below). The other PS-side port is for the Hokuyo.
set -euo pipefail

HOKUYO_IFACE="${HOKUYO_IFACE:-eth1}"      # PS-side port the LiDAR is cabled to
HOKUYO_HOST_IP="${HOKUYO_HOST_IP:-192.168.0.15/24}"  # LiDAR itself is 192.168.0.10
BLAZE_REPO="${BLAZE_REPO:-https://github.com/airacer-ucf/blaze_racer.git}"
BASE_DIR="$HOME/airacer_basecode"

step() { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
have_pkg() { dpkg -s "$1" >/dev/null 2>&1; }

step "0. Sanity checks"
. /etc/os-release
[[ "$VERSION_ID" == "22.04" && "$(uname -m)" == "aarch64" ]] || { echo "Needs Ubuntu 22.04 aarch64"; exit 1; }
tr -d '\0' < /proc/device-tree/model; echo
sudo -v

step "1. Boot firmware (must be done before apt upgrade)"
sudo xmutil bootfw_status || echo "xmutil bootfw_status failed - check firmware manually (UG1092)"
if [[ "${BOOTFW_OK:-0}" != "1" ]]; then
  read -r -p "Is the boot firmware already updated (>= 2022.1, persistent)? [y/N] " ans
  [[ "$ans" =~ ^[Yy]$ ]] || { echo "Update it first: sudo xmutil bootfw_update -i <BOOT.BIN>, reboot, then sudo xmutil bootfw_update -v"; exit 1; }
fi

step "2. Base packages"
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get install -y git curl build-essential cmake python3-pip htop bc rsync \
  linux-headers-"$(uname -r)" network-manager

step "3. ROS 2 Humble apt source"
if ! have_pkg ros2-apt-source && [[ ! -f /etc/apt/sources.list.d/ros2.sources ]]; then
  ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
fi

step "4. ROS 2 packages for the race stack"
sudo apt-get install -y \
  ros-humble-ros-base ros-dev-tools python3-rosdep \
  ros-humble-ackermann-msgs ros-humble-urg-node ros-humble-joy \
  ros-humble-librealsense2 ros-humble-realsense2-camera \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-map-server ros-humble-nav2-amcl ros-humble-nav2-lifecycle-manager \
  ros-humble-tf2-tools ros-humble-rosbag2-storage-mcap ros-humble-rmw-cyclonedds-cpp \
  ros-humble-cv-bridge \
  libopencv-dev libyaml-cpp-dev python3-opencv python3-yaml python3-numpy
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update

step "5. Device access (VESC serial, RealSense udev rules)"
sudo usermod -aG dialout,plugdev,video "$USER"
if [[ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]]; then
  sudo curl -L -o /etc/udev/rules.d/99-realsense-libusb.rules \
    https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
fi

step "6. Wi-Fi dongle driver (BrosTrend rtl8852bu, same as the KV260)"
if ! have_pkg rtl8852bu-dkms; then
  # Public repo key copied from the KV260's /etc/apt/trusted.gpg.d/brostrend.gpg.
  sudo install -m 644 "$(dirname "$0")/brostrend.gpg" /etc/apt/trusted.gpg.d/brostrend.gpg
  echo "deb https://linux.brostrend.com stable main previous" | sudo tee /etc/apt/sources.list.d/brostrend.list
  sudo apt-get update
  sudo apt-get install -y rtl8852bu-dkms
fi

step "7. Tailscale"
if ! command -v tailscale >/dev/null; then
  curl -fsSL https://tailscale.com/install.sh | sh
fi
tailscale status >/dev/null 2>&1 || echo "Run 'sudo tailscale up' to log this board into the tailnet"

step "8. Hokuyo network (${HOKUYO_IFACE} -> ${HOKUYO_HOST_IP})"
if ! nmcli -t -f NAME con show | grep -qx hokuyo; then
  sudo nmcli con add type ethernet ifname "$HOKUYO_IFACE" con-name hokuyo \
    ipv4.method manual ipv4.addresses "$HOKUYO_HOST_IP" ipv4.never-default yes ipv6.method ignore
fi
sudo nmcli con up hokuyo || echo "Hokuyo link not up yet (cable/power?) - it will come up when connected"

step "9. Workspaces"
# Code comes from the bundle (TEST_DAY.md 0.3: ~/2x1_w + ~/airacer_basecode/blaze_racer with the
# kria-race snapshot); cloning the public repo is only a fallback (it lacks the kria-race changes).
mkdir -p "$BASE_DIR"
[[ -d "$BASE_DIR/blaze_racer/subsystem_ws" ]] || git clone "$BLAZE_REPO" "$BASE_DIR/blaze_racer"
set +u; source /opt/ros/humble/setup.bash; set -u
( cd "$BASE_DIR/blaze_racer/subsystem_ws" && rosdep install --from-paths src --ignore-src -r -y \
  && colcon build --symlink-install )
# slam_mapping (slam_toolbox launch + loop-closure map saver) from autonomy_ws; the other autonomy
# packages (Python planners, YOLO26 on the CPU) are not used on the race car
( cd "$BASE_DIR/blaze_racer/autonomy_ws" && colcon build --packages-select slam_mapping )
if [[ -d "$HOME/2x1_w/race_ws/src" ]]; then
  ( cd "$HOME/2x1_w/race_ws" && rosdep install --from-paths src --ignore-src -r -y && colcon build )
else
  echo "race_ws not found: unpack the bundle first (TEST_DAY.md 0.3), then re-run this script"
fi

step "10. Shell environment"
if ! grep -q "# race-stack env" "$HOME/.bashrc"; then
  cat >> "$HOME/.bashrc" << 'EOF'
# race-stack env
source /opt/ros/humble/setup.bash
[ -f ~/airacer_basecode/blaze_racer/subsystem_ws/install/setup.bash ] && source ~/airacer_basecode/blaze_racer/subsystem_ws/install/setup.bash
[ -f ~/airacer_basecode/blaze_racer/autonomy_ws/install/setup.bash ] && source ~/airacer_basecode/blaze_racer/autonomy_ws/install/setup.bash
[ -f ~/2x1_w/race_ws/install/setup.bash ] && source ~/2x1_w/race_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
EOF
fi

step "Done"
echo "Log out and back in (group changes), then verify:"
echo "  cd ~/2x1_w && ./race.sh bringup     # /scan at 40 Hz, VESC on /dev/ttyACM0"
echo "  cd ~/2x1_w && ./race.sh check"
echo "DPU runtime (Vitis AI 3.5) is the next step: sudo BOARD=KR260 bash install_dpu_vai35.sh (TEST_DAY.md 0.5)"
