# KR260 status (2026-10-26)

Race board: `ssh kr260` = `ubuntu@100.93.28.26` (Tailscale node `kriakr`, hostname `kria`). Wi-Fi "AI Racer" via BrosTrend dongle (`wlx6815790b1868`, 192.168.1.93).

## Ready
- Ubuntu 22.04.4, kernel 5.15.0-1027 (not upgraded on purpose), boot FW K26-BootFW-01.02 (A and B).
- ROS 2 Humble + the KV260's package set; blaze_racer subsystem_ws, autonomy_ws/slam_mapping, race_ws built.
- `race_core` tests: 21 tests, 0 failures.
- Hokuyo network on eth0 (bottom PS RJ45), 192.168.0.15/24; LiDAR at 192.168.0.10.
- ROS defaults as on the KV260 (Fast DDS, domain 0); `~/.bashrc` sources all workspaces.
- Automatic apt updates disabled (CPU during runs).

## Known issues / not done
- **Wi-Fi dongle plugged in at power-on stalls boot ~4 min** (it first appears as a USB CD-ROM, 0bda:1a2b). Workaround: plug it in after boot. Fix (not applied): `LINUX_KERNEL_CMDLINE="usb-storage.quirks=0bda:1a2b:i"` in `/etc/default/flash-kernel`, then `sudo flash-kernel`.
- **No swap.** Two parallel compile jobs froze the board. Build with `MAKEFLAGS=-j1 colcon build --executor sequential` until a swap file is added.
- sudo needs the ubuntu password (passwordless sudo not set).
- DPU runtime not installed (`setup/install_dpu_vai35.sh`, TEST_DAY.md §0.5); installer cache in `~/kria-dpu-setup`.
- Boot FW 1.07 update deferred to a bench day.
- Joystick is an 8BitDo Ultimate 2C (2.4G dongle), not the F710: verify `/joy` indices LB=4, RB=5, A=0 before driving.

## First run on the car
1. Power on with everything plugged in except the Wi-Fi dongle; plug the dongle in ~1 min after boot.
2. `ls /dev/ttyACM0 /dev/input/js0 && lsusb | grep -i realsense && ping -c2 192.168.0.10`
3. `ros2 topic echo /joy` (after bringup): hold LB, RB, press A -> buttons 4, 5, 0.
4. Then TEST_DAY.md §1 onward (`./race.sh bringup`, `./race.sh check`).
