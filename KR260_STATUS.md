# KR260 status (updated 2026-10-26, overnight)

Race board: `ssh kr260` = `ubuntu@100.93.28.26` (Tailscale node `kriakr`, hostname `kria`). Wi-Fi "AI Racer" via a BrosTrend dongle (the one from the KV260, `wlxc83a35cc01d8`, 192.168.1.146). Claude runs on the RTX box (`claude` account, tmux session `race`).

## Ready and verified on the KR260
| Item | Result |
|---|---|
| OS / kernel | Ubuntu 22.04.4, **kernel 5.15.0-1080** (same as the KV260; see below why) |
| Boot firmware | K26-BootFW-01.02 on A and B (1.07 update: README todo) |
| Workspaces | blaze_racer subsystem_ws + autonomy_ws/slam_mapping, race_ws (race_core, dpu_detector with VART) |
| race_core tests | 21 tests, 0 failures |
| D435i + EKF (`./race.sh ekf color`) | color 30 Hz, IMU 200 Hz, `/odometry/filtered` 100 Hz; clean start/stop twice |
| DPU (`xdputil query`) | `DPUCZDX8G_ISA1_B4096`, fingerprint `0x101000056010407` (same as the KV260) |
| DPU check set (`dpu_detect_images`, v0 model) | TP 54, FP 4, missed 3; DPU p50 5.54 ms (identical to the KV260) |
| Live detector (`./race.sh detector`, v2neg) | `/perception/camera_dets` 30 Hz, total p50 9.2 ms, DPU 5.6 ms, latency ~50 ms |
| `dpu-overlay.service` | enabled, loads the DPU at boot (after PYNQ's `clear_pl_statefile`) |
| Swap | 4 GB `/swapfile`, swappiness 10 |
| Wi-Fi dongle at power-on | no boot stall (`usb-storage.quirks=0bda:1a2b:i` on the kernel command line) |
| Hokuyo network | eth0 = **bottom PS RJ45**, 192.168.0.15/24; LiDAR 192.168.0.10. **No link: LiDAR wire broken** |
| ROS | defaults as on the KV260 (Fast DDS, domain 0); `~/.bashrc` sources all workspaces |

Not tested yet (not connected): VESC (`/dev/ttyACM0`), joystick (`/dev/input/js0`), Hokuyo `/scan`.

## What changed overnight and why
- **Kernel 1027 -> 1080.** The image's 1027 kernel crashed (Oops in `uvcvideo`, `uvc_stop_streaming`) when the D435i stream was stopped, and the camera stayed dead until reboot. Only the kernel was upgraded (no full `apt upgrade`); it needs Ubuntu's flash-kernel 3.104ubuntu22, whose Kria boot script matches the old one. Rollback files: `/boot/firmware/image.fit.k1027`, `boot.scr.uimg.k1027`.
- **Boot quirk and swap** were applied at 02:10 on 10/26 by a command that showed as rejected in the UI; they work and were kept.
- **DPU runtime** installed (`setup/install_dpu_vai35.sh`, BOARD=KR260). Two fixes to that script: Kria-PYNQ's last line fails when eth0 has no IP, and `/etc/vart.conf` must point at `/usr/lib/dpu.xclbin`.
- **PYNQ boot units:** `jupyter.service` (port 9090, default password) is masked; `clear_pl_statefile.service` is overridden in /etc/systemd/system without its Jupyter dependency, and its script now sources `/etc/pynq_venv.sh`. Boot ends in state `running` with no failed units.
- **`race.sh check`** now reads the KR260's SoC temperature (ams hwmon) and warms up the ROS daemon (the first windows used to show NO DATA).
- Removed the deleted KV260 key (`kv260-claude`) from `~/.ssh/authorized_keys`.

## Known issues / not done
- **LiDAR wire broken** (README todo 2). Nothing that needs `/scan` (follow-the-gap, mapping, race mode) can run until it is fixed.
- sudo needs the ubuntu password (passwordless sudo not set).
- Boot to SSH takes about 1.5 min (systemd ~1.5 min; the first boot on the new kernel took ~6 min).
- The GNOME desktop still runs (~0.5 GB RAM); turning it off for races is optional.
- Joystick is an 8BitDo Ultimate 2C (2.4G dongle): verify `/joy` indices LB=4, RB=5, A=0 before driving.

## First run on the car
1. Power on with everything plugged in (the Wi-Fi dongle can stay in). Wait ~2 min, then `ssh kr260`.
2. `ls /dev/ttyACM0 /dev/input/js0 && lsusb | grep -i realsense && ping -c2 192.168.0.10`
3. TEST_DAY.md §1: `./race.sh bringup`, `./race.sh ekf`, `./race.sh check`; then `ros2 topic echo /joy --field buttons` (LB, RB, A -> 4, 5, 0).
