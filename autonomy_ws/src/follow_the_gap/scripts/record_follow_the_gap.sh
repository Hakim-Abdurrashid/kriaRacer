#!/usr/bin/env bash
set -euo pipefail

output_root="${1:-${HOME}/rosbags}"
timestamp="$(date -u +%Y%m%d_%H%M%S)"
bag_path="${output_root}/follow_the_gap_${timestamp}"

mkdir -p "$output_root"

echo "Recording /scan, /drive, and /odom to: $bag_path"
echo "Press Ctrl-C to stop recording and finalize the bag."
exec ros2 bag record --output "$bag_path" /scan /drive /odom
