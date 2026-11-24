#!/usr/bin/env bash
# Inside ros:humble-ros-base (CI job ros2-bag): install the MCAP storage
# plugin, show `ros2 bag info` for Robot Studio's exported bag, then read
# every message with rosbag2_py + rclpy and compare it with the recording.
#   docker run --rm -v "$PWD/out:/data:ro" -v "$PWD/scripts:/scripts:ro" \
#     ros:humble-ros-base bash /scripts/ros2_bag_check.sh
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq ros-humble-rosbag2-storage-mcap > /dev/null
dpkg -s ros-humble-rosbag2-storage-mcap | grep -E '^(Package|Version)'
set +u
source /opt/ros/humble/setup.bash
set -u
echo "== ros2 bag info /data/bag"
ros2 bag info /data/bag
echo "== rosbag2_py + rclpy: every message deserialized and compared with the recording"
python3 /scripts/ros2_bag_verify.py /data/bag /data/run
