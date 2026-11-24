#!/usr/bin/env bash
# Build and check the ROS 2 description packages exported by Robot Studio.
# Runs inside the official ros:humble-ros-base image (see the ros2-package
# job in .github/workflows/ci.yml) with the exported zips mounted at /pkgs:
#   colcon build, check_urdf (urdfdom's parser) on every URDF, the official
#   ROS xacro on every exported xacro, the launch file parsed by ros2 launch,
#   and robot_state_publisher started on each URDF.
# RViz and joint_state_publisher_gui are not in ros-base, so the launch file
# is parsed, not run.
set -euo pipefail

apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq liburdfdom-tools ros-humble-xacro > /dev/null
set +u
source /opt/ros/humble/setup.bash
set -u

mkdir -p /ws/src
for z in /pkgs/*.zip; do python3 -m zipfile -e "$z" /ws/src; done
cd /ws
colcon build --event-handlers console_cohesion+
set +u
source install/setup.bash
set -u

for pkg in $(ls src); do
  echo "=== $pkg"
  share="$(ros2 pkg prefix "$pkg")/share/$pkg"
  ls -R "$share"
  for u in "$share"/urdf/*.urdf; do check_urdf "$u"; done
  for x in "$share"/urdf/*.urdf.xacro; do
    [ -e "$x" ] || continue
    xacro "$x" > /tmp/expanded.urdf
    check_urdf /tmp/expanded.urdf
  done
  ros2 launch "$pkg" display.launch.py --show-args
  urdf=$(ls "$share"/urdf/*.urdf | head -n 1)
  # The description goes in a parameter file: rcl does not take multi-line XML in -p.
  python3 - "$urdf" > /tmp/rsp.yaml << 'PY'
import sys

import yaml

with open(sys.argv[1], encoding="utf-8") as f:
    text = f.read()
print(yaml.safe_dump({"robot_state_publisher": {"ros__parameters": {"robot_description": text}}}))
PY
  set +e
  timeout 6 ros2 run robot_state_publisher robot_state_publisher \
    --ros-args --params-file /tmp/rsp.yaml > /tmp/rsp.log 2>&1
  code=$?
  set -e
  cat /tmp/rsp.log
  test "$code" -eq 124 # still running when stopped: it accepted the description
  grep -q "got segment" /tmp/rsp.log # it parsed the links
  if grep -qiE "error|failed" /tmp/rsp.log; then
    echo "robot_state_publisher reported an error for $pkg"
    exit 1
  fi
done
echo "all packages built and checked"
