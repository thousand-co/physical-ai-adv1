#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/ros2_ws"
source /opt/ros/humble/setup.bash
crane_workspace="${CRANE_WS:-/home/ubuntu/airobot_ws}"
if [ ! -f "$crane_workspace/install/setup.bash" ]; then
  echo "CRANE+ workspace not found: $crane_workspace/install/setup.bash (set CRANE_WS)" >&2
  exit 1
fi
source "$crane_workspace/install/setup.bash"
colcon build --symlink-install --packages-select final_project_ros
source install/setup.bash
ros2 launch final_project_ros simple_delivery.launch.py "$@"
