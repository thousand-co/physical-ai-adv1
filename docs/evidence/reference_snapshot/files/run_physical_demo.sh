#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
crane_workspace="${CRANE_WS:-/home/ubuntu/airobot_ws}"
source "$crane_workspace/install/setup.bash"
python3 "$project_dir/tools/prepare_physical_delivery.py"
cd "$project_dir/ros2_ws"
colcon build --symlink-install --packages-select final_project_ros
source install/setup.bash
cd "$project_dir"
export GAZEBO_MODEL_DATABASE_URI=""
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
output_path="$project_dir/artifacts/physical_delivery/episode.json"
for argument in "$@"; do
  case "$argument" in output:=*) output_path="${argument#output:=}" ;; esac
done
run_started_at="$(date +%s)"
ros2 launch final_project_ros physical_delivery.launch.py "$@"
python3 "$project_dir/tools/check_physical_delivery.py" "$output_path" --newer-than "$run_started_at"
