#!/usr/bin/env bash
set -euo pipefail
experiment_dir="$(cd "$(dirname "$0")" && pwd)"
export PYTHONPATH="$experiment_dir/../.simdeps${PYTHONPATH:+:$PYTHONPATH}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
python3 "$experiment_dir/experiment.py" "$@"
