#!/usr/bin/env bash
set -euo pipefail
configuration=${1:-base}
shift || true
arm=false
[[ "$configuration" == arm ]] && arm=true
exec ros2 launch base101_mujoco mujoco.launch.py "arm:=$arm" "$@"
