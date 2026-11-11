#!/bin/bash
# `pixi run -e mujoco mujoco-setup`: builds mujoco_ros2_control into
# .mujoco_ws/ (gitignored), sourced by pixi/activate.sh in the mujoco env.
#
# Source: our fork of ros-controls/mujoco_ros2_control, checked out next to
# this repo (branch macos-support: the macOS fixes, meant for upstream).
# Override with MUJOCO_ROS2_CONTROL_SRC. On Linux, RoboStack's
# ros-jazzy-mujoco-ros2-control package would do; building the same way
# everywhere keeps one path.
set -euo pipefail

SRC="${MUJOCO_ROS2_CONTROL_SRC:-$PIXI_PROJECT_ROOT/../mujoco_ros2_control}"
WS="$PIXI_PROJECT_ROOT/.mujoco_ws"
if [ ! -d "$SRC/mujoco_ros2_control" ]; then
  echo "[mujoco-setup] no mujoco_ros2_control checkout at $SRC" >&2
  echo "  git clone -b macos-support <our fork> $SRC   (or set MUJOCO_ROS2_CONTROL_SRC)" >&2
  exit 1
fi

mkdir -p "$WS"
cd "$WS"
colcon build --base-paths "$SRC" \
  --packages-up-to mujoco_ros2_control mujoco_3d_lidar \
  --cmake-args -DBUILD_TESTING=OFF

if [ "$(uname)" = "Darwin" ]; then
  # ROS loads message type-support libraries by bare file name at runtime,
  # found via DYLD_LIBRARY_PATH or the running executable's rpath. pixi never
  # passes DYLD_* to the commands it runs, so custom messages built outside
  # the env (mujoco_ros2_control_msgs) are invisible to e.g. a Python
  # subscriber. Link them into the env's own lib/, which every executable
  # searches — how RoboStack's messages are found. A `pixi install` that
  # recreates the env drops the links: rerun this. See docs/macos.md.
  ENV_LIB="$CONDA_PREFIX/lib"
  find "$ENV_LIB" -maxdepth 1 -type l -lname "$WS/*" ! -exec test -e {} \; -delete  # stale
  n=0
  for lib in "$WS"/install/*/lib/*.dylib; do
    ln -sf "$lib" "$ENV_LIB/$(basename "$lib")"; n=$((n + 1))
  done
  echo "[mujoco-setup] linked $n libraries into $ENV_LIB"
fi
echo "[mujoco-setup] done — re-enter the env (pixi shell -e mujoco)"
