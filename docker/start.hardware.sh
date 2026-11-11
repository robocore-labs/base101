#!/usr/bin/env bash
set -euo pipefail
idle() { echo "$1 disabled"; exec sleep infinity; }
case "${1:-}" in
 bringup)
  [[ "${BASE101_ENABLE_HARDWARE:-false}" == true ]] || idle Bringup
  exec ros2 launch base101_bringup_hw robot.launch.py camera:=false nav:=false slam:=false agent:=false rosboard:=false \
    "arm:=${BASE101_ARM:-false}" "lidar:=${BASE101_ENABLE_LIDAR:-true}" \
    "lidar_serial_port:=${BASE101_LIDAR_DEVICE:-/dev/link101-lidar}"
  ;;
 camera)
  [[ "${BASE101_ENABLE_CAMERA:-false}" == true ]] || idle Camera
  exec ros2 launch base101_camera camera.launch.py
  ;;
 navigation)
  [[ "${BASE101_ENABLE_MAPPING:-false}" == true ]] || idle Mapping
  exec ros2 launch base101_bringup_hw autonomy.launch.py "nav:=${BASE101_ENABLE_NAVIGATION:-false}"
  ;;
 visualiser)
  exec ros2 run robocore_viz robocore_viz_node --ros-args -p port:=8888 -p use_sim_time:=false \
    -p config_file:=/etc/base101/dashboard.yaml -p 'publish_allowlist:=[disabled]'
  ;;
 *) echo 'Expected bringup, camera, navigation or visualiser' >&2; exit 2 ;;
esac
