#!/usr/bin/env bash
# Entrypoint for the Axon ROS 2 Jazzy test sidecar.
#
#   rosboard          (default) start the web dashboard on http://localhost:8888
#   shell | bash      interactive shell with ROS + rmw_zenoh sourced
#   <anything else>   run it as a ROS command, e.g.
#                       docker compose run --rm ros-sidecar ros2 topic list
#                       docker compose run --rm ros-sidecar \
#                         ros2 topic pub -1 /motor_manager/base_cmd \
#                         std_msgs/msg/Float64MultiArray '{data: [1,1,1,1]}'
set -e

source /opt/ros/jazzy/setup.bash

echo "[ros-sidecar] RMW=${RMW_IMPLEMENTATION} DOMAIN=${ROS_DOMAIN_ID} session=${ZENOH_SESSION_CONFIG_URI}"
echo "[ros-sidecar] router endpoint: $(grep -o 'tcp/[^"]*' "${ZENOH_SESSION_CONFIG_URI}" 2>/dev/null || echo '?')"

case "${1:-rosboard}" in
  rosboard)
    echo "[ros-sidecar] starting rosboard -> http://localhost:8888"
    # rosboard's launcher is the `run` script (imports rosboard.rosboard:main);
    # it must run from the repo root so the `rosboard` package is importable.
    cd /opt/rosboard
    exec python3 run
    ;;
  shell|bash)
    exec bash
    ;;
  *)
    exec "$@"
    ;;
esac
