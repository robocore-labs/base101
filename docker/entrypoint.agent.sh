#!/usr/bin/env bash
# Balena named volumes can retain root ownership from earlier releases. Repair
# only the Agent's writable directories, then run the Agent as its normal UID.
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/ringfour/setup.bash

if [[ "${1:-}" == "agent" ]]; then
    state_dir="${RINGFOUR_AGENT_STATE_DIR:-/var/lib/robocore}"
    socket_dir="$(dirname "${RINGFOUR_AGENT_SOCKET:-/run/robocore/agent.sock}")"
    ros_home=/home/ubuntu/.ros
    mkdir -p "$state_dir/audit" "$state_dir/maps" "$socket_dir" "$ros_home/log"
    chown ubuntu:ubuntu "$state_dir" "$state_dir/audit" "$state_dir/maps" \
        "$socket_dir" /home/ubuntu "$ros_home" "$ros_home/log"
    export HOME=/home/ubuntu
    export ROS_HOME="$ros_home"
    export ROS_LOG_DIR="$ros_home/log"
    shift
    exec setpriv --reuid=ubuntu --regid=ubuntu --init-groups \
        /opt/ringfour/lib/ringfour_agent/agent "$@"
fi

exec "$@"
