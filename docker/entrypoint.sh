#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/base101_ws/install/setup.bash
export PYTHONPATH=/workspace/base101/src/base101/base101_mujoco:/opt/base101-python/lib/python3.12/site-packages:${PYTHONPATH:-}
if [[ -f /workspace/base101/install/local_setup.bash ]]; then
    source /workspace/base101/install/local_setup.bash
fi
exec "$@"
