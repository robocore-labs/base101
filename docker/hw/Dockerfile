# Real-hardware bringup sidecar: everything base101_bringup_hw needs to run
# against the Axon 2 firmware over zenoh.
#
# Deliberately dumb, same as ../sim/Dockerfile: dependencies only, no
# workspace baked in. src/ is bind-mounted at runtime and built once into
# .hw/ by the `ws` service in ../docker-compose.hw.yaml.
#
# base101_bringup_hw's package.xml exec_depends on base101_slam, base101_nav
# and robocore_agent, and colcon enforces that closure even when they're
# --packages-select'd out — so slam_toolbox/nav2/robocore_agent's deps are
# installed below even though docker-compose.hw.yaml launches with
# nav:=false slam:=false agent:=false (no lidar or camera driver yet). No
# Gazebo, no GUI libs: nothing here opens a display.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# rmw_zenoh to talk to the firmware's zenoh router (see ../../firmware/docker),
# ros2_control + diff_drive_controller + twist_mux for the wheels, xacro +
# robot_state_publisher for the model, robot_localization + slam_toolbox +
# nav2_* to satisfy base101_slam/base101_nav's exec_depends (unused while
# nav:=false slam:=false), rosboard's declared deps (python3-pil,
# python3-tornado — see src/rosboard/package.xml).
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pip \
      ros-jazzy-rmw-zenoh-cpp \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-controller-manager ros-jazzy-hardware-interface \
      ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster \
      ros-jazzy-pluginlib \
      ros-jazzy-twist-mux \
      ros-jazzy-robot-localization ros-jazzy-slam-toolbox \
      ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-controller \
      ros-jazzy-nav2-lifecycle-manager ros-jazzy-nav2-map-server \
      ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-msgs \
      ros-jazzy-nav2-planner ros-jazzy-nav2-smac-planner \
      ros-jazzy-nav2-velocity-smoother \
      python3-tornado python3-pil \
    && rm -rf /var/lib/apt/lists/*

# robocore_agent needs pydantic v2 (Ubuntu 24.04's python3-pydantic is 1.10)
# — see docker/sim/Dockerfile's identical note. Unused while agent:=false,
# but required for colcon to build robocore_agent as part of the closure
# above.
RUN pip install --no-cache-dir --break-system-packages \
      'pydantic>=2.5' 'websockets>=12' 'numpy>=1.24'

# rmw_zenoh session config: connect as a client to the zenoh-axon router
# (tcp/localhost:7447 — reachable because both containers use host
# networking). Same file the firmware's own ROS sidecar uses; see its header
# comment for why shared_memory is disabled.
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_SESSION_CONFIG_URI=/config/session.json5
COPY firmware/docker/ros/session.json5 /config/session.json5

WORKDIR /bpe
