# ROS 2 Jazzy test sidecar for the Axon board.
#
# Connects to the serial-enabled zenoh router (the `zenoh-axon` service) as an
# rmw_zenoh CLIENT, so host-side ROS 2 tooling sees the firmware's topics:
#   pub  /motor_manager/joint_states           sensor_msgs/JointState
#   pub  /imu/data /imu/mag /imu/temperature   sensor_msgs/Imu, ...
#   sub  /motor_manager/base_cmd               std_msgs/Float64MultiArray (wheels)
#   sub  /motor_manager/arm_cmd                std_msgs/Float64MultiArray (servos)
#
# Bundles rosboard (web dashboard on :8888) plus the ros2 CLI for publishing
# test commands. The router (not this container) owns the serial link; this is
# a pure ROS client.
FROM ros:jazzy-ros-base

# rmw_zenoh + rosboard's Python deps. sensor_msgs/std_msgs/geometry_msgs come
# with ros-base. simplejpeg (image compression in rosboard) has no apt package,
# so pip it; the rest are apt to stay aligned with the system Python.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-zenoh-cpp \
        python3-tornado \
        python3-numpy \
        python3-pip \
        git \
    && pip3 install --no-cache-dir --break-system-packages simplejpeg \
    && rm -rf /var/lib/apt/lists/*

# rosboard — self-contained web visualizer (https://github.com/dheera/rosboard)
RUN git clone --depth 1 https://github.com/dheera/rosboard /opt/rosboard

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=0
# Node sessions connect to the Axon router as a client (see session.json5).
ENV ZENOH_SESSION_CONFIG_URI=/config/session.json5

COPY ros/session.json5 /config/session.json5
COPY ros/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 8888
ENTRYPOINT ["/entrypoint.sh"]
CMD ["rosboard"]
