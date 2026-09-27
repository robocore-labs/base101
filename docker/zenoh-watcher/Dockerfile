# Restarts zenoh_router when the Link101 board's zenoh serial device
# re-enumerates (new ttyACM index after e.g. a watchdog reboot) — see
# HARDWARE.md's "Host one-time setup" step 3 and watch.sh's own header for
# why a running zenohd doesn't pick that up on its own.
#
# Not a ROS node — this is a polling loop + `docker restart` over a mounted
# docker.sock. Built on ros:jazzy-ros-base anyway (not a minimal base like
# alpine) because forge's docker-compose templates unconditionally wrap
# every component's command in `bash -lc "source /opt/ros/<distro>/
# setup.bash && ... && <entrypoint>"` (docker-compose.zenoh*.j2, no
# per-component opt-out) — this image needs bash and a real
# /opt/ros/jazzy/setup.bash regardless of whether it ever uses ROS. Same
# base as ../hw/Dockerfile for the same reason.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# docker.io: only the CLI is used (`docker restart`, over the mounted
# docker.sock) — no dockerd runs in this container.
RUN apt-get update && apt-get install -y --no-install-recommends \
      docker.io \
    && rm -rf /var/lib/apt/lists/*

COPY watch.sh /watch.sh
RUN chmod +x /watch.sh

# No ENTRYPOINT/CMD here — forge's compose templates always generate their
# own `command:` from base101.yaml's `entrypoint:` field for this
# component (see the zenoh_watcher entry there), wrapped in the
# ROS-sourcing bash line mentioned above.
