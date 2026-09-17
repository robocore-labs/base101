# Dockerized zenoh router (serial transport)

The Axon firmware is a zenoh **client** that connects over a USB-CDC serial
link. The host needs a zenoh **router** listening on that serial port. The
official `eclipse/zenoh` image is built **without** the `transport_serial`
feature, so it can't do this out of the box.

This directory builds an image that:

1. rebuilds `zenohd` from source with `--features zenoh/transport_serial`, and
2. overlays it onto `eclipse/zenoh:latest` (so you keep the official runtime).

## Prerequisites

- The udev symlink `/dev/axon-zenoh` (run `../install.sh` once on the host).
- The router version must be protocol-compatible with the firmware's vendored
  zenoh-pico — currently **1.9.0** (`lib/zenoh-pico/include/zenoh-pico.h`).
  Bump `ZENOH_REF` in `docker-compose.yml` / `--build-arg` if you change it.

## Build & run

```bash
cd docker
docker compose up --build        # builds zenohd, starts the router
```

or with plain Docker:

```bash
cd docker
docker build -t axon-zenohd --build-arg ZENOH_REF=1.9.0 .
docker run --rm -it --network host \
    --device /dev/axon-zenoh:/dev/axon-zenoh \
    axon-zenohd
```

The container listens on:

- `serial//dev/axon-zenoh#baudrate=921600` — the board
- `tcp/[::]:7447` — for rmw_zenoh

(see `zenoh-serial.json5`; edit it to change endpoints).

## Using it with ROS 2 / rmw_zenoh

With this router as your single router, point rmw_zenoh's sessions at it
(it already listens on the default `tcp/7447`):

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# rmw_zenoh session config -> connect tcp/localhost:7447
ros2 topic echo /motor_manager/joint_states
```

To instead federate with an existing `rmw_zenohd`, add a `connect` endpoint
to `zenoh-serial.json5` pointing at your other router.

## ROS 2 test sidecar (Jazzy + rosboard)

If you don't have ROS 2 on the host, the `ros-sidecar` service is a self-
contained ROS 2 **Jazzy** container with `rmw_zenoh_cpp` (configured as a
client of the router above, see `ros/session.json5`) and **rosboard** for a
browser dashboard.

```bash
cd docker
docker compose up --build              # router + rosboard dashboard
```

Then open **http://localhost:8888** and click the firmware's topics to plot
them live (`/imu/data`, `/motor_manager/joint_states`, telemetry, …).

Drive the board from the same container (the firmware subscribes to these):

```bash
# spin all four wheels (rad/s): [front_left, front_right, back_left, back_right]
docker compose run --rm ros-sidecar \
  ros2 topic pub -1 /motor_manager/base_cmd \
  std_msgs/msg/Float64MultiArray '{data: [2.0, 2.0, 2.0, 2.0]}'

# move arm servos (position, by arm_cmd order)
docker compose run --rm ros-sidecar \
  ros2 topic pub -1 /motor_manager/arm_cmd \
  std_msgs/msg/Float64MultiArray '{data: [0.0]}'

# or just poke around
docker compose run --rm ros-sidecar ros2 topic list
docker compose run --rm ros-sidecar shell      # interactive ROS shell
```

Requirements: the firmware must be in **normal mode** (not config mode) so it
opens the zenoh client link, and the `zenoh-axon` router must be up. Both
containers use host networking, so `localhost:7447` and `localhost:8888` work
directly.

## Notes / troubleshooting

- **Permissions:** the official image runs as root, so it can open the mapped
  device. If you switch to a non-root user, add it to the device group
  (`group_add`) or relax the device mode.
- **glibc:** the binary is built on `rust:bookworm` and the base image is
  Debian-based, so they're compatible. If you change the base to a musl/Alpine
  image, build the binary with a matching target instead.
- **Wrong device:** if `/dev/axon-zenoh` doesn't exist, find the port with
  `ls -l /dev/serial/by-id/` and map that node to `/dev/axon-zenoh` in the
  compose `devices:` list.
