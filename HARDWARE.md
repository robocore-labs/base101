# Running base101 on real hardware

base101's motors, IMU and lidar are driven by the **Axon 2 board (RP2354B)**
running the [`base101-fw`](../base101-fw) firmware. It is a native ROS 2
node (Pico-ROS + zenoh-pico, compatible with `rmw_zenoh`): the host talks ROS
topics, not raw serial. There is no `ros2_control` on the host anymore —
locomotion (diff-drive kinematics) moved onto the firmware itself, so the
host just talks the usual robot-level topics straight to it over zenoh.
Odometry fusion did **not** move to the firmware, though — the firmware only
exposes raw wheel-encoder odometry and raw IMU; a host-side EKF (below)
fuses them into `odom → base_link` and `/odom`.

```
twist_mux ──▶ /link101/cmd_vel ── zenoh ──▶  Axon 2 firmware ──▶ 4× DDSM210
                                                    │
                      /link101/odom/raw, /link101/imu │
                   ◀────────────────────────────────────
                                    │
                                    ▼
                    host EKF (robot_localization)
                                    │
                         /odom, odom → base_link
```

## Topic contract (firmware ⇄ host)

Confirmed against the current `base101-fw`. Every firmware topic is
namespaced under `/link101/` — nothing publishes to plain `/cmd_vel`,
`/odom` or `/imu/*` directly, and the firmware does **not** publish `/tf` (or
`/link101/tf`) at all — see "Host-side fused odometry" below for who does.

### Host → firmware

| Topic | Type | Notes |
|---|---|---|
| `/link101/cmd_vel` | `geometry_msgs/TwistStamped` | `twist.linear.x` (m/s), `twist.angular.z` (rad/s); header ignored. Must arrive at least every 500 ms or the motors brake. |
| `/link101/odom/reset` | `std_msgs/Bool` | `data:true` resets the integrated odometry pose. |
| `/link101/gyro_calibrate` | `std_msgs/Bool` | `data:true` brakes the motors and restarts the 15 s stationary gyro calibration. |
| `/link101/time_sync/response` | `std_msgs/Int64MultiArray` | `[echoed_mcu_us, host_receive_ns, host_send_ns]` — see `base101_time/README.md`. |

### Firmware → host

| Topic | Type | Max rate | Frame |
|---|---|---|---|
| `/link101/odom/raw` | `nav_msgs/Odometry` | 50 Hz | `header.frame_id=odom`, `child_frame_id=base_link` |
| `/link101/imu` | `sensor_msgs/Imu` | 50 Hz | `imu_link` — angular velocity + linear acceleration only; `orientation_covariance[0] = -1` (no orientation estimate) |
| `/link101/imu/mag` | `sensor_msgs/MagneticField` | 50 Hz | `imu_link` — **not fused into anything yet**; don't wire it in for yaw without reviewing the EKF's existing gyro-derived yaw input first (correlated, not independent) |
| `/link101/imu/temperature` | `sensor_msgs/Temperature` | 50 Hz | `imu_link` |
| `/link101/imu/status` | `std_msgs/String` | 1 Hz | no header; publishes even while clock sync is unavailable |
| `/link101/time_sync/request` | `std_msgs/UInt64` | 1 Hz | MCU monotonic timestamp, microseconds |

The IMU is sampled at 208 Hz internally; ROS publication is capped at 50 Hz.
`/link101/odom/raw` is wheel-encoder-only — front-left/front-right motor
feedback, integrated into pose plus `twist.linear.x`/`twist.angular.z`. It
does **not** fuse gyro; that fusion now happens host-side (below).

**No `/joint_states`.** The firmware doesn't publish one — see
`base101.hardware.xacro`'s comment. Nothing on hardware currently needs it:
there's no `ros2_control`/`diff_drive_controller` to feed, and nav2/slam key
off `/odom` + TF + `/scan_filtered`. The only effect is the wheel joints sit
static in RViz. This only becomes a gap once arm hardware is wired up (still
sim-only — see "Tower / arms" below), and that's a separate joint_states
source (servo feedback), not this firmware.

## Host-side fused odometry

The firmware exposes raw sensors only — no fusion, no TF. A `robot_localization`
EKF (`ekf_filter_node`, `base101_control/config/ekf.hw.yaml`) owns that
instead, launched unconditionally from
`base101_bringup_hw/launch/robot.launch.py` — that package now owns the
*whole* graph (robot_state_publisher, twist_mux, this EKF, rosboard, lidar,
camera, SLAM/Nav2, and arm ros2_control if `arm:=true`; see "Bring up the
base" below), the same `nav`/`slam`/`agent` argument contract
`sim.launch.py` has. It's the **one and only** publisher of `/odom` and
`odom → base_link` on hardware:

| Input | Fused | Not fused |
|---|---|---|
| `/link101/odom/raw` | `twist.linear.x` (vx), `twist.angular.z` (wheel-derived wz) | position/orientation (the EKF integrates its own, rather than re-integrating the firmware's differential pose on top) |
| `/link101/imu` | `angular_velocity.z` (gyro-derived wz) | orientation (not provided — `orientation_covariance[0] = -1`), linear acceleration (not fused initially) |

Two independent yaw-rate estimates (wheel-derived and gyro) go in; the EKF's
own fused pose and twist come out on `/odom`, plus `odom → base_link` on
`/tf` (`publish_tf:true`). `two_d_mode:true`, `world_frame`/`odom_frame` both
`odom`. Frame tree:

```
odom
└── base_link
    ├── imu_link
    └── lidar_frame (via the chassis xacro chain — see base101_lidar)
```

`base_link → imu_link` and `base_link → lidar_frame` are fixed transforms
from the URDF/`robot_state_publisher`, same as always — only
`odom → base_link` is dynamic, and the EKF is its sole source.

Madgwick (or any other orientation filter) is deliberately **not** in this
pipeline. If one gets added later, give it its own filtered-IMU topic rather
than feeding its output back into this EKF's `imu0` — fusing both a
Madgwick-derived yaw *and* this EKF's existing gyro-derived yaw would double
up on the same underlying signal without a review of what's actually
independent between them.

## Host one-time setup

1. **udev rules** — exposes the board as stable device names:
   ```
   cd ~/Work/base101-fw && ./install.sh
   #  /dev/link101-zenoh  zenoh serial transport
   #  /dev/link101-lidar  RPLidar C1 UART passthrough
   #  /dev/link101-debug  firmware debug log
   ```
2. **RealSense udev rule** — the forge component's `privileged: true` +
   `devices:` mapping only grants the *container* USB access; the raw
   device node is still `root:root 0660` on the *host* by default, which
   isn't enough on its own (this bit an earlier OAK-D Lite integration with
   `X_LINK_DEVICE_NOT_FOUND`, a different camera but the same category of
   miss). librealsense ships its own rule
   (`librealsense/config/99-realsense-libusb.rules` upstream, per-product-ID)
   but that only takes effect installed on the *host* — installing
   `librealsense2-udev-rules` inside the container image doesn't reach the
   host's udev daemon. One-time, on whichever host the camera is physically
   plugged into:
   ```
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666"' | \
     sudo tee /etc/udev/rules.d/99-realsense.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   Unplug and replug the camera afterward. `8086` is Intel's USB vendor ID
   — verify with `lsusb | grep 8086`. **Not yet applied/verified on the
   real robot.**
3. **zenoh router** — bridges the board's serial zenoh to the host's
   `rmw_zenoh` sessions (TCP 7447). Use the firmware's compose file:
   ```
   cd ~/Work/base101-fw/docker && docker compose up -d   # uses zenoh-serial.json5
   ```
   (There's also a forge-managed path — `forge/base101.yaml`'s `zenoh:`
   block + `forge -p . -f forge/base101.yaml launch` — but forge itself is
   deprecated as of this repo's move to pixi-on-the-robot, see `PIXI.md`.
   Don't run both against the same host either way — same container
   name/image, they'd collide.)

   **The router must be restarted after a board re-enumeration** (e.g. its
   watchdog reboots it and it comes back as `ttyACM1` instead of `ttyACM0`).
   The host `/dev/link101-zenoh` symlink updates immediately, but `zenohd`
   opened the *old* device node at startup and holds that fd — it doesn't
   notice the symlink now points elsewhere until it reopens it, which only
   happens on restart. Two things needed together, not either alone:
   - mount the live host `/dev` tree into the router rather than a fixed
     `devices:` mapping resolved once at container start (otherwise a
     restart alone reopens a device the container doesn't even have access
     to anymore), and point the listener endpoint in `zenoh-serial.json5`
     at `serial//host-dev/link101-zenoh#baudrate=921600` — see
     `base101-fw`'s `docker/zenoh-serial.json5`, not this repo. Expressed
     as `router_docker: {volumes: [...], device_cgroup_rules: [...]}` in
     the forge config if you're still on that path.
   - a restart trigger, since nothing else prompts `zenohd` to reopen the
     device on its own. `docker/zenoh-watcher/` in this repo (a poll loop +
     `docker restart zenoh_router` over a mounted `/var/run/docker.sock` —
     see its `watch.sh` header for why polling rather than watching
     udev/netlink events directly) was built as a forge component and
     deployed the same way as the other forge-managed containers.
     **Not yet adapted for the pixi/no-Docker-on-hardware direction** — if
     you're not running forge, you need some other restart trigger here
     (a host systemd unit + udev rule is the non-Docker equivalent; ask
     if you want that instead of the container).
4. **rmw_zenoh** — every ROS 2 shell that should see the board:
   ```
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ```

## Bring up the base

```
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

ros2 launch base101_bringup_hw robot.launch.py
```

`robot.launch.py` owns the **whole** graph now — same shape as
`sim.launch.py`, per `docs/bringup-restructure.md`'s original design.
Unconditional: `robot_state_publisher` (from `base101.hardware.xacro`,
`simulator:=none`), `twist_mux` (`use_stamped:true`) in front of
`/link101/cmd_vel` — straight to the firmware, no controller_manager in
between for the wheels — the EKF described above, and `rosboard`.
Toggleable via launch args: `lidar:=`/`camera:=` (both default `true`),
`nav:=`/`slam:=` (both default `true`, SLAM/Nav2 delayed +10s so they have
`/tf` and `/scan_filtered` to work with), `agent:=` (default `false` — see
`PIXI.md` for why), and `arm:=` (see "Tower / arms" below). `base101_lidar`/
`base101_camera` remain independently launchable too, for bench-testing one
sensor without the whole graph.

**How you get a ROS 2 Jazzy environment to run this in, on the actual
robot, is presently unresolved.** It used to be forge-built Docker
containers (now deprecated, see `PIXI.md`); the pixi-native replacement for
hardware hasn't been built yet (`pixi.toml` only has `default`/`arm`, both
sim). Until one of those lands, this launch file works fine under any
Jazzy install that has the right packages (`robot_localization`,
`slam_toolbox`, `navigation2`, `rplidar_ros`, `realsense2_camera`,
`laser_filters`, `base101_control_plugin`'s build deps) sourced ahead of it
— `/opt/ros/jazzy` if you have one, or forge's old containers if you're not
ready to give those up yet.

Drive it:
```
ros2 topic pub /cmd_vel_key geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.1}}}' -r 10
```

### Verify
- `ros2 topic echo /link101/cmd_vel` reacts to `/cmd_vel_key`.
- `ros2 topic echo /link101/odom/raw` and `ros2 topic echo /link101/imu` are both publishing.
- `ros2 topic echo /odom` and `ros2 topic echo /tf` show `odom → base_link` moving — from the EKF, not the firmware.
- `ros2 topic info /link101/tf` should report no publishers and no subscribers — the firmware doesn't publish it and nothing on the host relays or listens for it anymore.

### Wheel direction calibration
If forward/back or turning is inverted, flip the wheel `direction` rows in the
firmware's `axon_config.h` (all four for fwd/back; per-side for spin) and
reflash — the host side stays unchanged.

## Sensors

- **Lidar (RPLidar C1)** on the firmware passthrough port:
  ```
  ros2 launch base101_lidar lidar.launch.py \
    serial_port:=/dev/link101-lidar frame_id:=lidar_frame
  ```
  Publishes the filtered `/scan_filtered` (self-hits on the payload deck
  stripped — see `base101_lidar/README.md`), not the raw driver output.
  `base101_slam`/`base101_nav` are already wired to consume that name.
- **IMU** needs no driver — the firmware publishes `/link101/imu` directly.

## Navigation / SLAM

On hardware, `base101_slam` + `base101_nav` are launched together directly
from `robot.launch.py` (`nav:=`/`slam:=`, see "Bring up the base" above) —
both remain independent packages with their own launch files (nav doesn't
require slam to be running, or vice versa); `robot.launch.py` only composes
them, the same way `sim.launch.py` already did for the sim side. (Used to
be a separate `base101_autonomy` package/forge component, specifically to
avoid pulling slam/nav's dependencies into `base101_bringup_hw` — moot now
that `base101_bringup_hw/package.xml` depends on them directly; the package
was removed.)

`base101_slam`'s `slam_toolbox` consumes standard `/tf` + `/scan_filtered`
and publishes only `map → odom` — it never touches `odom → base_link` (the
hardware EKF above owns that, unconditionally, whether or not slam is even
running). `base101_slam`'s own EKF instance (`ekf.sim.yaml`, launched from
`slam.launch.py`) is sim-only, supplementary to `diff_drive_controller`
there — see that file's comments; it is *not* the same EKF as hardware's.

`base101_nav`'s `controller_server`/`bt_navigator`/`velocity_smoother` all
read `/odom` on hardware (`*.hw.yaml`) vs `/diff_drive_controller/odom` in
sim (`*.sim.yaml`) — everything else in those configs is shared.

## Tower / arms

The deck-mounted mod101 arm (`arm:=true`) now runs on hardware too, via
`link101-fw`'s per-servo topics — unlike locomotion, the arm still goes
through `ros2_control` (MoveIt needs `FollowJointTrajectory`, which wants a
`JointTrajectoryController` underneath it):

| Topic | Type | Purpose |
|---|---|---|
| `/link101/servos/servo_<id>/command` | `std_msgs/Float64` | Position command (rad), one topic per discovered servo. `base101_control_plugin/ROS2ControlBridge` (below) addresses each joint by its configured `servo_id`, not by array position. |
| `/link101/joint_states` | `sensor_msgs/JointState` | Batch state (position + velocity), matched **by name** — requires the firmware's servo ID table to map IDs 1-5 to `arm_joint_base`...`arm_joint_wrist_roll` and ID 6 to `arm_6` (mod101's own Feetech ID convention: 1-5 arm, 6 jaw). An ID outside that table reports as `servo_<id>`, which won't match anything and simply never updates. |
| `/link101/servos/servo_<id>/{current,voltage,load,temperature}` | various | Per-servo telemetry, not consumed by ros2_control. |

`src/base101/base101_control_plugin` (`ROS2ControlBridge`) is the
`ros2_control` `SystemInterface` bridging to those topics —
`src/base101/base101_control/urdf/base101_arm.hardware.xacro` declares the 5
arm joints against it; the tool joint (`arm_6`) gets its own block from
`mod101_tool_jaws/urdf/tool.ros2control`, pointed at the same plugin/topics
via `mod101_macro.xacro`'s document-global xacro args (overridden in
`base101_bringup_hw/launch/robot.launch.py`) rather than mod101's own default
(`ros2_control_bridge/TopicBridge` on `/motor_manager/*`, its separate USB
Feetech bus — base101's arm servos share the Axon 2 board/firmware with the
wheels instead).

```
ros2 launch base101_bringup_hw robot.launch.py arm:=true
ros2 launch base101_bringup_hw robot.launch.py arm:=true hardware:=mock   # dry run, nothing moves
```

Only the `jaws` tool has a hardware path (same limitation as mod101's own
`mod101_hw_bringup` — see its README's "Not done yet").

The cross tower is parked out of the build in [`attic/`](attic/README.md) — its
deck mount no longer matches the re-exported chassis, and no real driver was
ever wired for its lift / pan / tilt.
