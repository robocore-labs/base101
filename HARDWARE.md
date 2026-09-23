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
`base101_bringup_hw/launch/robot.launch.py` — that package's whole scope is
robot_state_publisher + twist_mux + this EKF + rosboard now; SLAM/nav are a
separate `base101_autonomy` package/forge component (see "Navigation / SLAM"
below), not something `robot.launch.py` launches or even depends on. It's
the **one and only** publisher of `/odom` and `odom → base_link` on hardware:

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
   #  /dev/axon-lidar     RPLidar C1 UART passthrough
   #  /dev/axon-debug     firmware debug log
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
4. **rmw_zenoh** — every ROS 2 shell that should see the board:
   ```
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ```

## Bring up the base

```
source /opt/ros/jazzy/setup.bash
source ~/Work/base101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

ros2 launch base101_bringup_hw robot.launch.py
```

This starts `robot_state_publisher` (from `base101.hardware.xacro`,
`simulator:=none`, no `ros2_control` block), a `twist_mux` (`use_stamped:
true`) in front of `/link101/cmd_vel` — straight to the firmware, no
controller_manager in between — the EKF described above, and `rosboard`,
all unconditionally. That's the whole scope of this launch: SLAM/nav are a
separate `base101_autonomy` package, brought up with

```
ros2 launch base101_autonomy autonomy.launch.py
```

(its own forge component in `hardware.yaml`/`hardware.drive.yaml`, not
something `robot.launch.py` has flags for anymore).

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
    serial_port:=/dev/axon-lidar frame_id:=lidar_frame
  ```
  Publishes the filtered `/scan_filtered` (self-hits on the payload deck
  stripped — see `base101_lidar/README.md`), not the raw driver output.
  `base101_slam`/`base101_nav` are already wired to consume that name.
- **IMU** needs no driver — the firmware publishes `/link101/imu` directly.

## Navigation / SLAM

On hardware, `base101_slam` + `base101_nav` are launched together via
`base101_autonomy/launch/autonomy.launch.py` (its own forge component —
see "Bring up the base" above) — a separate small package that composes
the two without either depending on the other, and without pulling in
`base101_bringup_hw`'s own dependencies. Both packages remain independent
launch-wise (nav doesn't require slam to be running, or vice versa).

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

The deck-mounted mod101 arm (`base101_arm_*`) is **sim-only** for now. The Axon
firmware does expose `/motor_manager/arm_cmd` (ST3215 servos) for a future arm
bring-up.

The cross tower is parked out of the build in [`attic/`](attic/README.md) — its
deck mount no longer matches the re-exported chassis, and no real driver was
ever wired for its lift / pan / tilt.
