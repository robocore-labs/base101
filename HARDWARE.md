# Running base101 on real hardware

base101's motors, IMU and lidar are driven by the **Axon 2 board (RP2354B)**
running the [`base101-fw`](../base101-fw) firmware. It is a native ROS 2 node
(Pico-ROS + zenoh-pico, compatible with `rmw_zenoh`): the host talks ROS
topics, not raw serial. Locomotion bypasses `ros2_control` entirely on real
hardware — the firmware subscribes to `/cmd_vel` and publishes `/odom`
directly, so there is no `diff_drive_controller`/`base101_control_plugin`
bridge to keep in sync. (This is a host-side change landing ahead of the
firmware side — see `base101-fw` for status.)

```
twist_mux ──▶ /cmd_vel (geometry_msgs/Twist) ── zenoh ──▶ Axon 2 firmware ──▶ 4× DDSM210
                                                                │
host (EKF fuses with /imu/data) ◀── zenoh ◀── /odom (nav_msgs/Odometry) ◀──┘
```

## Topic contract (firmware ⇄ host)

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | host→fw | plain (unstamped) Twist; `twist_mux` output, `use_stamped:false` on hw |
| `/odom` | `nav_msgs/Odometry` | fw→host | wheel odometry computed by the firmware; frame `odom`→`base_link` |
| `/imu/data`, `/imu/mag`, `/imu/temperature` | `Imu` / `MagneticField` / `Temperature` | fw→host | BNO055, frame `imu_link`, 50 Hz |

Wheel geometry (separation, radius) and per-wheel joint order used to compute
`/odom` from the DDSM210 encoders now live entirely in the firmware's
`axon_config.h` — the host carries no copy of them for real hardware.

## Host one-time setup

1. **udev rules** — exposes the board as stable device names:
   ```
   cd ~/Work/base101-fw && ./install.sh
   #  /dev/axon-zenoh  zenoh serial transport
   #  /dev/axon-lidar  RPLidar C1 UART passthrough
   #  /dev/axon-debug  firmware debug log
   ```
2. **zenoh router** — bridges the board's serial zenoh to the host's
   `rmw_zenoh` sessions (TCP 7447). Use the firmware's compose file:
   ```
   cd ~/Work/base101-fw/docker && docker compose up -d   # uses zenoh-serial.json5
   ```
3. **rmw_zenoh** — every ROS 2 shell that should see the board:
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

This starts `robot_state_publisher` (from `base101.hardware.xacro`, i.e.
`simulator:=none`, no `ros2_control` block), a `twist_mux` in front of
`/cmd_vel`, and then SLAM + Nav2. Add `nav:=false slam:=false` for wheels
only.

Drive it:
```
ros2 topic pub /cmd_vel_key geometry_msgs/msg/Twist '{linear: {x: 0.1}}' -r 10
```

### Verify
- `ros2 topic echo /cmd_vel` reacts to `/cmd_vel_key` (or nav/agent/joystick)
  through `twist_mux`.
- `ros2 topic echo /odom` streams from the firmware once it's connected over
  zenoh.
- `ros2 topic echo /imu/data` streams from the BNO055.

### Wheel direction calibration
If forward/back or turning is inverted, flip the wheel `direction` rows in the
firmware's `axon_config.h` (all four for fwd/back; per-side for spin) and
reflash — the host side stays unchanged.

## Sensors

- **Lidar (RPLidar C1)** on the firmware passthrough port:
  ```
  ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/axon-lidar -p frame_id:=lidar_frame
  ```
- **IMU** needs no driver — the firmware publishes `/imu/data` directly.

## Navigation / SLAM

`base101_slam` and `base101_nav` run on top of the base unchanged, except that
`base101_bringup_hw` points `bt_navigator`/`velocity_smoother` at `/odom`
instead of sim's `/diff_drive_controller/odom` (see `nav.launch.py`'s
`odom_topic` argument). EKF (`robot_localization`, fusing `/odom` +
`/imu/data`) is the only publisher of `odom → base_link` on real hardware —
there is no controller to fight it, so `ekf.yaml` sets `publish_tf: true`.
Without SLAM running (`slam:=false`), nothing publishes that TF; the firmware
still drives fine, you just lose the transform for visualization/nav.

## Tower / arms

The deck-mounted mod101 arm (`base101_arm_*`) is **sim-only** for now. The Axon
firmware does expose `/motor_manager/arm_cmd` (ST3215 servos) for a future arm
bring-up.

The cross tower is parked out of the build in [`attic/`](attic/README.md) — its
deck mount no longer matches the re-exported chassis, and no real driver was
ever wired for its lift / pan / tilt.
