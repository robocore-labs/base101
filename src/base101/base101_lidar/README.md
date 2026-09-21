# base101_lidar

RPLidar C1 driver + self-filter for base101. Replaces launching
`rplidar_ros`'s own `rplidar_c1_launch.py` directly from forge (a
git-pulled package with no chance to filter its output) with a local launch
that remaps the raw driver onto `/scan_raw` and runs it through a
`laser_filters` chain before republishing the filtered result on
`/scan_filtered` — the name `base101_slam`/`base101_nav`'s configs already
expect (`slam_toolbox.yaml`'s `scan_topic`, `costmap.yaml`'s observation
source), not `/scan`.

## Why filter

`config/lidar_filters.yaml` keeps only the forward 180° of the physical
robot and blanks everything else, plus a small range floor independent of
that cutoff. It's a flat cutoff, not fitted to the chassis — the rear
payload deck (standoffs, `orp_link101_mount`, Jetson/Orin tray) that
motivated filtering in the first place starts around bearing 109° off
`lidar_frame`'s nominal +X, so a 180° window clears it with margin to
spare.

**Frame note:** confirmed (2026-09-21) raw `/scan`'s `angle=0` is the
physical rear, not `base_link`'s +X/front. Two independent fixes for two
independent consumers of that fact: this filter blanks scan bearing
-90°..+90° directly (it doesn't go through TF at all — see
`config/lidar_filters.yaml`'s comments), and
`base101_description/urdf/chassis.xacro`'s `lidar_frame_joint` carries a
matching `yaw=pi` so TF-based consumers (SLAM, costmaps) agree.

## Running

```bash
ros2 launch base101_lidar lidar.launch.py
ros2 launch base101_lidar lidar.launch.py serial_port:=/dev/ttyUSB0
```

Brought up as its own forge component (`lidar` in `hardware.yaml` /
`hardware.drive.yaml`), same as `camera` — not launched from
`base101_bringup_hw/launch/robot.launch.py`.
