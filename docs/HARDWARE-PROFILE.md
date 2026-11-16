# base101 hardware profile

Canonical file: profiles/base101_hw.yaml. Dockerfile.agent bakes this into /etc/robocore/profile.yaml. The profile is armless and validated against ringfour-agent:c-preview.

## Read-only evidence — Odin, 2026-11-12

A six-second subscription check received RGB and aligned depth (640×480, rgb8 / 16UC1), color CameraInfo, filtered lidar, IMU, fused odometry, diagnostics and map messages. RGB, aligned depth and calibration share camera_color_optical_frame. Odometry is odom -> base_link. map -> base_link resolves. /cmd_vel_agent has a subscriber and uses TwistStamped. Existing wheel tests established direct ROS movement and return to rest; agent-driven movement was not tested during this profile work.

The live graph exposes slam_toolbox serialize, deserialize, pause and reset services. It does not expose NavigateToPose or FollowWaypoints actions, so those fields are deliberately absent. There is no arm or verified battery topic in this profile.

## Configuration

- Commands: /cmd_vel_agent via twist_mux, priority 50. Dashboard joystick retains priority 100.
- Feedback: /odom. Caps: 0.3 m/s, 0.6 rad/s. Teleop watchdog cap: 0.5 s; stop on disconnect enabled.
- Camera: raw local RGB and aligned depth plus color CameraInfo. c-preview subscribes sensor_msgs/Image; compressed transport remains available for other consumers. SDK stream cap is 30 fps. The captured live deployment was still 15 fps; pending camera config requests 30 fps.
- Lidar: /scan_filtered. IMU: /link101/imu. Diagnostics: /diagnostics.
- State/audit: persistent robocore-state. Maps: separate robocore-maps volume at /var/lib/robocore/maps in agent and navigation containers, so slam_toolbox and agent can read/write the same files. Both images seed this directory as uid 1000 to avoid first-start ownership races. Existing named volumes retain their prior ownership.
- Restart recovery: the reviewed controller-manager baseline is `{}` because
  the armless hardware command path is `twist_mux -> /link101/cmd_vel ->
  Axon 2`; installing `diff_drive_controller` in an image does not start one.
  Stop verification uses fresh fused `/odom` samples for a 0.5 s dwell, with
  limits of 0.01 m/s and 0.02 rad/s. On 2026-11-14, stationary live samples
  showed 0.0 m/s and less than 0.0004 rad/s yaw noise.

## Fixes required in next release

Live camera TF used camera_camera_link as its root and was disconnected from the robot. RealSense prefixes camera_name to base_frame_id in parameters.cpp; both camera launch paths now pass link, yielding camera_link. Recheck base_link -> camera_color_optical_frame after pushing. No live fix was applied during this work.

No Nav2 or recovery stack was enabled. No deployment, motion commands, SLAM state changes or TCP permission changes were performed. The agent still starts with observer TCP permissions; declaring mobility does not grant network clients operator access. Use the existing local Unix socket/operator access or separately configure the intended trusted gateway before SDK motion testing.

## After your push

Check agent startup/profile version 2 and read-only SDK camera, lidar, IMU, pose and map calls. Confirm camera TF connectivity and both streams at the requested 30 fps. Then test agent lease, direct motion and stop while observing the wheels. Test map save/load with a disposable map name to qualify the shared volume and service path; service discovery alone does not prove those operations. No navigation goal completion is claimed.
