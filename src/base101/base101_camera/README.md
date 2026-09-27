# base101_camera

Intel RealSense D415 driver for base101. Wraps `realsense2_camera` (Intel's
own officially-maintained ROS 2 package — installed via rosdep/apt, no
source build) with base101-specific tuning, the same reason
`base101_lidar` wraps `rplidar_ros`.

This replaces an earlier OAK-D Lite integration. That one is worth knowing
about if you're revisiting camera choices: it worked in principle, but the
physical unit ended up wired through an unreliable, shared USB2 hub on the
real robot, causing intermittent `X_LINK_DEVICE_NOT_FOUND` boot failures no
amount of config could fully paper over. See git history for the full
account (`depthai_ros_driver_v3`'s parameter surface, the USB permissions
saga, the udev rule) if a future camera swap runs into similar issues.

## Why tune it

- **Close-range depth.** The mod101 arm's grasp work happens at ~20-30cm.
  `config/d415.yaml` runs depth/color at the D415's native 1280x720 (not a
  lower-res default) and enables spatial/temporal/hole-filling filters for
  cleaner depth at the cost of a little latency and fine detail — the right
  tradeoff for grasp planning over fast motion tracking. **Not yet verified
  on hardware:** the D415's commonly-cited native minimum depth range
  (~30-45cm) may sit outside the low end of that target — there's no
  IR-projector trick to lean on here the way there might be on other
  RealSense models. Confirm the real achievable minimum with
  `realsense-viewer` or by echoing `/camera/aligned_depth_to_color/
  image_raw` at a known close distance before assuming this covers the
  whole range.
- **No onboard IMU.** The D415 doesn't have one (unlike the D435i/D455) —
  there's nothing to enable. The robot's actual IMU is the Axon board's
  (see [`HARDWARE.md`](../../../HARDWARE.md)'s topic contract, fused by the
  host EKF); this camera contributes vision only.
- **TF integration.** Simpler than depthai_ros_driver turned out to be:
  realsense2_camera doesn't spawn its own robot_state_publisher/URDF for a
  parent frame, it only publishes static transforms *outward* from a given
  base frame to each optical frame. `launch/camera.launch.py` points
  `base_frame_id` at `camera_link` (base101_description's real mount frame,
  already in the robot's TF tree via chassis.xacro) and lets
  realsense2_camera fill in everything below it — see that file's docstring
  for the exact frame-name construction this relies on.

## Running

```bash
ros2 launch base101_camera camera.launch.py
```

Normally brought up by `base101_bringup_hw/launch/robot.launch.py`
(`camera:=true`, the default), which includes the same `realsense2_camera`
launch with the same params. This standalone launch is for bench-testing the
camera without the rest of the graph.

## USB passthrough (forge/Docker only)

Only relevant if you run this inside a container (the deprecated forge path,
`forge/base101.yaml`): map `/dev/bus/usb` via `devices:` (not `volumes:` — a
plain bind mount doesn't grant the container's cgroup permission to open the
device nodes) plus `privileged: true`; see forge's own `docs/tips.md` "USB
Devices"/"Camera Access" sections.

**Also needs a host udev rule**, same category of issue as the Movidius one
in `HARDWARE.md`'s "Host one-time setup": librealsense's own rule
(`librealsense/config/99-realsense-libusb.rules` upstream) grants Intel's
vendor ID broad USB access, but that rule only takes effect if it's
installed on the *host* — installing `librealsense2-udev-rules` inside the
container image doesn't reach the host's udev daemon. One-time, on
whichever host the camera is physically plugged into:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-realsense.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the camera afterward. `8086` is Intel's USB vendor ID;
confirm with `lsusb | grep 8086`. **Not yet applied/verified on the real
robot** — do this before expecting the camera to enumerate correctly, don't
assume it's already there the way the Movidius rule now is.

## Verify

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/color/camera_info --field header   # expect frame_id: camera_color_optical_frame
```

The frame_id expectation is traced through realsense2_camera's actual
frame-construction macros (see `launch/camera.launch.py`'s docstring), not
yet confirmed against the running camera on real hardware.
