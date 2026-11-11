# Pixi and multiROS

The default environment runs ROS 2 Jazzy and MuJoCo physics in a Linux container. macOS uses lighter rendering and a separate native MuJoCo viewer. tools/multiros and dependencies/mod101 are pinned Git submodules.

```sh
git submodule update --init --recursive
pixi install
pixi run setup
pixi run doctor
pixi run up
pixi run sim arm
```

`pixi run sim` defaults to arm and opens the native viewer on macOS. Use `pixi run sim base` for the base alone, `pixi run view` to reopen the viewer, and `pixi run dashboard` for http://localhost:8890. Use `pixi run sim-stop` to stop the graph and `pixi run down` to stop the container.

```sh
pixi run test
pixi run exec python3 checks/check_odometry.py
```

The first checks exercise sensors, map, base and arm. The second compares fused odometry to simulation ground truth. Navigation is still provisional; see [follow-up](docs/SIMULATION-FOLLOWUP.md). RoboCore SDK owns higher-level recovery and motion behavior.

The old native RoboStack manifest is preserved as pixi.native.toml and its instructions as docs/legacy/PIXI-native.md. It is not the current validated launch path.

## Hardware deployment

The root balena.yml and docker-compose.balena.yml define a separate hardware stack: serial Zenoh router, bringup, RealSense, navigation and visualiser. Hardware enable flags default false. This scaffold still requires on-device validation; it is not a simulation acceptance result.

```sh
pixi run balena-devices g_cristi_dragomir/ringfour_test
pixi run balena-prepare
pixi run balena-build g_cristi_dragomir/ringfour_test
pixi run balena-deploy g_cristi_dragomir/ringfour_test
pixi run balena-logs DEVICE_UUID
```

Build/deploy use the workstation Docker daemon and normal fleet releases, not device local mode. The staging allowlist excludes local environments and simulation runtime data.

### Stable serial identities on Balena

Balena services do not bind-mount the host /dev/serial directory. The privileged hardware images recreate the configured /dev/serial/by-id alias from USB sysfs identity at service startup. Link101 uses its serial number and interface 00 (Zenoh); interface 02 is the control port. The current lidar adapter is the sole CH340 1a86:7523 interface 00. Multiple identical CH340 adapters require a more specific identity strategy.

Set BASE101_ZENOH_DEVICE on zenoh-router to /dev/serial/by-id/usb-RoboCore_Link101_ROS_Node_983DBCDBA0D7E993-if00 and BASE101_LIDAR_DEVICE on bringup to /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0. Clear older ttyACM/ttyUSB service overrides. Services retry on failure when USB is temporarily unavailable. The arm remains disabled.

### Hardware validation notes

The distro rplidar_ros driver (SDK 1.12.0) reads C1 health but fails to start scanning. Hardware builds use pinned dependencies/rplidar_ros at 24cc9b6dea97e045bda1408eaa867ce730fd3fc3 (SDK 2.1.0), tested on Odin with explicit Standard mode and approximately 10 Hz /scan_raw. No release is pushed automatically.

A wheels-in-air test commanded 0.08 m/s for two seconds and measured up to 0.087 m/s raw encoder velocity, followed by zero. Physical observation of all four wheels remains pending. A later test lost encoder feedback after Link101 USB disconnected/re-enumerated; the router wrapper now detects USB identity changes and exits for restart. Kernel USB transport errors warrant further cable/power/firmware investigation if repeated.

### RoboCore agent

The separate robocore-agent service derives from the local ARM64 image robocore-agent:c-preview. Ensure that tag exists in the Docker daemon used by the Balena build (load the preview image archive there if needed). It shares host networking, ROS domain 0 and the Zenoh router at localhost:7447. Agent TCP is available on port 10101 with default observer permissions; trusted TCP operator mode is not enabled. Persistent safety/audit state uses the robocore-state named volume. Preserve this volume across upgrades.

The initial docker/agent-profile.yaml declares RGB, lidar and IMU only; no arm, mobility or autonomous navigation is advertised. Review live topics and commission additional capabilities before extending it. No extra BASE101 enable flag is required for the agent; its router must be enabled. Fleet pushes remain manual.
