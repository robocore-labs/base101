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
