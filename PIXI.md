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

### Ringfour Agent

The separate ringfour-agent service derives from the local ARM64 image ringfour-agent:c-preview. Ensure that tag exists in the Docker daemon used by the Balena build (load the preview image archive there if needed). It shares host networking, ROS domain 0 and the Zenoh router at localhost:7447. Agent TCP is available on robot loopback at port 10101 with default observer permissions; trusted TCP operator mode is not enabled. The Ringfour Runner uses the Agent's local Unix socket through the dedicated `robocore-agent-socket` volume. The socket is owned by UID 1000 and grants matching local clients operator permissions; the Runner mounts the socket volume read-only, which does not restrict socket commands. Persistent safety/audit state uses the robocore-state named volume. Preserve this volume across upgrades.

The canonical profiles/base101_hw.yaml declares direct mobility, aligned RGB-D, lidar, IMU, diagnostics and SLAM for the armless robot. Dockerfile.agent copies that profile; docker/agent-profile.yaml is a matching reference. No autonomous navigation action is advertised because it was absent from the live graph. See docs/HARDWARE-PROFILE.md for evidence and deployment checks. No extra BASE101 enable flag is required for the agent; its router must be enabled. Fleet pushes remain manual.

### Ringfour integration

The Balena Compose includes separate ringfour-daemon:latest and ringfour-runner:latest services from integrateme.md. Their local images must exist in the Docker daemon used for the Balena build. Thin Dockerfile wrappers package those images into the release, rather than asking Balena to pull the unqualified service images from Docker Hub. The daemon reaches the runner at http://ringfour-runner:8082 on the Compose network. Its local API binds to 127.0.0.1:8081. Persistent daemon/runner data and shared staging use separate named volumes; the runner mounts staging read-only.

Set PLATFORM_API_URL on ringfour-daemon and the same random RUNNER_AUTH_TOKEN on both ringfour-daemon and ringfour-runner as device-service variables in Balena. No credentials are baked into the source. Agent startup explicitly loads /etc/robocore/profile.yaml, copied from profiles/base101_hw.yaml in Dockerfile.agent. Agent TCP permissions remain unchanged.

The Runner uses `RINGFOUR_AGENT_URL=unix:///run/robocore/agent.sock`. Existing `robocore-*` volume names and `/var/lib/robocore`, `/run/robocore`, `/etc/robocore/profile.yaml` paths are retained for data and socket compatibility. Balena service variables previously scoped to `robocore-agent` must be moved to `ringfour-agent` if used. New mission code uses `from ringfour import Robot` and `ringfour-sdk==0.1.0.dev0`; the Runner retains the legacy wheel for existing mission revisions.

### Ringfour Logbook preview

Runner sends Logbook entries to `RUNNER_LOGBOOK_URL=http://ringfour-daemon:8081`
over the Compose bridge network. Its Agent connection remains the shared Unix
socket. Preserve the Agent socket, staging, daemon and Runner data volumes.
The existing local Agent, daemon and Runner images include this integration;
`ringfour-sdk:latest` is a development image and is not a robot service.

The Balena Compose also configures the optional Agent motion journal using the
dedicated `ringfour-motion-socket` volume. Agent and daemon use UID 1000; the
daemon mounts the socket volume read-only. The Agent state volume persists the
journal, and the control socket remains separate. See
`ringfour-balena-logbook-integration.md` for the current image IDs, test mission
and motion coverage limits.

Before testing, update the Platform backend and apply its EF migrations using
the Platform deployment procedure. Set a robot-reachable `PLATFORM_API_URL` on
the daemon and matching private `RUNNER_AUTH_TOKEN` values on daemon and Runner.
Do not use localhost for a backend on another computer. Missions receive a
derived per-execution Logbook credential rather than the shared Runner token.

After deploying the backend and robot services and resolving the reviewed
safety/commissioning requirements in the handoff, the demo can be run
from `/Users/cdr/Work/ringfour/platform` with the existing CLI login and selected
organization:

```sh
uv run --project apps/cli ringfour-cli ../missions/logbook-demo push --robot <robot-name-or-id> --start --follow
```

The demo writes Logbook entries, discovers capabilities and requests a short
0.05 m/s teleop test: it can turn the wheels. Its base101 attempt was refused
by a retained restart inhibit before a wheel command. Console can display
replicated mission and supported Agent motion records. Preserve safety state;
see the handoff for motion coverage and unresolved recovery requirements.
See [the integration handoff](ringfour-balena-logbook-integration.md).
