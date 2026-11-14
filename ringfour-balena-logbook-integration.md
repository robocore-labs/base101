# Ringfour Balena integration: Logbook preview

This note describes the locally built ARM64 images and the current base101
`docker-compose.balena.yml`. It is a deployment handoff; no Balena release was
pushed and the robot was not modified by writing this file.

## Local images

| Image | Local image ID prefix | Purpose |
| --- | --- | --- |
| `ringfour-agent:c-preview` / `ringfour-agent:latest` | `2213ff9641eb` | Agent and ROS integration; the existing Agent Dockerfile uses `c-preview` |
| `ringfour-daemon:latest` | `7c7433bef3d0` | Device connection, Agent motion handoff and local Logbook runtime |
| `ringfour-runner:latest` | `07b2eeb937b6` | Mission execution; contains the SDK and separate Platform runtime wheels |
| `ringfour-sdk:latest` | `a38d311036ca` | Optional standalone SDK development image; **not an additional robot service** |

All four were built for `linux/arm64`. The Runner includes the existing
`ringfour_sdk-0.1.0.dev0` wheel and the separately installed
`ringfour_platform_runtime-0.1.0.dev0` wheel. The SDK and Agent remain usable
without the Platform integration.

The existing base101 Dockerfiles use the first three local image names as
`FROM` images. Build the Balena release in an environment that can resolve
those local images, or make them available to the builder's registry. The SDK
image is not required in the Compose file. Existing image tags are local and
may point to different IDs after another build.

## Balena service configuration

Keep the existing Agent socket, Runner/daemon shared staging volume and
persistent data volumes. The local `docker-compose.balena.yml` now also mounts
a dedicated `ringfour-motion-socket` volume on Agent and daemon and sets
`RINGFOUR_AGENT_MOTION_SOCKET=/run/ringfour-motion/motion.sock` on both. The
daemon mounts that volume read-only; both services run as UID 1000. The socket
directory must be private and owned by that UID. The updated Agent and daemon
base images create it with mode 0700 for a newly initialized named volume.
Preserve the Agent state volume: it contains the motion journal and its stream
identity. The control socket remains a separate volume and is not mounted on
the daemon.

The existing **`ringfour-runner`** configuration includes:

```yaml
RUNNER_LOGBOOK_URL: http://ringfour-daemon:8081
```

The existing Runner setting remains:

```yaml
RINGFOUR_AGENT_URL: unix:///run/robocore/agent.sock
```

Set `RUNNER_AUTH_TOKEN` to the same private value on both `ringfour-runner`
and `ringfour-daemon`. Set `PLATFORM_API_URL` on the daemon to a Platform API
address reachable **from the robot**. Do not use `localhost` for a backend
running on a different computer. Preserve the daemon and Runner data volumes;
replacing them can lose enrollment and local state. The Runner derives a
per-execution Logbook credential; missions do not need the shared Runner token.

These URLs assume the current Docker bridge networking for Runner and daemon.
If their networking changes, use an address reachable from Runner to the
daemon's port 8081. The Agent itself uses host networking, but the Runner uses
the shared Agent Unix socket; the Logbook URL is a different connection.

## Backend and test mission

The Platform API must be updated to the current Platform source and apply its
EF migrations before it can ingest these motion entries. The local API was
rebuilt and restarted, and its readiness check passed. A hosted backend also
needs its own upgrade.

A mission is at
`/Users/cdr/Work/ringfour/missions/logbook-demo`. Its lock matches the SDK and
runtime wheels in the existing Runner. It writes deliberate Logbook entries,
discovers capabilities and requests a roughly one-second teleop wheel test at
0.05 m/s. **It can turn the wheels.** It does not assume odometry or sensor
values change. A measured-stop record appears only when the Agent obtains the
required stop evidence; absence of that record is not confirmation of a stop.

For example, after the updated backend and robot services are ready, from the
Platform checkout:

```sh
uv run --project apps/cli ringfour-cli ../missions/logbook-demo push --robot <robot-name-or-id> --start --follow
```

The CLI needs its existing Auth0 configuration, login and selected
organization. A successful call writes entries locally and replicates them to
the Platform; ordinary mission stdout is also available through CLI logs.

## Current limits

- Console displays mission entries and supported Agent motion records once
  replicated. Agent owner/session values are local attribution, not a verified
  Platform human or execution identity.
- The supported automatic motion coverage is teleop sessions, navigation and
  typed group tasks, measured stops and guards. Some other Agent motion paths,
  including vendor actions, are not yet covered. A `motion.history_incomplete`
  record indicates that history may contain a gap.
- The demo has **not completed successfully** on base101. The attempt reached
  the Agent, but its teleop request was refused
  before any wheel command because an unclean-restart inhibit was retained.
  The isolated local tests use synthetic motion evidence; they are not
  physical-device acceptance.

## Current base101 safety inhibit

On 2026-11-13, a no-session zero-velocity diagnostic reported an engaged
inhibit with two retained resources: `restart` (the previous Agent boot did not
complete verified cleanup) and `restart-model` (the model was unavailable at
startup). The current live model hash matched the old checkpoint hash. This
means the recorded missing startup observation is not evidence that the current
model differs; it does not establish what happened during the unclean shutdown.

The updated Agent preserves a known checkpoint hash when the startup model is
temporarily unavailable. Its authenticated local `safety.recover` handler also
supports explicitly reconciling `restart-model` only when the live model matches
the known hash. That operation does **not** clear `restart`, verify a physical
stop or release the inhibit. The local `profiles/base101_hw.yaml` does not
declare `safety.stop_verification` or `safety.restart_recovery_controllers`;
current teleop and restart recovery require reviewed configuration and measured
feedback. Restart recovery also requires a non-empty reviewed controller
baseline and an available controller-manager service; the local hardware notes
do not establish whether that is available on this armless base. Do not guess
thresholds, delete state or bypass the inhibit to make
the demo run. Inspect the real hardware and agree a recovery/commissioning path
before attempting further wheel motion. The rebuilt Agent image has not been
deployed to the device.
