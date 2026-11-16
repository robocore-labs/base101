# Ringfour Balena integration: Logbook preview

This note describes the locally built ARM64 images and the current base101
`docker-compose.balena.yml`. It is a deployment handoff; no Balena release was
pushed and the robot was not modified by writing this file.

## Local images

| Image | Local image ID prefix | Purpose |
| --- | --- | --- |
| `ringfour-agent:c-preview` / `ringfour-agent:latest` | `be157c6cee7d` | Agent and ROS integration, including controllerless restart recovery |
| `ringfour-daemon:latest` | `769bd2cc9b5d` | Device connection, Agent motion handoff and local Logbook runtime |
| `ringfour-runner:latest` | `7effa9a9557d` | Mission execution; contains the SDK and updated Platform runtime wheel |
| `ringfour-sdk:latest` | `79a7a65fdcaa` | Optional standalone SDK development image; **not an additional robot service** |

The Base101 packaging images are `base101-ringfour-agent:latest`
(`9d99e2af54cf`), `base101-ringfour-runner:latest` (`7effa9a9557d`) and
`base101-ringfour-daemon:latest` (`769bd2cc9b5d`). The Agent wrapper contains
profile version 3.

All four were built for `linux/arm64`. The Runner includes the existing
`ringfour_sdk-0.1.0.dev0` wheel and the updated
`ringfour_platform_runtime-0.1.0.dev0` wheel (SHA-256
`d0e666f1358f659cb42bfadc785d0f4b064b73495fae3da5234c6139ff95df34`). The
SDK and Agent remain usable without the Platform integration.

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

The Agent container starts its entrypoint as root only to repair ownership of
the persistent `/var/lib/robocore` state/audit directories and the shared
`/run/robocore` socket directory. It then drops to UID/GID 1000 (`ubuntu`)
before starting the Agent. The Runner's `app` account is also UID/GID 1000,
so it can connect to the Agent's mode-0600 Unix socket through the shared
volume. This also repairs volumes created by earlier releases with root-owned
state directories. It must be included in the next Balena release; no device
was redeployed as part of this source fix.

The entrypoint also resets `HOME` to `/home/ubuntu` and directs ROS logs to
`/home/ubuntu/.ros/log`, creating that directory with Agent ownership. Without
this, ROS inherited `HOME=/root` from the temporary root setup phase and failed
before opening the Agent socket.

Permission-fix image built locally from `docker/Dockerfile.agent` on
2026-11-16: `base101-ringfour-agent:latest`, ARM64, image ID
`sha256:5e6378044504f2f0e6c38f3dda7838a086fb07a7cd287afc212bf87b400fda29`.
With disposable named volumes deliberately set to root ownership, the
entrypoint repaired the state, audit, maps, and socket directories to UID/GID
1000. Agent startup passed both prior permission failures and continued until
the expected missing local Zenoh router. The Balena staging directory
included the entrypoint, and the staged Agent image built. No device motion or
Balena deployment was attempted.

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
- The first demo attempt reached the Agent but was refused before any wheel
  command because an unclean-restart inhibit was retained. On 2026-11-14,
  profile v3 restart recovery measured the body at rest, removed both retained
  checkpoint records and released the inhibit. Re-running the mission remains
  the end-to-end motion check.

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
stop or release the inhibit. The `profiles/base101_hw.yaml` v3 profile now
declares measured `/odom` stop verification and an explicit empty controller
baseline. The empty mapping is reviewed configuration for this controllerless
mobility path, where twist_mux publishes directly to the Axon firmware;
omission still means unreviewed. Full restart recovery can atomically reconcile
the legacy `restart-model` record whose previous hash was lost, but only after
the robot is idle, inhibited and its stop has been measured. Building these
changes does not itself recover the live device or command motion.
