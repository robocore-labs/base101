# Running base101 with pixi

[pixi](https://pixi.sh) gives you a complete ROS 2 Jazzy toolchain in a project-local
folder, pulled from the [RoboStack](https://robostack.github.io) conda channel. No
`apt`, no `/opt/ros`, no Docker - delete `.pixi/` and it's gone.

This is **sim only** — for real hardware, `HARDWARE.md` and `docker/hw/` are still
the reference (rmw_zenoh, the Axon 2/link101 firmware, forge deployment — none of
that is pixi's concern). This is for getting `base101_bringup_gazebo`'s sim stack
running natively on macOS, where the `docker/sim/` Dockerfile route isn't
practical (no GPU passthrough into Linux containers on Apple Silicon).

## Install pixi

```bash
curl -fsSL https://pixi.sh/install.sh | bash
exec $SHELL          # pick up the PATH change
```

## Quick start (armless)

```bash
pixi install         # solve + download ROS 2 Jazzy (first run: a few minutes)
pixi run build        # colcon build the workspace (skips base101_arm_moveit_config - see below)
pixi shell            # enter the environment - workspace overlay included
pixi run sim           # ros2 launch base101_bringup_gazebo sim.launch.py arm:=false agent:=false
```

`build`/`test`/`clean`/`stop` are identical in every environment (bare `pixi
run <task>` resolves fine). `sim` is the one task `-e arm` actually overrides
(arm:=true vs arm:=false) — pixi treats that as ambiguous for a bare `pixi
run sim` and refuses to guess, even though one environment is literally named
"default". Pass `-e default` or `-e arm` for `sim` specifically.

`pixi run <task>` runs a single command in the environment and exits. `pixi shell`
gives you an interactive shell with `ros2`, `rviz2`, `colcon`, etc. on `PATH` and
the built workspace sourced. Leave it with `exit`.

`agent:=false`: `robocore_agent` isn't vendored in this workspace (forge pulls it
as a separate container image for real deployment — see `base101.yaml`'s `drive`
vs a hypothetical `agent` component). Sim's own default is `agent:=true`; without
the package built, that fails to find the executable. Pass `agent:=true` yourself
once you have it sourced some other way.

## Environments

| Environment | Enter with          | Contents                                                              |
|-------------|----------------------|-------------------------------------------------------------------------|
| `default`   | `pixi shell`          | Armless sim: description, `ros2_control`, Gazebo, nav2, slam_toolbox, rosboard |
| `arm`       | `pixi shell -e arm`   | Same dependencies as `default` — just `sim` passing `arm:=true` instead of `false`. **No MoveIt** (see below) — **also needs the mod101 underlay built separately, see next section** |

Prefix `sim` with `-e arm` for the arm: `pixi run -e arm sim`. `build` is the
same task/dependency set either way, so `pixi run build` alone is enough.

### No MoveIt, on purpose — ros2_control only

`arm:=true` gets the arm's `ros2_control` interfaces (position/trajectory
controllers, same as hardware — see `base101_arm.hardware.xacro`), not motion
planning. `src/base101_arm/base101_arm_moveit_config` needs `moveit_ros_move_group`/
`moveit_planners_ompl`/etc., none of which this file installs — so it's
**always** skipped (`--packages-skip base101_arm_moveit_config` in `build`,
same in every environment), not just when mod101 isn't around. If MoveIt
support in sim becomes wanted later, that package would also need
`mod101_moveit_config` (the underlay, next section) sourced before it'll
build at all.

### Bringing up the arm in sim (`-e arm`)

mod101 is its own repo with its own `pixi.toml` — this project doesn't vendor or
build it. Build order:

```bash
# 1. mod101, in its own pixi project (see ~/Work/mod101 or wherever you cloned it)
cd ../mod101 && pixi install && pixi run build

# 2. base101, arm-enabled environment
cd ../base101 && pixi install -e arm && pixi run -e arm build
pixi run -e arm sim      # arm:=true
```

`pixi/activate.sh` sources `../mod101/install/setup.sh` (relative to this repo) on
top of this workspace's own overlay, if it exists — silently a no-op otherwise, so
the `default` environment is unaffected by whether mod101 happens to be built next
door. If your mod101 checkout lives somewhere other than a `../mod101` sibling,
symlink it there or edit the path in `pixi/activate.sh`.

## Tasks

Defined in `pixi.toml` under `[tasks]`, shared by every environment except
where `[feature.arm.tasks]` overrides one (see pixi's
[multi-environment docs](https://pixi.prefix.dev/latest/workspace/multi_environment/)):

| Task     | `-e default`                                                                    | `-e arm`                       |
|----------|-----------------------------------------------------------------------------------|----------------------------------|
| `build`  | `colcon build --symlink-install --packages-skip base101_arm_moveit_config ...`    | same — not overridden            |
| `sim`    | `ros2 launch base101_bringup_gazebo sim.launch.py agent:=false`                   | + `arm:=true`                    |
| `test`   | `colcon test && colcon test-result --verbose` (builds first)                      | same — not overridden            |
| `clean`  | `rm -rf build install log`                                                        | same — not overridden            |
| `stop`   | `bash pixi/stop.sh` — kills anything this workspace started, see below            | same — not overridden            |

Anything not listed still works inside `pixi shell` / `pixi run --`, e.g.
`pixi run -- ros2 launch base101_bringup_gazebo sim.launch.py headless:=true`.

## `pixi run stop` — actually stopping everything

Docker gives you `compose down` reaping a whole container's process tree for
free; a bare `ros2 launch` doesn't have an equivalent. Ctrl+C on the
foreground launch works fine (SIGINT propagates to the process group), but
anything else — a backgrounded/detached launch, a closed terminal tab, a
dropped SSH session, `kill <launch-pid>` directly — leaves every node it
spawned running independently, still bound to the ROS graph. Those pile up
silently across runs and start colliding with each other (stale processes
fighting over the same DDS shared-memory ports, duplicate `/clock` or `EKF`
instances on the same domain) — confirmed the hard way once already; see
the "known macOS Gazebo issue" section's history if curious.

`pixi run stop` (`pixi/stop.sh`) kills everything this workspace could have
started — matched by executable path (`$CONDA_PREFIX/lib/` or
`$PIXI_PROJECT_ROOT/install/`, not a hand-maintained node-name list, so it
doesn't go stale as launch files change), SIGTERM first with a 3s grace
period, SIGKILL for anything still alive after that, plus `ros2 daemon
stop`. Run it after every session, not just when something looks wrong —
stray processes give no visible symptom until a later run collides with
them.

## How the workspace overlay works

Same mechanism as mod101's own `PIXI.md` — `pixi.toml` registers
`pixi/activate.sh` as an activation script, sourced on every `pixi shell`/`pixi
run`. Before the first build it's a no-op; after, the overlay (and, for `-e arm`,
mod101's) is live automatically.

Re-run `pixi run build` after changing any `CMakeLists.txt`, `package.xml`,
or C++ source. `--symlink-install` means Python and launch/config edits are
picked up without a rebuild.

## Known macOS Gazebo issue: server + GUI in one process

`gz sim` normally runs the physics server and the Qt/Ogre2 GUI in a single
process. On macOS that doesn't work reliably — this is
[documented upstream](https://gazebosim.org/docs/harmonic/getstarted/), not a
base101-specific bug:

> On macOS, you will need to run Gazebo using two terminals, one for the server
> and another for the GUI... The GUI on macOS is currently known to be unstable.

`sim.launch.py` handles this: on `sys.platform == 'darwin'` (and `headless:=false`
— headless already only ever runs `-s`, no GUI, so it was never affected) it
starts `gz sim -s -r <world>` and `gz sim -g` as two separate processes instead of
one combined `gz sim -r <world>` call. Linux is unaffected — same single-process
invocation as before.

Two things to know if you hit trouble:
- The GUI is reported upstream as unstable on macOS beyond basic camera
  controls — don't be surprised by a plugin panel (e.g. Component Inspector)
  misbehaving. That's upstream, not something to chase down here.
- Both `gazebo-2` (server) and `gazebo-3` (GUI) log `[Ogre2RenderEngine.cc:752]
  Unable to load Ogre Plugin [.../lib/OGRE-Next]. Rendering will not be
  possible.` on every run, despite `OGRE_RESOURCE_PATH` being set correctly
  and the plugin files genuinely present on disk — **confirmed harmless**:
  the GUI window opens and the robot renders fine anyway (checked
  2026-09-27). Reads like Ogre2 failing to load one or two optional plugins
  from that directory while the render system actually in use
  (`RenderSystem_Metal`) loads through a different path — the error message
  just names the whole directory rather than the specific file, which makes
  it look far more alarming than it is. Don't chase it.
- A **separate**, more specific issue
  ([gz-sim#2877](https://github.com/gazebosim/gz-sim/issues/2877), open as of
  writing) crashes rendering-based **sensors** (cameras, depth) on macOS with a
  Metal pipeline error — unrelated to the server/GUI split above, and not
  something this launch file works around. If sim cameras crash on macOS, that's
  the issue to check against before assuming it's local misconfiguration.

## Gotchas

- **Don't mix with system ROS.** Never `source /opt/ros/jazzy/setup.bash` inside a
  pixi shell - the combined `PYTHONPATH` breaks both.
- **Gazebo needs a GPU + display.** No headless-only Mac setup for the GUI path;
  `headless:=true` (server-only, `--headless-rendering`) still works everywhere.
- **`ROS_DOMAIN_ID`.** pixi sets nothing here - export it yourself if you're on a
  network with a real robot on the ROS graph (`base101.yaml`'s forge config uses
  domain 0; pick something else for a laptop sim session on the same network).
- **Lockfile.** `pixi.lock` pins exact package versions and is committed. Commit it
  when you change dependencies so everyone resolves the same graph.

## Status

**Confirmed working end-to-end** on an M4 MacBook (osx-arm64), armless,
`default` environment — not just "doesn't crash," actually checked:

- `pixi install` / `pixi run build` — clean, no solve conflicts, no build
  errors.
- `pixi run sim` — Gazebo launches (server+GUI split), robot renders, drives,
  camera + depth publish real data.
- `ros2_control`/`diff_drive_controller` loads and activates (was silently
  dead until the `GZ_SIM_SYSTEM_PLUGIN_PATH` fix below).
- SLAM/Nav2: `slam_toolbox`, `planner_server`, `controller_server`,
  `bt_navigator`, `velocity_smoother` all reach lifecycle `active`; `/map`
  publishes; `map → odom` TF resolves with clean, monotonically-increasing
  timestamps over a live multi-second trace.
- `pixi run stop` reliably reaps everything, tested by deliberately
  reproducing the stray-process pile-up below and confirming a clean
  process list after.

**`arm:=true` in sim** (`-e arm`): solves and builds cleanly as of the
MoveIt removal below. Not yet checked with the same rigor as the armless
path (lifecycle states, TF, etc.) — worth a dedicated pass before trusting
it the same way.

**Not started:** the hardware path (`linux-aarch64`, real Jetson) — no pixi
environment exists for it yet, and `rplidar_ros`/`realsense2_camera`/
`rmw_zenoh_cpp` availability on RoboStack's ARM64-Linux build is unverified.

### Bugs found and fixed along the way, from real runs rather than reading package.xml

- `GZ_SIM_SYSTEM_PLUGIN_PATH` was empty — RoboStack's conda activation
  doesn't put the env's `lib/` on it, so `gz sim` never found
  `libgz_ros2_control-system.dylib` even though it was genuinely installed.
  No `GZ_SIM_SYSTEM_PLUGIN_PATH` meant no `controller_manager` in sim at
  all — every spawner hung forever. Fixed in `pixi/activate.sh`
  (`$CONDA_PREFIX`-based, not hardcoded to one environment name).
- `ros-jazzy-laser-filters` was missing entirely — a launch-time dependency
  of `sim.launch.py`'s `scan_filter` node, not one declared in
  `base101_bringup_gazebo`'s own `package.xml`, so a package.xml
  cross-reference alone missed it.
- `ros-jazzy-pick-ik` isn't published for osx-arm64 on RoboStack — and once
  MoveIt itself got dropped from the `arm` environment entirely (ros2_control
  only, see above), the question was moot anyway.
- `[build-dependencies]` is deprecated in current pixi; merged into
  `[dependencies]`.
- Bare `pixi run build` was ambiguous (defined identically in both `default`
  and `arm` at the time) — no longer an issue now that `build` isn't
  feature-overridden at all; `sim` still needs `-e default`/`-e arm`.
- The `sim.launch.py` "Ogre2 plugin fails to load" error is confirmed
  harmless (see the macOS Gazebo section) — logged on every run, doesn't
  affect the GUI actually rendering.
- Stray background processes from imperfectly-killed test runs (backgrounded
  `ros2 launch`, `kill <pid>` instead of Ctrl+C) piled up across a debugging
  session and started colliding — duplicate `/clock`/EKF instances on the
  same ROS domain, DDS shared-memory port conflicts — producing symptoms
  (TF "jump back in time" spam, missed EKF update rates, dead spawners) that
  looked like real bugs but weren't. `pixi run stop` above is the fix.

If something else is missing at runtime that wasn't in a `package.xml`
either, the same class of gap as `laser_filters` — grep the launch files
directly (`package=['"][a-z0-9_]+['"]` across `launch/*.py`) rather than
trusting package.xml alone.
