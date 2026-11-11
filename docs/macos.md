# base101 + mod101 on macOS (Apple Silicon)

Everything that runs natively on a Mac today, and every problem that had to be
solved to get there. Written for whoever runs or maintains this on a Mac next.
Verified on an M4 MacBook, macOS 26, osx-arm64, ROS 2 Jazzy from RoboStack,
September 2026.

Detail lives in two other docs; this one is the overview and the index:

- `PIXI.md` (this repo): the pixi environments, tasks and the Gazebo sim.
- `../mod101/mujoco.md`: the MuJoCo sim, the URDF→MJCF converter, scenes and
  grasping.

## What works

| | Status | Repo |
|---|---|---|
| ROS 2 Jazzy toolchain via pixi (no Docker, no `/opt/ros`) | works | both |
| Gazebo Harmonic sim, armless base101: drive, camera + depth, ros2_control, SLAM, Nav2 | works, checked end to end | base101 |
| Gazebo sim with the arm (`pixi run sim arm:=true`) | builds and runs; not checked as rigorously | base101 |
| MuJoCo sim of mod101 under `mujoco_ros2_control`: controllers, joint states, wrist camera (color + depth) | works | mod101 |
| MuJoCo viewer | works, as a separate process (`mujoco-view`) | mod101 |
| MuJoCo table scene, pick-and-place over the ROS controllers | scripted pick verified live; the `pick_place` node is new and tested offline only | mod101 |
| Hardware (Jetson, linux-aarch64) | out of scope: Docker / `HARDWARE.md` | base101 |

## Quick start

```bash
curl -fsSL https://pixi.sh/install.sh | bash && exec $SHELL

# Gazebo, base101
cd ~/Work/base101
pixi install && pixi run build
pixi run sim                     # add arm:=true for the arm (needs mod101 built next door)
pixi run stop                    # after every session, see below

# MuJoCo, mod101
cd ~/Work/mod101
pixi install -e mujoco
pixi run -e mujoco mujoco-setup  # one-time: builds mujoco_ros2_control, fetches scene assets
pixi run build
pixi run -e mujoco mujoco-sim    # headless physics + ros2_control
pixi run -e mujoco mujoco-view   # second terminal: viewer
pixi run sliders                 # third terminal: joint slider panel
```

## The saga, in order

### 1. Getting ROS onto the Mac: pixi + RoboStack

Docker was the Linux answer, but on Apple Silicon a Linux container gets no
GPU, which rules out Gazebo's renderer and MuJoCo's cameras. RoboStack
publishes ROS 2 Jazzy as conda packages for osx-arm64, and pixi manages them
per project (`.pixi/`, deleted and it's gone).

What came up:

| Problem | Cause | Fix |
|---|---|---|
| `ros-jazzy-pick-ik` unavailable | Not published for osx-arm64 | Dropped MoveIt from the sim environments entirely: the arm runs on ros2_control interfaces only, as it does on hardware |
| `laser_filters` not found at launch | A launch-time dependency not declared in any `package.xml` | Added to `pixi.toml`. Lesson: grep launch files for `package=` rather than trusting `package.xml` |
| `pixi run build` "ambiguous" | The same task defined in two environments | Only `sim` differs per environment now |
| `[build-dependencies]` deprecated | Current pixi | Merged into `[dependencies]` (still to do in mod101's `full` environment) |

### 2. Gazebo Harmonic

| Problem | Cause | Fix |
|---|---|---|
| Server + GUI in one `gz sim` process is unreliable | Known upstream macOS limitation | `gazebo.launch.py` starts `gz sim -s` and `gz sim -g` as two processes on `darwin` |
| Every controller spawner hangs forever | `GZ_SIM_SYSTEM_PLUGIN_PATH` empty: RoboStack's activation doesn't add the env's `lib/`, so `gz_ros2_control` was never loaded | `pixi/activate.sh` sets it from `$CONDA_PREFIX` |
| `Unable to load Ogre Plugin ... Rendering will not be possible` | Ogre2 failing on optional plugins; Metal rendering works regardless | Harmless, ignore |
| Duplicate `/ros_gz_bridge` node name | Two unnamed bridges | Clock bridge named explicitly |

Upstream issue to know about: [gz-sim#2877](https://github.com/gazebosim/gz-sim/issues/2877),
a Metal crash in rendering sensors on some setups. Camera + depth work on this
machine, but if they crash on yours, check that issue first.

### 3. Stray processes

Not a macOS bug, but it hit hardest here because there's no Docker to reap
everything. A `ros2 launch` that isn't stopped with Ctrl+C in its own terminal
(backgrounded, tab closed, `kill <pid>`) leaves its nodes running. They pile
up across runs: duplicate `/clock` and EKF instances, DDS shared-memory port
conflicts, TF "jump back in time" spam, missed update rates. All of it looks
like a real bug and isn't.

- `pixi run stop` (`pixi/stop.sh`) kills everything this workspace could
  have started, matched by executable path, then stops the ROS daemon. Run it
  after every session.
- The same symptoms appear with **two sims on one ROS domain**, e.g. a MuJoCo
  demo still running in another terminal: spawners talk to the wrong
  controller manager ("Failed loading controller") and `robot_state_publisher`
  logs "Moved backwards in time". Stop the other one.

### 4. MuJoCo: getting `mujoco_ros2_control` to exist

RoboStack ships `ros-jazzy-mujoco-ros2-control` for Linux only. It's built
from source into a gitignored underlay, `mod101/.mujoco_ws/`, by `pixi run -e
mujoco mujoco-setup` (`tools/mujoco/setup_mujoco_ros2_control.sh`, pinned to
upstream tag 0.1.2).

| Problem | Cause | Fix |
|---|---|---|
| RoboStack's `mujoco-vendor` 0.0.9 (MuJoCo 3.10) won't co-solve with the ROS env; conda-forge's `mujoco` Python needs a different `libabseil` than ROS | Dependency conflict | Pin `ros-jazzy-mujoco-vendor==0.0.8` (libmujoco **3.8.1**). Python bindings from the self-contained PyPI wheel, pinned to the **same** 3.8.1 so generated models load in the C++ side |
| Camera plugin requires EGL | EGL is Linux's headless GL; macOS has none | Patch: on Apple, use a **CGL** offscreen context, macOS's own GL API (what MuJoCo's Python uses with `MUJOCO_GL=cgl`) |
| `ld: unknown option: --push-state` | A GNU-ld-only link flag in upstream's CMake | Patch: skip it on Apple (macOS's two-level namespace doesn't need it) |
| `symbol not found in flat namespace '_PyExc_RuntimeError'` when the plugin loads | The C++ plugin links every message package's Python-binding library, whose Python symbols macOS resolves at load time | Link with `-Wl,-dead_strip_dylibs`, conda-forge's own macOS default. No source change |
| First launch sits in `dyld` for seconds | macOS validating signatures of freshly built libraries | Once only, nothing to fix |

The patch (`mod101/tools/mujoco/mujoco_ros2_control-macos.patch`, 4 files) is
small and worth upstreaming.

### 5. pixi drops `DYLD_LIBRARY_PATH`: the one that affects any custom message package

**Symptom:** everything worked when tested as `pixi run bash -c 'source
install/setup.bash && ros2 launch ...'`. The natural `pixi run -e mujoco --
ros2 launch ...` failed with `Library not loaded:
@rpath/libmujoco_ros2_control_msgs__rosidl_typesupport_cpp.dylib`.

**Cause:** ROS on macOS loads message type-support libraries by bare filename
at runtime, searching `DYLD_LIBRARY_PATH` or the running program's own
library path. pixi never passes `DYLD_*` variables to the commands it runs:
not from activation scripts, not even from a static `[activation.env]`. So a
message package built outside the pixi env is invisible. RoboStack's own
messages work only because they live in the env's `lib/`, which every
program already searches.

**Fix:** `mujoco-setup` bakes library paths into the built binaries
(`CMAKE_INSTALL_RPATH_USE_LINK_PATH=ON`) and symlinks the underlay's
libraries into `$CONDA_PREFIX/lib`. A `pixi install` that recreates the env
drops the symlinks: rerun `mujoco-setup`.

**This will hit base101 too**, the first time it builds its own custom
message package on macOS under pixi. Same fix.

### 6. GUI windows must live on the main thread

macOS only allows GLFW/AppKit windows on a process's main thread. Two places
broke:

| Where | Problem | Fix |
|---|---|---|
| Camera plugin | Its first choice, GLFW, crashes when started off the main thread (SIGTRAP) | The CGL patch above: offscreen, legal on any thread, no window at all |
| Built-in MuJoCo viewer (`headless:=false`) | The plugin opens its viewer window on a worker thread | Not patched (would mean restructuring the plugin's threads). Instead the sim runs headless, and `mujoco-view` (`tools/mujoco/mirror_viewer.py`, run with `mjpython`, which keeps the main thread for the window) mirrors it from `/joint_states` and `/free_joint_states`. The same server/GUI split Gazebo needs |

The mirror is view-only: it runs kinematics, never physics, so dragging
things in it does nothing. `mujoco-sim` defaults to `headless:=true` on macOS
for this reason. Linux can use the built-in viewer.

A side note: any GLFW window segfaults in a shell without window-server
access (SSH, some agent sandboxes). That's the environment, not the code.

### 7. Small things

- `/bin/bash` on macOS is 3.2: no `mapfile`, among others. Scripts run by
  pixi tasks use the system bash, so keep them bash-3.2 clean.
- Zsh globbing: `grep --include=*.xacro` fails with "no matches found" in
  zsh. Quote the pattern.

## Found on the Mac, but not macOS problems

These came up during the MuJoCo work, and would bite on Linux too. They're
in `mujoco.md`:

- MuJoCo collides every mesh as its convex hull, which filled the gripper
  jaw's own grasping gap. The converter now splits concave collision meshes
  into convex pieces (CoACD, cached).
- MuJoCo's URDF importer silently drops `<mimic>`. The converter turns it into
  joint equality constraints (parallel gripper, PincOpen).
- Light tool parts (PincOpen's 1 g coupler) blow up without some servo
  armature. The converter now defaults to 1e-3.
- The importer also drops actuators, fixed frames, visual meshes and mesh
  URIs, and chokes on zero inertias. All handled by the converter.

## Open

- Gazebo with the arm (`arm:=true`): not yet checked as rigorously as the
  armless sim.
- Hardware: no linux-aarch64 pixi environment. The Docker-or-pixi decision for
  the robot is pending.
- `mujoco_ros2_control` macOS patch: not yet offered upstream.
- mod101 `pixi.toml`: the `full` environment has `features = []` and still
  uses `[build-dependencies]`.
