# MuJoCo support — feasibility

**Constraint:** one robot description. The xacro stays the single source of
truth; MJCF is *generated* from it at launch time and never committed or
hand-edited. If that can't hold, MuJoCo isn't worth it.

**Verdict so far: it holds.** MuJoCo's own URDF importer
(`MjModel.from_xml_path("robot.urdf")`) loads today's xacro output — armless
*and* `arm:=true` — with three small, fully automatic preprocessing steps.
No hand-written MJCF.

## Spike (2026-09-28)

Environment: `pixi shell -e mujoco` (conda-forge `mujoco` 3.13.0 — Python
bindings + `simulate` viewer; solves cleanly in the same solve group as the
ROS stack). Input: `xacro base101.xacro simulator:=none arm:={false,true}`.

| Step | Armless | `arm:=true` |
|---|---|---|
| Raw URDF, no preprocessing | fails (inertia) | fails (inertia) |
| + clamp inertias, rewrite mesh URIs, 3 compiler flags | **loads**: 39 bodies, 4 joints, 64 geoms, 34 meshes | **loads**: 63 bodies, 10 joints (4 wheels + 5 arm + gripper), 106 geoms, 52 meshes |
| 4 s of `mj_step` | — | stable: no NaN, no warnings, 14.9 kg total |

### What broke, in order, and the automatic fix for each

1. **Invalid inertia tensors** — `standoff_{front,back}_{left,right}_1` have
   `izz="0"`: a ~10 g standoff whose axial inertia rounded to zero at the CAD
   exporter's 6-decimal precision. Gazebo silently tolerates this; MuJoCo
   rejects it *while parsing*, so the `boundinertia` compiler option doesn't
   help (it runs later). Fix: clamp non-positive principal moments to `1e-9`
   before import. Belongs in the converter, not a hand-edit of the xacro — a
   CAD re-export would regenerate the same zeros. The mod101 arm links are
   all valid.
2. **Fixed links fused away** — the importer's default `fusestatic="true"`
   merges every fixed-joint link into its parent: 38 links became 5 bodies,
   and `imu_link` / `lidar_frame` / `camera_optical_frame` disappeared —
   exactly the frames sensors need to attach to. Fix: `fusestatic="false"`.
3. **Every visual mesh silently dropped** — `discardvisual` defaults to true
   for URDF input; the first "successful" load had 0 meshes. Fix:
   `discardvisual="false"`.
4. **Mesh URIs not understood** — `file://` (50 in the arm URDF) and
   `package://` (6, from mod101). Fix: strip `file://`, resolve `package://`
   via `ament_index_python`; `strippath="false"` so MuJoCo uses the full path.

All meshes (100 across base101 + mod101) are STL, which MuJoCo loads
natively — no format conversion needed.

### What the importer doesn't give us (converter must add)

Loaded model has **0 actuators, 0 sensors, 0 cameras, no floor, and the root
welded to the world** (no free joint — the robot is bolted in place). All of
this is derivable from what the URDF already contains:

- **Actuators** — from the existing `<ros2_control>` joint tags (velocity for
  wheels, position for arm/gripper). No new source of truth.
- **Free joint on `base_link`** + a ground plane, for the mobile base. (A
  fixed-base arm scene wants the weld — make it a flag.)
- **Sensors** at existing frames: IMU at `imu_link`, camera at
  `camera_optical_frame`, lidar as a rangefinder ring at `lidar_frame`
  (MuJoCo has no lidar sensor; the pre-June `base101_mujoco/lidar_bridge.py`
  in git history — commit `bac62f9` — did this and is a starting point).
- **Collision geometry** — right now collision is whatever primitives the
  URDF declares; fine for the chassis, likely too coarse for grasping.
  Convex decomposition of the visual STLs if needed.
- **Physics tuning** (friction, contact softness, actuator gains) — the one
  non-derivable piece. A small overrides file, same role `chassis.gazebo` /
  `controllers.sim.yaml` already play for Gazebo. Per-simulator tuning, not a
  second description.

## Still open

- **`mujoco_ros2_control` on macOS — works headless, cameras included.**
  Built from source with a small patch; runs upstream's demo on osx-arm64
  (controllers, `/clock`, lidar, commands, and camera color + depth via a CGL
  offscreen context all verified). The built-in viewer still can't run on
  macOS; a separate mirror-viewer process replaces it (not yet confirmed on a
  real display). Details and the setup/view tasks: `~/Work/mod101/mujoco.md`.
- **Nothing ROS-side yet** — no clock, no controllers, no topics. The spike
  only proves the model loads and simulates.
- **Leftover hook:** `base101.xacro` still accepts `simulator:=mujoco` and
  emits `chassis.mujoco.ros2control`, which references a
  `base101_mujoco/scenes/` package deleted on 2026-06-18. Dead until
  replaced by the converter.

## Plan

Three pixi environments eventually: `hw`, `gazebo`, `mujoco`.

Learning happens on **mod101 first, in its own repo** — see
`~/Work/mod101/mujoco.md`. There the spike became a real converter
(`mod101/tools/urdf_to_mjcf.py`) and the arm actuates and tracks commands.
Two findings from there apply directly here:

- **Self-collision between mated parts** is the first thing that breaks
  (MuJoCo only auto-excludes parent/child contacts; `fusestatic="false"`
  makes mated parts non-parent pairs that grind forever). mod101's MoveIt
  SRDF collision matrix fixes it — and base101 already has one too
  (`base101_arm_moveit_config/config/collisions/*.srdf.xacro`,
  `gen_collision_matrix.py`), so the chassis↔arm pairs are covered the same
  way.
- **Actuators come from `<ros2_control>` tags**, so the xacro must emit them
  in a MuJoCo mode. base101's xacro already has a `simulator:=mujoco` arg;
  its dead `chassis.mujoco.ros2control` block becomes the real thing.

Then come back here and compose base101 + mod101 into one working MuJoCo
setup.
