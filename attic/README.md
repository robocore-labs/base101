# Parked packages

Out of `src/`, so colcon never sees them. Nothing here is built, tested, or
kept in sync with the chassis.

## console

Not a ROS/colcon package — a standalone FastAPI ops dashboard (Docker
container control + config hot-reload) for a forge-deployed robot. Parked
2026-09-27 alongside forge's own deprecation (see `PIXI.md`): its entire
premise is controlling `docker compose` files forge staged, which stops
being the deployment model once the robot runs `pixi` directly. Never
referenced from `README.md`/`HARDWARE.md`, so nothing else needs updating to
un-link it.

To bring it back: `git mv attic/console console`, and either keep it scoped
to whatever forge-managed hosts still exist, or adapt it to control/edit
whatever replaces `docker compose` in the pixi-native world (a
`pixi run <task>`-launching systemd unit, most likely) — see its own
`README.md` for the current (forge-shaped) design.

## base101_teleop

Standalone single-page web teleop (base + every joint) on `:8700`. Parked
2026-09-27: its `server.py` hardcodes topics for a **dual-arm + tower**
robot (`/tower_controller/commands`, `/left_arm_controller`,
`/right_arm_controller`, `/left_gripper_controller`, ...) — none of which
exist on the current single-arm, tower-less chassis (see `base101_tower`
below and `docs/bringup-restructure.md`). Last touched 2026-08-15, predates
the 2026-08-22 single-arm consolidation; almost certainly broken as shipped
against today's robot. rosboard's own Joint sliders card (`:8888`) covers
the same need today.

To bring it back: `git mv attic/base101_teleop src/base101_teleop`, then
rewrite `server.py`'s controller/topic names against the current
`arm_controller`/`gripper_controller` (see `base101_control/config/
controllers.{sim,hw}.yaml`) — there's no tower or second arm to restore.

## base101_tower

The lift column + pan/tilt head variant. Parked 2026-08-14 when the chassis CAD
was re-exported: the deck (`top_plate_1`) shrank from 340x240 mm to 180x240 mm
and rose 48 mm onto standoffs, so `tower_mount_xyz` no longer lands the column
anywhere real. The lift joint was rough to begin with.

To bring it back: `git mv attic/base101_tower src/base101_tower`, then re-derive
`tower_mount_xyz` in `urdf/tower.xacro` against the new deck.
