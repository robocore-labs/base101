# Parked packages

Out of `src/`, so colcon never sees them. Nothing here is built, tested, or
kept in sync with the chassis.

## base101_control_plugin

The `ros2_control` `SystemInterface` (`ROS2ControlBridge`) that bridged
`diff_drive_controller`'s per-wheel command/state interfaces to the Axon 2
firmware's `/motor_manager/*` topics. Parked 2026-09-17: locomotion moved
onto the firmware itself, which now owns the whole control loop (wheel
kinematics, odometry) and talks ROS topics directly over zenoh — there's no
`ros2_control` on the host for real hardware anymore, so nothing loads this
plugin. Still builds fine on its own if resurrected; nothing else in `src/`
depends on it.

To bring it back: `git mv attic/base101_control_plugin src/base101/base101_control_plugin`,
re-add it as an `exec_depend` of `base101_bringup_hw`, and put the
`<ros2_control>` block back in `base101_control/urdf/base101.hardware.xacro`.

## base101_tower

The lift column + pan/tilt head variant. Parked 2026-08-14 when the chassis CAD
was re-exported: the deck (`top_plate_1`) shrank from 340x240 mm to 180x240 mm
and rose 48 mm onto standoffs, so `tower_mount_xyz` no longer lands the column
anywhere real. The lift joint was rough to begin with.

To bring it back: `git mv attic/base101_tower src/base101_tower`, then re-derive
`tower_mount_xyz` in `urdf/tower.xacro` against the new deck.
