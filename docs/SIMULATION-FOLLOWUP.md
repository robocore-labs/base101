# Simulation follow-up — 2026-11-11

Hardware testing takes priority. Do not treat standalone Nav2 navigation as validated.

Validated: web arm and base control, RGB-D, lidar, live mapping and native MuJoCo viewer. The arm mounting plate uses namespaced mesh assets to avoid duplicate mesh-name collisions. Implicitfast integration avoids the prior unstable physics reset.

Odometry: fuse wheel forward velocity with simulated IMU yaw rate, matching hardware policy; do not use skid-steer wheel yaw as truth. Latest checks measured 4.4 mm position error and 0.0049 rad turn error against MuJoCo ground truth.

Unresolved: goal (0.8, -0.6) advanced to approximately 0.42 m remaining, then aborted with Nav2 error 105 (failed progress). Check command timestamps/timeout warnings and physical skid-steer turning response. Logs: /tmp/base101-navigation-qualified-result.log and /tmp/base101-nav-qualified-sim.log (local, ephemeral).

Design intent from owner: RoboCore SDK owns smoothing, recovery and higher-level navigation behavior; base101 is intentionally minimal. Current nav.launch.py nevertheless launches nav2_velocity_smoother. Reconcile this with SDK ownership before altering the command graph. Do not add autonomous recoveries or claim SDK end-to-end navigation acceptance from a bare NavigateToPose test. Success and cancellation remain unqualified.

The relaxed simulation progress checker is provisional; review with SDK behavior. Mapping works visually but still needs longer routes and turns checked for drift and map consistency.

Hardware/Balena configuration is an initial scaffold, not a tested deployment. Device interfaces and enable flags must be checked on Odin before activation. No hardware motion has been tested by this migration.
