# Ringfour motion safety on Base101

Base101 uses a supported controllerless mobility pattern. The armless hardware
stack publishes Agent velocity commands through `twist_mux` directly to the
Axon 2 firmware on `/link101/cmd_vel`. The bringup image contains ROS 2 control
packages, but the deployed `arm:=false` launch does not start a wheel
controller manager. The live graph on 2026-11-14 confirmed that the
controller-manager service had no provider.

`profiles/base101_hw.yaml` version 3 records this deliberately:

- `restart_recovery_controllers: {}` means the no-controller baseline was
  reviewed. Omitting the field still means the baseline is unknown and Agent
  recovery must refuse.
- Stop verification uses fresh fused `/odom` evidence for a 0.5 second dwell.
  Its limits are 0.01 m/s linear and 0.02 rad/s angular, above the stationary
  EKF noise observed on Odin and well below the robot's command limits.

The Agent supports this as a general robot integration pattern. Full restart
recovery still requires an authenticated local operator, an idle inhibited
Agent and successful measured-stop verification. A legacy `restart-model`
record with a lost previous hash can only be removed atomically as part of that
stronger restart recovery when its current hash matches the active checkpoint;
model-only recovery remains unavailable.

Building or deploying images does not clear the live quarantine and does not
command motion. Run recovery separately while the physical robot is supervised.

## Live recovery evidence

On 2026-11-14, Odin ran the profile version 3 recovery through the authenticated
local Agent socket. It atomically removed `restart` and `restart-model` after 11
progressive `/odom` samples over 0.506 seconds. The final measured linear speed
was effectively zero and angular speed was 0.00019 rad/s. The durable checkpoint
then contained an empty quarantine, and the operator inhibit was released. The
Agent reported `estop: false`. No nonzero motion command was issued during this
recovery.

