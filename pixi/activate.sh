# Sourced by `pixi shell` / `pixi run` (see [activation] in pixi.toml).
# Underlays first, this workspace last. Each is a no-op until it's built.

# gz_ros2_control's plugin (libgz_ros2_control-system.dylib) is genuinely
# installed by ros-jazzy-gz-ros2-control, but RoboStack's own conda
# activation doesn't put the env's lib/ on GZ_SIM_SYSTEM_PLUGIN_PATH — gz
# sim only searches its own bundled plugin dir by default, so third-party
# system plugins like this one silently fail to load ("Failed to load
# system plugin [gz_ros2_control-system]: Could not find shared library"),
# which means no controller_manager ever gets created in sim and every
# spawner hangs waiting on it. $CONDA_PREFIX (not a hardcoded env name) so
# this works whichever pixi environment is active.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$CONDA_PREFIX/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

# mujoco_ros2_control, built by `pixi run -e mujoco mujoco-setup`.
if [ "${PIXI_ENVIRONMENT_NAME:-}" = "mujoco" ] && [ -f "$PIXI_PROJECT_ROOT/.mujoco_ws/install/setup.sh" ]; then
  source "$PIXI_PROJECT_ROOT/.mujoco_ws/install/setup.sh"
fi

# mod101 underlay (see PIXI.md): a separate repo/pixi project at ../mod101,
# built with ITS OWN `pixi run build` first. Needed for arm:=true, and in the
# mujoco env for the MJCF tooling (mod101_mujoco). No-op if absent.
# local_setup.sh, NOT setup.sh: setup.sh also re-sources every prefix mod101
# was built on top of — mod101's own pixi env and .mujoco_ws — which would
# mix a second conda environment into this one.
if [ -f "$PIXI_PROJECT_ROOT/../mod101/install/local_setup.sh" ]; then
  source "$PIXI_PROJECT_ROOT/../mod101/install/local_setup.sh"
fi

if [ -f "$PIXI_PROJECT_ROOT/install/setup.sh" ]; then
  source "$PIXI_PROJECT_ROOT/install/setup.sh"
fi
