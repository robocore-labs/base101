# Sourced by `pixi shell` / `pixi run` (see [activation] in pixi.toml).
# Overlays the colcon workspace once it has been built. Safe to run before the
# first `pixi run build` - it just does nothing.
if [ -f "$PIXI_PROJECT_ROOT/install/setup.sh" ]; then
  source "$PIXI_PROJECT_ROOT/install/setup.sh"
fi

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

# mod101 underlay, `-e arm` only (see PIXI.md) - mod101 is a separate repo/pixi
# project (../mod101 relative to this one), built with ITS OWN `pixi run
# build` first. Sourced here rather than baked into this env's own install/,
# same reasoning arm.xacro's header gives for the plain (non-pixi) build:
# mod101 stays a standalone repo. No-op (silently) if it hasn't been built, or
# isn't checked out at all - only `-e arm` actually needs it.
if [ -f "$PIXI_PROJECT_ROOT/../mod101/install/setup.sh" ]; then
  source "$PIXI_PROJECT_ROOT/../mod101/install/setup.sh"
fi
