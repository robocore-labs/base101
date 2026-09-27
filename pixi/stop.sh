#!/bin/bash
# `pixi run stop` — the one thing Docker gave us for free (`compose down`
# reaping a container's whole process tree) that a bare `ros2 launch` doesn't:
# a reliable "kill everything, however it got left running" button.
#
# `ros2 launch` forwards SIGINT to its children fine when it's the
# foreground process in an interactive terminal (Ctrl+C works as expected)
# — the problem is everything else: a backgrounded/detached launch, a
# closed terminal tab, a dropped SSH session, `kill <launch-pid>` instead of
# Ctrl+C. `ros2 launch`'s own children get reparented and just keep running,
# independently, still bound to the ROS graph — and pile up across runs
# until they start colliding with each other (stale processes fighting over
# the same DDS shared-memory ports, multiple /clock or EKF instances on the
# same domain — exactly what happened during pixi.toml's own bring-up, see
# PIXI.md).
#
# Matches by executable PATH, not by a hand-maintained list of node names:
# every base101/mod101 node — ours or a conda-installed one like
# controller_manager/spawner or nav2_planner — runs out of either
# $CONDA_PREFIX/lib (installed packages) or $PIXI_PROJECT_ROOT/install (this
# workspace's own colcon build), so those two patterns catch anything this
# workspace could have started without needing to keep this list in sync
# with every launch file. Deliberately NOT a bare $PIXI_PROJECT_ROOT match —
# that would also catch an editor or shell that merely has the project open.
set -uo pipefail

PATTERNS=(
  "$CONDA_PREFIX/lib/"
  "$PIXI_PROJECT_ROOT/install/"
)

matches() {
  local pat
  for pat in "${PATTERNS[@]}"; do
    pgrep -f -- "$pat" 2>/dev/null
  done | sort -u
}

pids="$(matches)"
if [ -z "$pids" ]; then
  echo "[stop] nothing running"
else
  echo "[stop] sending SIGTERM to:"
  echo "$pids" | xargs -n1 -I{} ps -o pid=,command= -p {} 2>/dev/null
  echo "$pids" | xargs kill 2>/dev/null
  sleep 3
  remaining="$(matches)"
  if [ -n "$remaining" ]; then
    echo "[stop] still alive after 3s, sending SIGKILL:"
    echo "$remaining" | xargs -n1 -I{} ps -o pid=,command= -p {} 2>/dev/null
    echo "$remaining" | xargs kill -9 2>/dev/null
  fi
fi

# Discovery daemon persists across runs by design — but it's also cheap to
# restart, and a stale one occasionally holds onto graph state from a
# session that no longer exists.
ros2 daemon stop >/dev/null 2>&1 || true

echo "[stop] done"
