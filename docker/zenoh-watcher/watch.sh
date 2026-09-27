#!/bin/bash
# Restarts zenoh_router whenever the Link101 board's zenoh serial device
# re-enumerates.
#
# The host's udev rule updates /dev/link101-zenoh to point at the new
# ttyACM index immediately, live — that part already works via
# base101.yaml's `zenoh.router_docker` mounting the whole host /dev tree
# into zenoh_router instead of a fixed `devices:` mapping resolved once at
# container start. What it does NOT fix: zenohd itself opened the *old*
# device node at startup and holds that file descriptor — a symlink now
# pointing somewhere else doesn't make it reopen anything. Only a restart
# does. This container is what asks for that restart.
#
# Deliberately a poll loop, not a udev/netlink event watcher: watching udev
# events from inside a container needs host network namespace + real
# netlink access, which is more moving parts for the same outcome. Polling
# a symlink target every few seconds is simple, container-native, and the
# added latency (WATCH_INTERVAL) is irrelevant next to how long a board
# reboot + re-enumeration + firmware boot already takes.
set -euo pipefail

SYMLINK="${WATCH_SYMLINK:-/watch-dev/link101-zenoh}"
CONTAINER="${WATCH_CONTAINER:-zenoh_router}"
INTERVAL="${WATCH_INTERVAL:-3}"

log() { echo "[zenoh-watcher] $(date -Iseconds) $*"; }

log "watching $SYMLINK, will restart $CONTAINER on target change, every ${INTERVAL}s"

last=""
while true; do
  target="$(readlink -f "$SYMLINK" 2>/dev/null || true)"

  if [ -n "$target" ]; then
    if [ -z "$last" ]; then
      # First observation since this watcher started — just the baseline,
      # not something to react to (zenoh_router presumably started against
      # whatever this already resolves to).
      log "$SYMLINK -> $target (baseline)"
      last="$target"
    elif [ "$target" != "$last" ]; then
      log "$SYMLINK now -> $target (was $last), restarting $CONTAINER"
      if docker restart "$CONTAINER" >/dev/null 2>&1; then
        log "$CONTAINER restarted"
        last="$target"
      else
        # Don't update `last` on failure (container not up yet, docker.sock
        # hiccup, ...) — keep retrying every interval until it succeeds,
        # rather than silently giving up on this change.
        log "restart of $CONTAINER failed, will retry"
      fi
    fi
  else
    log "$SYMLINK not present"
  fi

  sleep "$INTERVAL"
done
