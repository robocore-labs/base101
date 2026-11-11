#!/usr/bin/env bash
set -euo pipefail
if [[ "${BASE101_ENABLE_HARDWARE:-false}" != true ]]; then
    echo 'Hardware serial router disabled'; exec sleep infinity
fi
serial=${BASE101_ZENOH_DEVICE:?Set BASE101_ZENOH_DEVICE to the board serial interface}
[[ "$serial" == /dev/* && "$serial" != *'"'* && "$serial" != *$'\n'* ]] || { echo 'Invalid serial path' >&2; exit 2; }
serial=$(/usr/local/bin/base101-serial-path "$serial")
printf '{mode:"router",listen:{endpoints:["serial/%s#baudrate=921600","tcp/[::]:7447"]}}\n' "$serial" > /tmp/zenoh.json5
# The serial listener may remain alive after USB EOF; restart on re-enumeration.
identity() {
 local node target
 target=$(readlink -f "$serial")
 node=$(readlink -f "/sys/class/tty/${target##*/}/device")
 while [[ "$node" != / && "$node" != /sys ]]; do
  if [[ -f "$node/devnum" ]]; then
   printf '%s:%s\n' "$target" "$(cat "$node/devnum")"; return
  fi
  node=${node%/*}
 done
 return 1
}
original=$(identity)
/usr/local/bin/zenohd -c /tmp/zenoh.json5 &
router_pid=$!
cleanup() { kill -TERM "$router_pid" 2>/dev/null || true; wait "$router_pid" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 0' TERM INT
while kill -0 "$router_pid" 2>/dev/null; do
 sleep 2
 /usr/local/bin/base101-serial-path "$serial" >/dev/null || exit 1
 current=$(identity) || exit 1
 if [[ "$current" != "$original" ]]; then
  echo 'Link101 USB re-enumerated; restarting router' >&2
  exit 1
 fi
done
wait "$router_pid"
