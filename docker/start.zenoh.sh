#!/usr/bin/env bash
set -euo pipefail
if [[ "${BASE101_ENABLE_HARDWARE:-false}" != true ]]; then
    echo 'Hardware serial router disabled'; exec sleep infinity
fi
serial=${BASE101_ZENOH_DEVICE:?Set BASE101_ZENOH_DEVICE to the board serial interface}
[[ "$serial" == /dev/* && "$serial" != *'"'* && "$serial" != *$'\n'* ]] || { echo 'Invalid serial path' >&2; exit 2; }
[[ -c "$serial" ]] || { echo "Serial device missing: $serial" >&2; exit 1; }
printf '{mode:"router",listen:{endpoints:["serial/%s#baudrate=921600","tcp/[::]:7447"]}}\n' "$serial" > /tmp/zenoh.json5
exec /usr/local/bin/zenohd -c /tmp/zenoh.json5
