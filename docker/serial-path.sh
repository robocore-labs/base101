#!/usr/bin/env bash
# Resolve known stable USB identities without a host /dev bind mount.
set -euo pipefail
wanted=${1:?Expected stable serial path}
[[ "$wanted" != /dev/serial/by-id/* && -c "$wanted" ]] && { printf '%s\n' "$wanted"; exit 0; }
name=${wanted##*/}
for tty in /sys/class/tty/tty*; do
 [[ -e "$tty/device" ]] || continue
 node=$(readlink -f "$tty/device")
 interface=''; serial=''; vendor=''; product=''
 while [[ "$node" != / && "$node" != /sys ]]; do
  [[ ! -f "$node/bInterfaceNumber" ]] || interface=$(cat "$node/bInterfaceNumber")
  [[ ! -f "$node/serial" ]] || serial=$(cat "$node/serial")
  [[ ! -f "$node/idVendor" ]] || vendor=$(cat "$node/idVendor")
  [[ ! -f "$node/idProduct" ]] || product=$(cat "$node/idProduct")
  [[ -z "$vendor" ]] || break
  node=${node%/*}
 done
 match=false
 if [[ "$vendor:$product" == 1209:ac01 && "$name" == "usb-RoboCore_Link101_ROS_Node_${serial}-if${interface}" ]]; then
  match=true
 elif [[ "$vendor:$product:$interface" == 1a86:7523:00 && "$name" == usb-1a86_USB_Serial-if00-port0 ]]; then
  match=true
 fi
 if [[ "$match" == true && -c "/dev/${tty##*/}" ]]; then
  [[ "$wanted" == /dev/serial/by-id/* ]] || { echo 'Unsupported stable serial path' >&2; exit 2; }
  mkdir -p /dev/serial/by-id
  ln -sfn "/dev/${tty##*/}" "$wanted"
  printf '%s\n' "$wanted"
  exit 0
 fi
done
echo "USB device not present: $wanted" >&2
exit 1
