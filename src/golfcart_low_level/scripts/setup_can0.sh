#!/usr/bin/env bash
set -euo pipefail

CAN_IF="${1:-can0}"
BITRATE="${2:-250000}"

# ros_can_bridge and steering_node both call this at launch. Two concurrent
# reconfigures race and one of them gets "RTNETLINK answers: Device or resource
# busy", so serialize on a lock file.
exec 9>"/tmp/setup_${CAN_IF}.lock"
flock 9

sudo modprobe can || true
sudo modprobe can_raw || true
sudo modprobe can_dev || true
sudo modprobe gs_usb || true

if ! ip link show "${CAN_IF}" >/dev/null 2>&1; then
  echo "CAN interface ${CAN_IF} was not found. Check the USB2CAN adapter and driver."
  exit 1
fi

cur_bitrate="$(ip -details link show "${CAN_IF}" \
  | awk '{for (i = 1; i < NF; i++) if ($i == "bitrate") print $(i + 1)}' \
  | head -n1)"

if ip link show "${CAN_IF}" | grep -q 'state UP' && [ "${cur_bitrate}" = "${BITRATE}" ]; then
  # Already configured correctly — leave the link alone. Bouncing it is not
  # free: while the interface is down the STM32 keeps transmitting 0x131/0x122
  # every 20 ms with nobody left to ACK, which drives its TEC to bus-off. The
  # firmware has no bus-off recovery, so it then stays silent until power-cycled.
  echo "${CAN_IF} already up at ${BITRATE} bps; leaving it untouched."
  ip -details -statistics link show "${CAN_IF}"
  exit 0
fi

sudo ip link set "${CAN_IF}" down || true

# restart-ms asks the driver for automatic bus-off recovery, but not every
# driver implements it -- gs_usb answers "Device doesn't support restart from
# Bus Off" and, under set -e, used to abort this script with the link left down.
if ! sudo ip link set "${CAN_IF}" up type can bitrate "${BITRATE}" restart-ms 100 2>/dev/null; then
  echo "${CAN_IF}: driver rejected restart-ms, bringing up without bus-off auto-recovery."
  sudo ip link set "${CAN_IF}" up type can bitrate "${BITRATE}"
fi

sudo ip link set "${CAN_IF}" txqueuelen 1000
ip -details -statistics link show "${CAN_IF}"
