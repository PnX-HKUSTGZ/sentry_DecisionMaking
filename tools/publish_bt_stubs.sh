#!/usr/bin/env bash
set -euo pipefail

# Publish mandatory BT input topics to avoid RMUC.xml short-circuit at startup.
# Ctrl+C to stop all publishers.

HEALTH="${HEALTH:-500}"
OUR_OUTPOST_HEALTH="${OUR_OUTPOST_HEALTH:-100}"
ENEMY_OUTPOST_HEALTH="${ENEMY_OUTPOST_HEALTH:-100}"
HEALTH_RATE="${HEALTH_RATE:-5}"
OUTPOST_RATE="${OUTPOST_RATE:-2}"

pids=()

cleanup() {
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

echo "[INFO] /ifhealth=${HEALTH} @ ${HEALTH_RATE}Hz"
ros2 topic pub -r "${HEALTH_RATE}" /ifhealth std_msgs/msg/UInt16 "{data: ${HEALTH}}" >/dev/null &
pids+=("$!")

echo "[INFO] /our_outpost_health=${OUR_OUTPOST_HEALTH} @ ${OUTPOST_RATE}Hz"
ros2 topic pub -r "${OUTPOST_RATE}" /our_outpost_health std_msgs/msg/UInt16 "{data: ${OUR_OUTPOST_HEALTH}}" >/dev/null &
pids+=("$!")

echo "[INFO] /enemy_outpost_health=${ENEMY_OUTPOST_HEALTH} @ ${OUTPOST_RATE}Hz"
ros2 topic pub -r "${OUTPOST_RATE}" /enemy_outpost_health std_msgs/msg/UInt16 "{data: ${ENEMY_OUTPOST_HEALTH}}" >/dev/null &
pids+=("$!")

wait
