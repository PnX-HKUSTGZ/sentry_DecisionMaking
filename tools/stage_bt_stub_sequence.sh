#!/usr/bin/env bash
set -euo pipefail

# Stage decision BT input topics through a deterministic sequence so joint
# RMUL2026 simulation recordings can exercise multiple branches.

PHASE_A_SEC="${PHASE_A_SEC:-18}"
PHASE_B_SEC="${PHASE_B_SEC:-18}"
PHASE_C_SEC="${PHASE_C_SEC:-18}"
HEALTH_RATE="${HEALTH_RATE:-5}"
OUTPOST_RATE="${OUTPOST_RATE:-2}"

pids=()

cleanup() {
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait "${pids[@]:-}" 2>/dev/null || true
  pids=()
}

start_publishers() {
  local health="$1"
  local our_outpost="$2"
  local enemy_outpost="$3"

  cleanup
  echo "[phase] health=${health} our_outpost=${our_outpost} enemy_outpost=${enemy_outpost}"
  ros2 topic pub -r "${HEALTH_RATE}" /ifhealth std_msgs/msg/UInt16 "{data: ${health}}" >/dev/null &
  pids+=("$!")
  ros2 topic pub -r "${OUTPOST_RATE}" /our_outpost_health std_msgs/msg/UInt16 "{data: ${our_outpost}}" >/dev/null &
  pids+=("$!")
  ros2 topic pub -r "${OUTPOST_RATE}" /enemy_outpost_health std_msgs/msg/UInt16 "{data: ${enemy_outpost}}" >/dev/null &
  pids+=("$!")
}

trap cleanup EXIT INT TERM

echo "[info] phase A: both outposts alive -> expect point 1"
start_publishers 500 100 100
sleep "${PHASE_A_SEC}"

echo "[info] phase B: enemy outpost dead -> expect point 4"
start_publishers 500 100 0
sleep "${PHASE_B_SEC}"

echo "[info] phase C: low health -> expect point 2"
start_publishers 100 100 100
sleep "${PHASE_C_SEC}"

echo "[done] staged BT sequence complete"
