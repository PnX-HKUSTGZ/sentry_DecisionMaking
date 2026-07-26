#!/usr/bin/env bash
set -euo pipefail

# Repro helper: ensure the BehaviorTree.ROS2 version validated on ROS 2 Jazzy exists.
# Usage:
#   bash tools/setup_behaviortree_ros2_dep.sh

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${REPO_DIR}/src"
DEP_DIR="${SRC_DIR}/BehaviorTree.ROS2"
DEP_URL="https://github.com/BehaviorTree/BehaviorTree.ROS2.git"
DEP_COMMIT="6c6aa078ee7bc52fec98984bed4964556abf5beb"
JAZZY_PATCH="${REPO_DIR}/tools/behaviortree_ros2_jazzy_cancel_goal.patch"

if [[ ! -d "${SRC_DIR}" ]]; then
  echo "[ERROR] Missing src directory: ${SRC_DIR}" >&2
  exit 1
fi

if [[ -d "${DEP_DIR}/.git" ]]; then
  echo "[INFO] BehaviorTree.ROS2 already exists, checking out pinned commit ${DEP_COMMIT}"
  git -C "${DEP_DIR}" fetch --all --tags
  git -C "${DEP_DIR}" checkout "${DEP_COMMIT}"
elif [[ -d "${DEP_DIR}" ]]; then
  echo "[WARN] ${DEP_DIR} exists but is not a git repo. Keeping local copy as-is."
  echo "[WARN] Ensure it matches commit ${DEP_COMMIT} manually if strict reproducibility is required."
else
  echo "[INFO] Cloning BehaviorTree.ROS2 into src/"
  git -C "${SRC_DIR}" clone "${DEP_URL}" BehaviorTree.ROS2
  git -C "${DEP_DIR}" checkout "${DEP_COMMIT}"
fi

if git -C "${DEP_DIR}" apply --reverse --check "${JAZZY_PATCH}" 2>/dev/null; then
  echo "[INFO] Jazzy action-cancel race patch is already applied"
elif git -C "${DEP_DIR}" apply --check "${JAZZY_PATCH}"; then
  git -C "${DEP_DIR}" apply "${JAZZY_PATCH}"
  echo "[INFO] Applied Jazzy action-cancel race patch"
else
  echo "[ERROR] BehaviorTree.ROS2 Jazzy patch does not apply cleanly" >&2
  exit 1
fi

echo "[DONE] Dependency ready at: ${DEP_DIR}"
