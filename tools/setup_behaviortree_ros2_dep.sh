#!/usr/bin/env bash
set -euo pipefail

# Repro helper: ensure BehaviorTree.ROS2 exists at a known commit.
# Usage:
#   bash tools/setup_behaviortree_ros2_dep.sh

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${REPO_DIR}/src"
DEP_DIR="${SRC_DIR}/BehaviorTree.ROS2"
DEP_URL="https://github.com/BehaviorTree/BehaviorTree.ROS2.git"
DEP_COMMIT="cc31ea7b97947f1aac6e8c37df6cec379c84a7d9"

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

echo "[DONE] Dependency ready at: ${DEP_DIR}"
