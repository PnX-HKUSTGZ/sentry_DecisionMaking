# RMUL_26 Repro Release (Decision + Navigation Pair)

Release date: 2026-02-18

This release note pins the cross-repo pair validated for RMUL_26 simulation.

## Pinned commits

- Decision repo: `924b5dc19ef7e16646dca32bb3b9e826447cde28`
- Navigation repo: `b93f3d4f6ed2a5b343b5555d47e102e34aeadcea`

## Runtime profile

- Launch file: `src/robot_decision/launch/decision_node_launch.py`
- Default strategy profile: `vp_center_control`
- Map profile: `RMUL_26` (alias of RMUL26 points config)
- Goal frame: `map`
- `use_sim_time`: `true`

## Mandatory BT input topics

This tree requires continuous data on:

- `/ifhealth` (`std_msgs/msg/UInt16`)
- `/our_outpost_health` (`std_msgs/msg/UInt16`)
- `/enemy_outpost_health` (`std_msgs/msg/UInt16`)

Use helper script:

```bash
cd sentry_DecisionMaking
source /opt/ros/humble/setup.bash
source install/setup.bash
bash tools/publish_bt_stubs.sh
```

## Decision startup command

```bash
cd sentry_DecisionMaking
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_decision decision_node_launch.py \
  map_profile:=RMUL_26 \
  strategy_profile:=vp_center_control \
  use_sim_time:=true
```

## Build dependency pin

BehaviorTree dependency pin helper:

```bash
cd sentry_DecisionMaking
bash tools/setup_behaviortree_ros2_dep.sh
```

Pinned commit in helper:

- `BehaviorTree.ROS2@cc31ea7b97947f1aac6e8c37df6cec379c84a7d9`
