# sentry_DecisionMaking

Ubuntu 24.04 / ROS 2 Jazzy decision package for sentry robot BehaviorTree control.

Detailed setup and reproducible runbook:
- `src/README.md`
- `docs/RMUL26_REPRO_RELEASE.md` (cross-repo pinned release note)

Quick start:

```bash
cd sentry_DecisionMaking
bash tools/setup_behaviortree_ros2_dep.sh
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

Run decision node (RMUL_26 profile):

```bash
ros2 launch robot_decision decision_node_launch.py \
  map_profile:=RMUL_26 \
  strategy_profile:=vp_center_control \
  use_sim_time:=true
```

RMUL_26 points are now maintained in:
- `src/robot_decision/behavior_trees/RMUL26_point.yaml`

Important:
- If the tree appears idle in simulation, publish BT stub topics first:

```bash
bash tools/publish_bt_stubs.sh
```

- Full reproducible details are in `src/README.md`.
