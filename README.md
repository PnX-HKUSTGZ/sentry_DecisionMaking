# sentry_DecisionMaking

ROS 2 Humble decision package for sentry robot BehaviorTree control.

Detailed setup and reproducible runbook:
- `src/README.md`

Quick start:

```bash
cd sentry_DecisionMaking
bash tools/setup_behaviortree_ros2_dep.sh
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
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
