# sentry_DecisionMaking Reproducible Guide (ROS 2 Humble)

This repository contains `robot_decision` and its BehaviorTree runtime dependency.
The commands below are tested for RMUL_26 simulation with Nav2 `NavigateToPose`.

## 1. Dependency Pinning

`robot_decision` depends on `BehaviorTree.ROS2`.

Pinned commit (recommended):
- `cc31ea7b97947f1aac6e8c37df6cec379c84a7d9`

Use helper script:

```bash
cd sentry_DecisionMaking
bash tools/setup_behaviortree_ros2_dep.sh
```

This ensures `src/BehaviorTree.ROS2` exists and is pinned when the folder is a git repository.

## 2. Build

### 2.1 System ROS 2 Humble

```bash
cd sentry_DecisionMaking
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

### 2.2 RoboStack

If navigation runs in RoboStack, source the same environment first, then:

```bash
cd sentry_DecisionMaking
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

## 3. Runtime Profiles (multi-map / multi-strategy)

Profile config file:
- `src/robot_decision/config/decision_profiles.yaml`

Default profiles:
- `default_map_profile: RMUL26`
- `default_strategy_profile: vp_center_control`

Map profiles currently provided:
- `RMUL26`
- `RMUL_26` (alias)
- `GENERIC`

Strategy profiles currently provided:
- `vp_center_control` (recommended for RMUL_26 VP center control)
- `rmuc_regression`
- `simple_health`
- `test_minimal`

## 4. Launch (with use_sim_time)

```bash
cd sentry_DecisionMaking
source install/setup.bash
ros2 launch robot_decision decision_node_launch.py \
  map_profile:=RMUL_26 \
  strategy_profile:=vp_center_control \
  use_sim_time:=true
```

Launch arguments:
- `profile_config_path`: path to profile YAML
- `map_profile`: key in `map_profiles`
- `strategy_profile`: key in `strategy_profiles`
- `bt_xml_override`: optional direct BT XML path
- `points_yaml_override`: optional direct points YAML path
- `target_frame_id`: optional override, default from map profile (`map`)
- `use_sim_time`: set `true` when running Gazebo/Nav2 sim clock

## 5. Mandatory Topic Stubs (important)

`RMUC.xml` starts with 3 subscriber checks inside an inverter.
If these topics have no messages, the tree can short-circuit and skip navigation actions.

Publish stubs with one command:

```bash
cd sentry_DecisionMaking
source install/setup.bash
bash tools/publish_bt_stubs.sh
```

Equivalent manual commands:

```bash
ros2 topic pub -r 5 /ifhealth std_msgs/msg/UInt16 "{data: 500}"
ros2 topic pub -r 2 /our_outpost_health std_msgs/msg/UInt16 "{data: 100}"
ros2 topic pub -r 2 /enemy_outpost_health std_msgs/msg/UInt16 "{data: 100}"
```

## 6. Joint Validation Checklist (with navigation repo)

After Nav2 bringup is up:

```bash
ros2 action list | grep navigate_to_pose
ros2 action info /navigate_to_pose
ros2 topic echo -n 1 /amcl_pose
```

Then run decision + stubs and check `NavigateToPose` goals keep being produced.

## 7. Files You Usually Tune

- Behavior tree logic: `src/robot_decision/behavior_trees/RMUC.xml`
- RMUL_26 points: `src/robot_decision/behavior_trees/RMUL26_point.yaml`
- Profile mapping: `src/robot_decision/config/decision_profiles.yaml`
- Launch args: `src/robot_decision/launch/decision_node_launch.py`
