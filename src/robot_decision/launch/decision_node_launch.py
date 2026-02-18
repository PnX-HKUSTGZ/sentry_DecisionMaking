from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = get_package_share_directory("robot_decision")
    default_profile = f"{pkg_share}/config/decision_profiles.yaml"

    profile_config_arg = DeclareLaunchArgument(
        "profile_config_path",
        default_value=default_profile,
        description="Path to decision profile config YAML",
    )
    map_profile_arg = DeclareLaunchArgument(
        "map_profile",
        default_value="",
        description="Map profile key in decision_profiles.yaml. Empty uses default_map_profile.",
    )
    strategy_profile_arg = DeclareLaunchArgument(
        "strategy_profile",
        default_value="",
        description="Strategy profile key in decision_profiles.yaml. Empty uses default_strategy_profile.",
    )
    bt_xml_override_arg = DeclareLaunchArgument(
        "bt_xml_override",
        default_value="",
        description="Optional override for BT XML file path.",
    )
    points_yaml_override_arg = DeclareLaunchArgument(
        "points_yaml_override",
        default_value="",
        description="Optional override for points YAML file path.",
    )
    target_frame_id_arg = DeclareLaunchArgument(
        "target_frame_id",
        default_value="",
        description="Optional override for goal frame_id. Empty uses map profile value.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Set true when running with Gazebo/Nav2 simulation clock.",
    )

    robot_decision_node = Node(
        name="robot_decision",
        package="robot_decision",
        executable="robot_decision",
        output="screen",
        parameters=[{
            "profile_config_path": LaunchConfiguration("profile_config_path"),
            "map_profile": LaunchConfiguration("map_profile"),
            "strategy_profile": LaunchConfiguration("strategy_profile"),
            "bt_xml_override": LaunchConfiguration("bt_xml_override"),
            "points_yaml_override": LaunchConfiguration("points_yaml_override"),
            "target_frame_id": LaunchConfiguration("target_frame_id"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }]
    )

    ld.add_action(profile_config_arg)
    ld.add_action(map_profile_arg)
    ld.add_action(strategy_profile_arg)
    ld.add_action(bt_xml_override_arg)
    ld.add_action(points_yaml_override_arg)
    ld.add_action(target_frame_id_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_decision_node)

    return ld
