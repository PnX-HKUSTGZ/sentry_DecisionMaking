#include "robot_decision/NavigateToPose.hpp"
#include "robot_decision/IfHealth.hpp"       
#include "robot_decision/IfHealthChanged.hpp" 
#include "robot_decision/Wait.hpp"
#include "robot_decision/check_our_base.hpp"
#include "robot_decision/check_our_outpost.hpp"
#include "robot_decision/check_enemy_outpost.hpp"
// New subscriber includes
#include "robot_decision/ifhealth_subscriber.hpp"
#include "robot_decision/our_base_health_subscriber.hpp"
#include "robot_decision/our_outpost_health_subscriber.hpp"
#include "robot_decision/enemy_outpost_health_subscriber.hpp"
#include "robot_decision/set_bool_node.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <signal.h>
#include <atomic>
#include <iostream>
#include <algorithm>
#include <vector>
#include <sstream>

//safe shutdown
std::atomic<bool> g_interrupt_requested(false);

// Signal handler for Ctrl+C
void signalHandler(int signum) {
  g_interrupt_requested = true;
  std::cout << "Interrupt received, safely shutting down..." << std::endl;
}

int main(int argc, char** argv)
{
  // Register signal handler
  signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);

  const std::string package_share_dir =
    ament_index_cpp::get_package_share_directory("robot_decision");

  auto resolve_to_absolute_path = [&](const std::string& path) -> std::string {
    if (path.empty()) {
      return {};
    }
    std::filesystem::path p(path);
    if (p.is_absolute()) {
      return p.string();
    }
    return (std::filesystem::path(package_share_dir) / p).string();
  };

  auto list_yaml_keys = [](const YAML::Node& map_node) -> std::string {
    if (!map_node || !map_node.IsMap()) {
      return "<none>";
    }
    std::vector<std::string> keys;
    keys.reserve(map_node.size());
    for (const auto& item : map_node) {
      keys.push_back(item.first.as<std::string>());
    }
    std::sort(keys.begin(), keys.end());
    std::ostringstream oss;
    for (size_t i = 0; i < keys.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << keys[i];
    }
    return oss.str();
  };

  // Configuration node: parameters can be set directly from launch.
  auto config_nh = std::make_shared<rclcpp::Node>("robot_decision");
  const std::string default_profile_config =
    (std::filesystem::path(package_share_dir) / "config" / "decision_profiles.yaml").string();

  const std::string profile_config_path =
    config_nh->declare_parameter<std::string>("profile_config_path", default_profile_config);
  const std::string requested_map_profile =
    config_nh->declare_parameter<std::string>("map_profile", "");
  const std::string requested_strategy_profile =
    config_nh->declare_parameter<std::string>("strategy_profile", "");
  const std::string bt_xml_override =
    config_nh->declare_parameter<std::string>("bt_xml_override", "");
  const std::string points_yaml_override =
    config_nh->declare_parameter<std::string>("points_yaml_override", "");
  const std::string target_frame_override =
    config_nh->declare_parameter<std::string>("target_frame_id", "");
  const bool use_sim_time =
    config_nh->declare_parameter<bool>("use_sim_time", true);

  auto apply_use_sim_time = [use_sim_time](const rclcpp::Node::SharedPtr& node) {
    if (!node->has_parameter("use_sim_time")) {
      node->declare_parameter<bool>("use_sim_time", use_sim_time);
    }
    node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
  };
  apply_use_sim_time(config_nh);

  YAML::Node profile_root;
  try {
    profile_root = YAML::LoadFile(profile_config_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(config_nh->get_logger(),
                 "Failed to load profile config: %s, error: %s",
                 profile_config_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  const std::string default_map_profile =
    profile_root["default_map_profile"] ? profile_root["default_map_profile"].as<std::string>() : "";
  const std::string default_strategy_profile =
    profile_root["default_strategy_profile"] ? profile_root["default_strategy_profile"].as<std::string>() : "";

  const std::string selected_map_profile =
    requested_map_profile.empty() ? default_map_profile : requested_map_profile;
  const std::string selected_strategy_profile =
    requested_strategy_profile.empty() ? default_strategy_profile : requested_strategy_profile;

  YAML::Node map_profiles = profile_root["map_profiles"];
  YAML::Node strategy_profiles = profile_root["strategy_profiles"];

  if (selected_map_profile.empty() || !map_profiles || !map_profiles[selected_map_profile]) {
    RCLCPP_ERROR(config_nh->get_logger(),
                 "Invalid map_profile '%s'. Available map profiles: %s",
                 selected_map_profile.c_str(), list_yaml_keys(map_profiles).c_str());
    rclcpp::shutdown();
    return 1;
  }
  if (selected_strategy_profile.empty() || !strategy_profiles || !strategy_profiles[selected_strategy_profile]) {
    RCLCPP_ERROR(config_nh->get_logger(),
                 "Invalid strategy_profile '%s'. Available strategy profiles: %s",
                 selected_strategy_profile.c_str(), list_yaml_keys(strategy_profiles).c_str());
    rclcpp::shutdown();
    return 1;
  }

  const YAML::Node map_profile = map_profiles[selected_map_profile];
  const YAML::Node strategy_profile = strategy_profiles[selected_strategy_profile];

  const std::string bt_xml_raw = bt_xml_override.empty()
    ? strategy_profile["bt_xml"].as<std::string>("")
    : bt_xml_override;
  const std::string points_yaml_raw = points_yaml_override.empty()
    ? map_profile["points_yaml"].as<std::string>("")
    : points_yaml_override;

  std::string target_frame_id = target_frame_override.empty()
    ? map_profile["target_frame_id"].as<std::string>("map")
    : target_frame_override;
  if (target_frame_id.empty()) {
    target_frame_id = "map";
  }

  const std::string bt_xml_path = resolve_to_absolute_path(bt_xml_raw);
  const std::string points_yaml_path = resolve_to_absolute_path(points_yaml_raw);

  if (bt_xml_path.empty() || !std::filesystem::exists(bt_xml_path)) {
    RCLCPP_ERROR(config_nh->get_logger(), "Behavior tree XML does not exist: %s", bt_xml_path.c_str());
    rclcpp::shutdown();
    return 1;
  }
  if (points_yaml_path.empty() || !std::filesystem::exists(points_yaml_path)) {
    RCLCPP_ERROR(config_nh->get_logger(), "Points YAML does not exist: %s", points_yaml_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(config_nh->get_logger(), "Using map_profile: %s", selected_map_profile.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "Using strategy_profile: %s", selected_strategy_profile.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "Behavior tree XML: %s", bt_xml_path.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "Points YAML: %s", points_yaml_path.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "Target frame: %s", target_frame_id.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "use_sim_time: %s", use_sim_time ? "true" : "false");

  //initiate node
  auto navigate_to_pose_nh = std::make_shared<rclcpp::Node>("navigate_to_pose_client");
  apply_use_sim_time(navigate_to_pose_nh);
  navigate_to_pose_nh->declare_parameter<std::string>("points_yaml_path", points_yaml_path);
  navigate_to_pose_nh->declare_parameter<std::string>("target_frame_id", target_frame_id);
  RosNodeParams navigate_to_pose_params;
  // lengthen the time of waiting for the action server to confirm the goal
  navigate_to_pose_params.server_timeout = std::chrono::milliseconds(5000);
  navigate_to_pose_params.nh = navigate_to_pose_nh;
  navigate_to_pose_params.default_port_value = "navigate_to_pose";

  auto set_bool_nh = std::make_shared<rclcpp::Node>("SetBool_client");
  apply_use_sim_time(set_bool_nh);
  RosNodeParams set_bool_params;
  set_bool_params.nh = set_bool_nh;
  set_bool_params.default_port_value = "set_bool";

  auto ifhealth_sub_nh = std::make_shared<rclcpp::Node>("IfHealthSub_subscriber");
  apply_use_sim_time(ifhealth_sub_nh);
  RosNodeParams ifhealth_sub_params;
  ifhealth_sub_params.nh = ifhealth_sub_nh;
  ifhealth_sub_params.default_port_value = "ifhealth";

  auto our_base_health_sub_nh = std::make_shared<rclcpp::Node>("OurBaseHealthSub_subscriber");
  apply_use_sim_time(our_base_health_sub_nh);
  RosNodeParams our_base_health_sub_params;
  our_base_health_sub_params.nh = our_base_health_sub_nh;
  our_base_health_sub_params.default_port_value = "our_base_health";

  auto our_outpost_health_sub_nh = std::make_shared<rclcpp::Node>("OurOutpostHealthSub_subscriber");
  apply_use_sim_time(our_outpost_health_sub_nh);
  RosNodeParams our_outpost_health_sub_params;
  our_outpost_health_sub_params.nh = our_outpost_health_sub_nh;
  our_outpost_health_sub_params.default_port_value = "our_outpost_health";

  auto enemy_outpost_health_sub_nh = std::make_shared<rclcpp::Node>("EnemyOutpostHealthSub_subscriber");
  apply_use_sim_time(enemy_outpost_health_sub_nh);
  RosNodeParams enemy_outpost_health_sub_params;
  enemy_outpost_health_sub_params.nh = enemy_outpost_health_sub_nh;
  enemy_outpost_health_sub_params.default_port_value = "enemy_outpost_health";

  //register nodes
  BehaviorTreeFactory factory;
  factory.registerNodeType<NavigateToPoseBT>("NavigateToPose", navigate_to_pose_params);
  factory.registerNodeType<SetBoolService>("Reloading", set_bool_params);

  // Register the new subscriber nodes
  factory.registerNodeType<robot_decision::IfHealthSubscriber>("IfHealthSubscriber", ifhealth_sub_params);
  factory.registerNodeType<robot_decision::OurBaseHealthSubscriber>("OurBaseHealthSubscriber", our_base_health_sub_params);
  factory.registerNodeType<robot_decision::OurOutpostHealthSubscriber>("OurOutpostHealthSubscriber", our_outpost_health_sub_params);
  factory.registerNodeType<robot_decision::EnemyOutpostHealthSubscriber>("EnemyOutpostHealthSubscriber", enemy_outpost_health_sub_params);
  
  // Register the modified action nodes (now regular nodes, not subscribers)
  factory.registerNodeType<robot_decision::IfHealth>("IfHealth");
  factory.registerNodeType<robot_decision::Wait>("Wait");
  factory.registerNodeType<robot_decision::CheckOutbase>("CheckOurBase");
  factory.registerNodeType<robot_decision::CheckOutposet>("CheckOurOutpost");
  factory.registerNodeType<robot_decision::CheckEnemyposet>("CheckEnemyOutpost");
  

  auto tree = factory.createTreeFromFile(bt_xml_path);
  
  BT::StdCoutLogger logger(tree);

  //运行行为树
  auto status = tree.tickOnce();
  std::cout << "--- status: " << toStr(status) << "\n\n";
  while(!g_interrupt_requested) 
  {
    // Sleep to avoid busy loops.
    // do NOT use other sleep functions!
    // Small sleep time is OK, here we use a large one only to
    // have less messages on the console.
    tree.sleep(std::chrono::milliseconds(500));

    //std::cout << "--- ticking\n";
    status = tree.tickOnce();
    //std::cout << "--- status: " << toStr(status) << "\n\n";
  }
  
  // Clean shutdown
  if (g_interrupt_requested) {
    std::cout << "Halting behavior tree..." << std::endl;
    tree.haltTree();
  }
  
  // Clean up ROS resources
  rclcpp::shutdown();
  std::cout << "Behavior tree execution completed." << std::endl;

  return 0;
}
