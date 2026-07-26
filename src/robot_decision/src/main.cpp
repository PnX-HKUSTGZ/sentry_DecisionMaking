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
#include "robot_decision/can_rebuild_outpost_subscriber.hpp"
#include "robot_decision/remain_ammo_subscriber.hpp"
#include "robot_decision/set_bool_node.hpp"
#include "robot_decision/decision_conditions.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <vector>
#include <sstream>

namespace
{
std::atomic_bool interrupt_requested{false};

void handleSignal(int)
{
  interrupt_requested.store(true);
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

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
  const std::string pre_match_yaml_raw =
    config_nh->declare_parameter<std::string>("pre_match_yaml_path", "config/decision_profiles.yaml");
  const std::string target_frame_override =
    config_nh->declare_parameter<std::string>("target_frame_id", "");
  const int navigate_to_pose_server_timeout_ms =
    config_nh->declare_parameter<int>("navigate_to_pose_server_timeout_ms", 20000);
  bool use_sim_time = false;
  try {
    use_sim_time = config_nh->declare_parameter<bool>("use_sim_time", false);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
    use_sim_time = config_nh->get_parameter("use_sim_time").as_bool();
  }

  auto apply_use_sim_time = [use_sim_time](const rclcpp::Node::SharedPtr& node) {
    try {
      if (!node->has_parameter("use_sim_time")) {
        node->declare_parameter<bool>("use_sim_time", use_sim_time);
      }
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
      // Launch/parameter overrides may predeclare this parameter for every
      // node created in-process. In that case we only need to set it.
    }
    node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
  };
  apply_use_sim_time(config_nh);

  const auto make_auxiliary_node = [&](const std::string& name) {
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    return std::make_shared<rclcpp::Node>(name, config_nh->get_namespace(), options);
  };

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
  const std::string pre_match_yaml_path = resolve_to_absolute_path(pre_match_yaml_raw);

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
  if (!pre_match_yaml_path.empty() && !std::filesystem::exists(pre_match_yaml_path)) {
    RCLCPP_ERROR(config_nh->get_logger(), "Pre-match YAML does not exist: %s", pre_match_yaml_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(config_nh->get_logger(), "Using map_profile: %s", selected_map_profile.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "Using strategy_profile: %s", selected_strategy_profile.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "Behavior tree XML: %s", bt_xml_path.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "Points YAML: %s", points_yaml_path.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "Pre-match YAML: %s", pre_match_yaml_path.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "Target frame: %s", target_frame_id.c_str());
  RCLCPP_INFO(config_nh->get_logger(), "use_sim_time: %s", use_sim_time ? "true" : "false");

  //initiate node
  auto navigate_to_pose_nh = make_auxiliary_node("navigate_to_pose_client");
  apply_use_sim_time(navigate_to_pose_nh);
  navigate_to_pose_nh->declare_parameter<std::string>("points_yaml_path", points_yaml_path);
  navigate_to_pose_nh->declare_parameter<std::string>("target_frame_id", target_frame_id);
  RosNodeParams navigate_to_pose_params;
  // Joint demo startup can overlap with Nav2 lifecycle and controller reconfiguration.
  // Keep this timeout configurable so decision does not misclassify transient startup
  // or replanning latency as SEND_GOAL_TIMEOUT.
  navigate_to_pose_params.server_timeout = std::chrono::milliseconds(
    std::max(navigate_to_pose_server_timeout_ms, 1000));
  navigate_to_pose_params.nh = navigate_to_pose_nh;
  navigate_to_pose_params.default_port_value = "navigate_to_pose";

  auto set_bool_nh = make_auxiliary_node("SetBool_client");
  apply_use_sim_time(set_bool_nh);
  RosNodeParams set_bool_params;
  set_bool_params.nh = set_bool_nh;
  set_bool_params.default_port_value = "set_bool";
  set_bool_params.server_timeout = std::chrono::milliseconds(3000);
  set_bool_params.wait_for_server_timeout = std::chrono::milliseconds(3000);

  auto follow_mark_nh = make_auxiliary_node("FollowMark_client");
  apply_use_sim_time(follow_mark_nh);
  RosNodeParams follow_mark_params;
  follow_mark_params.nh = follow_mark_nh;
  follow_mark_params.default_port_value = "/change_follow_mark";
  follow_mark_params.server_timeout = std::chrono::milliseconds(3000);
  follow_mark_params.wait_for_server_timeout = std::chrono::milliseconds(3000);

  auto ifhealth_sub_nh = make_auxiliary_node("IfHealthSub_subscriber");
  apply_use_sim_time(ifhealth_sub_nh);
  RosNodeParams ifhealth_sub_params;
  ifhealth_sub_params.nh = ifhealth_sub_nh;
  ifhealth_sub_params.default_port_value = "ifhealth";

  auto our_base_health_sub_nh = make_auxiliary_node("OurBaseHealthSub_subscriber");
  apply_use_sim_time(our_base_health_sub_nh);
  RosNodeParams our_base_health_sub_params;
  our_base_health_sub_params.nh = our_base_health_sub_nh;
  our_base_health_sub_params.default_port_value = "our_base_health";

  auto our_outpost_health_sub_nh = make_auxiliary_node("OurOutpostHealthSub_subscriber");
  apply_use_sim_time(our_outpost_health_sub_nh);
  RosNodeParams our_outpost_health_sub_params;
  our_outpost_health_sub_params.nh = our_outpost_health_sub_nh;
  our_outpost_health_sub_params.default_port_value = "our_outpost_health";

  auto enemy_outpost_health_sub_nh = make_auxiliary_node("EnemyOutpostHealthSub_subscriber");
  apply_use_sim_time(enemy_outpost_health_sub_nh);
  RosNodeParams enemy_outpost_health_sub_params;
  enemy_outpost_health_sub_params.nh = enemy_outpost_health_sub_nh;
  enemy_outpost_health_sub_params.default_port_value = "enemy_outpost_health";

  auto can_rebuild_outpost_sub_nh = make_auxiliary_node("CanRebuildOutpostSub_subscriber");
  apply_use_sim_time(can_rebuild_outpost_sub_nh);
  RosNodeParams can_rebuild_outpost_sub_params;
  can_rebuild_outpost_sub_params.nh = can_rebuild_outpost_sub_nh;
  can_rebuild_outpost_sub_params.default_port_value = "/can_rebuild_outpost";

  auto remain_ammo_sub_nh = make_auxiliary_node("RemainAmmoSub_subscriber");
  apply_use_sim_time(remain_ammo_sub_nh);
  RosNodeParams remain_ammo_sub_params;
  remain_ammo_sub_params.nh = remain_ammo_sub_nh;
  remain_ammo_sub_params.default_port_value = "/remain_ammo";

  auto configured_bool_nh = make_auxiliary_node("ConfiguredBool_condition");
  apply_use_sim_time(configured_bool_nh);
  configured_bool_nh->declare_parameter<std::string>("pre_match_yaml_path", pre_match_yaml_path);
  RosNodeParams configured_bool_params;
  configured_bool_params.nh = configured_bool_nh;

  //register nodes
  BehaviorTreeFactory factory;
  factory.registerNodeType<NavigateToPoseBT>("NavigateToPose", navigate_to_pose_params);
  factory.registerNodeType<SetBoolService>("Reloading", set_bool_params);
  factory.registerNodeType<SetBoolService>("SetFollowMark", follow_mark_params);

  // Register the new subscriber nodes
  factory.registerNodeType<robot_decision::IfHealthSubscriber>("IfHealthSubscriber", ifhealth_sub_params);
  factory.registerNodeType<robot_decision::OurBaseHealthSubscriber>("OurBaseHealthSubscriber", our_base_health_sub_params);
  factory.registerNodeType<robot_decision::OurOutpostHealthSubscriber>("OurOutpostHealthSubscriber", our_outpost_health_sub_params);
  factory.registerNodeType<robot_decision::EnemyOutpostHealthSubscriber>("EnemyOutpostHealthSubscriber", enemy_outpost_health_sub_params);
  factory.registerNodeType<robot_decision::CanRebuildOutpostSubscriber>("CanRebuildOutpostSubscriber", can_rebuild_outpost_sub_params);
  factory.registerNodeType<robot_decision::RemainAmmoSubscriber>("RemainAmmoSubscriber", remain_ammo_sub_params);
  
  // Register the modified action nodes (now regular nodes, not subscribers)
  factory.registerNodeType<robot_decision::IfHealth>("IfHealth");
  factory.registerNodeType<robot_decision::Wait>("Wait");
  factory.registerNodeType<robot_decision::CheckOutbase>("CheckOurBase");
  factory.registerNodeType<robot_decision::CheckOutposet>("CheckOurOutpost");
  factory.registerNodeType<robot_decision::CheckEnemyposet>("CheckEnemyOutpost");
  factory.registerNodeType<robot_decision::CheckBool>("CheckBool");
  factory.registerNodeType<robot_decision::CheckUInt16>("CheckUInt16");
  factory.registerNodeType<robot_decision::CheckConfiguredBool>("CheckConfiguredBool", configured_bool_params);
  factory.registerNodeType<robot_decision::CheckConfiguredUInt16>("CheckConfiguredUInt16", configured_bool_params);
  factory.registerNodeType<robot_decision::CheckCurrentPoint>("CheckCurrentPoint");
  factory.registerNodeType<robot_decision::CheckAmmoExchangeAvailable>("CheckAmmoExchangeAvailable");
  

  auto tree = factory.createTreeFromFile(bt_xml_path);
  
  BT::StdCoutLogger logger(tree);

  //运行行为树
  auto status = tree.tickOnce();
  std::cout << "--- status: " << toStr(status) << "\n\n";
  while (!interrupt_requested.load() && rclcpp::ok())
  {
    // Sleep to avoid busy loops.
    // do NOT use other sleep functions!
    // Small sleep time is OK, here we use a large one only to
    // have less messages on the console.
    tree.sleep(std::chrono::milliseconds(500));

    if (interrupt_requested.load() || !rclcpp::ok()) {
      break;
    }

    //std::cout << "--- ticking\n";
    status = tree.tickOnce();
    //std::cout << "--- status: " << toStr(status) << "\n\n";
  }
  
  std::cout << "Halting behavior tree..." << std::endl;
  try {
    tree.haltTree();
  } catch (const std::exception& e) {
    RCLCPP_WARN(config_nh->get_logger(), "Behavior tree halt reported: %s", e.what());
  }
  
  // Clean up ROS resources
  rclcpp::shutdown();
  std::cout << "Behavior tree execution completed." << std::endl;

  return 0;
}
