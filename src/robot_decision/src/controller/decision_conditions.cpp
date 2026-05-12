#include "robot_decision/decision_conditions.hpp"
#include "robot_decision/NavigateToPose.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>
#include <vector>

namespace robot_decision
{

namespace
{

std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::vector<std::string> splitKeyPath(const std::string& key)
{
  std::vector<std::string> parts;
  std::stringstream stream(key);
  std::string item;
  while (std::getline(stream, item, '.')) {
    if (!item.empty()) {
      parts.push_back(item);
    }
  }
  return parts;
}

YAML::Node findByPath(const YAML::Node& root, const std::string& key)
{
  if (!root || key.empty()) {
    return {};
  }

  YAML::Node current = root;
  for (const auto& part : splitKeyPath(key)) {
    if (!current || !current.IsMap() || !current[part]) {
      return {};
    }
    current = current[part];
  }
  return current;
}

bool yamlNodeToBool(const YAML::Node& node, bool& value)
{
  if (!node || !node.IsScalar()) {
    return false;
  }

  try {
    value = node.as<bool>();
    return true;
  } catch (const YAML::Exception&) {
  }

  const std::string raw = toLower(node.as<std::string>());
  if (raw == "true" || raw == "yes" || raw == "on" || raw == "1") {
    value = true;
    return true;
  }
  if (raw == "false" || raw == "no" || raw == "off" || raw == "0") {
    value = false;
    return true;
  }
  return false;
}

bool yamlNodeToUInt16(const YAML::Node& node, uint16_t& value)
{
  if (!node || !node.IsScalar()) {
    return false;
  }

  try {
    const int parsed = node.as<int>();
    if (parsed < 0 || parsed > std::numeric_limits<uint16_t>::max()) {
      return false;
    }
    value = static_cast<uint16_t>(parsed);
    return true;
  } catch (const YAML::Exception&) {
    return false;
  }
}

bool compareUInt16(uint16_t value, uint16_t threshold, const std::string& raw_comparison, bool& matched)
{
  const std::string comparison = toLower(raw_comparison);
  if (comparison == "lt" || comparison == "<") {
    matched = value < threshold;
    return true;
  }
  if (comparison == "le" || comparison == "<=") {
    matched = value <= threshold;
    return true;
  }
  if (comparison == "eq" || comparison == "==" || comparison == "=") {
    matched = value == threshold;
    return true;
  }
  if (comparison == "ge" || comparison == ">=") {
    matched = value >= threshold;
    return true;
  }
  if (comparison == "gt" || comparison == ">") {
    matched = value > threshold;
    return true;
  }
  if (comparison == "ne" || comparison == "!=") {
    matched = value != threshold;
    return true;
  }
  return false;
}

} // namespace

CheckBool::CheckBool(const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
}

BT::PortsList CheckBool::providedPorts()
{
  return {
    BT::InputPort<bool>("value"),
    BT::InputPort<bool>("expected")
  };
}

BT::NodeStatus CheckBool::tick()
{
  bool value = false;
  if (!getInput("value", value)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckBool"), "[%s] Missing value input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  bool expected = true;
  getInput("expected", expected);

  return value == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

CheckUInt16::CheckUInt16(const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
}

BT::PortsList CheckUInt16::providedPorts()
{
  return {
    BT::InputPort<uint16_t>("value"),
    BT::InputPort<uint16_t>("threshold"),
    BT::InputPort<std::string>("comparison")
  };
}

BT::NodeStatus CheckUInt16::tick()
{
  uint16_t value = 0;
  if (!getInput("value", value)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckUInt16"), "[%s] Missing value input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  uint16_t threshold = 0;
  if (!getInput("threshold", threshold)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckUInt16"), "[%s] Missing threshold input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::string comparison = "eq";
  getInput("comparison", comparison);
  comparison = toLower(comparison);

  bool matched = false;
  if (!compareUInt16(value, threshold, comparison, matched)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckUInt16"), "[%s] Unknown comparison: %s",
                 name().c_str(), comparison.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return matched ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

CheckConfiguredBool::CheckConfiguredBool(
  const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
  : BT::SyncActionNode(name, conf)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw BT::RuntimeError("CheckConfiguredBool requires a valid ROS node");
  }

  if (!node_->has_parameter("pre_match_yaml_path")) {
    node_->declare_parameter<std::string>("pre_match_yaml_path", "");
  }
  yaml_path_ = node_->get_parameter("pre_match_yaml_path").as_string();

  try {
    if (yaml_path_.empty()) {
      RCLCPP_WARN(logger(), "pre_match_yaml_path is empty");
      return;
    }
    config_ = YAML::LoadFile(yaml_path_);
    config_loaded_ = true;
    RCLCPP_INFO(logger(), "Loaded pre-match config: %s", yaml_path_.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "Failed to load pre-match config '%s': %s",
                 yaml_path_.c_str(), e.what());
  }
}

BT::PortsList CheckConfiguredBool::providedPorts()
{
  return {
    BT::InputPort<std::string>("key"),
    BT::InputPort<bool>("expected")
  };
}

rclcpp::Logger CheckConfiguredBool::logger() const
{
  if (node_) {
    return node_->get_logger();
  }
  return rclcpp::get_logger("CheckConfiguredBool");
}

bool CheckConfiguredBool::lookupBool(const std::string& key, bool& value) const
{
  YAML::Node node = findByPath(config_, key);
  if (!node && config_["pre_match"]) {
    node = findByPath(config_["pre_match"], key);
  }

  return yamlNodeToBool(node, value);
}

BT::NodeStatus CheckConfiguredBool::tick()
{
  if (!config_loaded_) {
    return BT::NodeStatus::FAILURE;
  }

  std::string key;
  if (!getInput("key", key)) {
    RCLCPP_ERROR(logger(), "[%s] Missing key input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  bool expected = true;
  getInput("expected", expected);

  bool configured_value = false;
  if (!lookupBool(key, configured_value)) {
    RCLCPP_ERROR(logger(), "[%s] Missing or non-bool pre-match key '%s' in %s",
                 name().c_str(), key.c_str(), yaml_path_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger(), "[%s] configured %s=%s, expected=%s",
              name().c_str(), key.c_str(),
              configured_value ? "true" : "false",
              expected ? "true" : "false");
  return configured_value == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

CheckConfiguredUInt16::CheckConfiguredUInt16(
  const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
  : BT::SyncActionNode(name, conf)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw BT::RuntimeError("CheckConfiguredUInt16 requires a valid ROS node");
  }

  if (!node_->has_parameter("pre_match_yaml_path")) {
    node_->declare_parameter<std::string>("pre_match_yaml_path", "");
  }
  yaml_path_ = node_->get_parameter("pre_match_yaml_path").as_string();

  try {
    if (yaml_path_.empty()) {
      RCLCPP_WARN(logger(), "pre_match_yaml_path is empty");
      return;
    }
    config_ = YAML::LoadFile(yaml_path_);
    config_loaded_ = true;
    RCLCPP_INFO(logger(), "Loaded pre-match config: %s", yaml_path_.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "Failed to load pre-match config '%s': %s",
                 yaml_path_.c_str(), e.what());
  }
}

BT::PortsList CheckConfiguredUInt16::providedPorts()
{
  return {
    BT::InputPort<std::string>("key"),
    BT::InputPort<uint16_t>("value"),
    BT::InputPort<std::string>("comparison")
  };
}

rclcpp::Logger CheckConfiguredUInt16::logger() const
{
  if (node_) {
    return node_->get_logger();
  }
  return rclcpp::get_logger("CheckConfiguredUInt16");
}

bool CheckConfiguredUInt16::lookupUInt16(const std::string& key, uint16_t& value) const
{
  YAML::Node node = findByPath(config_, key);
  if (!node && config_["pre_match"]) {
    node = findByPath(config_["pre_match"], key);
  }

  return yamlNodeToUInt16(node, value);
}

BT::NodeStatus CheckConfiguredUInt16::tick()
{
  if (!config_loaded_) {
    return BT::NodeStatus::FAILURE;
  }

  std::string key;
  if (!getInput("key", key)) {
    RCLCPP_ERROR(logger(), "[%s] Missing key input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  uint16_t value = 0;
  if (!getInput("value", value)) {
    RCLCPP_ERROR(logger(), "[%s] Missing value input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::string comparison = "le";
  getInput("comparison", comparison);

  uint16_t configured_value = 0;
  if (!lookupUInt16(key, configured_value)) {
    RCLCPP_ERROR(logger(), "[%s] Missing or non-UInt16 pre-match key '%s' in %s",
                 name().c_str(), key.c_str(), yaml_path_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  bool matched = false;
  if (!compareUInt16(value, configured_value, comparison, matched)) {
    RCLCPP_ERROR(logger(), "[%s] Unknown comparison: %s", name().c_str(), comparison.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger(), "[%s] value=%u, configured %s=%u, comparison=%s",
              name().c_str(), value, key.c_str(), configured_value, comparison.c_str());
  return matched ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

CheckCurrentPoint::CheckCurrentPoint(const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
}

BT::PortsList CheckCurrentPoint::providedPorts()
{
  return {
    BT::InputPort<int>("point_key")
  };
}

BT::NodeStatus CheckCurrentPoint::tick()
{
  int point_key = 0;
  if (!getInput("point_key", point_key)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckCurrentPoint"), "[%s] Missing point_key input",
                 name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const int current_point = NavigateToPoseBT::currentPointKey();
  RCLCPP_INFO(rclcpp::get_logger("CheckCurrentPoint"), "[%s] current point: %d, expected: %d",
              name().c_str(), current_point, point_key);
  return current_point == point_key ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

CheckAmmoExchangeAvailable::CheckAmmoExchangeAvailable(
  const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
}

BT::PortsList CheckAmmoExchangeAvailable::providedPorts()
{
  return {};
}

BT::NodeStatus CheckAmmoExchangeAvailable::tick()
{
  return BT::NodeStatus::SUCCESS;
}

} // namespace robot_decision
