#ifndef ROBOT_DECISION_DECISION_CONDITIONS_HPP
#define ROBOT_DECISION_DECISION_CONDITIONS_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace robot_decision
{

class CheckBool : public BT::SyncActionNode
{
public:
  CheckBool(const std::string& name, const BT::NodeConfig& conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class CheckUInt16 : public BT::SyncActionNode
{
public:
  CheckUInt16(const std::string& name, const BT::NodeConfig& conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class CheckConfiguredBool : public BT::SyncActionNode
{
public:
  CheckConfiguredBool(const std::string& name, const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Logger logger() const;
  bool lookupBool(const std::string& key, bool& value) const;

  rclcpp::Node::SharedPtr node_;
  std::string yaml_path_;
  YAML::Node config_;
  std::unordered_map<std::string, std::string> config_values_;
  bool config_loaded_ = false;
};

class CheckConfiguredUInt16 : public BT::SyncActionNode
{
public:
  CheckConfiguredUInt16(const std::string& name, const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Logger logger() const;
  bool lookupUInt16(const std::string& key, uint16_t& value) const;

  rclcpp::Node::SharedPtr node_;
  std::string yaml_path_;
  YAML::Node config_;
  std::unordered_map<std::string, std::string> config_values_;
  bool config_loaded_ = false;
};

class CheckCurrentPoint : public BT::SyncActionNode
{
public:
  CheckCurrentPoint(const std::string& name, const BT::NodeConfig& conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class CheckAmmoExchangeAvailable : public BT::SyncActionNode
{
public:
  CheckAmmoExchangeAvailable(const std::string& name, const BT::NodeConfig& conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace robot_decision

#endif // ROBOT_DECISION_DECISION_CONDITIONS_HPP
