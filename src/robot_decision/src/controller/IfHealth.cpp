#include "robot_decision/IfHealth.hpp"

namespace robot_decision
{

IfHealth::IfHealth(const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
}

BT::PortsList IfHealth::providedPorts()
{
  return {
    BT::InputPort<std::string>("message"),   
    BT::InputPort<int>("HPthreshold"),
    BT::InputPort<uint16_t>("health_value")
  };
}

BT::NodeStatus IfHealth::tick()
{
  std::string message;
  if (!getInput("message", message)) {
    return BT::NodeStatus::FAILURE;
  }
  
  int HPthreshold;
  if (!getInput("HPthreshold", HPthreshold)) {
    return BT::NodeStatus::FAILURE;
  }
  
  uint16_t health_value;
  if (!getInput("health_value", health_value)) {
    RCLCPP_ERROR(rclcpp::get_logger("IfHealth"), "[%s] Missing health_value input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  
  if (message == "checkhealth")
  {
    if(health_value > HPthreshold)
    {
      RCLCPP_INFO(rclcpp::get_logger("IfHealth"), "[%s] amount of blood: %d", name().c_str(), health_value);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  else // "checkunhealth"
  {
    if(health_value <= HPthreshold)
    {
      RCLCPP_INFO(rclcpp::get_logger("IfHealth"), "[%s] amount of blood: %d", name().c_str(), health_value);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
} 

}// namespace robot_decision