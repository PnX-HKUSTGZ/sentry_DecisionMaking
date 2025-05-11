#include "robot_decision/check_our_outpost.hpp"
#include <rclcpp/rclcpp.hpp>

namespace robot_decision
{

CheckOutposet::CheckOutposet(const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
}

BT::PortsList CheckOutposet::providedPorts()
{
  return {
    BT::InputPort<int>("HPthreshold"),
    BT::InputPort<uint16_t>("outpost_health_value")
  };
}

BT::NodeStatus CheckOutposet::tick()
{
  int HPthreshold;
  if (!getInput("HPthreshold", HPthreshold)) {
    return BT::NodeStatus::FAILURE;
  }
  
  uint16_t outpost_health;
  if (!getInput("outpost_health_value", outpost_health)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckOutposet"), "[%s] Missing outpost_health_value input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  
  // Check if outpost health is at or below threshold (dead)
  if (outpost_health <= HPthreshold)
  {
    RCLCPP_INFO(rclcpp::get_logger("CheckOutposet"), "[%s] Our outpost health: %d (dead/critical)", 
                name().c_str(), outpost_health);
    return BT::NodeStatus::SUCCESS;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("CheckOutposet"), "[%s] Our outpost health: %d (alive)", 
              name().c_str(), outpost_health);
  return BT::NodeStatus::FAILURE;
}

} // namespace robot_decision