#include "robot_decision/check_our_base.hpp"
#include <rclcpp/rclcpp.hpp>

namespace robot_decision
{

CheckOutbase::CheckOutbase(const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
}

BT::PortsList CheckOutbase::providedPorts()
{
  return {
    BT::InputPort<int>("HPthreshold"),
    BT::InputPort<uint16_t>("base_health_value")
  };
}

BT::NodeStatus CheckOutbase::tick()
{
  int HPthreshold;
  if (!getInput("HPthreshold", HPthreshold)) {
    return BT::NodeStatus::FAILURE;
  }
  
  uint16_t base_health;
  if (!getInput("base_health_value", base_health)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckOutbase"), "[%s] Missing base_health_value input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  
  // Check if base health is at or below threshold (critical)
  if (base_health <= HPthreshold)
  {
    RCLCPP_INFO(rclcpp::get_logger("CheckOutbase"), "[%s] Our base health: %d (critical)", 
                name().c_str(), base_health);
    return BT::NodeStatus::SUCCESS;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("CheckOutbase"), "[%s] Our base health: %d (good)", 
              name().c_str(), base_health);
  return BT::NodeStatus::FAILURE;
}

} // namespace robot_decision