#include "robot_decision/check_enemy_outpost.hpp"
#include <rclcpp/rclcpp.hpp>

namespace robot_decision
{

CheckEnemyposet::CheckEnemyposet(const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
}

BT::PortsList CheckEnemyposet::providedPorts()
{
  return {
    BT::InputPort<int>("HPthreshold"),
    BT::InputPort<uint16_t>("enemy_outpost_health_value")
  };
}

BT::NodeStatus CheckEnemyposet::tick()
{
  int HPthreshold;
  if (!getInput("HPthreshold", HPthreshold)) {
    return BT::NodeStatus::FAILURE;
  }
  
  uint16_t enemy_outpost_health;
  if (!getInput("enemy_outpost_health_value", enemy_outpost_health)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckEnemyposet"), "[%s] Missing enemy_outpost_health_value input", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  
  // Check if enemy outpost health is at or below threshold (dead)
  if (enemy_outpost_health <= HPthreshold)
  {
    RCLCPP_INFO(rclcpp::get_logger("CheckEnemyposet"), "[%s] Enemy outpost health: %d (dead/critical)", 
                name().c_str(), enemy_outpost_health);
    return BT::NodeStatus::SUCCESS;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("CheckEnemyposet"), "[%s] Enemy outpost health: %d (alive)", 
              name().c_str(), enemy_outpost_health);
  return BT::NodeStatus::FAILURE;
}

} // namespace robot_decision