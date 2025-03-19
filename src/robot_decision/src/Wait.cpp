#include "robot_decision/Wait.hpp"
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>

namespace robot_decision
{

Wait::Wait(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus Wait::tick()
{
  RCLCPP_INFO(rclcpp::get_logger("Wait"), "[%s] Waiting for 100ms", name().c_str());
  
  // Sleep for 100ms
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("Wait"), "[%s] Wait completed", name().c_str());
  return BT::NodeStatus::SUCCESS;
}

} // namespace robot_decision