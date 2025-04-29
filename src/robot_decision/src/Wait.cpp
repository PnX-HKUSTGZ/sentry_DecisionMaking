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
  int time_to_wait = 500;
  RCLCPP_INFO(rclcpp::get_logger("Wait"), "[%s] Waiting for %d ms", name().c_str(), time_to_wait);
  
  // Sleep for time_to_wait
  std::this_thread::sleep_for(std::chrono::milliseconds(time_to_wait));
  
  RCLCPP_INFO(rclcpp::get_logger("Wait"), "[%s] Wait completed", name().c_str());
  return BT::NodeStatus::SUCCESS;
}

} // namespace robot_decision