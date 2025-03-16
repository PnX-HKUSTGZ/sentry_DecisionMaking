#include "robot_decision/IfHealth.hpp"

namespace robot_decision
{

IfHealth::IfHealth(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<std_msgs::msg::UInt16>(name, conf, params)
{
}

BT::PortsList IfHealth::providedPorts()
{
  return {};
}

BT::NodeStatus IfHealth::onTick(const std::shared_ptr<std_msgs::msg::UInt16>& last_msg)
{
  if(!last_msg)
  {
    RCLCPP_ERROR(logger(), "[%s] invalid message", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if(last_msg->data> 150)  // empty if no new message received since the last tick
  {
    RCLCPP_INFO(logger(), "[%s] amount of blood: %s", name().c_str(),
                std::to_string(last_msg->data).c_str());
    std::cout<<"IfHealth::onTick and enter if conditon"<<std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  std::cout<<"IfHealth::onTick"<<std::endl;
  return BT::NodeStatus::FAILURE;
}

} // namespace robot_decision