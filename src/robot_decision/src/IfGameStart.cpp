#include "robot_decision/IfGameStart.hpp"

namespace robot_decision
{

IfGameStart::IfGameStart(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params)
{
}

BT::PortsList IfGameStart::providedPorts()
{
  return {};
}

BT::NodeStatus IfGameStart::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg)
{
  if(!last_msg)
  {
    RCLCPP_ERROR(logger(), "[%s] invalid message", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if(last_msg->data)  // empty if no new message received since the last tick
  {
    RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(),
                last_msg->data ? "true" : "false");
    std::cout<<"IfGameStart::onTick and enter if conditon"<<std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  std::cout<<"IfGameStart::onTick"<<std::endl;
  return BT::NodeStatus::FAILURE;
}

} // namespace robot_decision