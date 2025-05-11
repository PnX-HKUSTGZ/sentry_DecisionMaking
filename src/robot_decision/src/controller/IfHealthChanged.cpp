#include "robot_decision/IfHealthChanged.hpp"

namespace robot_decision
{

IfHealthChanged::IfHealthChanged(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<std_msgs::msg::UInt16>(name, conf, params)
{
}

BT::PortsList IfHealthChanged::providedPorts()
{
  return {};
}

BT::NodeStatus IfHealthChanged::onTick(const std::shared_ptr<std_msgs::msg::UInt16>& last_msg)
{
  //检查消息是否为空，防止指针访问错误
  if(!last_msg)
  {
    RCLCPP_ERROR(logger(), "[%s] invalid message", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  IfHealthChanged::curr_state = (last_msg->data >= 150);
  if(IfHealthChanged::curr_state != IfHealthChanged::prev_state)  // empty if no new message received since the last tick
  {
    RCLCPP_INFO(logger(), "state changed");
    IfHealthChanged::prev_state = IfHealthChanged::curr_state;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace robot_decision