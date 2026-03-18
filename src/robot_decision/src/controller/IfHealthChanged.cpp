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
    if(!waiting_for_first_message_logged_)
    {
      RCLCPP_WARN(logger(), "[%s] waiting for first valid message", name().c_str());
      waiting_for_first_message_logged_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }

  waiting_for_first_message_logged_ = false;
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
