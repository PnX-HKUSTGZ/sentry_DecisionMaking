#include "robot_decision/ifhealth_subscriber.hpp"

namespace robot_decision
{

IfHealthSubscriber::IfHealthSubscriber(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<std_msgs::msg::UInt16>(name, conf, params)
{
}

BT::PortsList IfHealthSubscriber::providedPorts()
{
  return providedBasicPorts({
    BT::OutputPort<uint16_t>("health_value")
  });
}

BT::NodeStatus IfHealthSubscriber::onTick(const std::shared_ptr<std_msgs::msg::UInt16>& last_msg)
{
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
  setOutput("health_value", last_msg->data);

  const void* current_identity = last_msg.get();
  if(current_identity != last_message_identity_)
  {
    RCLCPP_INFO(logger(), "[%s] health value: %u", name().c_str(), last_msg->data);
    last_message_identity_ = current_identity;
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace robot_decision
