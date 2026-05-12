#include "robot_decision/remain_ammo_subscriber.hpp"

namespace robot_decision
{

RemainAmmoSubscriber::RemainAmmoSubscriber(
  const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<std_msgs::msg::UInt16>(name, conf, params)
{
}

BT::PortsList RemainAmmoSubscriber::providedPorts()
{
  return providedBasicPorts({
    BT::OutputPort<uint16_t>("remain_ammo")
  });
}

BT::NodeStatus RemainAmmoSubscriber::onTick(
  const std::shared_ptr<std_msgs::msg::UInt16>& last_msg)
{
  if (!last_msg) {
    if (!waiting_for_first_message_logged_) {
      RCLCPP_WARN(logger(), "[%s] waiting for first valid message", name().c_str());
      waiting_for_first_message_logged_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }

  waiting_for_first_message_logged_ = false;
  setOutput("remain_ammo", last_msg->data);

  const void* current_identity = last_msg.get();
  if (current_identity != last_message_identity_) {
    RCLCPP_INFO(logger(), "[%s] remain ammo: %u", name().c_str(), last_msg->data);
    last_message_identity_ = current_identity;
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace robot_decision
