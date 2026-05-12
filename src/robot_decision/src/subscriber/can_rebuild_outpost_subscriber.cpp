#include "robot_decision/can_rebuild_outpost_subscriber.hpp"

namespace robot_decision
{

CanRebuildOutpostSubscriber::CanRebuildOutpostSubscriber(
  const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params)
{
}

BT::PortsList CanRebuildOutpostSubscriber::providedPorts()
{
  return providedBasicPorts({
    BT::OutputPort<bool>("can_rebuild_outpost")
  });
}

BT::NodeStatus CanRebuildOutpostSubscriber::onTick(
  const std::shared_ptr<std_msgs::msg::Bool>& last_msg)
{
  if (!last_msg) {
    if (!waiting_for_first_message_logged_) {
      RCLCPP_WARN(logger(), "[%s] waiting for first valid message", name().c_str());
      waiting_for_first_message_logged_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }

  waiting_for_first_message_logged_ = false;
  setOutput("can_rebuild_outpost", last_msg->data);

  const void* current_identity = last_msg.get();
  if (current_identity != last_message_identity_) {
    RCLCPP_INFO(logger(), "[%s] can rebuild outpost: %s",
                name().c_str(), last_msg->data ? "true" : "false");
    last_message_identity_ = current_identity;
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace robot_decision
