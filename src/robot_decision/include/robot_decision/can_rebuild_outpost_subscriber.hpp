#ifndef ROBOT_DECISION_CAN_REBUILD_OUTPOST_SUBSCRIBER_HPP
#define ROBOT_DECISION_CAN_REBUILD_OUTPOST_SUBSCRIBER_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/bool.hpp>
#include <string>

namespace robot_decision
{

class CanRebuildOutpostSubscriber : public BT::RosTopicSubNode<std_msgs::msg::Bool>
{
public:
  CanRebuildOutpostSubscriber(const std::string& name, const BT::NodeConfig& conf,
                              const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
  bool latchLastMessage() const override { return true; }

private:
  const void* last_message_identity_ = nullptr;
  bool waiting_for_first_message_logged_ = false;
};

} // namespace robot_decision

#endif // ROBOT_DECISION_CAN_REBUILD_OUTPOST_SUBSCRIBER_HPP
