#ifndef ROBOT_DECISION_RECEIVE_Bool_HPP
#define ROBOT_DECISION_RECEIVE_Bool_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/bool.hpp>
#include <string>

namespace robot_decision
{

class IfGameStart : public BT::RosTopicSubNode<std_msgs::msg::Bool>
{
public:
  IfGameStart(const std::string& name, const BT::NodeConfig& conf,
                const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
};

} // namespace robot_decision

#endif // ROBOT_DECISION_RECEIVE_Bool_HPP