#ifndef ROBOT_DECISION_CHECK_BLOOD_HPP
#define ROBOT_DECISION_CHECK_BLOOD_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/u_int16.hpp>
#include <string>


namespace robot_decision
{

class IfHealth : public BT::RosTopicSubNode<std_msgs::msg::UInt16>
{
public:
  IfHealth(const std::string& name, const BT::NodeConfig& conf,
                const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::UInt16>& last_msg) override;
};

} // namespace robot_decision

#endif // ROBOT_DECISION_RECEIVE_Bool_HPP