#ifndef ROBOT_DECISION_CHECK_OUR_OUTPOST_HPP
#define ROBOT_DECISION_CHECK_OUR_OUTPOST_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <string>

namespace robot_decision
{

class CheckOutposet : public BT::SyncActionNode
{
public:
  CheckOutposet(const std::string& name, const BT::NodeConfig& conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace robot_decision

#endif // ROBOT_DECISION_CHECK_OUR_OUTPOST_HPP