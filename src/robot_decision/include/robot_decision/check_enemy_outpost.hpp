#ifndef ROBOT_DECISION_CHECK_ENEMY_OUTPOST_HPP
#define ROBOT_DECISION_CHECK_ENEMY_OUTPOST_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <string>

namespace robot_decision
{

class CheckEnemyposet : public BT::SyncActionNode
{
public:
  CheckEnemyposet(const std::string& name, const BT::NodeConfig& conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace robot_decision

#endif