#ifndef ROBOT_DECISION_WAIT_HPP
#define ROBOT_DECISION_WAIT_HPP

#include <behaviortree_cpp/action_node.h>

namespace robot_decision
{

class Wait : public BT::SyncActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfig& config);

  // This method is invoked by the BT engine when the node is ticked
  BT::NodeStatus tick() override;

  // The node doesn't need any port
  static BT::PortsList providedPorts() { return {}; }
};

} // namespace robot_decision

#endif // ROBOT_DECISION_WAIT_HPP