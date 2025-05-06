#ifndef ROBOT_DECISION_WAIT_HPP
#define ROBOT_DECISION_WAIT_HPP

#include <behaviortree_cpp/action_node.h>
#include <chrono>

namespace robot_decision
{

/**
 * @brief A non-blocking wait node that returns RUNNING until the given duration elapses.
 */
class Wait : public BT::StatefulActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfiguration& config);

  // 声明一个名为 "waittime" 的输入端口，单位：毫秒
  static BT::PortsList providedPorts();

  // 从 IDLE → RUNNING 时调用一次
  BT::NodeStatus onStart() override;

  // 每次 tick 时调用，直到返回 SUCCESS 或 FAILURE
  BT::NodeStatus onRunning() override;

  // 如果节点被 halt()，调用此方法
  void onHalted() override;

private:
  std::chrono::system_clock::time_point deadline_;
};

} // namespace robot_decision

#endif // ROBOT_DECISION_WAIT_HPP
