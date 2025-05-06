#include "robot_decision/Wait.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>

namespace robot_decision
{

Wait::Wait(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{}

BT::PortsList Wait::providedPorts()
{
  return { BT::InputPort<int>("waittime") };
}

BT::NodeStatus Wait::onStart()
{
  int ms = 0;
  // 读取输入端口
  if (!getInput("waittime", ms))
  {
    throw BT::RuntimeError("Missing required input port [waittime]");
  }

  if (ms <= 0)
  {
    // 不需要等待，直接成功
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    // 记录截止时间
    deadline_ = std::chrono::system_clock::now() + std::chrono::milliseconds(ms);
    return BT::NodeStatus::RUNNING;
  }
}

BT::NodeStatus Wait::onRunning()
{
  // 如果当前时间没到 deadline，就保持 RUNNING
  if (std::chrono::system_clock::now() < deadline_)
  {
    return BT::NodeStatus::RUNNING;
  }
  // 超时后返回 SUCCESS
  return BT::NodeStatus::SUCCESS;
}

void Wait::onHalted()
{
  std::cout << "[Wait] Halted before completion\n";
}

} // namespace robot_decision
