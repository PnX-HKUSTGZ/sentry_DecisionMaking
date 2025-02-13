#ifndef NAVIGATE_TO_POSE_BT_HPP
#define NAVIGATE_TO_POSE_BT_HPP

#include <memory>
#include <vector>
#include <string>
#include "behaviortree_ros2/bt_action_node.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace BT;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavigateToPoseBT : public RosActionNode<NavigateToPose>
{
public:
  NavigateToPoseBT(const std::string& name, 
                   const NodeConfig& conf, 
                   const RosNodeParams& params);

  // 提供端口 (behavior tree input/output)
  static PortsList providedPorts();

protected:
  // 设置目标 (发送到 action server)
  bool setGoal(RosActionNode::Goal& goal) override;

  // 处理任务完成的返回值
  NodeStatus onResultReceived(const WrappedResult& wr) override;

  // 处理反馈信息
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  // 处理任务中断
  void onHalt() override;
};

#endif // NAVIGATE_TO_POSE_BT_HPP
