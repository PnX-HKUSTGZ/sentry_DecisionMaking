#include "robot_decision/NavigateToPose.hpp"
#include "behaviortree_ros2/plugins.hpp"

NavigateToPoseBT::NavigateToPoseBT(const std::string& name, 
                                   const NodeConfig& conf, 
                                   const RosNodeParams& params)
  : RosActionNode<NavigateToPose>(name, conf, params) 
{}

PortsList NavigateToPoseBT::providedPorts()
{
  return providedBasicPorts({
    InputPort<std::vector<double>>("goal"),   // 目标点坐标
  });
}

bool NavigateToPoseBT::setGoal(RosActionNode::Goal& goal)
{
  std::vector<double> target;
  if (!getInput("goal", target) || target.size() < 2)
  {
    RCLCPP_ERROR(logger(), "Invalid goal input!");
    return false;
  }

  goal.pose.pose.position.x = target[0];
  goal.pose.pose.position.y = target[1];
  goal.pose.pose.orientation.w = 1.0;  // 默认方向
  return true;
}

NodeStatus NavigateToPoseBT::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  // 检查任务是否成功完成
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // 任务成功
      RCLCPP_INFO(logger(), "Navigation Success!");
      return NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      // 任务中止
      RCLCPP_WARN(logger(), "Navigation aborted.");
      return NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      // 任务取消
      RCLCPP_WARN(logger(), "Navigation canceled.");
      return NodeStatus::FAILURE;

    default:
      // 任务失败或出现未知错误
      RCLCPP_ERROR(logger(), "Unknown result code");
      return NodeStatus::FAILURE;
  }
}


NodeStatus NavigateToPoseBT::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "Current progress: %f", feedback->distance_remaining);
  return NodeStatus::RUNNING;
}

void NavigateToPoseBT::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class NavigateToPoseBT will register with name  "NavigateToPose".
CreateRosNodePlugin(NavigateToPoseBT, "NavigateToPose");
