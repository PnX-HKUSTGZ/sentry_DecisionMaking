#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>
#include <memory>

using SetBool = std_srvs::srv::SetBool;

class SetBoolService : public BT::StatefulActionNode
{
public:
  explicit SetBoolService(const std::string& name, const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("value"),
      BT::InputPort<bool>("required", true, "Whether service failure should fail the BT node")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Logger logger() const;

  std::shared_ptr<rclcpp::Node> node_;
  std::string service_name_;
  std::chrono::milliseconds server_timeout_{3000};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<SetBool>::SharedPtr client_;
  SetBool::Request::SharedPtr pending_request_;
  std::shared_future<SetBool::Response::SharedPtr> future_response_;
  rclcpp::Time request_sent_time_{0, 0, RCL_ROS_TIME};
  bool request_required_{true};
};
