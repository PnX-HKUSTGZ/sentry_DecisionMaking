#include "robot_decision/set_bool_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

SetBoolService::SetBoolService(const std::string& name, const BT::NodeConfig& conf,
                               const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, conf)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw BT::RuntimeError("SetBoolService requires a valid ROS node");
  }

  service_name_ = params.default_port_value.empty() ? "set_bool" : params.default_port_value;
  server_timeout_ = params.server_timeout;

  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  client_ = node_->create_client<SetBool>(service_name_, rmw_qos_profile_services_default,
                                          callback_group_);
}

rclcpp::Logger SetBoolService::logger() const
{
  if (node_) {
    return node_->get_logger();
  }
  return rclcpp::get_logger("SetBoolService");
}

BT::NodeStatus SetBoolService::onStart()
{
  bool value = false;
  if (!getInput("value", value)) {
    RCLCPP_ERROR(logger(), "Failed to get decision input!");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(logger(), "SetBool service '%s' is not reachable.", service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  pending_request_ = std::make_shared<SetBool::Request>();
  pending_request_->data = value;
  future_response_ = client_->async_send_request(pending_request_).share();
  request_sent_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetBoolService::onRunning()
{
  executor_.spin_some();

  if (!future_response_.valid()) {
    RCLCPP_ERROR(logger(), "SetBool future lost its associated state");
    pending_request_.reset();
    return BT::NodeStatus::FAILURE;
  }

  if (future_response_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    const auto timeout = rclcpp::Duration::from_seconds(
      double(server_timeout_.count()) / 1000.0);
    if ((node_->now() - request_sent_time_) > timeout) {
      RCLCPP_ERROR(logger(), "SetBool service timed out after %ld ms", server_timeout_.count());
      future_response_ = {};
      pending_request_.reset();
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  try {
    const auto response = future_response_.get();
    future_response_ = {};
    pending_request_.reset();

    if (!response) {
      RCLCPP_ERROR(logger(), "SetBool service returned an empty response");
      return BT::NodeStatus::FAILURE;
    }

    if (response->success) {
      RCLCPP_INFO(logger(), "SetBool service succeeded.");
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(logger(), "SetBool service failed: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  } catch (const std::future_error& ex) {
    RCLCPP_ERROR(logger(), "SetBool future_error: %s", ex.what());
    future_response_ = {};
    pending_request_.reset();
    return BT::NodeStatus::FAILURE;
  }
}

void SetBoolService::onHalted()
{
  future_response_ = {};
  pending_request_.reset();
}
