#include "robot_decision/set_bool_node.hpp"


bool SetBoolService::setRequest(Request::SharedPtr& request)
{
  bool value;
  if (!getInput("value", value)) {
    RCLCPP_ERROR(logger(), "Failed to get decision input!");
    return false;
  }

  request->data = value;
  return true;
  std::cout<<"set request:"<<value<<std::endl;
}

BT::NodeStatus SetBoolService::onResponseReceived(const Response::SharedPtr& response)
{
  std::cout << "onResponseReceived " << std::endl;
  if(response->success)
  {
    RCLCPP_INFO(logger(), "SetBool service succeeded.");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(logger(), "SetBool service failed: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SetBoolService::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}

//-----------------------------------------------------------


