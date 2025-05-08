#include "robot_decision/ifhealth_subscriber.hpp"

namespace robot_decision
{

IfHealthSubscriber::IfHealthSubscriber(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<std_msgs::msg::UInt16>(name, conf, params)
{
}

BT::PortsList IfHealthSubscriber::providedPorts()
{
  return providedBasicPorts({
    BT::OutputPort<uint16_t>("health_value")
  });
}

BT::NodeStatus IfHealthSubscriber::onTick(const std::shared_ptr<std_msgs::msg::UInt16>& last_msg)
{
  if(!last_msg)
  {
    RCLCPP_ERROR(logger(), "[%s] invalid message", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  
  // Set the output port with the health value
  setOutput("health_value", last_msg->data);
  
  RCLCPP_INFO(logger(), "[%s] health value: %d", name().c_str(), last_msg->data);
  
  // Always return success when we receive a valid message
  return BT::NodeStatus::SUCCESS;
}

} // namespace robot_decision