#include "robot_decision/NavigateToPose.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <filesystem>

std::atomic<int> NavigateToPoseBT::current_point_key_{5};

NavigateToPoseBT::NavigateToPoseBT(const std::string& name, 
                                   const NodeConfig& conf, 
                                   const RosNodeParams& params)
  : RosActionNode<NavigateToPose>(name, conf, params) 
{
  // Load points from YAML file during initialization
  loadPointsFromYaml();
}

// Function to load points from YAML file
void NavigateToPoseBT::loadPointsFromYaml() {
  try {
    // Default path for backward compatibility.
    const std::string default_yaml_path =
      ament_index_cpp::get_package_share_directory("robot_decision") + "/behavior_trees/RMUC_point.yaml";

    points_yaml_path_ = default_yaml_path;
    target_frame_id_ = "map";

    // Read runtime configuration from the ROS node when available.
    if (auto node = node_.lock()) {
      if (!node->has_parameter("points_yaml_path")) {
        node->declare_parameter<std::string>("points_yaml_path", default_yaml_path);
      }
      if (!node->has_parameter("target_frame_id")) {
        node->declare_parameter<std::string>("target_frame_id", "map");
      }

      points_yaml_path_ = node->get_parameter("points_yaml_path").as_string();
      if (points_yaml_path_.empty()) {
        points_yaml_path_ = default_yaml_path;
      }

      target_frame_id_ = node->get_parameter("target_frame_id").as_string();
      if (target_frame_id_.empty()) {
        target_frame_id_ = "map";
      }
    }

    RCLCPP_INFO(logger(), "Loading points from: %s", points_yaml_path_.c_str());
    RCLCPP_INFO(logger(), "NavigateToPose target frame: %s", target_frame_id_.c_str());

    if (!std::filesystem::exists(points_yaml_path_)) {
      RCLCPP_ERROR(logger(), "Points YAML does not exist: %s", points_yaml_path_.c_str());
      return;
    }
    
    // Load YAML file
    YAML::Node config = YAML::LoadFile(points_yaml_path_);
    
    if (config["points"]) {
      points_map_ = config["points"];
      RCLCPP_INFO(logger(), "Points loaded successfully");
    } else {
      RCLCPP_ERROR(logger(), "Failed to find 'points' section in YAML file");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "Error loading points from YAML: %s", e.what());
  }
}

int NavigateToPoseBT::currentPointKey()
{
  return current_point_key_.load();
}

void NavigateToPoseBT::setCurrentPointKey(int point_key)
{
  current_point_key_.store(point_key);
}

PortsList NavigateToPoseBT::providedPorts()
{
  return providedBasicPorts({
    InputPort<int>("point_key"),   // 目标点的键值 (1, 2, 3, etc.)
    InputPort<std::string>("decision", "", "Debug description for why this target was selected"),
  });
}

bool NavigateToPoseBT::setGoal(RosActionNode::Goal& goal)
{
  if (!points_map_ || !points_map_.IsMap()) {
    RCLCPP_ERROR(logger(), "Points are not loaded. Check points_yaml_path parameter.");
    return false;
  }

  int point_key;
  if (!getInput("point_key", point_key))
  {
    RCLCPP_ERROR(logger(), "Failed to get point_key input!");
    return false;
  }
  std::string decision_info;
  getInput("decision", decision_info);
  active_point_key_ = point_key;

  // Convert to string for YAML lookup
  std::string key_str = std::to_string(point_key);
  
  // Check if key exists in points map
  if (!points_map_[key_str]) {
    RCLCPP_ERROR(logger(), "Point key %d not found in YAML file!", point_key);
    return false;
  }
  
  // Get coordinates from YAML
  std::vector<double> target = points_map_[key_str].as<std::vector<double>>();
  
  if (target.size() < 2) {
    RCLCPP_ERROR(logger(), "Invalid coordinates for point key %d!", point_key);
    return false;
  }
  
  goal.pose.header.frame_id = target_frame_id_;
  goal.pose.header.stamp = this->now();
  goal.pose.pose.position.x = target[0];
  goal.pose.pose.position.y = target[1];
  if (decision_info.empty()) {
    RCLCPP_INFO(logger(), "[DecisionTarget] point=%d x=%.3f y=%.3f frame=%s",
                point_key, goal.pose.pose.position.x, goal.pose.pose.position.y,
                target_frame_id_.c_str());
  } else {
    RCLCPP_INFO(logger(), "[DecisionTarget] %s -> point=%d x=%.3f y=%.3f frame=%s",
                decision_info.c_str(), point_key, goal.pose.pose.position.x,
                goal.pose.pose.position.y, target_frame_id_.c_str());
  }
  
  goal.pose.pose.orientation.w = 1.0;
  return true;
}

NodeStatus NavigateToPoseBT::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  std::cout<<"onResultReceived: "<<static_cast<int>(wr.code)<<std::endl;
  // 检查任务是否成功完成
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // 任务成功
      setCurrentPointKey(active_point_key_);
      RCLCPP_INFO(logger(), "Current point updated to %d", active_point_key_);
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
  std::cout<<"onFeedback: "<<feedback->distance_remaining<<std::endl;
  RCLCPP_INFO(logger(), "Current progress: %f", feedback->distance_remaining);
  return NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseBT::onFailure(BT::ActionNodeErrorCode code)
{
  RCLCPP_WARN(logger(), "NavigateToPose failure: %s", BT::toStr(code));
  return NodeStatus::FAILURE;
}


void NavigateToPoseBT::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class NavigateToPoseBT will register with name  "NavigateToPose".
CreateRosNodePlugin(NavigateToPoseBT, "NavigateToPose");
