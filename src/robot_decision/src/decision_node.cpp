#include "robot_decision/decision_node.hpp"
#include <iostream>
namespace rm_decision {

NavigateThroughPosesNode::NavigateThroughPosesNode(const rclcpp::NodeOptions & options)
: Node("navigate_through_poses_node" , options)
{
    this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
    this->timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&NavigateThroughPosesNode::send_goal, this));
}

void NavigateThroughPosesNode::send_goal()
{
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }
    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = get_poses();
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

std::vector<geometry_msgs::msg::PoseStamped> NavigateThroughPosesNode::get_poses()
{
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = 3;
    pose.pose.position.y = 0.46;
    pose.pose.orientation.w = 1.0;
    poses.push_back(pose);
    pose.pose.position.x = 3;
    pose.pose.position.y = 0.46;
    poses.push_back(pose);
    return poses;
}

} // namespace rm_decision

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<rm_decision::NavigateThroughPosesNode>();
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}
