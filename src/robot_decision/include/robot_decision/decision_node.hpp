#ifndef ROBOT_DECISION_DECISION_NODE_HPP_
#define ROBOT_DECISION_DECISION_NODE_HPP_

#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>

namespace rm_decision {

class NavigateThroughPosesNode : public rclcpp::Node
{
public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    NavigateThroughPosesNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal();

    std::vector<geometry_msgs::msg::PoseStamped> get_poses();
    
};

} // namespace rm_decision

#endif // ROBOT_DECISION_DECISION_NODE_HPP_