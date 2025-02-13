#include "robot_decision/NavigateToPose.hpp"
#include "behaviortree_cpp/bt_factory.h"

int main()
{
  auto navigate_to_pose_nh = std::make_shared<rclcpp::Node>("navigate_to_pose_client");
  RosNodeParams navigate_to_pose_params;
  navigate_to_pose_params.nh = navigate_to_pose_nh;
  navigate_to_pose_params.default_port_value = "navigate_to_pose";

  BehaviorTreeFactory factory;
  factory.registerNodeType<NavigateToPoseBT>("NavigateToPose",navigate_to_pose_params);
  
  auto tree = factory.createTreeFromFile("../behavior_tree.xml");
  
  //运行行为树
  auto status = tree.tickOnce();
  //std::cout << "--- status: " << toStr(status) << "\n\n";
  while(status == NodeStatus::RUNNING) 
  {
    // Sleep to avoid busy loops.
    // do NOT use other sleep functions!
    // Small sleep time is OK, here we use a large one only to
    // have less messages on the console.
    tree.sleep(std::chrono::milliseconds(100));

    //std::cout << "--- ticking\n";
    status = tree.tickOnce();
    //std::cout << "--- status: " << toStr(status) << "\n\n";
  }
  
  return 0;
}
