#include "robot_decision/NavigateToPose.hpp"
#include "robot_decision/IfHealth.hpp"       
#include "robot_decision/IfHealthChanged.hpp" 
#include "robot_decision/Wait.hpp"
#include "robot_decision/check_our_base.hpp"
#include "robot_decision/check_our_outpost.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <filesystem>
#include <signal.h>
#include <atomic>
#include <iostream>

//safe shutdown
std::atomic<bool> g_interrupt_requested(false);

// Signal handler for Ctrl+C
void signalHandler(int signum) {
  g_interrupt_requested = true;
  std::cout << "Interrupt received, safely shutting down..." << std::endl;
}

int main(int argc, char** argv)
{
  // Register signal handler
  signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);

  //initiate node
  auto navigate_to_pose_nh = std::make_shared<rclcpp::Node>("navigate_to_pose_client");
  RosNodeParams navigate_to_pose_params;
  // lengthen the time of waiting for the action server to confirm the goal
  navigate_to_pose_params.server_timeout = std::chrono::milliseconds(5000);
  navigate_to_pose_params.nh = navigate_to_pose_nh;
  navigate_to_pose_params.default_port_value = "navigate_to_pose";

  auto ifhealth_nh = std::make_shared<rclcpp::Node>("IfHealth_subscriber");
  RosNodeParams ifhealth_params;
  ifhealth_params.nh = ifhealth_nh;
  ifhealth_params.default_port_value = "ifhealth";

  auto ifhealthchanged_nh = std::make_shared<rclcpp::Node>("IfHealthChanged_subscriber");
  RosNodeParams ifhealthchanged_params;
  ifhealthchanged_params.nh = ifhealthchanged_nh;
  ifhealthchanged_params.default_port_value = "ifhealth";

  auto check_our_base_nh = std::make_shared<rclcpp::Node>("CheckOurBase_subscriber");
  RosNodeParams check_our_base_params;
  check_our_base_params.nh = check_our_base_nh;
  check_our_base_params.default_port_value = "our_base_health";

  auto check_our_outpost_nh = std::make_shared<rclcpp::Node>("CheckOurOutpost_subscriber");
  RosNodeParams check_our_outpost_params;
  check_our_outpost_params.nh = check_our_outpost_nh;
  check_our_outpost_params.default_port_value = "our_outpost_health";

  //register nodes
  BehaviorTreeFactory factory;
  factory.registerNodeType<NavigateToPoseBT>("NavigateToPose",navigate_to_pose_params);
  factory.registerNodeType<robot_decision::IfHealth>("IfHealth",ifhealth_params);
  factory.registerNodeType<robot_decision::Wait>("Wait");
  factory.registerNodeType<robot_decision::CheckOutbase>("CheckOurBase",check_our_base_params);
  factory.registerNodeType<robot_decision::CheckOutposet>("CheckOurOutpost",check_our_outpost_params);
  factory.registerNodeType<robot_decision::IfHealthChanged>("IfHealthChanged",ifhealthchanged_params);
  

  std::string bt_xml_path = ament_index_cpp::get_package_share_directory("robot_decision") + 
                          "/behavior_trees/RMUC.xml";
  auto tree = factory.createTreeFromFile(bt_xml_path);
  
  BT::StdCoutLogger logger(tree);

  //运行行为树
  auto status = tree.tickOnce();
  std::cout << "--- status: " << toStr(status) << "\n\n";
  while(status == NodeStatus::RUNNING && !g_interrupt_requested) 
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
  
  // Clean shutdown
  if (g_interrupt_requested) {
    std::cout << "Halting behavior tree..." << std::endl;
    tree.haltTree();
  }
  
  // Clean up ROS resources
  rclcpp::shutdown();
  std::cout << "Behavior tree execution completed." << std::endl;

  return 0;
}
 