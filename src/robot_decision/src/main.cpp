#include "robot_decision/NavigateToPose.hpp"
#include "robot_decision/IfGameStart.hpp"
#include "robot_decision/IfHealth.hpp"       
#include "robot_decision/IfHealthChanged.hpp" 
#include "robot_decision/Wait.hpp"
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

  //initiate navigate node
  auto navigate_to_pose_nh = std::make_shared<rclcpp::Node>("navigate_to_pose_client");
  RosNodeParams navigate_to_pose_params;
  navigate_to_pose_params.nh = navigate_to_pose_nh;
  navigate_to_pose_params.default_port_value = "navigate_to_pose";

  //initiate ifgamestart control node
  auto if_game_start_nh = std::make_shared<rclcpp::Node>("IfGameStart_subscriber");
  RosNodeParams if_game_start_params;
  if_game_start_params.nh = if_game_start_nh;
  if_game_start_params.default_port_value = "ifgamestart";

  auto ifhealth_nh = std::make_shared<rclcpp::Node>("IfHealth_subscriber");
  RosNodeParams ifhealth_params;
  ifhealth_params.nh = ifhealth_nh;
  ifhealth_params.default_port_value = "ifhealth";

  auto ifhealthchanged_nh = std::make_shared<rclcpp::Node>("IfHealthChanged_subscriber");
  RosNodeParams ifhealthchanged_params;
  ifhealthchanged_params.nh = ifhealthchanged_nh;
  ifhealthchanged_params.default_port_value = "ifhealth";

  //register nodes
  BehaviorTreeFactory factory;
  factory.registerNodeType<NavigateToPoseBT>("NavigateToPose",navigate_to_pose_params);
  factory.registerNodeType<robot_decision::IfGameStart>("IfGameStart",if_game_start_params);
  factory.registerNodeType<robot_decision::IfHealth>("IfHealth",ifhealth_params);
  factory.registerNodeType<robot_decision::IfHealthChanged>("IfHealthChanged",ifhealthchanged_params);
  factory.registerNodeType<robot_decision::Wait>("Wait");

  std::string bt_xml_path = ament_index_cpp::get_package_share_directory("robot_decision") + 
                          "/behavior_trees/behavior_tree.xml";
  auto tree = factory.createTreeFromFile(bt_xml_path);
  
  //BT::StdCoutLogger logger(tree);

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
 