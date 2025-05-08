#include "robot_decision/NavigateToPose.hpp"
#include "robot_decision/IfHealth.hpp"       
#include "robot_decision/IfHealthChanged.hpp" 
#include "robot_decision/Wait.hpp"
#include "robot_decision/check_our_base.hpp"
#include "robot_decision/check_our_outpost.hpp"
#include "robot_decision/check_enemy_outpost.hpp"
// New subscriber includes
#include "robot_decision/ifhealth_subscriber.hpp"
#include "robot_decision/our_base_health_subscriber.hpp"
#include "robot_decision/our_outpost_health_subscriber.hpp"
#include "robot_decision/enemy_outpost_health_subscriber.hpp"

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

  auto ifhealth_sub_nh = std::make_shared<rclcpp::Node>("IfHealthSub_subscriber");
  RosNodeParams ifhealth_sub_params;
  ifhealth_sub_params.nh = ifhealth_sub_nh;
  ifhealth_sub_params.default_port_value = "ifhealth";

  auto our_base_health_sub_nh = std::make_shared<rclcpp::Node>("OurBaseHealthSub_subscriber");
  RosNodeParams our_base_health_sub_params;
  our_base_health_sub_params.nh = our_base_health_sub_nh;
  our_base_health_sub_params.default_port_value = "our_base_health";

  auto our_outpost_health_sub_nh = std::make_shared<rclcpp::Node>("OurOutpostHealthSub_subscriber");
  RosNodeParams our_outpost_health_sub_params;
  our_outpost_health_sub_params.nh = our_outpost_health_sub_nh;
  our_outpost_health_sub_params.default_port_value = "our_outpost_health";

  auto enemy_outpost_health_sub_nh = std::make_shared<rclcpp::Node>("EnemyOutpostHealthSub_subscriber");
  RosNodeParams enemy_outpost_health_sub_params;
  enemy_outpost_health_sub_params.nh = enemy_outpost_health_sub_nh;
  enemy_outpost_health_sub_params.default_port_value = "enemy_outpost_health";

  //register nodes
  BehaviorTreeFactory factory;
  factory.registerNodeType<NavigateToPoseBT>("NavigateToPose", navigate_to_pose_params);
  
  // Register the new subscriber nodes
  factory.registerNodeType<robot_decision::IfHealthSubscriber>("IfHealthSubscriber", ifhealth_sub_params);
  factory.registerNodeType<robot_decision::OurBaseHealthSubscriber>("OurBaseHealthSubscriber", our_base_health_sub_params);
  factory.registerNodeType<robot_decision::OurOutpostHealthSubscriber>("OurOutpostHealthSubscriber", our_outpost_health_sub_params);
  factory.registerNodeType<robot_decision::EnemyOutpostHealthSubscriber>("EnemyOutpostHealthSubscriber", enemy_outpost_health_sub_params);
  
  // Register the modified action nodes (now regular nodes, not subscribers)
  factory.registerNodeType<robot_decision::IfHealth>("IfHealth");
  factory.registerNodeType<robot_decision::Wait>("Wait");
  factory.registerNodeType<robot_decision::CheckOutbase>("CheckOurBase");
  factory.registerNodeType<robot_decision::CheckOutposet>("CheckOurOutpost");
  factory.registerNodeType<robot_decision::CheckEnemyposet>("CheckEnemyOutpost");
  

  std::string bt_xml_path = ament_index_cpp::get_package_share_directory("robot_decision") + 
                          "/behavior_trees/RMUC.xml";
  auto tree = factory.createTreeFromFile(bt_xml_path);
  
  BT::StdCoutLogger logger(tree);

  //运行行为树
  auto status = tree.tickOnce();
  std::cout << "--- status: " << toStr(status) << "\n\n";
  while(!g_interrupt_requested) 
  {
    // Sleep to avoid busy loops.
    // do NOT use other sleep functions!
    // Small sleep time is OK, here we use a large one only to
    // have less messages on the console.
    tree.sleep(std::chrono::milliseconds(200));

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
