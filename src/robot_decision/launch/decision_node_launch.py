from argparse import Namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    robot_decision_node = Node(
        name="robot_decision",
        package="robot_decision",
        executable="robot_decision_node",
        output="screen"
    )

    ld.add_action(robot_decision_node)

    return ld
