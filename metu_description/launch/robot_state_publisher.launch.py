import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
import xacro


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('metu_description'))

    xacro_file = os.path.join(pkg_path,'urdf','rover.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description_config.toxml(), "use_sim_time": True}

    robot_state_publisher_cmd = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params]
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_cmd)

    return ld
