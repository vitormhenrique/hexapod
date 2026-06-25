"""
Launch file for HexNav.

Usage:
    ros2 launch HexNav_description display.launch.py
    ros2 launch HexNav_description display.launch.py namespace:=robot1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("HexNav_description")

    xacro_file = os.path.join(pkg_dir, "urdf", "HexNav.urdf.xacro")
    rviz_file = os.path.join(pkg_dir, "rviz", "display.rviz")
    controllers_file = os.path.join(pkg_dir, "config", "ros2_controllers.yaml")

    ns = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " prefix:=", prefix]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS 2 namespace for topics and nodes",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="URDF link/joint name prefix (passed to xacro)",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch the joint slider control GUI",
            ),
            GroupAction(
                [
                    PushRosNamespace(ns),
                    # Robot state publisher
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        name="robot_state_publisher",
                        parameters=[{"robot_description": robot_description}],
                        output="screen",
                    ),
                    # ros2_control controller manager using generated mock hardware
                    Node(
                        package="controller_manager",
                        executable="ros2_control_node",
                        name="controller_manager",
                        parameters=[
                            {"robot_description": robot_description},
                            controllers_file,
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        name="spawn_joint_state_broadcaster",
                        arguments=[
                            "joint_state_broadcaster",
                            "--controller-manager",
                            "controller_manager",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        name="spawn_position_controller",
                        arguments=[
                            "position_controller",
                            "--controller-manager",
                            "controller_manager",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        name="spawn_velocity_controller",
                        arguments=[
                            "velocity_controller",
                            "--controller-manager",
                            "controller_manager",
                        ],
                        output="screen",
                    ),
                    # RViz2
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        arguments=["-d", rviz_file],
                        output="screen",
                    ),
                    # Joint slider control GUI (publishes /position_controller/commands)
                    Node(
                        package="HexNav_description",
                        executable="joint_gui.py",
                        name="joint_gui",
                        output="screen",
                        condition=IfCondition(gui),
                    ),
                ]
            ),
        ]
    )
