"""Launch ControllerCore against the named mock ros2_control feedback path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("HexNav_description")
    description_launch = os.path.join(
        description_share, "launch", "display.launch.py"
    )
    sil_rviz_config = os.path.join(description_share, "rviz", "sil.rviz")
    namespace = LaunchConfiguration("namespace")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace for the complete SIL graph",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2 with the SIL graph.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch),
                launch_arguments={
                    "namespace": namespace,
                    "prefix": "",
                    "gui": "false",
                    "rviz": rviz,
                    # The SIL controller broadcasts odom -> base_footprint, so
                    # RViz can use the world-fixed odom frame and show the
                    # robot translating while it walks.
                    "rvizconfig": sil_rviz_config,
                    "velocity_controller": "false",
                }.items(),
            ),
            Node(
                package="hexapod_controller_ros",
                executable="hexapod_controller_ros_node",
                name="hexapod_controller_core",
                namespace=namespace,
                parameters=[
                    {
                        "autostart": True,
                        "feedback_topic": "joint_states",
                        "simulation_command_topic": "position_controller/commands",
                        # No physical servos hold torque in SIL, and the
                        # simulated rc_armed input cannot provide the arm
                        # release edge required after an idle auto-disarm.
                        "idle_disarm_ms": 0,
                    }
                ],
                output="screen",
            ),
        ]
    )