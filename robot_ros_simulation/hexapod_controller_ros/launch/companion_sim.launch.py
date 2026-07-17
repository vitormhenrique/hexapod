"""Launch the full ROS SIL graph and its local companion-app endpoint."""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)


def generate_launch_description():
    package_share = get_package_share_directory("hexapod_controller_ros")
    package_prefix = get_package_prefix("hexapod_controller_ros")
    bridge_script = os.path.join(
        package_prefix,
        "lib",
        "hexapod_controller_ros",
        "companion_sim_bridge",
    )
    sil_launch = PythonLaunchDescriptionSource(
        f"{package_share}/sil.launch.py"
    )
    namespace = LaunchConfiguration("namespace")
    rviz = LaunchConfiguration("rviz")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    token = LaunchConfiguration("token")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace passed to the SIL controller graph.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz2 with the companion simulation graph.",
            ),
            DeclareLaunchArgument(
                "host",
                default_value="127.0.0.1",
                description="Loopback host for the companion simulation endpoint.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="5560",
                description="TCP port for the companion simulation endpoint.",
            ),
            DeclareLaunchArgument(
                "token",
                default_value="hexapod-sim",
                description="Local TCP endpoint token required by the companion.",
            ),
            IncludeLaunchDescription(
                sil_launch,
                launch_arguments={
                    "namespace": namespace,
                    "rviz": rviz,
                }.items(),
            ),
            # The installed script uses /usr/bin/env in its shebang. On macOS
            # that system launcher strips DYLD_LIBRARY_PATH, which makes the
            # generated hexapod_msgs type support unavailable. Invoke it with
            # this launch process's Pixi Python interpreter instead.
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    bridge_script,
                    "--ros-args",
                    "-r",
                    "__node:=companion_sim_bridge",
                    "-r",
                    ["__ns:=/", namespace],
                    "-p",
                    ["host:=", host],
                    "-p",
                    ["port:=", port],
                    "-p",
                    ["token:=", token],
                ],
                output="screen",
            ),
        ]
    )