from tkinter.messagebox import NO
from typing import List

from launch_ros.actions import Node

import launch
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # argument
    launch_arguments: List[DeclareLaunchArgument] = []
    launch_arguments.append(
        DeclareLaunchArgument(
            "use_radar_threshold_filter",
            default_value=False,
        )
    )

    # node
    launch_nodes: List[Node] = []
    launch_nodes.append(
        Node(
            package="radar_threshold_filter",
            node_executable="radar_threshold_filter_node",
            output="screen",
        )
    )

    return launch.LaunchDescription(launch_arguments + launch_nodes)
