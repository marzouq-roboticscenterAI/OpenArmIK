#!/usr/bin/env python3
"""Launch the measured virtual controller and its sole robot-state TF authority."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("openarm_ik_ros"))
    robot_description = (share / "urdf" / "openarm_v10_bimanual.urdf").read_text()
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="openarm_ik_ros",
            executable="openarm_ik_ros_node",
            name="openarm_ik_ros",
            output="screen",
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(share / "rviz" / "openarm_ik.rviz")],
        ),
    ])
