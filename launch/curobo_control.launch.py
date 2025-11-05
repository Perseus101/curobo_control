#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            Node(
                package="curobo_control",
                executable="joy_cmd_vel_node",
                name="joy_cmd_vel_node",
                output="screen",
            ),
            Node(
                package="curobo_control",
                executable="curobo_control",
                name="curobo_control",
                output="screen",
            ),
        ]
    )
