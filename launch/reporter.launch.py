#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mission_reporter",
                executable="reporter",
                name="reporter",
                respawn=True,
                respawn_delay=1,
                parameters=[
                    {
                        "robot_id": "helhest",
                        "path_topic": "/plan",
                    }
                ],
            )
        ]
    )
