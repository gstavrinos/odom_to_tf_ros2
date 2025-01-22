#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory("odom_to_tf_ros2"), "config", "odom_to_tf.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="odom_to_tf_ros2",
                executable="odom_to_tf",
                name="odom_to_tf",
                output="screen",
                parameters=[config_file_path],
                remappings=[],
            )
        ]
    )
