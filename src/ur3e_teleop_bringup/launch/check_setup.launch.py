#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Simple test launch that starts the joy_dummy node from ur3e_teleop_joy."""
    return LaunchDescription([
        Node(
            package='ur3e_teleop_joy',          # Provided by our custom Python package
            executable='joy_dummy',            # Registered via console_scripts
            name='joy_dummy_from_bringup',
            output='screen',
        )
    ])
