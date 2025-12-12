#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    bringup_share = get_package_share_directory('ur3e_teleop_bringup')
    servo_config_path = os.path.join(bringup_share, 'config', 'ur3e_servo_joint.yaml')

    # MoveIt Servo node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='ur3e_servo',
        output='screen',
        parameters=[
            servo_config_path,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'dev': '/dev/input/js0'},
        ],
    )

    # Joint-space teleoperation node
    joint_teleop_node = Node(
        package='ur3e_teleop_joy',
        executable='joint_teleop',
        name='ur3e_joint_teleop',
        output='screen',
        parameters=[
            {
                'joy_topic': '/joy',
                'enable_button': 5,    # Default RB button
                'deadzone': 0.15,
                'joint_scale': 1.0,
                'joint_names': [
                    'shoulder_pan_joint',
                    'shoulder_lift_joint',
                    'elbow_joint',
                    'wrist_1_joint',
                    'wrist_2_joint',
                    'wrist_3_joint',
                ],
                'joint_axes': [1, 0, 4, 3, -1, -1],
                'joint_jog_topic': '/ur3e_servo/joint_jog',
            }
        ],
    )

    #  Automatically switch Servo to command_type = 0 (JointJog mode)
    switch_command_type = TimerAction(
    period=3.0,    # Allow 3 seconds for servo_node to start
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/ur3e_servo/switch_command_type',
                    'moveit_msgs/srv/ServoCommandType',
                    '{command_type: 0}',
                ],
                output='screen',
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(servo_node)
    ld.add_action(joy_node)
    ld.add_action(joint_teleop_node)
    ld.add_action(switch_command_type)

    return ld
