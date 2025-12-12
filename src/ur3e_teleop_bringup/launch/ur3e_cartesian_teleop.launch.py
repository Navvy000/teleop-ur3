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

    # ================================
    # 1. Custom MoveIt Servo node (Twist mode)
    # ================================
    servo_config_path = os.path.join(
        bringup_share, 'config', 'ur3e_servo_cartesian.yaml'
    )

    ur_kinematics_path = os.path.join(
        bringup_share, 'config', 'ur3e_servo_kinematics.yaml'
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='ur3e_servo',
        output='screen',
        parameters=[
            servo_config_path,          # Servo configuration
            ur_kinematics_path,         # KDL inverse kinematics plugin config
            {'use_sim_time': use_sim_time},
        ],
    )

    # ================================
    # 2. Joy node
    # ================================
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'dev': '/dev/input/js0'},
        ],
    )

    # ================================
    # 3. Cartesian teleoperation node
    # ================================
    cartesian_teleop_node = Node(
        package='ur3e_teleop_joy',
        executable='cartesian_teleop',
        name='ur3e_cartesian_teleop',
        output='screen',
        parameters=[
            {
                'joy_topic': '/joy',
                'enable_button': 5,
                'deadzone': 0.15,

                'linear_scale': 0.1,    # m/s
                'angular_scale': 0.8,   # rad/s

                'twist_topic': '/ur3e_servo/cartesian_twist',
                'command_frame': 'tool0',

                'axis_left_x': 0,
                'axis_left_y': 1,
                'axis_right_x': 3,
                'axis_right_y': 4,
                'axis_lt': 2,
                'axis_rt': 5,
            }
        ],
    )

    # ================================
    # 4. Switch Servo into Twist mode (command_type=1)
    # ================================
    switch_command_type = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/ur3e_servo/switch_command_type',
                    'moveit_msgs/srv/ServoCommandType',
                    '{command_type: 1}',
                ],
                output='screen',
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(servo_node)
    ld.add_action(joy_node)
    ld.add_action(cartesian_teleop_node)
    ld.add_action(switch_command_type)

    return ld
