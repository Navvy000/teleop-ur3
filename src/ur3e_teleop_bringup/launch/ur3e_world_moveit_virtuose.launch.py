#!/usr/bin/env python3
"""Integrated bringup using Virtuose as the Cartesian input device.

Starts:
  1) Gazebo (custom world) + UR3e spawn + ros2_control + MoveIt + RViz
  2) MoveIt Servo
  3) Virtuose bridge (out_virtuose_speed -> /ur3e_servo/cartesian_twist)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    publish_obstacles = LaunchConfiguration("publish_obstacles")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_ur_type = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3e",
        description="UR robot type (e.g. ur3e)",
    )
    declare_launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch RViz together with MoveIt",
    )
    declare_publish_obstacles = DeclareLaunchArgument(
        "publish_obstacles",
        default_value="true",
        description="If true, publish obstacles.yaml into MoveIt PlanningScene",
    )

    bringup_share = get_package_share_directory("ur3e_teleop_bringup")
    custom_sim_moveit_launch = os.path.join(
        bringup_share, "launch", "ur3e_sim_moveit_custom_world.launch.py"
    )
    sim_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_sim_moveit_launch),
        launch_arguments={
            "ur_type": ur_type,
        }.items(),
    )

    servo_config_path = os.path.join(bringup_share, "config", "ur3e_servo_cartesian.yaml")
    ur_kinematics_path = os.path.join(bringup_share, "config", "ur3e_servo_kinematics.yaml")
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="ur3e_servo",
        output="screen",
        parameters=[
            servo_config_path,
            ur_kinematics_path,
            {"use_sim_time": use_sim_time},
        ],
    )

    bridge_share = get_package_share_directory("ur3e_virtuose_bridge")
    bridge_config = os.path.join(bridge_share, "config", "virtuose_bridge.yaml")
    bridge_node = Node(
        package="ur3e_virtuose_bridge",
        executable="virtuose_bridge",
        name="ur3e_virtuose_bridge",
        output="screen",
        parameters=[bridge_config, {"use_sim_time": use_sim_time}],
    )

    switch_command_type = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/ur3e_servo/switch_command_type",
                    "moveit_msgs/srv/ServoCommandType",
                    "{command_type: 1}",
                ],
                output="screen",
            )
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_ur_type)
    ld.add_action(declare_launch_rviz)
    ld.add_action(declare_publish_obstacles)

    ld.add_action(sim_and_moveit)
    ld.add_action(servo_node)
    ld.add_action(bridge_node)
    ld.add_action(switch_command_type)

    return ld
