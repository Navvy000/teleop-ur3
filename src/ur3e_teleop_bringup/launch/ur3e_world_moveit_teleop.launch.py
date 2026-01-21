#!/usr/bin/env python3
"""Integrated bringup for the thesis demos.

Starts:
  1) Gazebo (gz sim) using our custom world SDF (3 obstacles + 1 target)
  2) UR3e robot spawn + ros2_control (from ur_simulation_gz)
  3) MoveIt + RViz (from ur_moveit_config)
  4) MoveIt Servo + joystick teleop
  5) (Optional) publish obstacles.yaml as MoveIt collision objects

Why this file exists:
  The upstream `ur_sim_moveit.launch.py` always uses `empty.sdf` by default and
  doesn't expose a world override. The lower-level `ur_sim_control.launch.py`
  *does* support `world_file`, so we stitch it here and point it to our SDF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    haptic_share = get_package_share_directory("ur3e_haptic_scene")
    # 1) Gazebo (custom world) + UR spawn + ros2_control + MoveIt + RViz
    # We reuse upstream launch structure to keep robot_description + SRDF consistent.
    custom_sim_moveit_launch = os.path.join(
        bringup_share, "launch", "ur3e_sim_moveit_custom_world.launch.py"
    )
    sim_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_sim_moveit_launch),
        launch_arguments={
            "ur_type": ur_type,
        }.items(),
    )

    # 3) Servo
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

    # 4) Joy + teleop
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{"dev": "/dev/input/js0"}],
    )
    cartesian_teleop_node = Node(
        package="ur3e_teleop_joy",
        executable="cartesian_teleop",
        name="ur3e_cartesian_teleop",
        output="screen",
        parameters=[
            {
                "joy_topic": "/joy",
                "enable_button": 5,
                "deadzone": 0.15,
                "linear_scale": 0.1,
                "angular_scale": 0.8,
                "twist_topic": "/ur3e_servo/cartesian_twist",
                "command_frame": "tool0",
                "axis_left_x": 0,
                "axis_left_y": 1,
                "axis_right_x": 3,
                "axis_right_y": 4,
                "axis_lt": 2,
                "axis_rt": 5,
            }
        ],
    )

    # Switch Servo into Twist mode (command_type=1)
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

    # 5) (Optional) publish obstacles.yaml into MoveIt PlanningScene
    add_obstacles = Node(
        package="ur3e_haptic_scene",
        executable="add_obstacles",
        name="ur3e_add_obstacles",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    add_obstacles_delayed = TimerAction(
        period=6.0,
        actions=[add_obstacles],
        condition=IfCondition(publish_obstacles),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_ur_type)
    ld.add_action(declare_launch_rviz)
    ld.add_action(declare_publish_obstacles)

    # Order: simulation+MoveIt -> Servo/Teleop
    ld.add_action(sim_and_moveit)
    ld.add_action(servo_node)
    ld.add_action(joy_node)
    ld.add_action(cartesian_teleop_node)
    ld.add_action(switch_command_type)
    ld.add_action(add_obstacles_delayed)

    return ld
