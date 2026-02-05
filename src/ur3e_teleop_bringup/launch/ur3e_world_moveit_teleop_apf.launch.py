#!/usr/bin/env python3
"""Integrated bringup for APF targeted teleop demos.

This launch reuses the existing integrated simulation + MoveIt + Servo + joystick
teleop pipeline, and layers the APF-related nodes from `ur3e_haptic_scene` on top:

  - target_selector: keyboard index -> /selected_target
  - virtual_force: APF attractive + repulsive, publishes /virtual_force + markers

Defaults are chosen to match existing project configuration:
  - base_frame defaults to Servo `planning_frame` (base_link)
  - ee_frame defaults to Servo `ee_frame` (tool0)
  - yaml_file defaults to ur3e_haptic_scene/config/obstacles.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---------------------------------------------------------------------
    # Bringup layer (simulation + MoveIt + Servo + joystick teleop)
    # ---------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    publish_obstacles = LaunchConfiguration("publish_obstacles")

    bringup_share = get_package_share_directory("ur3e_teleop_bringup")
    world_moveit_teleop_launch = os.path.join(
        bringup_share, "launch", "ur3e_world_moveit_teleop.launch.py"
    )
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_moveit_teleop_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "ur_type": ur_type,
            "launch_rviz": launch_rviz,
            "publish_obstacles": publish_obstacles,
        }.items(),
    )

    # ---------------------------------------------------------------------
    # APF layer (haptic scene nodes)
    # ---------------------------------------------------------------------
    haptic_share = get_package_share_directory("ur3e_haptic_scene")
    default_yaml = os.path.join(haptic_share, "config", "obstacles.yaml")

    yaml_file = LaunchConfiguration("yaml_file")
    base_frame = LaunchConfiguration("base_frame")
    ee_frame = LaunchConfiguration("ee_frame")

    # Selector defaults
    start_target_selector = LaunchConfiguration("start_target_selector")
    publish_initial_target = LaunchConfiguration("publish_initial_target")
    initial_target_index = LaunchConfiguration("initial_target_index")

    # Virtual force defaults / tuning (only pass parameters that node actually declares)
    start_virtual_force = LaunchConfiguration("start_virtual_force")
    virtual_force_rate = LaunchConfiguration("virtual_force_rate")
    d_safe = LaunchConfiguration("d_safe")
    d_stop = LaunchConfiguration("d_stop")

    target_selector_node = Node(
        package="ur3e_haptic_scene",
        executable="target_selector",
        name="target_selector",
        output="screen",
        parameters=[
            {"yaml_file": yaml_file},
            {"publish_initial_target": publish_initial_target},
            {"initial_target_index": initial_target_index},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(start_target_selector),
    )

    virtual_force_node = Node(
        package="ur3e_haptic_scene",
        executable="virtual_force",
        name="ur3e_virtual_force",
        output="screen",
        parameters=[
            {"yaml_file": yaml_file},
            {"base_frame": base_frame},
            {"ee_frame": ee_frame},
            {"rate": virtual_force_rate},
            {"F_att_max": LaunchConfiguration("F_att_max")},
            {"d_safe": d_safe},
            {"d_stop": d_stop},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(start_virtual_force),
    )

    # ---------------------------------------------------------------------
    # Optional pre-teleop staging pose (MoveIt plan+execute to a safe posture)
    # ---------------------------------------------------------------------
    run_staging = LaunchConfiguration("run_staging")
    staging_delay = LaunchConfiguration("staging_delay")

    default_staging_yaml = os.path.join(bringup_share, "config", "staging_pose.yaml")
    staging_yaml = LaunchConfiguration("staging_yaml")

    staging_node = Node(
        package="ur3e_teleop_bringup",
        executable="staging_pose_executor.py",
        name="staging_pose_executor",
        output="screen",
        parameters=[staging_yaml, {"use_sim_time": use_sim_time}],
        condition=IfCondition(run_staging),
    )
    staging_node_delayed = TimerAction(
        period=staging_delay,
        actions=[staging_node],
        condition=IfCondition(run_staging),
    )

    # ---------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------
    declared_arguments = []

    # Bringup args
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            description="UR robot type (e.g. ur3e)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz together with MoveIt",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_obstacles",
            default_value="true",
            description="If true, publish obstacles.yaml into MoveIt PlanningScene",
        )
    )

    # Staging args
    declared_arguments.append(
        DeclareLaunchArgument(
            "run_staging",
            default_value="true",
            description="If true, execute a pre-teleop staging joint pose using MoveIt plan+execute",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "staging_delay",
            default_value="10.0",
            description="Delay before running staging pose [s] (wait for move_group/controllers)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "staging_yaml",
            default_value=default_staging_yaml,
            description="YAML file containing staging pose parameters",
        )
    )

    # APF args (frame defaults are derived from ur3e_servo_cartesian.yaml)
    declared_arguments.append(
        DeclareLaunchArgument(
            "yaml_file",
            default_value=default_yaml,
            description="Scene objects YAML used by add_obstacles/virtual_force/target_selector",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="TF base frame for APF computation (Servo planning_frame)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_frame",
            default_value="tool0",
            description="TF end effector frame for APF computation (Servo ee_frame)",
        )
    )

    # Selector args
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_target_selector",
            default_value="false",
            description=(
                "Start the keyboard target selector node. "
                "Recommended workflow: run target_selector in a separate terminal."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_initial_target",
            default_value="true",
            description="If true, selector publishes an initial target at startup",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_target_index",
            default_value="1",
            description="Initial target index (must match obstacles.yaml 'index' field)",
        )
    )

    # Virtual force args
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_virtual_force",
            default_value="true",
            description="Start the APF virtual force node",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "virtual_force_rate",
            default_value="50.0",
            description="Virtual force update rate [Hz]",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "F_att_max",
            default_value="35.0",
            description="Attractive force saturation [N]",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "d_safe",
            default_value="0.15",
            description="Repulsive activation distance [m]",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "d_stop",
            default_value="0.02",
            description="Repulsive stop distance [m]",
        )
    )
    return LaunchDescription(
        declared_arguments + [bringup, staging_node_delayed, target_selector_node, virtual_force_node]
    )
