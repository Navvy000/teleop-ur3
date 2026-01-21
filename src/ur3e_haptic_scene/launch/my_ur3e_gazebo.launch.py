from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch Gazebo with the custom world my_ur3e_world.sdf.
    """
    pkg_share = get_package_share_directory("ur3e_haptic_scene")
    world_path = os.path.join(pkg_share, "worlds", "my_ur3e_world.sdf")

    # gz sim in Jazzy supports "-r" (run immediately). Older usage of "--pause"
    # is not supported by this frontend, so we emulate "paused" by omitting "-r".
    paused = LaunchConfiguration("paused")
    declare_paused = DeclareLaunchArgument(
        "paused",
        default_value="true",
        description="If true, start paused (do not pass -r). If false, start running immediately (-r).",
    )

    # Start paused (default)
    gz_cmd_paused = ["gz", "sim", world_path]
    # Start running immediately
    gz_cmd_run = ["gz", "sim", "-r", world_path]

    return LaunchDescription(
        [
            declare_paused,
            ExecuteProcess(
                condition=IfCondition(paused),
                cmd=gz_cmd_paused,
                output="screen",
            ),
            ExecuteProcess(
                condition=UnlessCondition(paused),
                cmd=gz_cmd_run,
                output="screen",
            ),
        ]
    )
