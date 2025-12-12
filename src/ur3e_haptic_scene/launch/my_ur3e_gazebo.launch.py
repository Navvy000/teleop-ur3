from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch Gazebo with the custom world my_ur3e_world.sdf.
    """
    pkg_share = get_package_share_directory("ur3e_haptic_scene")
    world_path = os.path.join(pkg_share, "worlds", "my_ur3e_world.sdf")

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gz", "sim", "--pause", world_path],
                output="screen"
            ),
        ]
    )
