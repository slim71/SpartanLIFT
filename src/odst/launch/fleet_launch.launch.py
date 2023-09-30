from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from odst.logs import print_argument  # Gives error in VSCode but it's valid


def generate_launch_description():
    launch_description = LaunchDescription()

    launch_middleware = "launch"
    launch_file = "any_model_spawn.launch.py"
    this_pkg_share = PathJoinSubstitution(
        [FindPackageShare("odst"), launch_middleware, launch_file]
    )

    launch_file_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(this_pkg_share),
        launch_arguments={
            'x': '-10', 'y': '-10', 'model': 'X3'
            }.items(),
        )

    launch_description.add_action(launch_file_description)

    return launch_description
