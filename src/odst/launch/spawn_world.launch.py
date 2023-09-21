"""
Older version, used with Ignition

Returns:
    _type_: _description_
"""
# from pathlib import Path  # Debug
from launch import LaunchDescription  # , LaunchContext  # Debug
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    launch_description = LaunchDescription()
    # LaunchConfigurations are used to store values of launch arguments in the above
    # variables and to pass them to required actions
    use_sim_time = LaunchConfiguration("use_sim_time")

    world_pkg_share = FindPackageShare("reach")
    world_middleware = "worlds/tugbot_depot/"
    world_filename = "tugbot_depot.sdf"

    ignition_pkg = FindPackageShare("ros_gz_sim")
    ignition_middleware = "launch"
    ignition_launchfile = "gz_sim.launch.py"

    # Debug
    # print(world_pkg_share.describe())
    # sub = PathJoinSubstitution([world_pkg_share, world_middleware, world_filename])
    # print(Path(sub.perform(LaunchContext())))

    # Arguments to be used in simulations only, not with real hw
    # From the docs: "A launch argument is stored in a "launch configuration" of the same name."

    # TODO: consider checking for headless mode?
    # headless_argument = DeclareLaunchArgument(
    #     name="headless",
    #     default_value="False",
    #     description="Whether to execute gzclient",
    # )

    use_sim_time_argument = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (a.k.a. Ignition) clock if true",
    )

    # TODO: spawn in second screen if available

    # Include the Ignition launch file
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [ignition_pkg, ignition_middleware, ignition_launchfile]
            )
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [world_pkg_share, world_middleware, world_filename]
            ),
        }.items(),
    )

    # Declare the launch options
    # Notice: you need to add this, otherwise the argument is not passed
    launch_description.add_action(use_sim_time_argument)

    # Add any actions
    launch_description.add_action(ignition)

    return launch_description
