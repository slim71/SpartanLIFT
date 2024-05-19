import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from odst.logs import LaunchfileLogger
from odst.helpers import print_launch_configuration


def generate_launch_description():
    """
    Core functionality needed to actually use a launch file in ROS2.

    Returns:
        LaunchDescription: launch.LaunchDescription object at which actions
                           have been added
    """
    model_pkg = "cargo"
    model_middleware = "models/"
    sdf_model = "model.sdf"
    model_pkg_share = FindPackageShare(model_pkg)

    logger = LaunchfileLogger()
    # Parse loglevel argument manually to use it in this script too
    for arg in sys.argv:
        if arg.startswith("loglevel:="):
            logger.set_level(str(arg.split(":=")[1]))

    launch_description = LaunchDescription()
    world = LaunchConfiguration("world", default="empty.sdf")
    box_name = LaunchConfiguration("name", default="my_box")
    model_name = LaunchConfiguration("model", default="box_w_handles")
    spawn_x = LaunchConfiguration("x", default="0.0")
    spawn_y = LaunchConfiguration("y", default="0.0")
    spawn_z = LaunchConfiguration("z", default="0.0")
    spawn_roll = LaunchConfiguration("roll", default="0.0")
    spawn_pitch = LaunchConfiguration("pitch", default="0.0")
    spawn_yaw = LaunchConfiguration("yaw", default="0.0")

    # Pose where we want to spawn the box
    # A launch argument is stored in a "launch configuration" of the same name
    box_name_argument = DeclareLaunchArgument(
        name="box_name",
        default_value="my_box",
        description="Name identificating the cargo spawned",
    )
    model_name_argument = DeclareLaunchArgument(
        name="model_name",
        default_value="box_w_handles",
        description="Name of the sdf model used",
    )
    spawn_x_argument = DeclareLaunchArgument(
        name="spawn_x",
        default_value="0.0",
        description="Position along x-axis",
    )
    spawn_y_argument = DeclareLaunchArgument(
        name="spawn_y",
        default_value="0.0",
        description="Position along y-axis",
    )
    spawn_z_argument = DeclareLaunchArgument(
        name="spawn_z",
        default_value="0.0",
        description="Position along z-axis",
    )
    spawn_roll_argument = DeclareLaunchArgument(
        name="spawn_roll",
        default_value="0.0",
        description="Rotation around x-axis",
    )
    spawn_pitch_argument = DeclareLaunchArgument(
        name="spawn_pitch",
        default_value="0.0",
        description="Rotation around y-axis",
    )
    spawn_yaw_argument = DeclareLaunchArgument(
        name="spawn_yaw",
        default_value="0.0",
        description="Rotation around z-axis",
    )
    world_argument = DeclareLaunchArgument(
        name="world",
        default_value="empty.sdf",
        description="World in which to spawn everything in",
    )

    ros_gz_sim_pkg_share = FindPackageShare("ros_gz_sim")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [ros_gz_sim_pkg_share, "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": world}.items(),
    )

    # Debug
    path_to_file = os.path.join(
        get_package_share_directory(model_pkg), model_middleware
    )
    logger.print(f"Path in which I'm searching the model: {path_to_file}")

    # Spawn entity
    box = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            box_name,
            "-file",
            PathJoinSubstitution(
                [model_pkg_share, model_middleware, model_name, sdf_model]
            ),
            "-allow_renaming",
            "true",  # Rename entity if name already used
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-R",
            spawn_roll,
            "-P",
            spawn_pitch,
            "-Y",
            spawn_yaw,
        ],
        output="screen",
    )

    launch_description.add_action(gazebo)
    launch_description.add_action(box_name_argument)
    launch_description.add_action(model_name_argument)
    launch_description.add_action(spawn_x_argument)
    launch_description.add_action(spawn_y_argument)
    launch_description.add_action(spawn_z_argument)
    launch_description.add_action(spawn_roll_argument)
    launch_description.add_action(spawn_pitch_argument)
    launch_description.add_action(spawn_yaw_argument)
    launch_description.add_action(world_argument)
    launch_description.add_action(box)

    # Print argument values
    if logger.get_level() == "debug":
        print_launch_config = OpaqueFunction(
            function=print_launch_configuration
        )
        launch_description.add_action(print_launch_config)

    return launch_description
