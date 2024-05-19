import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from odst.helpers import print_launch_configuration
from odst.logs import LaunchfileLogger


def generate_launch_description():
    """
    Core functionality needed to actually use a launch file in ROS2.

    Returns:
        LaunchDescription: launch.LaunchDescription object at which actions have been added
    """
    main_pkg = "pelican"
    model_middleware = "models/"
    sdf_model = "model.sdf"
    model_pkg_share = FindPackageShare(main_pkg)

    logger = LaunchfileLogger()
    # Parse loglevel argument manually to use it in this script too
    for arg in sys.argv:
        if arg.startswith("loglevel:="):
            logger.set_level(str(arg.split(":=")[1]))

    launch_description = LaunchDescription()

    world = LaunchConfiguration("world", default="empty.sdf")
    drone_name = LaunchConfiguration("name", default="my_drone")
    model_name = LaunchConfiguration("model", default="X3")
    spawn_x = LaunchConfiguration("x", default="0.0")
    spawn_y = LaunchConfiguration("y", default="0.0")
    spawn_z = LaunchConfiguration("z", default="0.0")
    spawn_roll = LaunchConfiguration("roll", default="0.0")
    spawn_pitch = LaunchConfiguration("pitch", default="0.0")
    spawn_yaw = LaunchConfiguration("yaw", default="0.0")

    # Pose where we want to spawn the robot
    # A launch argument is stored in a 'launch configuration' of the same name
    drone_name_argument = DeclareLaunchArgument(
        name="drone_name",
        default_value="my_drone",
        description="Name identificating the drone spawned",
    )
    model_name_argument = DeclareLaunchArgument(
        name="model_name",
        default_value="X3",
        description="Name identificating the model used in Gazebo for the drone spawned",
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
            PathJoinSubstitution([ros_gz_sim_pkg_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": world}.items(),
    )

    # Debug
    path_to_file = os.path.join(get_package_share_directory(main_pkg), model_middleware)
    logger.print(f"Path in which I'm searching the model: {path_to_file}")

    # Spawn entity
    drone = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            drone_name,
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
    launch_description.add_action(world_argument)
    launch_description.add_action(drone_name_argument)
    launch_description.add_action(model_name_argument)
    launch_description.add_action(spawn_x_argument)
    launch_description.add_action(spawn_y_argument)
    launch_description.add_action(spawn_z_argument)
    launch_description.add_action(spawn_roll_argument)
    launch_description.add_action(spawn_pitch_argument)
    launch_description.add_action(spawn_yaw_argument)
    launch_description.add_action(drone)

    # Print argument values
    if logger.get_level() == "debug":
        print_launch_config = OpaqueFunction(function=print_launch_configuration)
        launch_description.add_action(print_launch_config)

    return launch_description
