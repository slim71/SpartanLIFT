import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from odst.helpers import print_launch_configuration
from odst.logs import LaunchfileLogger


def generate_launch_description():
    """
    Core functionality needed to actually use a launch file in ROS2.

    Returns:
        LaunchDescription: launch.LaunchDescription object at which actions
                           have been added
    """
    launch_pkg = "odst"
    model_pkg = "cargo"
    model_middleware = "models/"
    config_middleware = "config"
    config_yaml = "nominal.yaml"
    sdf_model = "model.sdf"

    launch_description = LaunchDescription()

    logger = LaunchfileLogger()
    # Parse loglevel argument manually to use it in this script too
    for arg in sys.argv:
        if arg.startswith("loglevel:="):
            logger.set_level(str(arg.split(":=")[1]))

    # Get the filepath to your config file
    config_file = os.path.join(
        get_package_share_directory(launch_pkg), config_middleware, config_yaml
    )
    # Load the parameters specific to your ComposableNode
    with open(config_file, "r", encoding="utf8") as file:
        config_params = yaml.safe_load(file)["simulation"]["cargo"]

    # Extract details abouth of the cargo
    cargo_name = config_params["name"]
    cargo_x = config_params["x"]
    cargo_y = config_params["y"]
    cargo_z = config_params["z"]
    logger.print(
        f"Model {cargo_name} will be spawned at "
        f"position ({cargo_x},{cargo_y},{cargo_z})"
    )

    # Gather log-level argument to use in the nodes
    log_level = LaunchConfiguration("loglevel", default="info")
    log_level_argument = DeclareLaunchArgument(
        name="loglevel",
        default_value="info",
        description="Log level of the launch file itself",
    )
    launch_description.add_action(log_level_argument)

    # # Spawn entity
    cargo_model = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            cargo_name,
            "-file",
            os.path.join(
                get_package_share_directory(model_pkg),
                model_middleware,
                cargo_name,
                sdf_model,
            ),
            "-allow_renaming",
            "true",  # Rename entity if name already used
            "-x",
            str(cargo_x),
            "-y",
            str(cargo_y),
            "-z",
            str(cargo_z),
        ],
        output="screen",
    )
    launch_description.add_action(cargo_model)

    # Start the ROS2 node too
    cargo_node = Node(
        package="cargo",
        executable="cargo",
        # Using `ros_arguments` is equivalent to using `arguments` with
        # a prepended '--ros-args' item.
        ros_arguments=[
            "--log-level",
            log_level,
        ],
    )
    launch_description.add_action(cargo_node)

    # Print argument values
    if logger.get_level() == "debug":
        print_launch_config = OpaqueFunction(function=print_launch_configuration)
        launch_description.add_action(print_launch_config)

    return launch_description
