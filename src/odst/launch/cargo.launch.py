import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from odst.logs import LaunchfileLogger
from odst.helpers import print_launch_configuration


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
    config_file = "cargo.yaml"
    sdf_model = "model.sdf"

    logger = LaunchfileLogger()
    # Parse loglevel argument manually to use it in this script too
    for arg in sys.argv:
        if arg.startswith("loglevel:="):
            logger.set_level(str(arg.split(":=")[1]))

    # Get the filepath to your config file
    config_file = os.path.join(
        get_package_share_directory(launch_pkg), config_middleware, config_file
    )
    # Load the parameters specific to your ComposableNode
    with open(config_file, "r", encoding="utf8") as file:
        config_params = yaml.safe_load(file)["cargo"]

    # Extract details abouth of the cargo
    cargo_name = config_params["name"]
    cargo_x = config_params["x"]
    cargo_y = config_params["y"]
    cargo_z = config_params["z"]
    logger.print(
        f"Model {cargo_name} will be spawned at "
        f"position ({cargo_x},{cargo_y},{cargo_z})"
    )

    launch_description = LaunchDescription()

    # # Spawn entity
    box = Node(
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

    launch_description.add_action(box)

    # Print argument values
    if logger.get_level() == "debug":
        print_launch_config = OpaqueFunction(
            function=print_launch_configuration
        )
        launch_description.add_action(print_launch_config)

    return launch_description
