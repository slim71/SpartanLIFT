import os
import sys
import yaml
from odst.logs import LogDebug  # Gives error in VSCode but it's valid
from pathlib import Path  # Debug
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = "odst"
    main_pkg = "pelican"
    config_middleware = "config/"

    logger = LogDebug()
    launch_description = LaunchDescription()
    config_pkg_share = FindPackageShare(main_pkg)

    # Parse loglevel argument manually to use it in this script too
    for arg in sys.argv:
        if arg.startswith("loglevel:="):
            logger.setLevel(str(arg.split(":=")[1]))

    # Get the filepath to your config file
    config_file = os.path.join(get_package_share_directory(pkg), "config", "fleet.yaml")
    # Load the parameters specific to your ComposableNode
    with open(config_file, "r", encoding="utf8") as file:
        config_params = yaml.safe_load(file)["launchfile"]

    # Extract the number of agents
    agent_num = config_params["fleet_size"]
    logger.print(f"Agents to spawn: {agent_num}")

    # Gather log-level argument to use in the nodes
    log_level = LaunchConfiguration("loglevel", default="info")
    log_level_argument = DeclareLaunchArgument(
        name="loglevel",
        default_value="info",
        description="Log level of the launch file itself",
    )
    launch_description.add_action(log_level_argument)

    # Define the config files name to use
    files = [f"copter{x}.yaml" for x in range(1, agent_num + 1)]
    sub = PathJoinSubstitution([config_pkg_share, config_middleware, files[0]])
    logger.print(f"Path to config files: {Path(sub.perform(LaunchContext()))}")

    # Actually populate launch actions with nodes
    logger.print(f"Starting {agent_num} agents...")
    for config_file in files:
        pelican_node = Node(
            package="pelican",
            executable="pelican",
            # Using `ros_arguments` is equivalent to using `arguments` with
            # a prepended '--ros-args' item.
            ros_arguments=[
                "--params-file",
                PathJoinSubstitution(
                    [config_pkg_share, config_middleware, config_file]
                ),
                "--log-level",
                log_level,
            ],
        )
        launch_description.add_action(pelican_node)

    return launch_description
