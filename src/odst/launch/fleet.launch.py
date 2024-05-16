import os
import sys
import yaml
from odst.logs import LogDebug
from launch import LaunchDescription
from launch.event_handlers import OnShutdown
from launch.substitutions import LocalSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo


def generate_launch_description():
    """
    Prepare everything that needs to be launched for the overall application.

    Returns:
        LaunchDescription: the object comprised of the needed actions to launch
    """
    level = "info"
    pkg = "odst"
    launchfile = "ros_agents.launch.py"

    logger = LogDebug()
    launch_description = LaunchDescription()

    # Parse loglevel argument manually to use it in this script too
    for arg in sys.argv:
        if arg.startswith("loglevel:="):
            logger.setLevel(str(arg.split(":=")[1]))
            level = logger.getLevel()

    # Get the filepath to your config file
    config_file = os.path.join(get_package_share_directory(pkg), "config", "fleet.yaml")
    # Load the parameters specific to your ComposableNode
    with open(config_file, "r", encoding="utf8") as file:
        config_params = yaml.safe_load(file)["launchfile"]

    # Extract the number of agents
    agent_num = config_params["fleet_size"]

    # Script used when starting Gazebo
    additional_source_file = os.path.join(
        get_package_share_directory(pkg), "resource", "include_px4_models.sh"
    )

    source_local_wos = "source install/setup.bash"

    # World to use in Gazebo
    world_name = config_params["world"]
    logger.print(f"Using world {world_name}")

    # List of topics to bridge from Gazebo to ROS2
    bridges = ""

    # Get the filepath to your config file
    script_file = os.path.join(get_package_share_directory(pkg), "temp", "terminal.sh")
    # Write every gnome-terminal in a temp file and use that to spawn
    # everything in the same gnome-terminal window
    with open(script_file, "w+", encoding="utf8") as tmp:
        tmp.write("#!/bin/bash")  # Bash shebang
        tmp.write("\n")

        # Add the MicroXRCE agent command
        agent_cmd = (
            "gnome-terminal --tab -t 'MicroXRCE agent' "
            f"-- bash -c '{source_local_wos}; MicroXRCEAgent udp4 -p 8888'"
        )
        logger.print(agent_cmd)
        tmp.write(agent_cmd)
        tmp.write("\n")

        # Source a custom script to include the PX4 model folder, then manually start Gazebo;
        # without this, Gazebo would not find all needed models.
        # '-r' is needed to start the simulation right away, since PX4
        # checks for the Gazebo's clock values
        gazebo_server_cmd = (
            "gnome-terminal --tab -t 'Gazebo' "
            f"-- bash -c 'source {additional_source_file}; gz sim -r {world_name}.sdf'"
        )
        logger.print(gazebo_server_cmd)
        tmp.write(gazebo_server_cmd)
        tmp.write("\n")

        # Prepare each PX4 instance
        simulation_headstart = 10
        for count in range(agent_num):
            code = config_params["codes"][count]
            x = config_params["xs"][count]
            y = config_params["ys"][count]
            model = config_params["models"][count]

            # Add the command to start the PX4 simulation for the model
            px4_cmd = (
                f"PX4_SYS_AUTOSTART={code} PX4_GZ_MODEL_POSE='{x},{y}' "
                f"PX4_GZ_MODEL={model} ./build/px4_sitl_default/bin/px4 -i {count+1}"
            )
            # Add some wait time for subsequent spawns
            if count > 0:
                simulation_headstart += count
                px4_cmd = f"sleep {simulation_headstart}; {px4_cmd}"
            px4_cmd = (
                f"gnome-terminal --tab -t 'PX4 - {model}' "
                f"-- bash -c 'cd PX4-Autopilot; {px4_cmd}'"
            )
            logger.print(px4_cmd)
            tmp.write(px4_cmd)
            tmp.write("\n")

            bridges += f"/model/{model}_{count+1}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry "

        # Run ros_gz_bridge, as we need data from a Gazebo topic for the local position
        bridge_cmd = (
            "gnome-terminal --tab -t 'ROS2-Gazebo bridge' "
            f"-- bash -c '{source_local_wos}; "
            f"ros2 run ros_gz_bridge parameter_bridge {bridges}'"
        )
        logger.print(bridge_cmd)
        tmp.write(bridge_cmd)
        tmp.write("\n")

        # Manually execute launch file in order to get the logs in a standalone gnome terminal tab
        # Give an additional 10s to be sure everything has started correctly
        sleep_part = f"sleep {10 + simulation_headstart}"
        agents_launchfile = f"ros2 launch {pkg} {launchfile} loglevel:={level}"
        launch_cmd = (
            "gnome-terminal --tab -t 'ROS2 nodes' "
            f"-- bash -c ' {sleep_part}; {source_local_wos}; {agents_launchfile}'"
        )
        logger.print(launch_cmd)
        tmp.write(launch_cmd)
        tmp.write("\n")

        # Datapad node
        datapad_cmd = (
            "gnome-terminal --tab -t 'Datapad' "
            f"-- bash -c ' {sleep_part}; {source_local_wos}; "
            "ros2 run datapad datapad'"
        )
        logger.print(datapad_cmd)
        tmp.write(datapad_cmd)
        tmp.write("\n")

        # Interactive shell
        shell_cmd = "gnome-terminal --tab -t 'Interactive shell' -- bash"
        logger.print(shell_cmd)
        tmp.write(shell_cmd)
        tmp.write("\n")

    # Ensure the right permissions are set, to be able to execute the script
    os.chmod(script_file, 0o777)

    # Actually initiate spawn of everything
    bash_script = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--window",
            "--maximize",
            "--",
            f"{script_file}",
        ],
    )

    # Build the launch description
    launch_description.add_action(bash_script)

    return launch_description
