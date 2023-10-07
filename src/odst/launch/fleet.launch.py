import os
import sys
import yaml
from odst.logs import LogDebug  # Gives error in VSCode but it's valid
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    logger = LogDebug()

    # Parse arguments manually to use them in this script too
    for arg in sys.argv:
        if arg.startswith('loglevel:='):
            logger.setLevel(str(arg.split(':=')[1]))

    launch_description = LaunchDescription()

    # Get the filepath to your config file
    config_file = os.path.join(
        get_package_share_directory('odst'),
        'config',
        'fleet.yaml'
    )
    # Load the parameters specific to your ComposableNode
    with open(config_file, 'r', encoding='utf8') as file:
        config_params = yaml.safe_load(file)['launchfile']

    # Extract the number of agents
    agent_num = config_params['fleet_size']

    # Spawn RVIZ
    rviz = Node(package='rviz2', namespace='', executable='rviz2', name='rviz2')

    # Get the filepath to your config file
    script_file = os.path.join(
        get_package_share_directory('odst'),
        'temp',
        'terminal.sh'
    )
    # Write every gnome-terminal in a temp file and use that to spawn everything in the same gnome-terminal window
    with open(script_file, 'w+', encoding='utf8') as tmp:
        tmp.write('#!/bin/bash')  # Bash shebang
        tmp.write('\n')

        # Add the MicroXRCE agent command
        agent_cmd = 'gnome-terminal --tab -t MicroXRCE agent -- bash -c \'source install/setup.bash; MicroXRCEAgent udp4 -p 8888\''
        logger.print(agent_cmd)
        tmp.write(agent_cmd)
        tmp.write('\n')

        # Prepare each PX4 instance
        for count in range(agent_num):
            code = config_params['codes'][count]
            x = config_params['xs'][count]
            y = config_params['ys'][count]
            model = config_params['models'][count]

            # Add the command to start the PX4 simulation for the model
            px4_cmd = f'PX4_SYS_AUTOSTART={code} PX4_GZ_MODEL_POSE=\'{x},{y}\' PX4_GZ_MODEL={model} ./build/px4_sitl_default/bin/px4 -i {count}'
            # Add some wait time for subsequent spawns
            if count > 0:
                px4_cmd = f'sleep {5+count}; {px4_cmd}'
            px4_cmd = f'gnome-terminal --tab -t \'PX4 - {model}\' -- bash -c \'cd PX4-Autopilot; {px4_cmd}\''
            logger.print(px4_cmd)
            tmp.write(px4_cmd)
            tmp.write('\n')

    # Ensure the right permissions are set, to be able to execute the script
    os.chmod(script_file, 0o777)

    # Actually initiate spawn of everything
    px4 = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--window',
            '--maximize',
            '--',
            f'{script_file}',
        ],
    )

    # Build the launch description
    launch_description.add_action(px4)
    # launch_description.add_action(rviz)

    return launch_description
