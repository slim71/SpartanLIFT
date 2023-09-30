import sys
from odst.logs import print_argument  # Gives error in VSCode but it's valid
from pathlib import Path  # Debug
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

DEFAULT_AGENT_NUM = 3
DEFAULT_LOG_LEVEL = 'info'


def generate_launch_description():
    # Parse arguments manually to use them in this script too
    n_args = DEFAULT_AGENT_NUM
    level = DEFAULT_LOG_LEVEL
    for arg in sys.argv:
        if arg.startswith('agent_num:='):
            n_args = int(arg.split(':=')[1])
        if arg.startswith('loglevel:='):
            level = str(arg.split(':=')[1])

    launch_description = LaunchDescription()

    log_level = LaunchConfiguration('loglevel', default='info')
    log_level_argument = DeclareLaunchArgument(
        name='loglevel',
        default_value='info',
        description='Log level of the launch file itself',
    )
    launch_description.add_action(log_level_argument)

    files = [f'copter{x}.yaml' for x in range(1, n_args+1)]

    config_pkg_share = FindPackageShare('pelican')
    config_middleware = 'config/'

    if level == 'debug':
        sub = PathJoinSubstitution([config_pkg_share, config_middleware, files[0]])
        print(f'[DEBUG] Path to config files: {Path(sub.perform(LaunchContext()))}')

        print_launch_argument = OpaqueFunction(function=print_argument)
        launch_description.add_action(print_launch_argument)


    LogInfo(msg=f'Starting {n_args} agents...')
    for config_file in files:

        # ... Populate launch file like this:
        pelican_node = Node(
            package='pelican',
            executable='pelican',
            # Using `ros_arguments` is equivalent to using `arguments` with a prepended '--ros-args' item.
            ros_arguments=['--params-file', PathJoinSubstitution([config_pkg_share, config_middleware, config_file]),
                           '--log-level', log_level]
        )

        launch_description.add_action(pelican_node)

    return launch_description
