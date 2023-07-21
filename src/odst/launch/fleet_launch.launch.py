from pathlib import Path  # Debug
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

AGENT_NUM = 3  # TODO: add optionally as input from CLI


def generate_launch_description():
    ld = LaunchDescription()

    files = ['copter{}.yaml'.format(x) for x in range(1,AGENT_NUM+1)]

    config_pkg_share = FindPackageShare("pelican")
    config_middleware = "config/"

    for config_file in files:
        # Debug
        print(config_pkg_share.describe())
        sub = PathJoinSubstitution([config_pkg_share, config_middleware, config_file])
        print(Path(sub.perform(LaunchContext())))

        # ... Populate launch file like this:
        pelican_unit = Node(
            package='pelican',
            executable='pelican_listener',
            # Using `ros_arguments` is equivalent to using `arguments` with a prepended '--ros-args' item.
            ros_arguments=['--params-file', PathJoinSubstitution([config_pkg_share, config_middleware, config_file])]
        )
        
        ld.add_action(pelican_unit)

    return ld