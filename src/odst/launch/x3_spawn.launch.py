# from pathlib import Path  # Debug
from launch import LaunchDescription  # , LaunchContext  # Debug
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    launch_description = LaunchDescription()
    drone_name = LaunchConfiguration("name", default='my_drone')
    spawn_x = LaunchConfiguration("x", default='0.0')
    spawn_y = LaunchConfiguration("y", default='0.0')
    spawn_z = LaunchConfiguration("z", default='0.0')
    spawn_roll = LaunchConfiguration("roll", default='0.0')
    spawn_pitch = LaunchConfiguration("pitch", default='0.0')
    spawn_yaw = LaunchConfiguration("yaw", default='0.0')

    model_pkg_share = FindPackageShare("pelican")
    model_middleware = "models/X3/"
    sdf_model = "model.sdf"

    # Pose where we want to spawn the robot
    # A launch argument is stored in a "launch configuration" of the same name
    drone_name_argument = DeclareLaunchArgument(
        name="drone_name",
        default_value="my_drone",
        description="Name identificating the drone spawned",
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

    # Debug
    # print(model_pkg_share.describe())
    # sub = PathJoinSubstitution([model_pkg_share, model_middleware, sdf_model])
    # print(Path(sub.perform(LaunchContext())))

    # Spawn entity
    drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', drone_name,
                   '-file', PathJoinSubstitution([model_pkg_share, model_middleware, sdf_model]),
                    '-allow_renaming', 'true',  # Rename entity if name already used
                    "-x", spawn_x,
                    "-y", spawn_y,
                    "-z", spawn_z,
                    "-R", spawn_roll,
                    "-P", spawn_pitch,
                    "-Y", spawn_yaw,
                    ],
        output='screen'
    )

    launch_description.add_action(drone)
    launch_description.add_action(drone_name_argument)
    launch_description.add_action(spawn_x_argument)
    launch_description.add_action(spawn_y_argument)
    launch_description.add_action(spawn_z_argument)
    launch_description.add_action(spawn_roll_argument)
    launch_description.add_action(spawn_pitch_argument)
    launch_description.add_action(spawn_yaw_argument)

    return launch_description
