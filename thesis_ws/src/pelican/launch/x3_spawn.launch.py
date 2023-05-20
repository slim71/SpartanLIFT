# from pathlib import Path  # Debug
from launch import LaunchDescription  # , LaunchContext  # Debug
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


# TODO: move to launch package?
def generate_launch_description():
    launch_description = LaunchDescription()

    model_pkg_share = FindPackageShare("pelican")
    model_middleware = "models/X3/"
    sdf_model = "model.sdf"

    # Pose where we want to spawn the robot
    spawn_x = "0.0"
    spawn_y = "0.0"
    spawn_z = "0.0"
    spawn_yaw = "0.0"
    spawn_pitch = "0.0"
    spawn_roll = "0.0"

    # Debug
    # print(model_pkg_share.describe())
    # sub = PathJoinSubstitution([model_pkg_share, model_middleware, sdf_model])
    # print(Path(sub.perform(LaunchContext())))

    # Spawn entity
    drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_bot',
                   '-file', PathJoinSubstitution([model_pkg_share, model_middleware, sdf_model]),
                    '-allow_renaming', 'true',
                    "-x", spawn_x,
                    "-y", spawn_y,
                    "-z", spawn_z,
                    "-X", spawn_roll,
                    "-Y", spawn_pitch,
                    "-Z", spawn_yaw,
                    ],
        output='screen'
    )

    launch_description.add_action(drone)

    return launch_description
