from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def drone_setup(context, *args, **kwargs):
    drone_name = LaunchConfiguration('name', default='my_drone').perform(context)
    model_name = LaunchConfiguration('model', default='X3').perform(context)
    spawn_x = LaunchConfiguration('x', default='0.0').perform(context)
    spawn_y = LaunchConfiguration('y', default='0.0').perform(context)
    spawn_z = LaunchConfiguration('z', default='0.0').perform(context)
    spawn_roll = LaunchConfiguration('roll', default='0.0').perform(context)
    spawn_pitch = LaunchConfiguration('pitch', default='0.0').perform(context)
    spawn_yaw = LaunchConfiguration('yaw', default='0.0').perform(context)

    model_pkg_share = FindPackageShare('pelican')
    model_middleware = f'models/{model_name}/'
    sdf_model = 'model.sdf'

    # Spawn entity
    drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', drone_name,
                   '-file', PathJoinSubstitution([model_pkg_share, model_middleware, sdf_model]),
                    '-allow_renaming', 'true',  # Rename entity if name already used
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z,
                    '-R', spawn_roll,
                    '-P', spawn_pitch,
                    '-Y', spawn_yaw,
                    ],
        output='screen'
    )

    return [drone]

def generate_launch_description():
    launch_description = LaunchDescription()

    world = LaunchConfiguration('world', default='empty.sdf')

    # Pose where we want to spawn the robot
    # A launch argument is stored in a 'launch configuration' of the same name
    drone_name_argument = DeclareLaunchArgument(
        name='drone_name',
        default_value='my_drone',
        description='Name identificating the drone spawned',
    )
    model_name_argument = DeclareLaunchArgument(
        name='model_name',
        default_value='X3',
        description='Name identificating the model used in Gazebo for the drone spawned',
    )
    spawn_x_argument = DeclareLaunchArgument(
        name='spawn_x',
        default_value='0.0',
        description='Position along x-axis',
    )
    spawn_y_argument = DeclareLaunchArgument(
        name='spawn_y',
        default_value='0.0',
        description='Position along y-axis',
    )
    spawn_z_argument = DeclareLaunchArgument(
        name='spawn_z',
        default_value='0.0',
        description='Position along z-axis',
    )
    spawn_roll_argument = DeclareLaunchArgument(
        name='spawn_roll',
        default_value='0.0',
        description='Rotation around x-axis',
    )
    spawn_pitch_argument = DeclareLaunchArgument(
        name='spawn_pitch',
        default_value='0.0',
        description='Rotation around y-axis',
    )
    spawn_yaw_argument = DeclareLaunchArgument(
        name='spawn_yaw',
        default_value='0.0',
        description='Rotation around z-axis',
    )
    world_argument = DeclareLaunchArgument(
        name='world',
        default_value='empty.sdf',
        description='World in which to spawn everything in',
    )

    ros_gz_sim_pkg_share = FindPackageShare('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim_pkg_share, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={
                'gz_args': world
                }.items(),
        )

    drone_opaquefun = OpaqueFunction(function = drone_setup)

    launch_description.add_action(gazebo)
    launch_description.add_action(drone_opaquefun)
    launch_description.add_action(world_argument)
    launch_description.add_action(drone_name_argument)
    launch_description.add_action(model_name_argument)
    launch_description.add_action(spawn_x_argument)
    launch_description.add_action(spawn_y_argument)
    launch_description.add_action(spawn_z_argument)
    launch_description.add_action(spawn_roll_argument)
    launch_description.add_action(spawn_pitch_argument)
    launch_description.add_action(spawn_yaw_argument)

    return launch_description
