import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('x_init',
            default_value='-2.5',
            description='robot initial x'),
    DeclareLaunchArgument('y_init',
            default_value='-0.5',
            description='robot initial y'),
    DeclareLaunchArgument('z_init',
            default_value='0.0',
            description='robot initial z'),
    DeclareLaunchArgument('yaw_init',
            default_value='0.0',
            description='robot initial yaw')
]

def generate_launch_description():

    model_path = PathJoinSubstitution(
        [FindPackageShare("maze_solving"), "models"]
    )

    gz_model = SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=model_path
    )

    world_file = PathJoinSubstitution(
        [FindPackageShare("maze_solving"),
        "worlds",
        "maze_4_metal_6x6.world"],
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("maze_solving"),
        "launch",
        "empty_world.launch.py"],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world_path': world_file}.items(),
    )
    
    tekbot_description_path = PathJoinSubstitution(
        [FindPackageShare("tekbot_description"),
        "launch",
        "description.launch.py"]
    )
    tekbot_description_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tekbot_description_path]))
    
    tekbot_teleop_path = PathJoinSubstitution(
        [FindPackageShare("tekbot_control"),
        "launch",
        "tekbot_teleop_joy.launch.py"]
    )
    tekbot_teleop_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tekbot_teleop_path]))

    # Pose we want to spawn the robot
    spawn_x_val = LaunchConfiguration('x_init')
    spawn_y_val = LaunchConfiguration('y_init')
    spawn_z_val = LaunchConfiguration('z_init')
    spawn_yaw_val = LaunchConfiguration('yaw_init')

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', "tekbot", 
                    '-topic', 'robot_description',
                        '-x', spawn_x_val,
                        '-y', spawn_y_val,
                        '-z', spawn_z_val,
                        '-Y', spawn_yaw_val],
                        output='screen',
        parameters=[{'use_sim_time': True}])
    
    rqt = Node(
                package='rqt_gui',
                executable='rqt_gui',
                name='rqt',
                output='screen'
            )
    
    # Create the launch description and populate
    ld = LaunchDescription(ARGUMENTS)

    # Add any actions
    ld.add_action(gz_model)
    ld.add_action(gazebo_sim)
    ld.add_action(tekbot_description_launch)
    ld.add_action(tekbot_teleop_launch)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(rqt)

    return ld
