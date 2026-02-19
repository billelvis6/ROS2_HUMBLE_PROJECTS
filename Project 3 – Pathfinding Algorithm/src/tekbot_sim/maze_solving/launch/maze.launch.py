from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
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

    ld = LaunchDescription()
    ld.add_action(gz_model)
    ld.add_action(gazebo_sim)

    return ld
