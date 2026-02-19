from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("tekbot_navigation"),
        "rviz",
        "tekbot_config_b2ms.rviz"]
    )

    return LaunchDescription([

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
