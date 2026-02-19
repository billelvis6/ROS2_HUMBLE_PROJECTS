from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    # Chemin de la configuration de nav2 
    params_file = PathJoinSubstitution([
        FindPackageShare("tekbot_navigation"),
        "config",
        "nav2_params.yaml"
    ])

    # Chemin du fichier RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("tekbot_navigation"),
        "rviz",
        "tekbot_config_nav_b2ms.rviz"
    ])

    # Chemin de la carte YAML
    my_map = PathJoinSubstitution([
        FindPackageShare("tekbot_navigation"),
        "maps",
        "tekbot_map.yaml"
    ])

    # Lancement de la localisation (amcl)
    nav2_launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "localization_launch.py"
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': my_map
        }.items()
    )

    # Lancement de la navigation

    nav2_launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
            'map_subscriber_transient_local': 'true'
        }.items()
    )

    # Lancement de RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        nav2_launch_localization,
        nav2_launch_navigation,
        rviz_node
    ])
