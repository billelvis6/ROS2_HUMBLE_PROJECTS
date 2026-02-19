import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction

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

    tekbot_maze_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [PathJoinSubstitution([
            FindPackageShare("maze_solving"),
            "launch",
            "tekbot_maze.launch.py"
        ])]
    ),
    launch_arguments={
        'x_init': LaunchConfiguration('x_init'),
        'y_init': LaunchConfiguration('y_init'),
        'z_init': LaunchConfiguration('z_init'),
        'yaw_init': LaunchConfiguration('yaw_init'),
    }.items()
)


    # Chemin de la configuration de nav2 (avec Hybrid-A*)
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
            'map': my_map,
        }.items()
    )


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


    # donne la position initiale après 3s via /initialpose
    publish_initialpose = TimerAction(
    period=3.0,  
    actions=[
        ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
             '{header: {frame_id: "map"}, pose: {pose: {position: {x: -2.5, y: -0.5, z: 0.0}}}}'
        ],
        output='screen'
        )
        ]
    )
   
    # donne la position à atteindre après 6s via /goal_pose
    publish_goalpose = TimerAction(
    period=6.0,  
    actions=[
        ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/goal_pose', 'geometry_msgs/msg/PoseStamped',
             '{header: {frame_id: "odom"}, pose: {position: {x: 2.5, y: 2.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
        ],
        output='screen'
        )
        ]
    )
    
    
    # Create the launch description and populate
    ld = LaunchDescription(ARGUMENTS)
    
    ld.add_action(tekbot_maze_launch)


    ld.add_action(nav2_launch_localization)
    ld.add_action(nav2_launch_navigation)
    ld.add_action(rviz_node)

    ld.add_action(publish_initialpose) 
    ld.add_action(publish_goalpose) 


    return ld
