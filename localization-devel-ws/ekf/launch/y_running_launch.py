from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rival_name = LaunchConfiguration('rival_name')
    side = LaunchConfiguration('side')

    rival_config_path = PathJoinSubstitution([
        FindPackageShare('rival_localization'),
        'config',
        'rival_localization.yaml'
    ])

    local_filter_launch_path = PathJoinSubstitution([
        FindPackageShare('local_filter'),
        'launch',
        'local_filter_whole.xml'
    ])

    # rplidar_launch = PathJoinSubstitution([
    #     FindPackageShare('lidar_localization_pkg'),
    #     'launch',
    #     'firmware',
    #     'rplidar_s3_launch.py'
    # ])
    ydlidar_launch = PathJoinSubstitution([
        FindPackageShare('lidar_localization_pkg'),
        'launch',
        'firmware',
        'ydlidar_launch.py'
    ])

    obstacle_extractor_launch = PathJoinSubstitution([
        FindPackageShare('lidar_localization_pkg'),
        'launch',
        'obstacle_extractor_launch.xml'
    ])

    healthcheck_node = Node(
        package='healthcheck',
        executable='healthcheck_node',
        name='healthcheck_node',
        output='screen'
    )

    ekf_node = Node(
        package='ekf',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[{
            'use_cam': 0,
            'robot_parent_frame_id': 'map',
            'robot_frame_id': 'base_footprint',
            '/use_sim_time': False,
            'q_linear': 1.2e-5,
            'q_angular': 1.7e-6,
            'r_camra_linear': 1e-2,
            'r_camra_angular': 0.15,
            'r_threshold_xy': 1e-3,
            'r_threshold_theta': 1e-2
        }],
        remappings=[
            ('initalpose', ['initial_pose'])
        ]

    )

    lidar_node = Node(
        package='lidar_localization_pkg',
        executable='lidar_localization',
        name='lidar_localization',
        output='screen',
        parameters=[{
            'side': side,
            'debug_mode': False,
            'visualize_candidate': True,
            'likelihood_threshold': 0.8,
            'consistency_threshold': 0.95,
            'lidar_multiplier': 0.987
        }]
    )

    local_filter_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(local_filter_launch_path)
    )

    rival_node = Node(
        package='rival_localization',
        executable='rival_localization',
        name='rival_localization',
        output='screen',
        parameters=[rival_config_path],
        remappings=[
            ('raw_pose', [rival_name, '/raw_pose']),
            ('final_pose', [rival_name, '/final_pose'])
        ]
    )

    rival_obstacle_node = GroupAction([
        SetLaunchConfiguration('ros_namespace', rival_name),
        Node(
            package='obstacle_detector',
            executable='obstacle_extractor_node',
            name='obstacle_detector_to_map',
            parameters=[
                rival_config_path,
                {'frame_id': 'map'}
            ],
            remappings=[
                ('raw_obstacles', '/obstacles_to_map'),
                ('scan', '/scan')
            ]
        )
    ])

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_to_laser',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '-1.6057',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'laser'
        ]
    )

    # rplidar_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(rplidar_launch))
    ydlidar_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(ydlidar_launch))
    obstacle_extractor_include = IncludeLaunchDescription(AnyLaunchDescriptionSource(obstacle_extractor_launch))

    return LaunchDescription([
        DeclareLaunchArgument('rival_name', default_value='rival'),
        DeclareLaunchArgument('side', default_value='0'),

        static_tf,
        ydlidar_include,
        obstacle_extractor_include,

        healthcheck_node,

        TimerAction(period=2.0, actions=[ekf_node]),
        TimerAction(period=4.0, actions=[lidar_node]),
        TimerAction(period=6.0, actions=[local_filter_launch]),
        TimerAction(period=8.0, actions=[rival_node, rival_obstacle_node])
    ])
