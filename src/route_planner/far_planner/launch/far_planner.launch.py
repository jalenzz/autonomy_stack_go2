from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Launch FAR Planner, its RViz view, and the graph decoder."""
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        DeclareLaunchArgument('config', default_value='default'),

        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            output='screen',
            parameters=[
                PythonExpression([
                    '"',
                    get_package_share_directory('far_planner'),
                    '/config/',
                    LaunchConfiguration('config'),
                    '.yaml"',
                ])
            ],
            remappings=[
                ('/odom_world', '/state_estimation'),
                ('/terrain_cloud', '/terrain_map_ext'),
                ('/scan_cloud', '/terrain_map'),
                ('/terrain_local_cloud', '/registered_scan'),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='far_rviz',
            arguments=[
                '-d',
                PythonExpression([
                    '"',
                    get_package_share_directory('far_planner'),
                    '/rviz/',
                    LaunchConfiguration('config'),
                    '.rviz"',
                ]),
            ],
            respawn=False,
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('graph_decoder'),
                '/launch/decoder.launch.py',
            ])
        ),
    ])
