import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the visibility graph decoder."""
    graph_decoder_path = get_package_share_directory('graph_decoder')
    config_path = graph_decoder_path + '/config/default.yaml'

    with open(config_path, 'r') as config_file:
        config = yaml.safe_load(config_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'graph_topic',
            default_value='/planner_nav_graph',
            description='Graph topic to be used',
        ),
        Node(
            package='graph_decoder',
            executable='graph_decoder',
            name='graph_decoder',
            output='screen',
            parameters=[config],
            remappings=[
                ('/planner_nav_graph', LaunchConfiguration('graph_topic')),
            ],
        ),
    ])
