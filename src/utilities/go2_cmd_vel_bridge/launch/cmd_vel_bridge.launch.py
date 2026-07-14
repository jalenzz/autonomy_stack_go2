from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'sport_request_topic', default_value='/api/sport/request'),
        DeclareLaunchArgument(
            'wireless_controller_topic', default_value='/wirelesscontroller'),
        DeclareLaunchArgument('max_linear_x', default_value='0.8'),
        DeclareLaunchArgument('max_linear_y', default_value='0.6'),
        DeclareLaunchArgument('max_angular_z', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_timeout', default_value='0.5'),
        DeclareLaunchArgument('stream_rate', default_value='10.0'),

        Node(
            package='go2_cmd_vel_bridge',
            executable='cmd_vel_bridge',
            name='go2_cmd_vel_bridge',
            output='screen',
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'sport_request_topic': LaunchConfiguration('sport_request_topic'),
                'wireless_controller_topic': LaunchConfiguration(
                    'wireless_controller_topic'),
                'max_linear_x': LaunchConfiguration('max_linear_x'),
                'max_linear_y': LaunchConfiguration('max_linear_y'),
                'max_angular_z': LaunchConfiguration('max_angular_z'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
                'stream_rate': LaunchConfiguration('stream_rate'),
            }],
        ),
    ])
