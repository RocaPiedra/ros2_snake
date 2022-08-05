from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='snake_tf2',
            executable='snake_tf2_broadcaster',
            name='snake_master',
            parameters=[
                {'snakename': 'turtle1'}
            ]
        ),
        Node(
            package='snake_tf2',
            executable='snake_tf2_broadcaster',
            name='snake_body1',
            parameters=[
                {'snakename': 'body1'}
            ]
        ),
        Node(
            package='snake_tf2',
            executable='snake_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
    ])