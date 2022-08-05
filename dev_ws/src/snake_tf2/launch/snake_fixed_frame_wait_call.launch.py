import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
   snake_base_nodes = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('snake_tf2'), 'launch'),
            '/snake_wait_call.launch.py']),
      launch_arguments={'target_frame': 'tail1'}.items(),
      )

   return LaunchDescription([
      snake_base_nodes,
      Node(
            package='snake_tf2',
            executable='start_snake_tf2_tail_frame_broadcaster',
            name='fixed_broadcaster',
      ),
   ])