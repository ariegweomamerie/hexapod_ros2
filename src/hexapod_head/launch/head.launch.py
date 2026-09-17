import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('hexapod_head'), 'config', 'head.yaml')
    head = Node(
        package='hexapod_head',
        executable='head_node',
        name='hexapod_head',
        output='screen',
        # Wall clock, same reasoning as the gait: moves are relative durations
        # (time_from_start), so the node never needs Gazebo's /clock.
        parameters=[config, {'use_sim_time': False}],
    )
    return LaunchDescription([head])
