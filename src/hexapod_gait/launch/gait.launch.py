from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gait = Node(
        package='hexapod_gait',
        executable='gait_node',
        name='hexapod_gait',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([gait])
