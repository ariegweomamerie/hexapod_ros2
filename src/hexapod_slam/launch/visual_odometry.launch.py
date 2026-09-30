"""RGB-D visual odometry for the hexapod (Stage 2.3) - odometry only, no SLAM.

Runs rtabmap_odom/rgbd_odometry on the head camera's synchronised colour and
depth streams. It publishes odom -> base_footprint, the transform deliberately
left free when Gazebo's ground-truth TF was unbridged in Stage 2.2.

Ground truth (/odom_ground_truth) is NOT an input here in any form. Comparing the
two trajectories happens afterwards, in hexapod_slam/evaluate_odometry.py.

    ros2 launch hexapod_slam visual_odometry.launch.py

Needs the facility and the gait running:
    ./run_facility.sh
    ros2 launch hexapod_gait gait.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('hexapod_slam'), 'config',
                          'rgbd_odometry.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value='/face_camera/image'),
        DeclareLaunchArgument('depth_topic', default_value='/face_camera/depth_image'),
        DeclareLaunchArgument('info_topic', default_value='/face_camera/camera_info'),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[config],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('depth/image', LaunchConfiguration('depth_topic')),
                ('rgb/camera_info', LaunchConfiguration('info_topic')),
                # odom + odom_info keep their default names: /odom, /odom_info
            ],
        ),
    ])
