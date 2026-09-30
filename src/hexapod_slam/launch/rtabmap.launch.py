"""RTAB-Map SLAM for the hexapod (Stage 2.4), on top of Stage 2.3's odometry.

This adds the mapping and loop-closure layer only. The camera, the bridge and
rgbd_odometry are untouched: RTAB-Map consumes the same validated RGB-D topics
and subscribes to /odom for the pose estimate.

TF ownership stays split, with nothing publishing the same edge twice:
    rgbd_odometry  ->  odom -> base_footprint
    rtabmap        ->  map  -> odom

    ros2 launch hexapod_slam rtabmap.launch.py

Needs the facility, the gait and the odometry already running:
    ./run_facility.sh
    ros2 launch hexapod_gait gait.launch.py
    ros2 launch hexapod_slam visual_odometry.launch.py

delete_db_on_start:=false keeps the previous session's database, which is how a
map is reused; the default starts each run from an empty map.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('hexapod_slam'), 'config', 'rtabmap.yaml')
    database = LaunchConfiguration('database_path')
    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value='/face_camera/image'),
        DeclareLaunchArgument('depth_topic', default_value='/face_camera/depth_image'),
        DeclareLaunchArgument('info_topic', default_value='/face_camera/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('delete_db_on_start', default_value='true'),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.join(os.path.expanduser('~'), '.ros', 'hexapod_rtabmap.db')),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='open the RTAB-Map GUI (needs ros-jazzy-rtabmap-viz)'),
        # rtabmap takes --delete_db_on_start on the command line rather than as a
        # parameter, so the two cases are two nodes under opposite conditions;
        # exactly one ever starts.
        *[Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[config, {'database_path': database}],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('depth/image', LaunchConfiguration('depth_topic')),
                ('rgb/camera_info', LaunchConfiguration('info_topic')),
                ('odom', LaunchConfiguration('odom_topic')),
            ],
            arguments=args,
            condition=cond(LaunchConfiguration('delete_db_on_start')),
        ) for args, cond in ((['--delete_db_on_start'], IfCondition), ([], UnlessCondition))],
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            parameters=[{'use_sim_time': True, 'frame_id': 'base_footprint',
                         'subscribe_depth': True, 'approx_sync': False}],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('depth/image', LaunchConfiguration('depth_topic')),
                ('rgb/camera_info', LaunchConfiguration('info_topic')),
                ('odom', LaunchConfiguration('odom_topic')),
            ],
        ),
    ])
