"""Bring up the hexapod inside the SLAM test facility.

Same stack as Hexapod_Robot_description/gazebo.launch.py (Gazebo, ros2_control,
the bridge), only with the facility world and the robot placed at the start pose
in the south corridor. GZ_SIM_RESOURCE_PATH is extended so the world's
`model://` references (landmarks and shared textures) resolve.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hexapod_worlds.layout import START


try:                       # the SLAM view lives with the SLAM package; if that
                           # package is not built, RViz just opens its default view
    RVIZ_CONFIG = os.path.join(
        get_package_share_directory('hexapod_slam'), 'rviz', 'stage2_4_slam.rviz')
    if not os.path.exists(RVIZ_CONFIG):
        RVIZ_CONFIG = None
except Exception:
    RVIZ_CONFIG = None


def generate_launch_description():
    worlds_share = get_package_share_directory('hexapod_worlds')
    description_share = get_package_share_directory('Hexapod_Robot_description')
    world = os.path.join(worlds_share, 'worlds', 'hexapod_facility.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        # passed straight through to gazebo.launch.py; empty means "leave the
        # command line alone", so normal runs are unaffected
        DeclareLaunchArgument('render_engine_server', default_value=''),
        # RViz comes up with Gazebo every time: the physics view alone hides the
        # TF tree, the map and what the perception stack believes. rviz:=false
        # turns it off for headless or benchmark runs.
        DeclareLaunchArgument('rviz', default_value='true'),
        # let Gazebo find model://pillar, model://slam_assets/... and friends
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(worlds_share, 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': world,
                'headless': LaunchConfiguration('headless'),
                'render_engine_server': LaunchConfiguration('render_engine_server'),
                'x': str(START['x']),
                'y': str(START['y']),
                'z': '0.32',
                'yaw': str(START['yaw']),
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=(['-d', RVIZ_CONFIG] if RVIZ_CONFIG else []),
            parameters=[{'use_sim_time': True}],
        ),
    ])
