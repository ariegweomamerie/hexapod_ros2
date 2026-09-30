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
from launch.substitutions import LaunchConfiguration

from hexapod_worlds.layout import START


def generate_launch_description():
    worlds_share = get_package_share_directory('hexapod_worlds')
    description_share = get_package_share_directory('Hexapod_Robot_description')
    world = os.path.join(worlds_share, 'worlds', 'hexapod_facility.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        # passed straight through to gazebo.launch.py; empty means "leave the
        # command line alone", so normal runs are unaffected
        DeclareLaunchArgument('render_engine_server', default_value=''),
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
    ])
