"""The whole Stage 3 localisation stack in one command, started in order.

Same shape as Stage 2.4's slam_stack.launch.py, and for the same reason: a
stack brought up by hand in four terminals ends up with duplicate nodes, and
that has already cost this project a run. One command, one of everything.

    ./run_localization.sh run_dir:=verification/runs/stage3_L0_<stamp>

Startup order matters. The gait needs controllers up; the odometry needs the
camera streaming; RTAB-Map needs /odom to exist before it starts matching
against the reference. The delays are the ones Stage 2.4 validated.

Arguments:
    run_dir              where the reference working copy and artefacts go
    headless             true = no Gazebo GUI (benchmark runs)
    rviz                 RViz comes up with Gazebo by default
    localization         false = sensing + odometry only, no RTAB-Map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    slam_share = get_package_share_directory('hexapod_slam')
    worlds_share = get_package_share_directory('hexapod_worlds')
    gait_share = get_package_share_directory('hexapod_gait')
    include = lambda path, **kw: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=kw.items())

    return LaunchDescription([
        DeclareLaunchArgument('run_dir',
                              description='run directory for the reference working copy'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),

        include(os.path.join(worlds_share, 'launch', 'slam_world.launch.py'),
                headless=LaunchConfiguration('headless'),
                rviz=LaunchConfiguration('rviz')),

        TimerAction(period=12.0, actions=[
            include(os.path.join(gait_share, 'launch', 'gait.launch.py'))]),

        TimerAction(period=20.0, actions=[
            include(os.path.join(slam_share, 'launch', 'visual_odometry.launch.py'))]),

        TimerAction(period=30.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_share, 'launch', 'localization.launch.py')),
                launch_arguments={'run_dir': LaunchConfiguration('run_dir')}.items(),
                condition=IfCondition(LaunchConfiguration('localization')))]),
    ])
