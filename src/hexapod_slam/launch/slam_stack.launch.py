"""The whole Stage 2.4 stack in one command, started in dependency order.

    ros2 launch hexapod_slam slam_stack.launch.py

    facility (Gazebo + RViz)          t = 0
      -> gait node                    t = 12 s, once the controllers are up
      -> RGB-D visual odometry        t = 20 s, once the camera is streaming
      -> RTAB-Map SLAM                t = 30 s, once /odom exists

The delays matter. RTAB-Map refuses images while no odometry is published, and
RViz shows every display in error until its fixed frame (map) exists, so
starting these by hand in the wrong order produces alarming-looking failures
that are nothing but a race. Starting them from one launch file also means one
Ctrl-C stops everything, with no leftover duplicates.

Arguments:
    headless:=true      no Gazebo GUI (higher real-time factor for benchmarks)
    rviz:=false         no RViz
    slam:=false         odometry only, no RTAB-Map (the Stage 2.3 stack)
    delete_db_on_start:=false   keep the previous map database
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
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('delete_db_on_start', default_value='true'),
        # each experiment gets its own database file, so a new run can never
        # overwrite the one a previous run is still being scored against
        DeclareLaunchArgument('database_path',
                              default_value='~/.ros/hexapod_rtabmap.db'),

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
                    os.path.join(slam_share, 'launch', 'rtabmap.launch.py')),
                launch_arguments={
                    'delete_db_on_start': LaunchConfiguration('delete_db_on_start'),
                    'database_path': LaunchConfiguration('database_path')}.items(),
                condition=IfCondition(LaunchConfiguration('slam')))]),
    ])
