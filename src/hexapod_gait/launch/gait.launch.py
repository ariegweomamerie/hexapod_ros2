from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gait = Node(
        package='hexapod_gait',
        executable='gait_node',
        name='hexapod_gait',
        output='screen',
        # Simulation time: the 50 Hz control timer, gait phase, start/stop
        # transitions and the /cmd_vel watchdog all follow Gazebo's /clock, the
        # same clock as the physics, controllers and (later) Nav2. With wall-clock
        # time the gait ran too fast for the physics whenever Gazebo fell behind
        # real time (real-time factor < 1), which made its behaviour load-dependent.
        # If /clock stops (Gazebo paused or gone), the gait simply pauses.
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([gait])
