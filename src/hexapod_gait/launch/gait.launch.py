from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gait = Node(
        package='hexapod_gait',
        executable='gait_node',
        name='hexapod_gait',
        output='screen',
        # use_sim_time is intentionally False for THIS node (Gazebo,
        # ros2_control, controllers and RViz stay on simulation time).
        # Reasoning specific to this project's current architecture:
        #   * the gait is a relative-duration JointTrajectory streamer, so its
        #     control loop does not depend on Gazebo's absolute /clock;
        #   * with use_sim_time:=true the 50 Hz timer runs on sim time and will
        #     FREEZE if /clock does not reach this node (a DDS discovery issue we
        #     hit), stalling the whole gait;
        #   * wall-clock timing makes the gait timer independent of /clock and
        #     immune to that failure.
        parameters=[{'use_sim_time': False}],
    )
    return LaunchDescription([gait])
