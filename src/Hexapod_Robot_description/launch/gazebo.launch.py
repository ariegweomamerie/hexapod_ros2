import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro
from os.path import join


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('Hexapod_Robot_description')

    # headless:=true runs Gazebo server-only (no GUI). Use it when the display
    # can't provide OpenGL >=3.3 for the gz GUI (it crash-loops otherwise);
    # visualize with RViz instead. Default keeps the normal GUI.
    headless = LaunchConfiguration('headless')
    declare_headless = DeclareLaunchArgument('headless', default_value='false')
    # world:= an SDF world file (or a built-in name). Default: the empty world the
    # gait was verified in; hexapod_worlds passes the facility world for SLAM.
    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument('world', default_value='empty.sdf')
    # spawn pose, so a world can place the robot at its own start point
    declare_pose = [DeclareLaunchArgument(n, default_value=v) for n, v in
                    (('x', '0.0'), ('y', '0.0'), ('z', '0.32'), ('yaw', '0.0'))]
    gz_args = PythonExpression(
        ["'-s -r -v 4 ' + '", world, "' if '", headless, "' == 'true' else '-r -v 4 ' + '",
         world, "'"]
    )

    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'Hexapod_Robot.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Publishes TF from /joint_states. use_sim_time so TF stamps match Gazebo.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Start Gazebo with an empty world (running, verbose); GUI unless headless.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items()
    )

    # Spawn the robot from the /robot_description topic.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "Hexapod_Robot",
            "-allow_renaming", "false",
            "-x", LaunchConfiguration('x'), "-y", LaunchConfiguration('y'),
            "-z", LaunchConfiguration('z'), "-Y", LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    # Bridge (currently just /clock so ROS shares Gazebo's sim time).
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen',
    )

    # --- Controller spawners (loaded via the controller_manager gz plugin) ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_controller'],
        output='screen',
    )

    face_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['face_controller'],
        output='screen',
    )

    # Deterministic ordering: robot spawned -> JSB -> leg + face controllers.
    load_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    load_controllers_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg_controller_spawner, face_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_headless,
        declare_world,
        *declare_pose,
        gazebo,
        robot_state_publisher,
        ros_gz_bridge,
        spawn_robot,
        load_jsb_after_spawn,
        load_controllers_after_jsb,
    ])
