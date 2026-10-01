"""RTAB-Map localisation against the frozen Stage 2.4 reference map (Stage 3).

Stage 2.4's rtabmap.launch.py builds a map. This one loads one and does not
change it. The camera, the bridge and rgbd_odometry are untouched: they are the
same validated Stage 2.3/2.4 components, and RTAB-Map still consumes the same
RGB-D topics and takes its pose estimate from /odom.

TF ownership is unchanged, with nothing publishing the same edge twice:
    rgbd_odometry  ->  odom -> base_footprint
    rtabmap        ->  map  -> odom     (a correction, not a map)

The canonical reference database is NEVER opened by an experiment. This launch
copies it to a per-run working file first, because RTAB-Map opens its database
read-write even in localisation mode. The copy is made here rather than in a
shell script so that no run can start by accident against the canonical file.

    ros2 launch hexapod_slam localization.launch.py \\
        run_dir:=verification/runs/stage3_L0_20261001_101500

Needs the facility, the gait and the odometry already running - or use
localization_stack.launch.py, which starts all of it in order.
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WS = "/home/general/hexapod_ros_robot_ws"


def _reference_manifest():
    """The repository's record of which map Stage 3 localises against."""
    import yaml
    path = os.path.join(get_package_share_directory('hexapod_slam'),
                        'config', 'stage3_reference.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['stage3_reference']


def _prepare(context, *args, **kwargs):
    """Copy the frozen reference to a per-run working file, verifying it first.

    Refusing to launch is the right behaviour here. A localisation run against a
    reference that is the wrong file - or a corrupt one - produces numbers that
    look like measurements, and that has already happened once on this project:
    the first proposed reference was a malformed SQLite image with an empty
    visual vocabulary that passed a size check.
    """
    import hashlib
    import sqlite3

    ref = _reference_manifest()
    canonical = os.path.join(WS, ref['path'])
    run_dir = LaunchConfiguration('run_dir').perform(context)
    if not os.path.isabs(run_dir):
        run_dir = os.path.join(WS, run_dir)

    if not os.path.exists(canonical):
        raise RuntimeError(f"Stage 3 reference is missing: {canonical}")

    size = os.path.getsize(canonical)
    if size != ref['size_bytes']:
        raise RuntimeError(
            f"Stage 3 reference is the wrong size: {size} != {ref['size_bytes']}")

    digest = hashlib.sha256()
    with open(canonical, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            digest.update(chunk)
    got = digest.hexdigest()
    if got != ref['sha256']:
        raise RuntimeError("Stage 3 reference sha256 does not match the manifest:\n"
                           f"  expected {ref['sha256']}\n  got      {got}")

    # A torn database raises out of sqlite rather than returning a verdict, so
    # the whole probe is wrapped: the point of this gate is to say clearly WHY
    # the run is not starting, not to abort with a stack trace.
    try:
        con = sqlite3.connect(f"file:{canonical}?mode=ro", uri=True)
        try:
            integrity = con.execute("PRAGMA integrity_check;").fetchone()[0]
            words = con.execute("SELECT COUNT(*) FROM Word;").fetchone()[0]
            feats = con.execute("SELECT COUNT(*) FROM Feature;").fetchone()[0]
        finally:
            con.close()
    except sqlite3.DatabaseError as exc:
        raise RuntimeError(
            f"Stage 3 reference is not a readable SQLite database ({exc}). "
            "A database copied while RTAB-Map was still writing looks like this; "
            "see the run directory's REFERENCE_README.md.") from None
    if integrity != "ok":
        raise RuntimeError(f"Stage 3 reference fails PRAGMA integrity_check: {integrity}")
    if words == 0 or feats == 0:
        raise RuntimeError(
            f"Stage 3 reference has no visual content (words={words}, features={feats}); "
            "RTAB-Map cannot relocalise against it")

    os.makedirs(run_dir, exist_ok=True)
    working = os.path.join(run_dir, ref['per_run_copy'])
    shutil.copyfile(canonical, working)
    os.chmod(working, 0o644)                      # the copy IS written to; the canonical is not
    with open(os.path.join(run_dir, 'reference_sha256_before.txt'), 'w') as f:
        f.write(f"{got}  {ref['path']}\n")

    print(f"[stage3] reference verified: {words} words, {feats} features, integrity ok")
    print(f"[stage3] working copy: {working}")

    config = os.path.join(get_package_share_directory('hexapod_slam'),
                          'config', 'rtabmap_localization.yaml')
    # Optional single-parameter override for a controlled experiment. Unset -
    # the default - the parameter list is exactly what it was before this
    # argument existed, so the baseline launch path is unchanged. When set, the
    # file is appended AFTER the config so it overrides, and only the keys it
    # contains are affected.
    params = [config, {'database_path': working}]
    override = LaunchConfiguration('rtabmap_params_file').perform(context).strip()
    if override:
        if not os.path.isabs(override):
            override = os.path.join(WS, override)
        if not os.path.exists(override):
            raise RuntimeError(f"rtabmap_params_file does not exist: {override}")
        params.append(override)
        print(f"[stage3] rtabmap parameter override: {override}")
    return [Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=params,
        remappings=[
            ('rgb/image', LaunchConfiguration('rgb_topic')),
            ('depth/image', LaunchConfiguration('depth_topic')),
            ('rgb/camera_info', LaunchConfiguration('info_topic')),
            ('odom', LaunchConfiguration('odom_topic')),
        ],
        # NO --delete_db_on_start: that flag would wipe the reference map, which
        # is the entire input to this stage.
        arguments=[],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value='/face_camera/image'),
        DeclareLaunchArgument('depth_topic', default_value='/face_camera/depth_image'),
        DeclareLaunchArgument('info_topic', default_value='/face_camera/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'run_dir',
            description='run directory; the reference working copy is written here'),
        DeclareLaunchArgument(
            'rtabmap_params_file', default_value='',
            description='optional ROS 2 parameter file layered over '
                        'rtabmap_localization.yaml; empty means no override'),
        OpaqueFunction(function=_prepare),
    ])
