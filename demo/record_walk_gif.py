#!/usr/bin/env python3
"""Record a clean walking GIF of the hexapod straight from Gazebo.

How it works:
  1. Adds an invisible, fixed "demo camera" to the Gazebo world, placed in front
     of and to the left of wherever the robot currently is (a 3/4 view).
  2. Drives the robot with the normal ROS commands: /cmd_vel (walk forward,
     turn) and, if hexapod_head is running, /head/cmd (look around).
  3. Grabs the camera frames directly over Gazebo transport (no ROS bridge
     needed), then turns them into a GIF with ../make_gif.sh.
  4. Removes the demo camera again.

Frames come from a Gazebo sensor at a fixed simulation-time rate, so the GIF is
steady and plays at real-time speed, with no desktop recording or window framing
involved.

Needs, in other terminals:
  ./run_gazebo.sh
  ros2 launch hexapod_gait gait.launch.py
  ros2 launch hexapod_head head.launch.py      (optional: the look-around)

Usage (from the workspace root, with the workspace sourced):
  python3 demo/record_walk_gif.py                   # -> docs/media/hexapod_walking.gif
  python3 demo/record_walk_gif.py --preview         # save one still to check framing
  python3 demo/record_walk_gif.py --out my.gif --fps 10 --width 560
"""
import argparse
import math
import os
import queue
import shutil
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from PIL import Image as PILImage
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = "empty"
ROBOT = "Hexapod_Robot"
CAMERA = "demo_camera"
CAMERA_TOPIC = "/demo_camera/image"
BODY_CENTRE = (0.115, 0.115)      # body centre in the robot's base_footprint frame

# Choreography: (duration s, label, cmd_vel (linear.x, angular.z) or None, head [pan, tilt] or None)
SCRIPT = [
    (1.5, "stand",      (0.0, 0.0),  None),
    (3.5, "forward",    (0.15, 0.0), None),
    (3.5, "turn left",  (0.0, 0.6),  None),
    (1.5, "stop",       (0.0, 0.0),  None),
    (1.2, "look left",  (0.0, 0.0),  (0.30, 0.0)),
    (1.4, "look right", (0.0, 0.0),  (-0.35, 0.0)),
    (1.2, "look up",    (0.0, 0.0),  (0.0, 0.35)),
    (1.2, "straight",   (0.0, 0.0),  (0.0, 0.0)),
]


def robot_pose(gz, timeout=5.0):
    """(x, y, yaw) of the robot model in the world, from /world/<w>/pose/info."""
    got = queue.Queue()

    def cb(msg):
        for p in msg.pose:
            if p.name == ROBOT:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                got.put((p.position.x, p.position.y, yaw))

    topic = f"/world/{WORLD}/pose/info"
    gz.subscribe(Pose_V, topic, cb)
    try:
        return got.get(timeout=timeout)
    except queue.Empty:
        raise SystemExit(f"No pose for '{ROBOT}' on {topic} - is Gazebo running?")
    finally:
        gz.unsubscribe(topic)


def camera_pose(robot, distance, height, side_angle_deg):
    """Place the camera `distance` m from the robot's body centre, `side_angle_deg`
    to the robot's left of straight ahead, looking at a point a little ahead of
    the robot (the middle of its walk)."""
    x, y, yaw = robot
    c, s = math.cos(yaw), math.sin(yaw)
    fwd = (s, -c)                       # robot forward = its -Y axis
    left = (c, s)                       # robot left    = its +X axis
    cx = x + c * BODY_CENTRE[0] - s * BODY_CENTRE[1]
    cy = y + s * BODY_CENTRE[0] + c * BODY_CENTRE[1]
    look = (cx + 0.18 * fwd[0], cy + 0.18 * fwd[1], 0.07)
    a = math.radians(side_angle_deg)
    horiz = distance * math.cos(math.atan2(height, distance))
    px = look[0] + horiz * (math.cos(a) * fwd[0] + math.sin(a) * left[0])
    py = look[1] + horiz * (math.cos(a) * fwd[1] + math.sin(a) * left[1])
    pz = look[2] + height
    cam_yaw = math.atan2(look[1] - py, look[0] - px)
    cam_pitch = math.atan2(pz - look[2], math.hypot(look[0] - px, look[1] - py))
    return px, py, pz, cam_pitch, cam_yaw


def camera_sdf(pose, fps, width, height):
    px, py, pz, pitch, yaw = pose
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{CAMERA}">
    <static>true</static>
    <pose>{px:.4f} {py:.4f} {pz:.4f} 0 {pitch:.4f} {yaw:.4f}</pose>
    <link name="link">
      <sensor name="{CAMERA}" type="camera">
        <always_on>1</always_on>
        <update_rate>{fps}</update_rate>
        <topic>{CAMERA_TOPIC.lstrip('/')}</topic>
        <camera>
          <horizontal_fov>0.9</horizontal_fov>
          <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>30</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>"""


def gz_call(gz, service, req, req_type, timeout_ms=10000):
    ok, rep = gz.request(service, req, req_type, Boolean, timeout_ms)
    return ok and rep.data


def remove_camera(gz):
    return gz_call(gz, f"/world/{WORLD}/remove/blocking",
                   Entity(name=CAMERA, type=Entity.MODEL), Entity)


def spawn_camera(gz, sdf):
    remove_camera(gz)                   # clear a leftover camera from an earlier run
    if not gz_call(gz, f"/world/{WORLD}/create/blocking", EntityFactory(sdf=sdf), EntityFactory):
        raise SystemExit("Could not add the demo camera to Gazebo")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=os.path.join(WS, "docs", "media", "hexapod_walking.gif"))
    ap.add_argument("--fps", type=int, default=12, help="camera and GIF frame rate")
    ap.add_argument("--width", type=int, default=640, help="GIF width in pixels")
    ap.add_argument("--distance", type=float, default=1.7, help="camera distance (m)")
    ap.add_argument("--height", type=float, default=0.55, help="camera height above target (m)")
    ap.add_argument("--side", type=float, default=40.0, help="degrees to the robot's left")
    ap.add_argument("--preview", action="store_true", help="save one still and exit")
    ap.add_argument("--keep-frames", action="store_true", help="keep the PNG frames")
    args = ap.parse_args()

    # render at 2x the GIF width (16:9) and downscale, for smooth edges
    render_w = 2 * args.width
    render_h = round(render_w * 9 / 16 / 2) * 2

    gz = GzNode()
    robot = robot_pose(gz)
    spawn_camera(gz, camera_sdf(camera_pose(robot, args.distance, args.height, args.side),
                                args.fps, render_w, render_h))

    frames = queue.Queue()
    gz.subscribe(GzImage, CAMERA_TOPIC, lambda m: frames.put(m))
    frame_dir = tempfile.mkdtemp(prefix="hexapod_gif_")
    try:
        first = frames.get(timeout=15)
        if args.preview:
            out = os.path.splitext(args.out)[0] + "_preview.png"
            PILImage.frombytes("RGB", (first.width, first.height), first.data).save(out)
            print(f"preview saved: {out}")
            return

        rclpy.init()
        node = rclpy.create_node("gif_recorder")
        cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        head_pub = node.create_publisher(Float64MultiArray, "/head/cmd", 10)
        t0 = time.time()
        while cmd_pub.get_subscription_count() == 0 and time.time() - t0 < 10:
            time.sleep(0.1)
        if cmd_pub.get_subscription_count() == 0:
            raise SystemExit("Nothing subscribes to /cmd_vel - is the gait running?")
        has_head = head_pub.get_subscription_count() > 0 or _wait_sub(head_pub, 3.0)
        if not has_head:
            print("hexapod_head is not running: recording without the look-around")

        while not frames.empty():       # start the clip from "now"
            frames.get_nowait()
        count = 0

        def drain():
            nonlocal count
            while not frames.empty():
                m = frames.get_nowait()
                PILImage.frombytes("RGB", (m.width, m.height), m.data).save(
                    os.path.join(frame_dir, f"{count:05d}.png"), compress_level=1)
                count += 1

        for duration, label, (vx, wz), head in SCRIPT:
            if head is not None and not has_head:
                continue
            print(f"  {label}")
            if head is not None:
                head_pub.publish(Float64MultiArray(data=[float(v) for v in head]))
            end = time.time() + duration
            while time.time() < end:
                twist = Twist()
                twist.linear.x, twist.angular.z = vx, wz
                cmd_pub.publish(twist)
                drain()
                time.sleep(0.05)
        cmd_pub.publish(Twist())        # make sure the robot is left standing still
        time.sleep(0.3)
        drain()
        node.destroy_node()
        rclpy.shutdown()

        print(f"captured {count} frames ({count / args.fps:.1f} s of sim time)")
        subprocess.run([os.path.join(WS, "make_gif.sh"), os.path.join(frame_dir, "%05d.png"),
                        args.out, str(args.fps), str(args.width)], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"GIF saved: {args.out} ({os.path.getsize(args.out) / 1e6:.1f} MB)")
    finally:
        gz.unsubscribe(CAMERA_TOPIC)
        if not remove_camera(gz):
            print(f"note: could not remove '{CAMERA}' from Gazebo (it is invisible and harmless)")
        if args.keep_frames:
            print(f"frames kept in {frame_dir}")
        else:
            shutil.rmtree(frame_dir, ignore_errors=True)


def _wait_sub(pub, secs):
    end = time.time() + secs
    while time.time() < end:
        if pub.get_subscription_count() > 0:
            return True
        time.sleep(0.1)
    return False


if __name__ == "__main__":
    main()
