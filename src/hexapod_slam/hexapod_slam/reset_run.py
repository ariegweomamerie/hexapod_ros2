#!/usr/bin/env python3
"""Put the robot and the visual odometry back to a known state between runs.

Teleports Hexapod_Robot to the facility START pose through Gazebo's set_pose
service, lets it settle, then zeroes rgbd_odometry through /reset_odom. Both are
setup steps outside the odometry's own pipeline - nothing here is fed to it as a
measurement.

    ros2 run hexapod_slam reset_run
"""
import math
import os
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_srvs.srv import Empty

WORLD = "hexapod_facility"
MODEL = "Hexapod_Robot"
START = (3.3, 3.3, 0.32, 1.5708)          # x, y, z, yaw - matches slam_world.launch.py


def main():
    x, y, z, yaw = START
    req = (f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    out = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose",
                          "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                          "--timeout", "3000", "--req", req],
                         capture_output=True, text=True)
    print(f"teleport to START ({x}, {y}, yaw {math.degrees(yaw):.0f} deg): "
          f"{out.stdout.strip() or out.stderr.strip()}")

    rclpy.init()
    node = rclpy.create_node("reset_run",
                             parameter_overrides=[Parameter("use_sim_time",
                                                            Parameter.Type.BOOL, True)])
    gt = []
    node.create_subscription(Odometry, "/odom_ground_truth", lambda m: gt.append(m), 10)
    t0 = time.time()
    while time.time() - t0 < 6.0:             # let the legs settle after the teleport
        rclpy.spin_once(node, timeout_sec=0.05)

    client = node.create_client(Empty, "/rgbd_odometry/reset_odom")
    if client.wait_for_service(timeout_sec=5.0):
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        print("visual odometry reset to its origin")
    else:
        print("no /rgbd_odometry/reset_odom - is visual_odometry.launch.py running?")

    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if gt:
        p = gt[-1].pose.pose
        print(f"ground truth now ({p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f})")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
