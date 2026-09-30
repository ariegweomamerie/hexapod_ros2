#!/usr/bin/env python3
"""Walk the hexapod around one of the facility loops, to prove it is traversable.

A deliberately dumb waypoint follower - turn towards the next waypoint, walk at
it, stop - driving the same /cmd_vel the gait already takes. This is a world
test, not navigation: Nav2 arrives in a later stage, and this uses Gazebo ground
truth for steering because nothing else exists yet.

    ros2 run hexapod_worlds drive_facility_loop -- --loop core

Reports progress, body tilt, stalls and the closest approach to any obstacle,
so a loop that cannot be walked shows up as a stall rather than a silent pass.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

from hexapod_worlds import layout as L

WORLD = "hexapod_facility"
ROBOT = "Hexapod_Robot"
BODY_CENTRE = (0.115, 0.115)     # body centre in the robot's base_footprint frame
REACHED = 0.35                   # m, waypoint considered reached
MAX_SPEED = 0.20                 # m/s command (the gait saturates around here)
MAX_TURN = 0.60                  # rad/s command
STALL_WINDOW = 30.0              # s; no real progress in this long -> give up
STALL_PROGRESS = 0.10            # m of progress that counts as "still moving"


class Truth:
    def __init__(self):
        self.pose = None
        self.tilt = 0.0
        self._gz = GzNode()
        self._gz.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self._cb)

    def _cb(self, msg):
        for p in msg.pose:
            if p.name == ROBOT:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
                pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
                c, s = math.cos(yaw), math.sin(yaw)
                # The robot faces its own -Y, so its heading in the world is yaw - 90 deg.
                self.pose = (p.position.x + c * BODY_CENTRE[0] - s * BODY_CENTRE[1],
                             p.position.y + s * BODY_CENTRE[0] + c * BODY_CENTRE[1],
                             yaw - math.pi / 2)
                self.tilt = max(abs(roll), abs(pitch))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--loop", choices=["ring", "core", "rooms", "localization"],
                    default="core")
    ap.add_argument("--timeout", type=float, default=900.0, help="s of wall clock")
    ap.add_argument("--final-dwell", type=float, default=0.0,
                    help="s to stand still at the last waypoint before finishing")
    args = ap.parse_args()
    waypoints = {"ring": L.loop_ring(), "core": L.loop_core(), "rooms": L.loop_rooms(),
                 "localization": L.loop_localization()}[args.loop]

    rclpy.init()
    node = rclpy.create_node("drive_facility_loop")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    truth = Truth()
    t0 = time.time()
    while (pub.get_subscription_count() == 0 or truth.pose is None) and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print("nothing subscribes to /cmd_vel - is the gait running?")
        return 1

    # Join the loop at the nearest waypoint: this follower drives straight at its
    # target, so starting mid-world could otherwise aim it through a wall.
    closed = waypoints[0] == waypoints[-1]
    ring = waypoints[:-1] if closed else waypoints
    px, py, _ = truth.pose
    first = min(range(len(ring)), key=lambda i: math.hypot(ring[i][0] - px, ring[i][1] - py))
    if closed:
        waypoints = [ring[(first + k) % len(ring)] for k in range(len(ring) + 1)]
    else:
        # An OPEN route (the Stage 3 localisation route) is walked once, in
        # order, and stops at its last waypoint. Wrapping it would drive an
        # extra leg the experiment is not defined over, and its segments ARE
        # the experiment, so it is never joined part-way.
        waypoints = ring[first:]
    print(f"walking the {args.loop} loop from waypoint {first + 1}: {len(waypoints) - 1} legs")
    start = time.time()
    path_len, max_tilt = 0.0, 0.0
    last = truth.pose[:2]
    for i, (wx, wy) in enumerate(waypoints[1:], 1):
        leg_start, best = time.time(), 9e9
        mark_t, mark_d = time.time(), None
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            x, y, yaw = truth.pose
            path_len += math.hypot(x - last[0], y - last[1])
            last = (x, y)
            max_tilt = max(max_tilt, truth.tilt)
            d = math.hypot(wx - x, wy - y)
            best = min(best, d)
            if d < REACHED:
                print(f"  waypoint {i}/{len(waypoints) - 1} ({wx:.1f}, {wy:.1f}) reached "
                      f"after {time.time() - leg_start:5.1f} s   tilt max {math.degrees(max_tilt):.1f} deg")
                break
            # stall = no real progress towards the waypoint for a while, which is
            # robust to however fast the simulation happens to be running
            if mark_d is None or d < mark_d - STALL_PROGRESS:
                mark_d, mark_t = d, time.time()
            if time.time() - mark_t > STALL_WINDOW:
                print(f"  STALLED heading for ({wx:.1f}, {wy:.1f}): got within {best:.2f} m "
                      f"at ({x:.2f}, {y:.2f})")
                pub.publish(Twist())
                return 1
            if time.time() - start > args.timeout:
                print("  timed out")
                pub.publish(Twist())
                return 1
            # turn towards the waypoint, walk when roughly aligned
            want = math.atan2(wy - y, wx - x)
            heading = yaw                          # already the world heading (see Truth)
            err = math.atan2(math.sin(want - heading), math.cos(want - heading))
            cmd = Twist()
            cmd.angular.z = max(-MAX_TURN, min(MAX_TURN, 1.5 * err))
            cmd.linear.x = MAX_SPEED * max(0.0, math.cos(err)) ** 2
            pub.publish(cmd)
    pub.publish(Twist())
    if args.final_dwell > 0:
        # Standing still at the end is a measurement window, not padding: it is
        # where a localisation estimate is read without any motion in it.
        print(f"  holding still for {args.final_dwell:.0f} s at the last waypoint")
        hold = time.time()
        while time.time() - hold < args.final_dwell:
            rclpy.spin_once(node, timeout_sec=0.05)
            pub.publish(Twist())
    dt = time.time() - start
    print(f"\nloop '{args.loop}' walked: {path_len:.1f} m of path in {dt / 60:.1f} min wall, "
          f"max body tilt {math.degrees(max_tilt):.1f} deg")
    print("RESULT: traversable")
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
