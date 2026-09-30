#!/usr/bin/env python3
"""Show what the camera saw when the visual odometry lost tracking.

Finds the first lost frame in a recorded run, then pulls the colour and depth
images from either side of it out of the bag and counts ORB features in each -
the same thing the odometry is trying to match. A wall filling the view, or a
surface too close for the depth camera, shows up immediately here.

    ros2 run hexapod_slam inspect_failure -- verification/runs/stage2_vo_.../bag
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from hexapod_slam.evaluate_odometry import LOST_COVARIANCE


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--before", type=float, default=3.0, help="s of context before the loss")
    ap.add_argument("--after", type=float, default=1.0)
    ap.add_argument("--every", type=float, default=0.5, help="s between saved frames")
    args = ap.parse_args()

    out_dir = os.path.join(os.path.dirname(os.path.abspath(args.bag.rstrip("/"))), "failure")
    os.makedirs(out_dir, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # first pass: when was tracking lost, and where was the robot?
    loss_t, gt = None, []
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/odom", "/odom_ground_truth"]))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == "/odom_ground_truth":
            gt.append((stamp(msg), msg.pose.pose.position.x, msg.pose.pose.position.y,
                       math.degrees(math.atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z),
                                               1 - 2 * msg.pose.pose.orientation.z ** 2))))
        elif loss_t is None and (msg.pose.covariance[0] >= LOST_COVARIANCE
                                 or not math.isfinite(msg.pose.pose.position.x)):
            loss_t = stamp(msg)
    if loss_t is None:
        print("tracking was never lost in this run")
        return 0
    near = min(gt, key=lambda g: abs(g[0] - loss_t))
    print(f"tracking lost at sim t={loss_t:.2f} s, robot at "
          f"({near[1]:.2f}, {near[2]:.2f}) facing {near[3]:+.0f} deg")

    # second pass: the images around that moment
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=["/face_camera/image", "/face_camera/depth_image"]))
    orb = cv2.ORB_create(1000)
    saved, last = {}, {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        t = stamp(msg)
        if not (loss_t - args.before <= t <= loss_t + args.after):
            continue
        if t - last.get(topic, -1e9) < args.every:
            continue
        last[topic] = t
        rel = t - loss_t
        if topic.endswith("depth_image"):
            arr = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.width)
            good = np.isfinite(arr) & (arr > 0)
            name = f"depth_{rel:+.2f}s.png"
            vis = np.zeros_like(arr)
            vis[good] = arr[good]
            cv2.imwrite(os.path.join(out_dir, name),
                        cv2.applyColorMap(
                            np.clip(vis / 6.0 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO))
            print(f"  depth {rel:+5.2f}s  valid {100 * good.mean():3.0f}% of pixels, "
                  f"near {arr[good].min() if good.any() else float('nan'):.2f} m, "
                  f"median {np.median(arr[good]) if good.any() else float('nan'):.2f} m")
        else:
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
            kp = orb.detect(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), None)
            name = f"colour_{rel:+.2f}s_{len(kp)}feat.png"
            cv2.imwrite(os.path.join(out_dir, name),
                        cv2.drawKeypoints(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), kp, None,
                                          color=(0, 255, 0)))
            print(f"  colour {rel:+5.2f}s  {len(kp):4d} ORB features")
        saved[name] = t
    print(f"\n{len(saved)} images written to {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
