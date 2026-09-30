#!/usr/bin/env python3
"""Score a recorded run from its bag, with no simulator involved.

Reads /odom (visual odometry) and /odom_ground_truth out of a rosbag2 and runs
the same scoring as a live experiment, so a run can be re-measured after the
fact - and so every run in the report is scored by identical code.

    ros2 run hexapod_slam score_bag -- verification/runs/stage2_vo_runA_ring_.../bag
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from hexapod_slam.evaluate_odometry import LOST_COVARIANCE, align_ground_truth, resample, yaw_of


def read(bag_path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        out[topic].append(deserialize_message(data, get_message(types[topic])))
    return out


def as_series(msgs):
    vo, lost = [], []
    for m in msgs:
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if m.pose.covariance[0] >= LOST_COVARIANCE or not math.isfinite(p.x):
            lost.append(t)
        else:
            vo.append((t, p.x, p.y, p.z, yaw_of(o)))
    return vo, lost


def score(vo, gt, lost):
    """Same measurements as a live run: both trajectories from a common start."""
    gt = align_ground_truth(gt)
    t, x0, y0, z0, yaw0 = vo[0]
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    vo = [(t, (x - x0) * c - (y - y0) * s, (x - x0) * s + (y - y0) * c, z - z0,
           math.atan2(math.sin(yw - yaw0), math.cos(yw - yaw0))) for t, x, y, z, yw in vo]
    times = [v[0] for v in vo]
    gt_s = resample(gt, times)
    err = np.array([math.hypot(v[1] - g[1], v[2] - g[2]) for v, g in zip(vo, gt_s)])
    yaw_err = np.array([math.degrees(abs(math.atan2(math.sin(v[4] - g[4]),
                                                    math.cos(v[4] - g[4]))))
                        for v, g in zip(vo, gt_s)])
    travel = [0.0]
    for a, b in zip(gt, gt[1:]):
        travel.append(travel[-1] + math.hypot(b[1] - a[1], b[2] - a[2]))
    gt_t = np.array([g[0] for g in gt])
    at = lambda tt: travel[int(np.argmin(np.abs(gt_t - tt)))]
    losses = [l for l in lost if l >= times[0]]
    tracked = at(times[-1])
    return dict(
        ground_truth_path_m=travel[-1], visual_odometry_path_m=sum(
            math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(vo, vo[1:])),
        tracked_distance_m=tracked,
        ate_rmse_m=float(np.sqrt((err ** 2).mean())), ate_mean_m=float(err.mean()),
        final_position_error_m=float(err[-1]), max_position_error_m=float(err.max()),
        drift_per_metre=float(err[-1] / tracked) if tracked else float("nan"),
        final_yaw_error_deg=float(yaw_err[-1]), max_yaw_error_deg=float(yaw_err.max()),
        vo_messages=len(vo), lost_frames=len(losses),
        lost_fraction=len(losses) / max(1, len(losses) + len(vo)),
        first_loss_after_m=at(min(losses)) if losses else None,
    ), vo, gt, gt_s, err, yaw_err, travel


def plot(path, title, vo, gt, gt_s, err, yaw_err):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))
    ax1.plot([g[1] for g in gt], [g[2] for g in gt], color="#1a8c46", lw=2.5, label="ground truth")
    ax1.plot([v[1] for v in vo], [v[2] for v in vo], color="#2a5db0", lw=2.0,
             label="RGB-D visual odometry")
    ax1.plot(0, 0, "o", color="#111", ms=9, label="start")
    ax1.plot(gt[-1][1], gt[-1][2], "s", color="#1a8c46", ms=9, label="ground-truth end")
    ax1.plot(vo[-1][1], vo[-1][2], "X", color="#b03a2a", ms=11, label="odometry ends (tracking lost)"
             if len(vo) and abs(vo[-1][0] - gt[-1][0]) > 2 else "odometry end")
    ax1.set_aspect("equal"); ax1.grid(alpha=0.3); ax1.legend(fontsize=9)
    ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)"); ax1.set_title(title)
    d = np.cumsum([0] + [math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(gt_s, gt_s[1:])])
    ax2.plot(d, err, color="#b03a2a", lw=2)
    ax2b = ax2.twinx(); ax2b.plot(d, yaw_err, color="#8a6d1f", lw=1.4, alpha=0.8)
    ax2.set_xlabel("distance travelled (m)"); ax2.set_ylabel("position error (m)")
    ax2b.set_ylabel("yaw error (deg)"); ax2.grid(alpha=0.3)
    ax2.set_title("drift against ground truth")
    fig.tight_layout(); fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--out", default=None, help="where to write results (default: beside the bag)")
    args = ap.parse_args()
    data = read(args.bag, ["/odom", "/odom_ground_truth"])
    vo, lost = as_series(data["/odom"])
    gt = [(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
           m.pose.pose.position.y, m.pose.pose.position.z, yaw_of(m.pose.pose.orientation))
          for m in data["/odom_ground_truth"]]
    if len(vo) < 20 or len(gt) < 20:
        print(f"not enough data in the bag (vo {len(vo)}, gt {len(gt)})")
        return 1
    results, vo, gt, gt_s, err, yaw_err, _ = score(vo, gt, lost)
    out = args.out or os.path.dirname(os.path.abspath(args.bag.rstrip("/")))
    label = os.path.basename(out)
    with open(os.path.join(out, "results_from_bag.json"), "w") as f:
        json.dump(results, f, indent=2)
    plot(os.path.join(out, "trajectory.png"), label, vo, gt, gt_s, err, yaw_err)
    for k, v in results.items():
        print(f"  {k:26s} {v if not isinstance(v, float) else round(v, 4)}")
    print(f"\nwritten to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
