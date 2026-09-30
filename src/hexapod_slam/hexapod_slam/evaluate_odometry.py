#!/usr/bin/env python3
"""Run one visual-odometry experiment and score it against ground truth.

Drives the hexapod around a facility loop while recording the RGB-D visual
odometry (/odom, from rgbd_odometry) and Gazebo's ground truth
(/odom_ground_truth). Ground truth is only ever read here, for scoring - it is
never fed to the odometry, which sees nothing but colour, depth and intrinsics.

    ros2 run hexapod_slam run_odometry_experiment -- --loop ring --label runA

Needs running: ./run_facility.sh, the gait, and
ros2 launch hexapod_slam visual_odometry.launch.py

Writes into verification/runs/stage2_vo_<label>_<time>/:
    results.json   every measurement below
    trajectory.png ground truth vs visual odometry, plus error over time
    bag/           rosbag2 of the run (unless --no-bag)

Measures: absolute trajectory error, final position and yaw error, drift per
metre, worst deviation, tracking losses and gaps, stream rates and Gazebo's
real-time factor.
"""
import argparse
import datetime
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, Image

WS = "/home/general/hexapod_ros_robot_ws"
WORLD = "hexapod_facility"
BAG_TOPICS = ["/face_camera/image", "/face_camera/depth_image", "/face_camera/camera_info",
              "/odom", "/odom_ground_truth", "/tf", "/tf_static", "/cmd_vel", "/clock",
              "/joint_states", "/imu"]
LOST_COVARIANCE = 9999.0        # rtabmap marks a lost frame with this


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self):
        super().__init__("vo_experiment",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.vo, self.gt, self.lost, self.rgb_t, self.depth_t, self.info_t = [], [], [], [], [], []
        q = QoSProfile(depth=50)
        self.create_subscription(Odometry, "/odom", self._on_vo, q)
        self.create_subscription(Odometry, "/odom_ground_truth", self._on_gt, q)
        self.create_subscription(Image, "/face_camera/image",
                                 lambda m: self.rgb_t.append(stamp(m)), q)
        self.create_subscription(Image, "/face_camera/depth_image",
                                 lambda m: self.depth_t.append(stamp(m)), q)
        self.create_subscription(CameraInfo, "/face_camera/camera_info",
                                 lambda m: self.info_t.append(stamp(m)), q)
        try:                                     # optional: richer tracking detail
            from rtabmap_msgs.msg import OdomInfo
            self.odom_info = []
            self.create_subscription(OdomInfo, "/odom_info",
                                     lambda m: self.odom_info.append(
                                         (stamp(m), m.features, m.inliers, m.matches, m.lost)), q)
        except ImportError:
            self.odom_info = None

    def _on_vo(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        if m.pose.covariance[0] >= LOST_COVARIANCE or not math.isfinite(p.x):
            self.lost.append(stamp(m))           # null odom: tracking lost this frame
        else:
            self.vo.append((stamp(m), p.x, p.y, p.z, yaw_of(o)))

    def _on_gt(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        self.gt.append((stamp(m), p.x, p.y, p.z, yaw_of(o)))


class Load:
    """Samples Gazebo's real-time factor and the CPU/GPU cost of running the
    odometry, so the report can say what Stage 2.3 costs the simulation."""

    def __init__(self):
        self.rtf, self.cpu, self.gpu = [], [], []
        self.procs = {}
        try:
            from gz.msgs10.world_stats_pb2 import WorldStatistics
            from gz.transport13 import Node as GzNode
            self._gz = GzNode()
            self._gz.subscribe(WorldStatistics, f"/world/{WORLD}/stats",
                               lambda m: self.rtf.append(m.real_time_factor))
        except Exception as exc:                 # pragma: no cover - tooling only
            print(f"   (no real-time factor: {exc})")
        import psutil
        for p in psutil.process_iter(["name", "cmdline"]):
            cmd = " ".join(p.info["cmdline"] or [])
            for key, pattern in (("gz_server", "gz sim server"), ("gz_gui", "gz sim gui"),
                                 ("bridge", "parameter_bridge"),
                                 ("rgbd_odometry", "rtabmap_odom/rgbd_odometry"),
                                 ("gait", "hexapod_gait/gait_node")):
                if pattern in cmd and key not in self.procs:
                    self.procs[key] = p
                    p.cpu_percent(None)          # prime the counter
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._sample, daemon=True)

    def _sample(self):
        while not self._stop.wait(2.0):
            row = {}
            for key, proc in self.procs.items():
                try:
                    row[key] = proc.cpu_percent(None)
                except Exception:
                    row[key] = float("nan")
            self.cpu.append(row)
            try:
                out = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu",
                                      "--format=csv,noheader,nounits"],
                                     capture_output=True, text=True, timeout=4)
                self.gpu.append(float(out.stdout.split()[0]))
            except Exception:
                pass

    def start(self):
        self._thread.start()

    def summary(self):
        out = {}
        if self.rtf:
            out["rtf_mean"] = float(np.mean(self.rtf))
            out["rtf_min"] = float(min(self.rtf))
        for key in self.procs:
            vals = [row[key] for row in self.cpu if math.isfinite(row.get(key, float("nan")))]
            if vals:
                out[f"cpu_{key}_mean_pct"] = float(np.mean(vals))
                out[f"cpu_{key}_max_pct"] = float(max(vals))
        if self.gpu:
            out["gpu_mean_pct"] = float(np.mean(self.gpu))
            out["gpu_max_pct"] = float(max(self.gpu))
        return out

    def stop(self):
        self._stop.set()


def rate(times):
    if len(times) < 3:
        return float("nan")
    return (len(times) - 1) / (times[-1] - times[0])


def align_ground_truth(gt):
    """Express ground truth relative to its first pose, so it shares the visual
    odometry's convention of starting at the origin facing along +x."""
    t0, x0, y0, z0, yaw0 = gt[0]
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    out = []
    for t, x, y, z, yaw in gt:
        dx, dy = x - x0, y - y0
        out.append((t, dx * c - dy * s, dx * s + dy * c, z - z0,
                    math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))))
    return out


def resample(series, times):
    """Nearest-in-time sample of `series` for each t in `times`."""
    arr = np.array([s[0] for s in series])
    out = []
    for t in times:
        i = int(np.argmin(np.abs(arr - t)))
        out.append(series[i])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--loop", choices=["ring", "core", "rooms"], default="ring")
    ap.add_argument("--label", default="run")
    ap.add_argument("--no-bag", action="store_true")
    ap.add_argument("--straight", type=float, default=0.0,
                    help="instead of a loop, walk straight for this many seconds")
    ap.add_argument("--settle", type=float, default=6.0, help="s of standing before driving")
    args = ap.parse_args()

    out_dir = os.path.join(WS, "verification", "runs",
                           f"stage2_vo_{args.label}_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out_dir)

    rclpy.init()
    node = Recorder()
    t0 = time.time()
    while time.time() - t0 < 25 and not (node.vo and node.gt):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.vo:
        print("no /odom from rgbd_odometry - is visual_odometry.launch.py running?")
        return 1
    if not node.gt:
        print("no /odom_ground_truth - is the simulation running?")
        return 1

    bag = None
    if not args.no_bag:
        bag = subprocess.Popen(
            ["ros2", "bag", "record", "-o", os.path.join(out_dir, "bag"), "--storage", "mcap",
             "--storage-preset-profile", "zstd_fast", *BAG_TOPICS],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(3.0)

    # let the odometry settle while standing still, then drive the loop
    t0 = time.time()
    while time.time() - t0 < args.settle:
        rclpy.spin_once(node, timeout_sec=0.05)
    start_idx = (len(node.vo), len(node.gt))
    load = Load()
    load.start()
    print(f"driving {'straight for %.0f s' % args.straight if args.straight else args.loop + ' loop'}...",
          flush=True)
    if args.straight:
        drive = subprocess.Popen(
            ["timeout", str(args.straight), "ros2", "topic", "pub", "-r", "10", "/cmd_vel",
             "geometry_msgs/msg/Twist", "{linear: {x: 0.095}}", "--print", "100"], cwd=WS,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    else:
        drive = subprocess.Popen(["ros2", "run", "hexapod_worlds", "drive_facility_loop",
                                  "--loop", args.loop], cwd=WS,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    drive_log = []

    def read_drive():                            # waypoints are minutes apart, so the
        for line in drive.stdout:                # log is read on its own thread and we
            drive_log.append(line.rstrip())      # never stop spinning the subscriptions
            print("   " + line.rstrip(), flush=True)

    reader = threading.Thread(target=read_drive, daemon=True)
    reader.start()
    while drive.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.05)
    reader.join(timeout=5)
    if args.straight:                            # the loop driver stops itself; this does not
        subprocess.run(["ros2", "topic", "pub", "-1", "/cmd_vel", "geometry_msgs/msg/Twist", "{}"],
                       capture_output=True)
        t0 = time.time()
        while time.time() - t0 < 4:              # let it come to a stand
            rclpy.spin_once(node, timeout_sec=0.05)
    t0 = time.time()
    while time.time() - t0 < 3:                 # catch the last messages
        rclpy.spin_once(node, timeout_sec=0.05)
    load.stop()
    if bag:
        os.killpg(bag.pid, signal.SIGINT)
        try:
            bag.wait(timeout=25)
        except subprocess.TimeoutExpired:
            os.killpg(bag.pid, signal.SIGKILL)

    vo = node.vo[start_idx[0]:]
    gt = align_ground_truth(node.gt[start_idx[1]:])
    if len(vo) < 20 or len(gt) < 20:
        print(f"not enough data (vo {len(vo)}, gt {len(gt)})")
        return 1
    # visual odometry also starts at its own origin; re-zero it on the drive start
    vt0, vx0, vy0, vz0, vyaw0 = vo[0]
    c, s = math.cos(-vyaw0), math.sin(-vyaw0)
    vo = [(t, (x - vx0) * c - (y - vy0) * s, (x - vx0) * s + (y - vy0) * c, z - vz0,
           math.atan2(math.sin(yaw - vyaw0), math.cos(yaw - vyaw0))) for t, x, y, z, yaw in vo]

    times = [v[0] for v in vo]
    gt_s = resample(gt, times)
    err = np.array([math.hypot(v[1] - g[1], v[2] - g[2]) for v, g in zip(vo, gt_s)])
    yaw_err = np.array([math.degrees(abs(math.atan2(math.sin(v[4] - g[4]),
                                                    math.cos(v[4] - g[4]))))
                        for v, g in zip(vo, gt_s)])
    gt_path = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(gt, gt[1:]))
    vo_path = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(vo, vo[1:]))

    gaps = [b - a for a, b in zip(times, times[1:])]
    # how far the robot had walked when tracking first failed, and where it was
    losses = [t for t in node.lost if t >= times[0]]
    gt_travel = [0.0]
    for a, b in zip(gt, gt[1:]):
        gt_travel.append(gt_travel[-1] + math.hypot(b[1] - a[1], b[2] - a[2]))
    def travelled_at(t):
        return gt_travel[int(np.argmin(np.abs(np.array([g[0] for g in gt]) - t)))]
    def gt_at(t):
        return gt[int(np.argmin(np.abs(np.array([g[0] for g in gt]) - t)))]
    first_loss = min(losses) if losses else None
    # camera gaps tell us whether an odometry gap was the odometry falling behind
    # or Gazebo simply not rendering a frame during a stall
    rgb_in_run = [t for t in node.rgb_t if t >= times[0]]
    rgb_gaps = [b - a for a, b in zip(rgb_in_run, rgb_in_run[1:])] or [0.0]
    results = dict(
        label=args.label, loop="straight" if args.straight else args.loop,
        date=datetime.datetime.now().isoformat(timespec="seconds"),
        ground_truth_path_m=gt_path, visual_odometry_path_m=vo_path,
        ate_rmse_m=float(np.sqrt((err ** 2).mean())), ate_mean_m=float(err.mean()),
        final_position_error_m=float(err[-1]), max_position_error_m=float(err.max()),
        drift_per_metre=float(err[-1] / gt_path) if gt_path else float("nan"),
        final_yaw_error_deg=float(yaw_err[-1]), max_yaw_error_deg=float(yaw_err.max()),
        vo_messages=len(vo), lost_frames=len(losses),
        lost_fraction=len(losses) / max(1, len(losses) + len(vo)),
        tracked_distance_m=travelled_at(times[-1]),
        first_loss_after_m=travelled_at(first_loss) if first_loss else None,
        first_loss_position=gt_at(first_loss)[1:3] if first_loss else None,
        vo_rate_hz=float(rate(times)), rgb_rate_hz=float(rate(node.rgb_t)),
        depth_rate_hz=float(rate(node.depth_t)), info_rate_hz=float(rate(node.info_t)),
        worst_gap_s=float(max(gaps)) if gaps else None,
        gaps_over_0p5s=int(sum(1 for g in gaps if g > 0.5)),
        worst_camera_gap_s=float(max(rgb_gaps)),
        camera_gaps_over_0p5s=int(sum(1 for g in rgb_gaps if g > 0.5)),
        drive_log=drive_log,
        bag=None if args.no_bag else os.path.join(os.path.basename(out_dir), "bag"),
        **load.summary(),
    )
    if node.odom_info:
        feats = [f for _, f, _, _, _ in node.odom_info]
        inl = [i for _, _, i, _, _ in node.odom_info]
        results.update(features_mean=float(np.mean(feats)), features_min=int(min(feats)),
                       inliers_mean=float(np.mean(inl)), inliers_min=int(min(inl)),
                       odom_info_lost=int(sum(1 for *_, lost in node.odom_info if lost)))

    with open(os.path.join(out_dir, "series.json"), "w") as f:
        json.dump(dict(vo=vo, gt=gt, lost=node.lost), f)   # re-scorable without re-walking
    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(results, f, indent=2)

    # plot: the two paths, and the error as the robot travels
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))
    ax1.plot([g[1] for g in gt], [g[2] for g in gt], "-", color="#1a8c46", lw=2.5,
             label="ground truth")
    ax1.plot([v[1] for v in vo], [v[2] for v in vo], "-", color="#2a5db0", lw=2.0,
             label="RGB-D visual odometry")
    ax1.plot(gt[0][1], gt[0][2], "o", color="#111", ms=9, label="start")
    ax1.plot(gt[-1][1], gt[-1][2], "s", color="#1a8c46", ms=9, label="ground-truth end")
    ax1.plot(vo[-1][1], vo[-1][2], "s", color="#2a5db0", ms=9, label="odometry end")
    ax1.set_aspect("equal"); ax1.grid(alpha=0.3); ax1.legend(loc="best", fontsize=9)
    ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
    ax1.set_title(f"{args.label}: {args.loop} loop, {gt_path:.1f} m walked")
    travel = np.cumsum([0] + [math.hypot(b[1] - a[1], b[2] - a[2])
                              for a, b in zip(gt_s, gt_s[1:])])
    ax2.plot(travel, err, color="#b03a2a", lw=2, label="position error")
    ax2b = ax2.twinx()
    ax2b.plot(travel, yaw_err, color="#8a6d1f", lw=1.4, alpha=0.8, label="yaw error")
    ax2.set_xlabel("distance travelled (m)"); ax2.set_ylabel("position error (m)")
    ax2b.set_ylabel("yaw error (deg)")
    ax2.grid(alpha=0.3); ax2.set_title("drift against ground truth")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "trajectory.png"), dpi=110)

    print(f"\n{'ground truth path':28s} {gt_path:7.2f} m")
    print(f"{'visual odometry path':28s} {vo_path:7.2f} m")
    print(f"{'ATE (RMSE)':28s} {results['ate_rmse_m']:7.3f} m")
    print(f"{'final position error':28s} {results['final_position_error_m']:7.3f} m "
          f"({100 * results['drift_per_metre']:.1f}% of distance)")
    print(f"{'max position error':28s} {results['max_position_error_m']:7.3f} m")
    print(f"{'final yaw error':28s} {results['final_yaw_error_deg']:7.2f} deg")
    print(f"{'max yaw error':28s} {results['max_yaw_error_deg']:7.2f} deg")
    print(f"{'odometry rate':28s} {results['vo_rate_hz']:7.1f} Hz "
          f"(rgb {results['rgb_rate_hz']:.1f}, depth {results['depth_rate_hz']:.1f})")
    if results["first_loss_after_m"] is not None:
        x, y = results["first_loss_position"]
        print(f"{'TRACKING LOST after':28s} {results['first_loss_after_m']:7.2f} m "
              f"(at aligned x {x:+.2f}, y {y:+.2f}); "
              f"{100 * results['lost_fraction']:.0f}% of frames lost")
    print(f"{'lost frames':28s} {results['lost_frames']:7d}   "
          f"gaps > 0.5 s: {results['gaps_over_0p5s']}  worst gap "
          f"{results['worst_gap_s'] or 0:.2f} s "
          f"(worst camera gap {results['worst_camera_gap_s']:.2f} s)")
    if node.odom_info:
        print(f"{'features / inliers (mean)':28s} {results['features_mean']:7.0f} / "
              f"{results['inliers_mean']:.0f}   min inliers {results['inliers_min']}")
    if "rtf_mean" in results:
        print(f"{'real-time factor':28s} {results['rtf_mean']:7.2f} mean, "
              f"{results['rtf_min']:.2f} min")
    cpu = {k[4:-9]: v for k, v in results.items() if k.startswith("cpu_") and k.endswith("_mean_pct")}
    if cpu:
        print(f"{'CPU (mean %)':28s} " + "  ".join(f"{k} {v:.0f}" for k, v in cpu.items()))
    if "gpu_mean_pct" in results:
        print(f"{'GPU':28s} {results['gpu_mean_pct']:7.0f}% mean, {results['gpu_max_pct']:.0f}% max")
    print(f"\nresults: {out_dir}")
    sys.stdout.flush()      # os._exit skips the flush that a normal exit would do
    os._exit(0)             # rclpy's shutdown can hang on the gz transport thread


if __name__ == "__main__":
    sys.exit(main())
