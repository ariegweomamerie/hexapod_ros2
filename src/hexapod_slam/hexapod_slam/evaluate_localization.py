#!/usr/bin/env python3
"""Stage 3 localisation experiment runner and segment-aware scorer.

Drives the Stage 3 route, records what the localisation stack believes, and
scores it against Gazebo ground truth. Ground truth is read ONLY here, for
scoring; nothing in the localisation pipeline ever sees it.

    ros2 run hexapod_slam run_localization_experiment -- --experiment L2

The one thing this file is careful about, above everything else: it does NOT
treat "map -> odom is being published" as evidence that localisation works. A
transform can be published continuously while RTAB-Map has recognised nothing
at all, in which case the robot is running on dead reckoning wearing a map's
clothes. Four quantities are therefore measured independently:

    1. TF availability        - can map -> odom -> base_footprint be resolved?
    2. VO continuity          - is odom -> base_footprint valid per frame?
    3. Recognition events     - is RTAB-Map actually matching the reference?
    4. Pose consistency       - does map -> base_footprint track ground truth?

A run in which 1 and 2 are perfect and 3 is empty is a localisation failure,
and this scorer reports it as one.
"""
import argparse
import datetime
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from hexapod_slam.evaluate_odometry import LOST_COVARIANCE, yaw_of

WS = "/home/general/hexapod_ros_robot_ws"
BAG_TOPICS = ["/face_camera/image", "/face_camera/depth_image", "/face_camera/camera_info",
              "/odom", "/odom_ground_truth", "/tf", "/tf_static", "/cmd_vel", "/clock",
              "/joint_states", "/imu", "/info", "/localization_pose",
              "/map", "/mapPath"]

# The reference map's frame is the Stage 2.4 run's start pose: base_footprint at
# world START with body yaw 1.5708. So map +x runs along world +y, and:
#     world_x = START_x - map_y      world_y = START_y + map_x
#     world_heading = map_yaw        (body yaw = heading + 90 deg)
# Verified against the reference: node 138 at map (4.28, -8.32) decodes to world
# (11.62, 7.58), the ring's NE corner.
REACHED = 0.35          # same waypoint tolerance the follower uses


def map_to_world(mx, my, myaw, start):
    return start[0] - my, start[1] + mx, myaw


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Recorder(Node):
    """Everything the scorer needs, sampled live."""

    def __init__(self):
        super().__init__("stage3_localization_recorder")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        import tf2_ros
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.gt = []            # (t, x, y, yaw)              world
        self.loc = []           # (t, x, y, yaw)              world, from map->base_footprint
        self.vo_lost = []       # (t, bool)
        self.tf_avail = []      # (t, map->odom, odom->base, map->base)
        self.map_odom = []      # (t, x, y, yaw)              the correction itself
        self.events = []        # (t, kind, id)
        # How many /info messages actually reached the callback. Without it,
        # "recognition events: 0" cannot be told apart from "no message ever
        # arrived" - which is exactly what a subscription to the wrong topic
        # produced before. Counted over the recorder's lifetime, the same span
        # the recognition count covers, so the two are directly comparable.
        self.info_received = 0

        q = 20
        self.create_subscription(Odometry, "/odom_ground_truth", self._on_gt, q)
        self.create_subscription(Odometry, "/odom", self._on_odom, q)
        try:
            from rtabmap_msgs.msg import Info
            self.create_subscription(Info, "/info", self._on_info, q)
            self.have_info = True
        except Exception:
            self.have_info = False
        self.create_timer(0.1, self._sample_tf)          # 10 Hz TF availability

    # ------------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_gt(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, o = m.pose.pose.position, m.pose.pose.orientation
        self.gt.append((t, p.x, p.y, yaw_of(o)))

    def _on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.vo_lost.append((t, m.pose.covariance[0] >= LOST_COVARIANCE))

    def _on_info(self, m):
        self.info_received += 1
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if m.loop_closure_id > 0:
            self.events.append((t, "loop_closure", int(m.loop_closure_id)))
        if m.proximity_detection_id > 0:
            self.events.append((t, "proximity", int(m.proximity_detection_id)))

    def _sample_tf(self):
        import rclpy.time
        t = self._now()
        got = {}
        for name, a, b in (("map_odom", "map", "odom"),
                           ("odom_base", "odom", "base_footprint"),
                           ("map_base", "map", "base_footprint")):
            try:
                tr = self.buf.lookup_transform(a, b, rclpy.time.Time())
                got[name] = tr
            except Exception:
                got[name] = None
        self.tf_avail.append((t, got["map_odom"] is not None,
                              got["odom_base"] is not None, got["map_base"] is not None))
        if got["map_base"] is not None:
            tr = got["map_base"].transform
            self.loc.append((t, tr.translation.x, tr.translation.y, yaw_of(tr.rotation)))
        if got["map_odom"] is not None:
            tr = got["map_odom"].transform
            self.map_odom.append((t, tr.translation.x, tr.translation.y, yaw_of(tr.rotation)))


# --------------------------------------------------------------- segmentation
def segment(gt, route, segments):
    """Label every ground-truth sample with a route segment.

    Replays the follower's own logic - drive at waypoint k until within REACHED,
    then target k+1 - so a window is defined by where the robot actually was
    rather than by wall-clock timing, which has bitten this project before. The
    route's 4.2 m overlap is handled naturally: J comes after I in time, so its
    samples land in J even though the ground was already walked in B.
    """
    labels = [None] * len(gt)

    # Deciding WHEN the robot was driving has to be robust to ground-truth
    # jitter. A per-sample "did it move at all" test is not: millimetre noise
    # while standing perfectly still makes almost every sample look like
    # motion, and a stationary L0 run then scores as a successful walk down the
    # south leg. So: a run counts as having a drive phase only if it travels
    # MIN_TRAVEL from where it started, and motion is judged from a smoothed
    # speed rather than consecutive samples.
    MIN_TRAVEL = 0.50        # m from the start pose before a run has "driven"
    MIN_SPEED = 0.02         # m/s, well under the gait's 0.20 m/s cruise
    SPEED_WIN = 0.50         # s to average speed over

    x0, y0 = gt[0][1], gt[0][2]
    travelled = max(math.hypot(g[1] - x0, g[2] - y0) for g in gt)
    if travelled < MIN_TRAVEL:
        # Stationary run (L0). Every sample is the opening dwell; no movement
        # segment may be claimed, and no movement window may be scored.
        return [segments[0][0]] * len(gt)

    speeds = []
    j = 0
    for i in range(len(gt)):
        while gt[i][0] - gt[j][0] > SPEED_WIN and j < i:
            j += 1
        dt = gt[i][0] - gt[j][0]
        # Only trust a speed once the lookback window is actually long enough.
        # Early on the window is short, so jitter divided by a few tens of
        # milliseconds reads as metres per second and the opening dwell gets
        # truncated to a single sample.
        if dt < 0.8 * SPEED_WIN:
            speeds.append(0.0)
            continue
        d = math.hypot(gt[i][1] - gt[j][1], gt[i][2] - gt[j][2])
        speeds.append(d / dt)
    moving = [i for i, s in enumerate(speeds) if s > MIN_SPEED]
    t_move0 = gt[moving[0]][0] if moving else gt[-1][0]
    t_move1 = gt[moving[-1]][0] if moving else gt[-1][0]

    straight = [s for s in segments if s[3] != "stationary"]
    target = 1                                   # waypoint index being driven at
    seg_of_leg = {}                              # leg k -> (straight seg, turn seg)
    legs = [s for s in straight if s[3] in ("straight", "revisit")]
    turns = [s for s in straight if s[3] == "turn"]
    for k in range(len(route) - 1):
        seg_of_leg[k + 1] = (legs[k][0] if k < len(legs) else legs[-1][0],
                             turns[k][0] if k < len(turns) else None)

    for i, (t, x, y, _) in enumerate(gt):
        if t < t_move0:
            labels[i] = segments[0][0]                      # A, the opening dwell
            continue
        if t > t_move1:
            labels[i] = segments[-1][0]                     # K, the closing dwell
            continue
        # Advance exactly as the follower does: a waypoint within REACHED counts
        # as arrived, and the next one becomes the target.
        while target < len(route) - 1 and math.hypot(route[target][0] - x,
                                                     route[target][1] - y) < REACHED:
            target += 1
        leg = min(target, len(route) - 1)
        lab, _turn = seg_of_leg.get(leg, (segments[1][0], None))
        # The turn happens just AFTER that advance: the follower is now aiming
        # at the next waypoint while still standing at the corner it just
        # reached, and its forward term (cos(err)^2) is ~0 until it has swung
        # round. So the turn window is proximity to the corner already LEFT,
        # route[leg - 1] - not to the one being driven at.
        prev = leg - 1
        if 1 <= prev <= len(route) - 2:
            _l, turn_here = seg_of_leg.get(prev, (None, None))
            if turn_here and math.hypot(route[prev][0] - x,
                                        route[prev][1] - y) < REACHED:
                lab = turn_here
        labels[i] = lab
    return labels


def resample_to(src, times):
    """Nearest-in-time lookup of src (t, ...) at each t in times."""
    if not src:
        return [None] * len(times)
    arr = np.array([s[0] for s in src])
    return [src[int(np.argmin(np.abs(arr - t)))] for t in times]


# -------------------------------------------------------------------- scoring
def score(rec, route, segments, windows, start, experiment):
    gt = sorted(rec.gt)
    if len(gt) < 50:
        raise SystemExit(f"not enough ground truth ({len(gt)} samples)")
    labels = segment(gt, route, segments)
    times = [g[0] for g in gt]

    loc_world = [(t, *map_to_world(x, y, yw, start)) for t, x, y, yw in rec.loc]
    loc_at = resample_to(loc_world, times)
    lost_at = resample_to(rec.vo_lost, times)
    tf_at = resample_to(rec.tf_avail, times)

    rows = []
    for i, (t, gx, gy, gyaw) in enumerate(gt):
        L = loc_at[i]
        row = dict(t=t, seg=labels[i], gt_x=gx, gt_y=gy, gt_yaw=gyaw)
        if L is not None and abs(L[0] - t) < 0.5:
            # ground truth yaw is body yaw; the localised heading from the map
            # frame is a heading, so put both on the same footing
            row.update(loc_x=L[1], loc_y=L[2],
                       pos_err=math.hypot(L[1] - gx, L[2] - gy),
                       yaw_err=abs(math.degrees(wrap(L[3] - (gyaw - math.pi / 2)))))
        row["vo_lost"] = bool(lost_at[i][1]) if lost_at[i] else None
        if tf_at[i]:
            row.update(tf_map_odom=tf_at[i][1], tf_odom_base=tf_at[i][2],
                       tf_map_base=tf_at[i][3])
        rows.append(row)

    def stats(sel):
        e = [r["pos_err"] for r in sel if "pos_err" in r]
        y = [r["yaw_err"] for r in sel if "yaw_err" in r]
        n = len(sel)
        return dict(
            samples=n,
            localized_samples=len(e),
            coverage=round(len(e) / n, 4) if n else None,
            pos_rms=round(float(np.sqrt(np.mean(np.square(e)))), 4) if e else None,
            pos_max=round(float(np.max(e)), 4) if e else None,
            yaw_rms=round(float(np.sqrt(np.mean(np.square(y)))), 4) if y else None,
            yaw_max=round(float(np.max(y)), 4) if y else None,
            vo_lost_samples=sum(1 for r in sel if r.get("vo_lost")),
            tf_map_base_unavailable=sum(1 for r in sel if r.get("tf_map_base") is False),
        )

    per_segment = {s[0]: stats([r for r in rows if r["seg"] == s[0]]) for s in segments}
    per_window = {}
    for w, segs in windows.items():
        s = stats([r for r in rows if r["seg"] in segs])
        # A window with no samples was NOT exercised by this run. Saying so
        # explicitly matters: an empty window prints as zeros and blanks, and
        # a reader skimming a table should never be able to mistake "this
        # experiment did not happen" for "this experiment passed".
        s["exercised"] = s["samples"] > 0
        s["segments"] = segs
        per_window[w] = s

    # --- the four continuity quantities, measured separately ------------------
    tf = rec.tf_avail
    def longest_gap(pred, series):
        worst = cur = 0.0
        prev = None
        for s in series:
            bad = pred(s)
            if bad and prev is not None:
                cur += s[0] - prev
                worst = max(worst, cur)
            else:
                cur = 0.0
            prev = s[0]
        return round(worst, 3)

    tf_outage = longest_gap(lambda s: not s[3], tf)                 # map->base missing
    vo_outage = longest_gap(lambda s: s[1], sorted(rec.vo_lost))    # odom lost
    ev = sorted(rec.events)
    if ev:
        gaps = [ev[i][0] - ev[i - 1][0] for i in range(1, len(ev))]
        reco = dict(count=len(ev), first_t=ev[0][0], last_t=ev[-1][0],
                    longest_interval_s=round(max(gaps), 3) if gaps else None,
                    median_interval_s=round(float(np.median(gaps)), 3) if gaps else None,
                    unique_reference_nodes=len({e[2] for e in ev}))
    else:
        reco = dict(count=0, longest_interval_s=None, unique_reference_nodes=0)
    reco["info_messages_received"] = rec.info_received

    # map->odom correction magnitude: how far the pose was yanked each time
    jumps = []
    for i in range(1, len(rec.map_odom)):
        a, b = rec.map_odom[i - 1], rec.map_odom[i]
        d = math.hypot(b[1] - a[1], b[2] - a[2])
        if d > 1e-4:
            jumps.append((b[0], round(d, 4),
                          round(abs(math.degrees(wrap(b[3] - a[3]))), 3)))

    # bounded error growth: the property that separates localisation from
    # dead reckoning. A ratio near 1 means the map is genuinely correcting.
    errs = [r["pos_err"] for r in rows if "pos_err" in r]
    half = len(errs) // 2
    growth = None
    if half > 20:
        a = float(np.sqrt(np.mean(np.square(errs[:half]))))
        b = float(np.sqrt(np.mean(np.square(errs[half:]))))
        growth = round(b / a, 3) if a > 1e-9 else None

    return dict(
        experiment=experiment,
        samples=len(rows),
        overall=stats(rows),
        per_segment=per_segment,
        per_window=per_window,
        continuity=dict(
            tf_longest_outage_s=tf_outage,
            vo_longest_outage_s=vo_outage,
            vo_lost_frames=sum(1 for _, b in rec.vo_lost if b),
            recognition=reco,
            map_odom_corrections=len(jumps),
            map_odom_largest_jump_m=round(max((j[1] for j in jumps), default=0.0), 4),
            map_odom_largest_jump_deg=round(max((j[2] for j in jumps), default=0.0), 3),
        ),
        error_growth_2nd_half_over_1st=growth,
        # The headline distinction. Everything else can look healthy while this
        # is false, and then the robot is dead-reckoning with a map attached.
        localization_actually_recognised=reco["count"] > 0,
    ), rows


# ------------------------------------------------------------------- database
def db_counts(path):
    import sqlite3
    con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        return dict(nodes=con.execute("SELECT COUNT(*) FROM Node;").fetchone()[0],
                    links=con.execute("SELECT COUNT(*) FROM Link;").fetchone()[0],
                    words=con.execute("SELECT COUNT(*) FROM Word;").fetchone()[0])
    finally:
        con.close()


def sha256(path):
    import hashlib
    d = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            d.update(chunk)
    return d.hexdigest()


def main():
    from hexapod_worlds import layout as L

    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--experiment", choices=["L0", "L1", "L2", "L3L4"], default="L0")
    ap.add_argument("--run-dir", default=None,
                    help="existing run dir (the one the launch copied the reference into)")
    # Dwell A and dwell K are different things and are produced by different
    # processes. A is owned by THIS script and always happens; K is owned by
    # the waypoint follower and only exists when the robot drives. For L0 there
    # is no drive at all, so --settle IS the entire L0 measurement window and
    # --final-dwell is meaningless.
    ap.add_argument("--settle", type=float, default=30.0,
                    help="s of dwell A: stationary, before any driving. For L0 this is "
                         "the whole measurement window")
    ap.add_argument("--final-dwell", type=float, default=20.0,
                    help="s of dwell K: stationary, after driving. Ignored for L0, "
                         "which never drives")
    ap.add_argument("--bag", action="store_true", help="record a full rosbag")
    ap.add_argument("--timeout", type=float, default=1200.0)
    args = ap.parse_args()

    route = L.loop_localization()
    segments = L.SEGMENTS_LOCALIZATION
    windows = L.SCORING_WINDOWS
    start = (L.START["x"], L.START["y"])

    out_dir = args.run_dir or os.path.join(
        WS, "verification", "runs",
        f"stage3_{args.experiment}_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out_dir, exist_ok=True)

    working = os.path.join(out_dir, "reference.db")
    before = db_counts(working) if os.path.exists(working) else None

    rclpy.init()
    rec = Recorder()
    t0 = time.time()
    while time.time() - t0 < 30 and not rec.gt:
        rclpy.spin_once(rec, timeout_sec=0.1)
    if not rec.gt:
        print("no /odom_ground_truth - is the simulation running?")
        return 1
    if not rec.have_info:
        print("WARNING: rtabmap_msgs not importable; recognition events will be empty")

    # Guard against a second recorder or a second follower. The experiment
    # script is started by hand in a second terminal, so "run it twice by
    # accident" is a real failure mode, and a stray recorder has already run
    # for 3 h 18 m across three experiments on this project.
    ps = subprocess.run("ps -eo args", shell=True, capture_output=True,
                        text=True).stdout.splitlines()
    def already(pat):
        return sum(1 for l in ps if pat in l and "grep" not in l
                   and "run_localization_experiment" not in l)

    # A waypoint follower already running is fatal for EVERY experiment, and
    # most of all for L0: static localisation measured while something else is
    # walking the robot is not static localisation.
    if already("drive_facility_loop"):
        print("refusing to start: a waypoint follower is already running - the robot "
              "would be driven by two things at once")
        return 1
    if args.bag and already("bag record"):
        print("refusing to start: a rosbag recorder is already running")
        return 1

    bag = None
    if args.bag:
        bag = subprocess.Popen(
            ["ros2", "bag", "record", "-o", os.path.join(out_dir, "bag"), "--storage", "mcap",
             "--storage-preset-profile", "zstd_fast", *BAG_TOPICS],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(3.0)

    # dwell A: stand still while localisation settles against the reference
    print(f"dwell A: standing still for {args.settle:.0f} s", flush=True)
    t0 = time.time()
    while time.time() - t0 < args.settle:
        rclpy.spin_once(rec, timeout_sec=0.05)

    drive = None
    if args.experiment == "L0":
        # L0 is static localisation. Nothing drives, dwell K does not exist,
        # and the scorer will label every sample dwell A.
        if args.final_dwell:
            print(f"note: --final-dwell {args.final_dwell:g} ignored - L0 does not drive, "
                  f"its measurement window is the {args.settle:g} s dwell A")
    else:
        cmd = ["ros2", "run", "hexapod_worlds", "drive_facility_loop", "--",
               "--loop", "localization", "--final-dwell", str(args.final_dwell)]
        print("driving the localisation route: " + " ".join(cmd), flush=True)
        drive = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, start_new_session=True)
        log = []
        threading.Thread(target=lambda: [log.append(l) or print("   " + l.rstrip(), flush=True)
                                         for l in drive.stdout], daemon=True).start()
        t0 = time.time()
        while drive.poll() is None and time.time() - t0 < args.timeout:
            rclpy.spin_once(rec, timeout_sec=0.05)
        if drive.poll() is None:
            drive.kill()
            print("drive timed out")

    if bag:
        bag.terminate()
        try:
            bag.wait(timeout=20)
        except Exception:
            bag.kill()

    results, rows = score(rec, route, segments, windows, start, args.experiment)

    # hard integrity: the reference pose graph must not have grown
    after = db_counts(working) if os.path.exists(working) else None
    results["reference_graph"] = dict(before=before, after=after,
                                      unchanged=(before == after))
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        man = yaml.safe_load(open(os.path.join(
            get_package_share_directory("hexapod_slam"), "config",
            "stage3_reference.yaml")))["stage3_reference"]
        canonical = os.path.join(WS, man["path"])
        got = sha256(canonical)
        results["canonical_reference"] = dict(
            sha256_expected=man["sha256"], sha256_after_run=got,
            unchanged=(got == man["sha256"]))
        with open(os.path.join(out_dir, "reference_sha256_after.txt"), "w") as f:
            f.write(f"{got}  {man['path']}\n")
    except Exception as exc:
        results["canonical_reference"] = dict(error=str(exc))

    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(results, f, indent=2)
    with open(os.path.join(out_dir, "series.json"), "w") as f:
        json.dump(dict(rows=rows, events=rec.events, map_odom=rec.map_odom), f)

    # ------------------------------------------------------------------ report
    o = results["overall"]
    c = results["continuity"]
    print(f"\n{'localised samples':34s} {o['localized_samples']}/{o['samples']} "
          f"({100 * (o['coverage'] or 0):.1f}%)")
    print(f"{'position error  RMS / max':34s} "
          f"{o['pos_rms']} m / {o['pos_max']} m")
    print(f"{'yaw error       RMS / max':34s} {o['yaw_rms']} deg / {o['yaw_max']} deg")
    print(f"{'error growth (2nd half/1st)':34s} "
          f"{results['error_growth_2nd_half_over_1st']}")
    print(f"\n-- the four continuity quantities, measured separately --")
    print(f"{'1. TF longest outage':34s} {c['tf_longest_outage_s']} s")
    print(f"{'2. VO longest outage':34s} {c['vo_longest_outage_s']} s "
          f"({c['vo_lost_frames']} lost frames)")
    print(f"{'3. recognition events':34s} {c['recognition']['count']} "
          f"({c['recognition']['unique_reference_nodes']} distinct reference nodes)")
    print(f"{'   /info messages received':34s} {c['recognition']['info_messages_received']}")
    print(f"{'   longest gap between them':34s} "
          f"{c['recognition']['longest_interval_s']} s")
    print(f"{'4. pose consistency':34s} see position/yaw error above")
    print(f"\n{'map->odom corrections':34s} {c['map_odom_corrections']}, "
          f"largest {c['map_odom_largest_jump_m']} m / {c['map_odom_largest_jump_deg']} deg")
    if not results["localization_actually_recognised"]:
        print("\n*** NO RECOGNITION EVENTS: map -> odom may have been published "
              "throughout, but RTAB-Map never matched the reference. This is a "
              "localisation failure regardless of the pose error above. ***")
    print(f"\n{'reference graph unchanged':34s} {results['reference_graph']['unchanged']}")
    print(f"{'canonical reference unchanged':34s} "
          f"{results.get('canonical_reference', {}).get('unchanged')}")
    print(f"\nper window (segments in brackets):")
    for w, s in results["per_window"].items():
        tag = "".join(s["segments"])
        if not s["exercised"]:
            print(f"  {w:5s} [{tag:6s}] NOT EXERCISED by this run - no samples, nothing scored")
            continue
        print(f"  {w:5s} [{tag:6s}] samples {s['samples']:5d}  "
              f"localised {100*(s['coverage'] or 0):5.1f}%  "
              f"pos RMS {s['pos_rms']}  max {s['pos_max']}  yaw RMS {s['yaw_rms']}")
    print(f"\nresults: {out_dir}")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
