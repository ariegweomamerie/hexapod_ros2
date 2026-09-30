#!/usr/bin/env python3
"""Stage 3 preflight: refuse to start a localisation experiment on bad footing.

22 checks - 20 MANDATORY and 2 ADVISORY - run before every Stage 3 experiment.
It exists because this project has already lost runs to problems that were
invisible until the data was being scored:

  * a reference database that passed a size check and was a malformed SQLite
    image with an empty visual vocabulary,
  * a stray `ros2 bag record` that ran for 3 h 18 m across three experiments,
  * a screen recorder at 906 % CPU that invalidated a baseline,
  * duplicate nodes from a stack brought up by hand.

Every one of those is cheap to detect beforehand and expensive to discover
afterwards.

    ros2 run hexapod_slam stage3_preflight                 # LIVE - the real gate
    ros2 run hexapod_slam stage3_preflight -- --offline    # static checks only

MODES
    live     all 22 checks run. Passing means "cleared for launch".
    offline  checks 1-13 and 21 run; the ROS-dependent checks 14-20 and 22 are
             reported SKIPPED, never PASSED. An offline run can therefore never
             say "cleared for launch" - it has not looked at a single sensor.

STATUSES
    PASS  ok           FAIL  mandatory failure, do not start
    SKIP  not run in this mode (offline)          every check is also tagged
                                                  mandatory or advisory

Exit 0 = cleared for launch (live mode, no mandatory failure).
Exit 1 = a mandatory check failed.
Exit 2 = offline mode: static checks passed, live checks not performed.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

WS = "/home/general/hexapod_ros_robot_ws"

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"
MANDATORY, ADVISORY = "mandatory", "advisory"

# The authoritative list. The script, its output and the reports all count from
# this, so "how many checks are there" has exactly one answer.
CHECKS = [
    (1,  "reference database exists",                        MANDATORY),
    (2,  "reference size",                                   MANDATORY),
    (3,  "reference sha256",                                 MANDATORY),
    (4,  "reference SQLite integrity",                       MANDATORY),
    (5,  "reference vocabulary and features",                MANDATORY),
    (6,  "reference node/link counts",                       MANDATORY),
    (7,  "localisation configuration",                       MANDATORY),
    (8,  "odometry configuration frozen at Stage 2.4",       MANDATORY),
    (9,  "world identity",                                   MANDATORY),
    (10, "git commit recorded",                              MANDATORY),
    (11, "git tree state recorded",                          MANDATORY),
    (12, "no duplicate/stale ROS or Gazebo processes",       MANDATORY),
    (13, "no stray recorder or screencast",                  MANDATORY),
    (14, "/clock publishing",                                MANDATORY),
    (15, "RGB image stream",                                 MANDATORY),
    (16, "depth image stream",                               MANDATORY),
    (17, "camera info",                                      MANDATORY),
    (18, "/odom from visual odometry",                       MANDATORY),
    (19, "/odom_ground_truth (evaluation layer only)",       MANDATORY),
    (20, "head joints stationary",                           MANDATORY),
    (21, "canonical reference is write-protected (444)",     ADVISORY),
    (22, "expected dynamic TF edges present",                ADVISORY),
]
NAMES = {n: name for n, name, _ in CHECKS}
KIND = {n: kind for n, _, kind in CHECKS}
LIVE_CHECKS = [14, 15, 16, 17, 18, 19, 20, 22]
N_MANDATORY = sum(1 for _, _, k in CHECKS if k == MANDATORY)
N_ADVISORY = sum(1 for _, _, k in CHECKS if k == ADVISORY)


class Checks:
    def __init__(self):
        self.rows = {}

    def set(self, n, status, detail):
        self.rows[n] = dict(n=n, name=NAMES[n], kind=KIND[n], status=status, detail=detail)

    def ok(self, n, detail):
        self.set(n, PASS, detail)

    def bad(self, n, detail):
        self.set(n, FAIL, detail)

    def skip(self, n, detail):
        self.set(n, SKIP, detail)

    def missing(self):
        return [n for n, _, _ in CHECKS if n not in self.rows]

    @property
    def failures(self):
        return [r for r in self.rows.values()
                if r["status"] == FAIL and r["kind"] == MANDATORY]

    @property
    def skipped_mandatory(self):
        return [r for r in self.rows.values()
                if r["status"] == SKIP and r["kind"] == MANDATORY]


def reference_manifest():
    from ament_index_python.packages import get_package_share_directory
    import yaml
    path = os.path.join(get_package_share_directory('hexapod_slam'),
                        'config', 'stage3_reference.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['stage3_reference'], path


def sha256(path):
    d = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            d.update(chunk)
    return d.hexdigest()


def sh(cmd, timeout=20):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=timeout).stdout.strip()
    except Exception:
        return ""


# ------------------------------------------------------- 1-6, 21  reference
def check_reference(c):
    try:
        ref, _ = reference_manifest()
    except Exception as exc:
        for n in (1, 2, 3, 4, 5, 6):
            c.bad(n, f"reference manifest unreadable: {exc}")
        c.skip(21, "manifest unreadable")
        return
    canonical = os.path.join(WS, ref['path'])

    if not os.path.exists(canonical):
        c.bad(1, canonical)
        for n in (2, 3, 4, 5, 6):
            c.bad(n, "reference file missing")
        c.skip(21, "reference file missing")
        return
    c.ok(1, canonical)

    size = os.path.getsize(canonical)
    c.ok(2, f"{size} bytes") if size == ref['size_bytes'] \
        else c.bad(2, f"{size} != {ref['size_bytes']}")

    got = sha256(canonical)
    c.ok(3, f"{got} matches the manifest") if got == ref['sha256'] \
        else c.bad(3, f"expected {ref['sha256']}  got {got}")

    mode = oct(os.stat(canonical).st_mode)[-3:]
    c.ok(21, f"mode {mode}") if mode == "444" else c.set(21, FAIL, f"mode {mode}, want 444")

    import sqlite3
    try:
        con = sqlite3.connect(f"file:{canonical}?mode=ro", uri=True)
        try:
            integrity = con.execute("PRAGMA integrity_check;").fetchone()[0]
            words = con.execute("SELECT COUNT(*) FROM Word;").fetchone()[0]
            feats = con.execute("SELECT COUNT(*) FROM Feature;").fetchone()[0]
            nodes = con.execute("SELECT COUNT(*) FROM Node;").fetchone()[0]
            l0 = con.execute("SELECT COUNT(*) FROM Link WHERE type=0;").fetchone()[0]
            l1 = con.execute("SELECT COUNT(*) FROM Link WHERE type=1;").fetchone()[0]
            # Counting Word rows is not enough. A database can hold thousands
            # of words and still be missing most of the dictionary entries its
            # own features refer to - which is what the Stage 2.4 reference
            # turned out to be (5159 rows, 56382 referenced, 51223 missing).
            # RTAB-Map masks it by rebuilding the dictionary at load time and
            # only warning, so nothing downstream notices unless this is
            # measured directly.
            distinct_fw = con.execute(
                "SELECT COUNT(DISTINCT word_id) FROM Feature;").fetchone()[0]
            missing_fw = con.execute(
                "SELECT COUNT(DISTINCT f.word_id) FROM Feature f "
                "LEFT JOIN Word w ON w.id = f.word_id WHERE w.id IS NULL;").fetchone()[0]
            # Which nodes those missing ids belong to decides whether they can
            # affect localisation: only nodes in the optimised pose graph are
            # loaded into working memory and matched against.
            missing_in_graph = con.execute(
                "SELECT COUNT(DISTINCT f.word_id) FROM Feature f "
                "LEFT JOIN Word w ON w.id = f.word_id WHERE w.id IS NULL AND EXISTS "
                "(SELECT 1 FROM Link l WHERE l.type=0 AND "
                "(l.from_id=f.node_id OR l.to_id=f.node_id));").fetchone()[0]
        finally:
            con.close()
    except sqlite3.DatabaseError as exc:
        # A database copied while RTAB-Map was writing reads like this. The
        # first proposed Stage 3 reference failed exactly here while passing
        # the size check above.
        c.bad(4, f"{exc} - a database copied while RTAB-Map was writing looks like this")
        c.bad(5, "unreadable")
        c.bad(6, "unreadable")
        return

    c.ok(4, integrity) if integrity == "ok" else c.bad(4, integrity)

    detail = (f"word_rows={words} distinct_feature_word_ids={distinct_fw} "
              f"missing_feature_word_ids={missing_fw} "
              f"(of which {missing_in_graph} belong to pose-graph nodes)")
    if words == 0 or feats == 0:
        c.bad(5, f"{detail} - RTAB-Map cannot relocalise at all")
    elif words != ref['vocabulary_words'] or feats != ref['visual_features']:
        c.bad(5, f"{detail} - expected word_rows={ref['vocabulary_words']}, "
                 f"features={ref['visual_features']}")
    elif missing_fw:
        # Word rows present but the dictionary is incomplete. RTAB-Map will
        # rebuild what it can at load and carry on, so this fails here rather
        # than being discovered from a run's results.
        c.bad(5, f"{detail} - dictionary incomplete")
    else:
        c.ok(5, detail)

    if nodes != ref['nodes'] or l0 != ref['links_neighbour'] or l1 != ref['links_accepted']:
        c.bad(6, f"nodes={nodes}/{ref['nodes']} neighbour={l0}/{ref['links_neighbour']} "
                 f"accepted={l1}/{ref['links_accepted']}")
    else:
        c.ok(6, f"{nodes} nodes, {l0} neighbour links, {l1} accepted links")


# ---------------------------------------------------------- 7-8  configuration
def check_configs(c):
    from ament_index_python.packages import get_package_share_directory
    import yaml
    share = get_package_share_directory('hexapod_slam')

    want = {"Mem/IncrementalMemory": "false", "Mem/InitWMWithAllNodes": "true"}
    try:
        p = yaml.safe_load(open(os.path.join(
            share, 'config', 'rtabmap_localization.yaml')))['rtabmap']['ros__parameters']
        wrong = {k: p.get(k) for k, v in want.items() if str(p.get(k)).lower() != v}
        if wrong:
            c.bad(7, f"wrong: {wrong}")
        elif p.get('publish_tf') is not True or p.get('frame_id') != 'base_footprint':
            c.bad(7, "frame_id/publish_tf unexpected")
        else:
            c.ok(7, "IncrementalMemory=false, InitWMWithAllNodes=true, publish_tf=true")
    except Exception as exc:
        c.bad(7, f"{exc}")

    frozen = {"Odom/ResetCountdown": "0", "Vis/PnPVarianceMedianRatio": "2",
              "Vis/MinInliers": "20"}
    try:
        p = yaml.safe_load(open(os.path.join(
            share, 'config', 'rgbd_odometry.yaml')))['rgbd_odometry']['ros__parameters']
        wrong = {k: p.get(k) for k, v in frozen.items() if str(p.get(k)) != v}
        c.bad(8, f"changed: {wrong}") if wrong \
            else c.ok(8, "ratio 2, ResetCountdown 0, MinInliers 20")
    except Exception as exc:
        c.bad(8, f"{exc}")


# ----------------------------------------------------------- 9-11  provenance
def check_provenance(c):
    from ament_index_python.packages import get_package_share_directory
    world = os.path.join(get_package_share_directory('hexapod_worlds'),
                         'worlds', 'hexapod_facility.sdf')
    c.ok(9, f"sha256 {sha256(world)}") if os.path.exists(world) \
        else c.bad(9, "hexapod_facility.sdf not found")

    head = sh(f"git -C {WS} rev-parse HEAD")
    c.ok(10, head) if head else c.bad(10, "git rev-parse failed")

    # Mandatory that the state is DETERMINED and will be recorded, not that it
    # is clean: implementing a stage means an uncommitted tree, and a run is
    # reproducible as long as the exact diff travels with it.
    probe = subprocess.run(f"git -C {WS} status --short", shell=True,
                           capture_output=True, text=True)
    if probe.returncode != 0:
        c.bad(11, "git status failed - run provenance cannot be recorded")
    else:
        n = len(probe.stdout.strip().splitlines())
        c.ok(11, "clean" if n == 0 else f"{n} modified file(s) - recorded with the run")


# ------------------------------------------------------ 12-13  process hygiene
def check_processes(c):
    ps = sh("ps -eo pid,args")
    lines = ps.splitlines()

    def match(pat):
        return [l for l in lines if pat in l and "grep" not in l]

    dupes = []
    for label, pat in (("gz sim server", "gz sim server"),
                       ("rgbd_odometry", "rtabmap_odom/rgbd_odometry"),
                       ("rtabmap", "rtabmap_slam/rtabmap"),
                       ("gait_node", "hexapod_gait/gait_node"),
                       ("parameter_bridge", "ros_gz_bridge/parameter_bridge"),
                       ("drive_facility_loop", "drive_facility_loop")):
        n = len(match(pat))
        if n > 1:
            dupes.append(f"{label} x{n}")
    c.bad(12, ", ".join(dupes)) if dupes else c.ok(12, "at most one of each")

    recorders = match("bag record")
    screencast = [l for l in lines
                  if ("Screencast" in l or "ffmpeg" in l or "/obs " in l)
                  and "grep" not in l]
    if recorders:
        c.bad(13, f"{len(recorders)} rosbag recorder(s) already running")
    elif screencast:
        c.bad(13, f"{len(screencast)} screen recorder process(es) running")
    else:
        c.ok(13, "no rosbag recorder, no screencast")


# -------------------------------------------------------- 14-20, 22  live ROS
def check_live(c, settle=6.0):
    """Actually look at the running system. Never called in offline mode."""
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image, JointState
        from tf2_msgs.msg import TFMessage
    except Exception as exc:
        for n in LIVE_CHECKS:
            c.bad(n, f"ROS client library unavailable: {exc}")
        return

    try:
        rclpy.init()
    except Exception as exc:
        for n in LIVE_CHECKS:
            c.bad(n, f"rclpy.init failed: {exc}")
        return

    node = Node("stage3_preflight")
    seen = {k: [] for k in ("clock", "rgb", "depth", "info", "odom", "gt", "joints")}
    tf_pubs = {}

    def on_tf(msg):
        for t in msg.transforms:
            k = f"{t.header.frame_id}->{t.child_frame_id}"
            tf_pubs[k] = tf_pubs.get(k, 0) + 1

    q = 10
    node.create_subscription(Clock, "/clock", lambda m: seen["clock"].append(m), q)
    node.create_subscription(Image, "/face_camera/image", lambda m: seen["rgb"].append(m), q)
    node.create_subscription(Image, "/face_camera/depth_image",
                             lambda m: seen["depth"].append(m), q)
    node.create_subscription(CameraInfo, "/face_camera/camera_info",
                             lambda m: seen["info"].append(m), q)
    node.create_subscription(Odometry, "/odom", lambda m: seen["odom"].append(m), q)
    node.create_subscription(Odometry, "/odom_ground_truth", lambda m: seen["gt"].append(m), q)
    node.create_subscription(JointState, "/joint_states",
                             lambda m: seen["joints"].append(m), q)
    node.create_subscription(TFMessage, "/tf", on_tf, 50)

    t0 = time.time()
    while time.time() - t0 < settle:
        rclpy.spin_once(node, timeout_sec=0.05)

    for n, key, what in ((14, "clock", "/clock"), (15, "rgb", "/face_camera/image"),
                         (16, "depth", "/face_camera/depth_image"),
                         (17, "info", "/face_camera/camera_info"),
                         (18, "odom", "/odom"),
                         (19, "gt", "/odom_ground_truth")):
        c.ok(n, f"{len(seen[key])} msgs in {settle:.0f} s") if seen[key] \
            else c.bad(n, f"nothing on {what}")

    # The camera is mounted on the head, so a moving head changes the camera
    # pose and breaks a controlled variable.
    head_var = None
    if seen["joints"]:
        names = seen["joints"][0].name
        idx = [i for i, nm in enumerate(names)
               if "face" in nm or "head" in nm or "neck" in nm]
        if idx:
            import statistics
            series = [[m.position[i] for m in seen["joints"] if i < len(m.position)]
                      for i in idx]
            head_var = max((statistics.pvariance(s) if len(s) > 1 else 0.0) for s in series)
    if not seen["joints"]:
        c.bad(20, "no /joint_states - cannot confirm the head is still")
    elif head_var is None:
        c.bad(20, "no head joints found in /joint_states")
    elif head_var > 1e-6:
        c.bad(20, f"variance {head_var:.2e} rad^2 - the head is moving")
    else:
        c.ok(20, f"variance {head_var:.2e} rad^2")

    want_edges = ("map->odom", "odom->base_footprint")
    have = {k: v for k, v in tf_pubs.items() if k in want_edges}
    c.set(22, PASS if have else FAIL,
          ", ".join(f"{k} x{v}" for k, v in have.items())
          or "neither map->odom nor odom->base_footprint seen")

    node.destroy_node()
    rclpy.shutdown()


def skip_live(c):
    for n in LIVE_CHECKS:
        c.skip(n, "offline mode - the running system was not inspected")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--offline", action="store_true",
                    help="static checks only; the ROS-dependent checks are SKIPPED, "
                         "never passed")
    ap.add_argument("--settle", type=float, default=6.0)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    c = Checks()
    check_reference(c)
    check_configs(c)
    check_provenance(c)
    check_processes(c)
    skip_live(c) if args.offline else check_live(c, args.settle)

    for n in c.missing():                       # never silently omit a check
        c.bad(n, "check did not run")

    mode = "OFFLINE (static checks only)" if args.offline else "LIVE"
    width = max(len(v) for v in NAMES.values())
    print(f"\nStage 3 preflight - mode: {mode}")
    print(f"{N_MANDATORY} mandatory checks, {N_ADVISORY} advisory\n")
    print(f"{'':4s} {'check':{width}s}  {'':6s} {'':10s} detail")
    for n, _, kind in CHECKS:
        r = c.rows[n]
        mark = {PASS: " ok ", FAIL: "FAIL", SKIP: "skip"}[r["status"]]
        tag = "" if kind == MANDATORY else "(advisory)"
        print(f"{n:3d}  {r['name']:{width}s}  [{mark}] {tag:10s} {r['detail']}")

    failures, skipped = c.failures, c.skipped_mandatory
    adv_fail = [r for r in c.rows.values()
                if r["status"] == FAIL and r["kind"] == ADVISORY]
    print()
    if failures:
        print(f"PREFLIGHT FAILED: {len(failures)} of {N_MANDATORY} mandatory checks failed. "
              "DO NOT START THE EXPERIMENT.")
        for r in failures:
            print(f"  - {r['n']:2d} {r['name']}: {r['detail']}")
        code = 1
    elif skipped:
        # Offline. Say plainly that no sensor was looked at, so this can never
        # be mistaken for clearance.
        print(f"OFFLINE PRE-CHECK PASSED: {N_MANDATORY - len(skipped)} of {N_MANDATORY} "
              f"mandatory checks passed; {len(skipped)} live checks were SKIPPED "
              f"({', '.join(str(r['n']) for r in skipped)}).")
        print("NOT CLEARED FOR LAUNCH - no sensor, topic or TF state was inspected. "
              "Re-run without --offline once the stack is up.")
        code = 2
    else:
        print(f"PREFLIGHT PASSED: all {N_MANDATORY} mandatory checks passed - "
              "cleared for launch.")
        code = 0
    if adv_fail:
        print(f"  advisory not met: "
              + "; ".join(f"{r['n']} {r['name']} ({r['detail']})" for r in adv_fail))

    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)) or ".", exist_ok=True)
        with open(args.json, "w") as f:
            json.dump(dict(mode="offline" if args.offline else "live",
                           mandatory=N_MANDATORY, advisory=N_ADVISORY,
                           cleared_for_launch=(code == 0),
                           checks=[c.rows[n] for n, _, _ in CHECKS]), f, indent=2)
        print(f"written: {args.json}")
    return code


if __name__ == "__main__":
    sys.exit(main())
