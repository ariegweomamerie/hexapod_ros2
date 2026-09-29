#!/usr/bin/env python3
"""Stage 1 verification: stable basic gait (simulation).

Drives the running hexapod through a fixed sequence of /cmd_vel commands and
checks, against Gazebo ground truth, that it:
  * stands correctly (height, level, joint angles, all six feet on the ground)
  * walks forward and backward, turns left and right, strafes, and arcs
  * uses a proper tripod gait (never more than one tripod in the air) and walks
    continuously while commands keep coming
  * starts and stops without dragging planted feet, and returns to its stand pose
  * stops by itself when /cmd_vel goes silent (command-loss watchdog)
  * ignores tiny commands (deadband) and survives oversized ones
  * logs no gait warnings/errors and keeps a steady 50 Hz leg command stream

The measured values and pass/fail results are written to results.json. With
--bag, the run is also recorded with ros2 bag (MCAP) for replay/analysis:
    verification/runs/stage1_basic_gait_<date_time>/{results.json, bag/}

Needs, in other terminals:
    ./run_gazebo.sh
    ros2 launch hexapod_gait gait.launch.py

Usage (workspace sourced):
    python3 verification/stage1_basic_gait.py          # full run (~2.5 min)
    python3 verification/stage1_basic_gait.py --bag    # also record a ros2 bag

Exit code 0 = all required checks passed.

ROS interface used (all pre-existing, nothing new is added):
    publishes   /cmd_vel                              the command under test
    subscribes  /joint_states /foot_contacts /imu
                /leg_controller/joint_trajectory /rosout
Ground truth comes from Gazebo (/world/empty/pose/info over gz transport).

Timing: every duration, rate and time window is measured in SIMULATION time
(Gazebo /clock and message stamps), the same clock the gait runs on, so results
do not depend on how fast Gazebo runs on this machine. Gazebo's real-time factor
is recorded and reported for reference.
"""
import argparse
import datetime
import json
import math
import os
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int8MultiArray
from trajectory_msgs.msg import JointTrajectory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node as GzNode

from hexapod_gait.gait_node import STAND_Q
from hexapod_gait.kinematics import LEGS

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = "empty"
ROBOT = "Hexapod_Robot"
GAIT_NODE = "hexapod_gait"
BODY_CENTRE = (0.115, 0.115)        # body centre in base_footprint (m)
FOOT_RADIUS = 0.013                 # foot pad sphere radius (m)
ON_GROUND = FOOT_RADIUS + 0.006     # foot centre below this height = on the ground
TRIPODS = ({"leg_l1", "leg_l3", "leg_r2"}, {"leg_r1", "leg_r3", "leg_l2"})
STAND_HEIGHT = 0.143                # design body height (m)
MAX_SLIDE = 0.005                   # a planted foot sliding farther than this is "dragged"
START_WINDOW = 0.3                  # s after a walk command in which starting is judged
CMD_TIMEOUT = 0.5                   # s, expected /cmd_vel watchdog timeout

BAG_TOPICS = [
    "/robot_description", "/cmd_vel", "/joint_states", "/foot_contacts", "/imu",
    "/odom", "/tf", "/tf_static", "/clock", "/leg_controller/joint_trajectory",
    "/leg_controller/controller_state", "/rosout",
]


# --------------------------------------------------------------------------- data
def quat_rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def quat_matrix(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def stamp_s(stamp):
    return stamp.sec + stamp.nsec * 1e-9 if hasattr(stamp, "nsec") else stamp.sec + stamp.nanosec * 1e-9


class GroundTruth:
    """Latest robot pose and world foot heights from Gazebo, stamped in sim time,
    plus Gazebo's real-time factor."""

    def __init__(self):
        self.latest = None
        self.sim_t = 0.0            # simulation clock for the whole test (from Gazebo)
        self.rtf = []               # (sim t, real-time factor)
        self._gz = GzNode()
        self._gz.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self._cb)
        self._gz.subscribe(WorldStatistics, f"/world/{WORLD}/stats", self._on_stats)

    def _on_stats(self, msg):
        self.sim_t = stamp_s(msg.sim_time)
        self.rtf.append((self.sim_t, msg.real_time_factor))

    def _cb(self, msg):
        model, feet = None, {}
        for p in msg.pose:
            if p.name == ROBOT:
                model = p
            elif p.name.endswith("_foot") and p.name[:-5] in LEGS:
                feet[p.name[:-5]] = p
        if model is None or len(feet) != len(LEGS):
            return
        rot = quat_matrix(model.orientation)
        origin = np.array([model.position.x, model.position.y, model.position.z])
        roll, pitch, yaw = quat_rpy(model.orientation)
        # link poses are relative to the model frame -> transform to world
        feet_w = {leg: origin + rot @ np.array([f.position.x, f.position.y, f.position.z])
                  for leg, f in feet.items()}
        self.sim_t = stamp_s(msg.header.stamp)
        self.latest = dict(t=self.sim_t, x=model.position.x, y=model.position.y,
                           z=model.position.z, roll=roll, pitch=pitch, yaw=yaw, feet=feet_w,
                           foot_z={leg: float(p[2]) for leg, p in feet_w.items()})


class Probe(Node):
    """Publishes /cmd_vel and records what the robot reports."""

    def __init__(self, gt):
        # Simulation time comes from Gazebo (`gt`), not from a /clock subscription:
        # /clock ticks at 1 kHz and processing it in Python would load the machine
        # enough to slow Gazebo down during the test.
        super().__init__("stage1_gait_verification")
        self._gt = gt
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.last_send = None       # sim time of the last /cmd_vel sent
        self.joint_err = []         # (t, max |q - STAND_Q| rad over all leg joints)
        self.contacts = []          # (t, tuple)
        self.imu_rp = []            # (t, roll, pitch)
        self.traj_t = []            # t of each leg trajectory message
        self.gait_logs = []         # (t, level, text) WARN+ from the gait node
        self.create_subscription(JointState, "/joint_states", self._on_js, qos_profile_sensor_data)
        self.create_subscription(Int8MultiArray, "/foot_contacts",
                                 lambda m: self.contacts.append((self.now(), tuple(m.data))), 10)
        self.create_subscription(Imu, "/imu", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(JointTrajectory, "/leg_controller/joint_trajectory",
                                 lambda m: self.traj_t.append(self.now()), 10)
        self.create_subscription(Log, "/rosout", self._on_log, 100)

    def _on_js(self, m):
        pos = dict(zip(m.name, m.position))
        errs = [abs(pos[f"{leg}_{j}"] - STAND_Q[leg][i])
                for leg in LEGS for i, j in enumerate(("coxa", "femur", "tibia"))
                if f"{leg}_{j}" in pos]
        if len(errs) == 3 * len(LEGS):
            self.joint_err.append((stamp_s(m.header.stamp), max(errs)))

    def _on_imu(self, m):
        roll, pitch, _ = quat_rpy(m.orientation)
        self.imu_rp.append((stamp_s(m.header.stamp), roll, pitch))

    def _on_log(self, m):
        if m.name == GAIT_NODE and m.level >= Log.WARN:
            self.gait_logs.append((self.now(), m.level, m.msg))

    def now(self):
        """Simulation time (s), from Gazebo's pose/stats stream."""
        return self._gt.sim_t

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(t)
        self.last_send = self.now()


# --------------------------------------------------------------------------- analysis
def body_centre(s):
    c, sn = math.cos(s["yaw"]), math.sin(s["yaw"])
    return np.array([s["x"] + c * BODY_CENTRE[0] - sn * BODY_CENTRE[1],
                     s["y"] + sn * BODY_CENTRE[0] + c * BODY_CENTRE[1]])


def yaw_change(samples):
    total = 0.0
    for a, b in zip(samples, samples[1:]):
        total += math.atan2(math.sin(b["yaw"] - a["yaw"]), math.cos(b["yaw"] - a["yaw"]))
    return total


def motion_metrics(samples):
    """Body motion expressed in the robot's heading at the start of the window."""
    s0, s1 = samples[0], samples[-1]
    d = body_centre(s1) - body_centre(s0)
    fwd = np.array([math.sin(s0["yaw"]), -math.cos(s0["yaw"])])     # robot forward = -Y
    left = np.array([math.cos(s0["yaw"]), math.sin(s0["yaw"])])     # robot left = +X
    duration = s1["t"] - s0["t"]
    lifted_sets = [{leg for leg, z in s["foot_z"].items() if z > ON_GROUND} for s in samples]
    return dict(
        duration=duration,
        forward=float(d @ fwd), left=float(d @ left), distance=float(np.linalg.norm(d)),
        yaw_deg=math.degrees(yaw_change(samples)),
        tilt_max_deg=math.degrees(max(max(abs(s["roll"]), abs(s["pitch"])) for s in samples)),
        z_min=min(s["z"] for s in samples), z_max=max(s["z"] for s in samples),
        z_mean=float(np.mean([s["z"] for s in samples])),
        max_lifted=max(len(ls) for ls in lifted_sets),
        tripod_ok=float(np.mean([any(ls <= tri for tri in TRIPODS) for ls in lifted_sets])),
        all_down=float(np.mean([not ls for ls in lifted_sets])),
        foot_z_max_mm=1000 * max(max(s["foot_z"].values()) for s in samples),
    )


def planted_slide(samples):
    """Worst horizontal slide of a foot while it stays on the ground: for every
    continuous ground contact, the farthest the foot moves from where that
    contact began. Returns (metres, leg)."""
    worst, worst_leg = 0.0, None
    for leg in LEGS:
        anchor = None
        for s in samples:
            if s["foot_z"][leg] > ON_GROUND:
                anchor = None               # lifted: next contact starts a new anchor
                continue
            if anchor is None:
                anchor = s["feet"][leg][:2]
            slide = float(np.linalg.norm(s["feet"][leg][:2] - anchor))
            if slide > worst:
                worst, worst_leg = slide, leg
    return worst, worst_leg


def in_window(series, t0, t1):
    """Samples in [t0, t1). Half-open: the simulation clock has a finite tick, so an
    inclusive end would also pick up the first sample of the next phase."""
    return [v for v in series if t0 <= v[0] < t1]


# --------------------------------------------------------------------------- runner
class Verifier:
    def __init__(self, probe, gt):
        self.probe, self.gt = probe, gt
        self.phases = []
        self.checks = []

    def spin_for(self, duration, cmd=(0.0, 0.0, 0.0), publish=True):
        """Run one phase for `duration` s of SIM time: stream `cmd` at 20 Hz (sim
        time) if publish, and collect ground-truth samples."""
        samples, t0 = [], self.probe.now()
        next_pub, last = t0, None
        while self.probe.now() - t0 < duration:
            if publish and self.probe.now() >= next_pub:
                self.probe.send(*cmd)
                next_pub += 0.05
            rclpy.spin_once(self.probe, timeout_sec=0.005)
            s = self.gt.latest
            if s is not None and s is not last:
                samples.append(s)
                last = s
        return samples, t0, self.probe.now()

    def phase(self, name, duration, cmd=(0.0, 0.0, 0.0), publish=True):
        print(f"  {name:<24} cmd vx={cmd[0]:+.3f} vy={cmd[1]:+.3f} wz={cmd[2]:+.3f}"
              f"{'' if publish else '  (publisher silent)'}  {duration:.2f} s (sim)", flush=True)
        samples, t0, t1 = self.spin_for(duration, cmd, publish)
        ph = dict(name=name, cmd=list(cmd), publish=publish, t0=t0, t1=t1,
                  metrics=motion_metrics(samples), samples=samples)
        ph["traj_hz"] = len(in_window([(t,) for t in self.probe.traj_t], t0, t1)) / (t1 - t0)
        rtf = [r for t, r in in_window(self.gt.rtf, t0, t1)]
        ph["rtf_mean"] = float(np.mean(rtf)) if rtf else float("nan")
        ph["rtf_min"] = float(min(rtf)) if rtf else float("nan")
        self.phases.append(ph)
        return ph

    def check(self, group, name, passed, measured, criterion, required=True):
        self.checks.append(dict(group=group, name=name, passed=bool(passed),
                                measured=measured, criterion=criterion, required=required))

    # ---- check groups ----
    def check_stand(self, group, ph):
        m, p = ph["metrics"], self.probe
        self.check(group, "Body height", abs(m["z_mean"] - STAND_HEIGHT) <= 0.010,
                   f"{m['z_mean'] * 1000:.1f} mm", f"{STAND_HEIGHT * 1000:.0f} ± 10 mm")
        self.check(group, "Body level (ground truth)", m["tilt_max_deg"] <= 2.0,
                   f"max tilt {m['tilt_max_deg']:.2f}°", "≤ 2°")
        imu = in_window(p.imu_rp, ph["t0"], ph["t1"])
        imu_tilt = math.degrees(max(max(abs(r), abs(q)) for _, r, q in imu)) if imu else float("nan")
        self.check(group, "Body level (IMU)", imu_tilt <= 2.0,
                   f"max tilt {imu_tilt:.2f}°", "≤ 2°")
        je = in_window(p.joint_err, ph["t1"] - 1.0, ph["t1"])
        worst = math.degrees(max(e for _, e in je)) if je else float("nan")
        self.check(group, "Joints at stand pose", worst <= 1.0,
                   f"max error {worst:.2f}°", "≤ 1°")
        self.check(group, "All six feet on the ground", m["all_down"] == 1.0,
                   f"highest foot {m['foot_z_max_mm']:.1f} mm", f"all < {ON_GROUND * 1000:.0f} mm")
        fc = in_window(p.contacts, ph["t0"], ph["t1"])
        odd = [(round(t - ph["t0"], 2), c) for t, c in fc if not all(c)]
        self.check(group, "/foot_contacts all down", fc and not odd,
                   f"{sum(all(c) for _, c in fc)}/{len(fc)} msgs all 1"
                   + (f"; first odd at +{odd[0][0]} s {odd[0][1]}" if odd else ""),
                   "all msgs all 1")
        self.check(group, "Holds still", m["distance"] <= 0.005 and abs(m["yaw_deg"]) <= 0.5,
                   f"drift {m['distance'] * 1000:.1f} mm, {m['yaw_deg']:+.2f}°",
                   "≤ 5 mm, ≤ 0.5°")

    def check_walking(self, group, ph):
        m = ph["metrics"]
        self.check(group, "Body stable while walking",
                   m["tilt_max_deg"] <= 5.0 and m["z_min"] >= STAND_HEIGHT - 0.020
                   and m["z_max"] <= STAND_HEIGHT + 0.020,
                   f"max tilt {m['tilt_max_deg']:.1f}°, height {m['z_min'] * 1000:.0f}–"
                   f"{m['z_max'] * 1000:.0f} mm", "tilt ≤ 5°, height 143 ± 20 mm")
        self.check(group, "Tripod gait (feet in the air)",
                   m["tripod_ok"] == 1.0 and m["max_lifted"] <= 3,
                   f"{m['tripod_ok'] * 100:.0f}% samples OK, max {m['max_lifted']} lifted",
                   "only one tripod lifted, 100%")
        self.check(group, "Leg command stream rate", 45.0 <= ph["traj_hz"] <= 55.0,
                   f"{ph['traj_hz']:.1f} Hz (sim time)", "50 ± 5 Hz")
        fc = in_window(self.probe.contacts, ph["t0"] + 1.0, ph["t1"])
        pauses = sum(all(c) for _, c in fc)
        self.check(group, "Walks continuously (no false stops)", fc and pauses == 0,
                   f"{pauses}/{len(fc)} contact msgs with all feet down", "0 after the first 1 s")
        start = [s for s in ph["samples"] if s["t"] <= ph["t0"] + START_WINDOW]
        slide, leg = planted_slide(start)
        self.check(group, "No foot dragging when starting", slide <= MAX_SLIDE,
                   f"worst planted-foot slide {slide * 1000:.1f} mm{f' ({leg})' if leg else ''}",
                   f"≤ {MAX_SLIDE * 1000:.0f} mm in the first {START_WINDOW:.1f} s")

    def check_stop(self, ph, t_last_cmd=None):
        """Stop transition after the last command at t_last_cmd (default: phase start)."""
        t_ref = ph["t0"] if t_last_cmd is None else t_last_cmd
        je = in_window(self.probe.joint_err, t_ref, ph["t1"])
        settle = next((t - t_ref for t, e in je if math.degrees(e) <= 1.0), float("inf"))
        tail = [s for s in ph["samples"] if s["t"] >= ph["t1"] - 1.0]
        tm = motion_metrics(tail)
        slide, leg = planted_slide(ph["samples"])
        return dict(settle=settle, rest_mm=tm["distance"] * 1000, rest_yaw=abs(tm["yaw_deg"]),
                    feet_down=tm["all_down"] == 1.0, slide_mm=slide * 1000, slide_leg=leg,
                    tripod_ok=ph["metrics"]["tripod_ok"] == 1.0)


def start_bag(run_dir):
    bag_dir = os.path.join(run_dir, "bag")
    proc = subprocess.Popen(
        ["ros2", "bag", "record", "-o", bag_dir, "--storage", "mcap",
         "--storage-preset-profile", "zstd_fast", *BAG_TOPICS],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    return proc, bag_dir


def stop_bag(proc):
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)


def main():
    ap = argparse.ArgumentParser(description="Stage 1 verification: stable basic gait")
    ap.add_argument("--bag", action="store_true", help="also record the run with ros2 bag")
    args = ap.parse_args()

    run_dir = os.path.join(WS, "verification", "runs",
                           "stage1_basic_gait_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir)

    rclpy.init()
    gt = GroundTruth()
    probe = Probe(gt)
    v = Verifier(probe, gt)

    # ---- preconditions ----
    print("Preconditions")
    t0 = time.time()
    while time.time() - t0 < 20 and not (probe.cmd_pub.get_subscription_count() and probe.joint_err
                                          and probe.contacts and gt.latest and probe.now() > 0):
        rclpy.spin_once(probe, timeout_sec=0.1)
    ready = dict(sim_clock=probe.now() > 0,
                 cmd_vel_subscriber=probe.cmd_pub.get_subscription_count() > 0,
                 joint_states=bool(probe.joint_err), foot_contacts=bool(probe.contacts),
                 gazebo_pose=gt.latest is not None)
    for k, ok in ready.items():
        print(f"  {'ok ' if ok else 'MISSING'} {k}")
    if not all(ready.values()):
        raise SystemExit("Preconditions not met: start Gazebo and the gait first.")

    bag = None
    if args.bag:
        bag, bag_dir = start_bag(run_dir)
        time.sleep(3.0)                 # let the recorder discover the topics
        print(f"Recording ros2 bag -> {os.path.relpath(bag_dir, WS)}")

    print("Sequence")
    moves = {}
    stops = []
    v.phase("settle", 2.0)
    stand0 = v.phase("stand (initial)", 8.0)
    # Durations differ by fractions of a gait cycle (1 s) so the stops sample
    # different gait phases instead of always stopping at the same point.
    plan = [
        ("forward",      10.0, (0.10, 0.0, 0.0)),
        ("backward",     10.25, (-0.10, 0.0, 0.0)),
        ("turn left",     9.5, (0.0, 0.0, 0.4)),
        ("turn right",    9.75, (0.0, 0.0, -0.4)),
        ("strafe left",   8.1, (0.0, 0.10, 0.0)),
        ("strafe right",  8.4, (0.0, -0.10, 0.0)),
        ("arc forward-left", 8.6, (0.08, 0.0, 0.3)),
    ]
    for name, dur, cmd in plan:
        moves[name] = v.phase(name, dur, cmd)
        stops.append((name, v.phase(f"stop after {name}", 3.0)))
    deadband = v.phase("deadband (tiny cmd)", 4.0, (0.005, 0.0, 0.0))
    saturate = v.phase("oversized cmd", 5.35, (1.0, 0.0, 1.0))
    stops.append(("oversized cmd", v.phase("stop after oversized cmd", 3.0)))
    v.phase("cmd then publisher stops", 2.15, (0.10, 0.0, 0.0))
    t_last_cmd = probe.last_send
    silent = v.phase("publisher silent", 3.0, (0.10, 0.0, 0.0), publish=False)
    v.phase("explicit stop", 3.0)
    stand1 = v.phase("stand (final)", 5.0)

    if bag:
        stop_bag(bag)

    # ---- checks ----
    v.check_stand("Stand (initial)", stand0)

    fw, bw = moves["forward"]["metrics"], moves["backward"]["metrics"]
    for label, m, sign in (("Walk forward", fw, 1), ("Walk backward", bw, -1)):
        v.check(label, "Moves in the commanded direction", sign * m["forward"] >= 0.10,
                f"{m['forward'] * 100:+.1f} cm ({m['forward'] / m['duration'] * 100:+.2f} cm/s)",
                f"{'≥ +10' if sign > 0 else '≤ −10'} cm in {m['duration']:.1f} s")
        v.check(label, "Walks straight", abs(m["left"]) <= 0.15 * abs(m["forward"]),
                f"sideways {m['left'] * 100:+.1f} cm", "≤ 15% of forward distance")
        v.check(label, "Holds heading", abs(m["yaw_deg"]) <= 5.0,
                f"{m['yaw_deg']:+.1f}°", "≤ 5°")
        v.check_walking(label, moves["forward" if sign > 0 else "backward"])

    tl, tr = moves["turn left"]["metrics"], moves["turn right"]["metrics"]
    for label, m, sign in (("Turn left", tl, 1), ("Turn right", tr, -1)):
        v.check(label, "Turns in the commanded direction", sign * m["yaw_deg"] >= 10.0,
                f"{m['yaw_deg']:+.1f}° ({m['yaw_deg'] / m['duration']:+.2f}°/s)",
                f"{'≥ +10°' if sign > 0 else '≤ −10°'} in {m['duration']:.1f} s")
        v.check(label, "Turns in place", m["distance"] <= 0.05,
                f"body centre moved {m['distance'] * 100:.1f} cm", "≤ 5 cm")
        v.check_walking(label, moves["turn left" if sign > 0 else "turn right"])

    sl, sr = moves["strafe left"]["metrics"], moves["strafe right"]["metrics"]
    for label, m, sign in (("Strafe left", sl, 1), ("Strafe right", sr, -1)):
        v.check(label, "Moves in the commanded direction", sign * m["left"] >= 0.08,
                f"{m['left'] * 100:+.1f} cm sideways",
                f"{'≥ +8' if sign > 0 else '≤ −8'} cm in {m['duration']:.1f} s")
        v.check(label, "Moves sideways only",
                abs(m["forward"]) <= 0.25 * abs(m["left"]) and abs(m["yaw_deg"]) <= 5.0,
                f"forward {m['forward'] * 100:+.1f} cm, yaw {m['yaw_deg']:+.1f}°",
                "forward ≤ 25% of sideways, yaw ≤ 5°")
        v.check_walking(label, moves["strafe left" if sign > 0 else "strafe right"])

    arc = moves["arc forward-left"]["metrics"]
    v.check("Arc (forward + turn)", "Moves forward while turning left",
            arc["forward"] >= 0.05 and arc["yaw_deg"] >= 5.0,
            f"forward {arc['forward'] * 100:+.1f} cm, yaw {arc['yaw_deg']:+.1f}°",
            "forward ≥ 5 cm and yaw ≥ +5°")
    v.check_walking("Arc (forward + turn)", moves["arc forward-left"])

    stop_results = [(name, v.check_stop(ph)) for name, ph in stops]
    worst_settle = max(r["settle"] for _, r in stop_results)
    worst_rest = max(r["rest_mm"] for _, r in stop_results)
    worst_rest_yaw = max(r["rest_yaw"] for _, r in stop_results)
    v.check("Stopping", f"Returns to stand pose ({len(stop_results)} stops)", worst_settle <= 1.0,
            f"slowest {worst_settle:.2f} s", "joints within 1° in ≤ 1 s")
    v.check("Stopping", "Comes to rest", worst_rest <= 5.0 and worst_rest_yaw <= 0.5,
            f"worst drift {worst_rest:.1f} mm, {worst_rest_yaw:.2f}° (last 2 s)", "≤ 5 mm, ≤ 0.5°")
    v.check("Stopping", "All feet down after stopping", all(r["feet_down"] for _, r in stop_results),
            f"{sum(r['feet_down'] for _, r in stop_results)}/{len(stop_results)} stops", "all")
    worst_slide = max(stop_results, key=lambda nr: nr[1]["slide_mm"])
    v.check("Stopping", "No foot dragging while stopping", worst_slide[1]["slide_mm"] <= MAX_SLIDE * 1000,
            f"worst planted-foot slide {worst_slide[1]['slide_mm']:.1f} mm "
            f"({worst_slide[1]['slide_leg']}, {worst_slide[0]})", f"≤ {MAX_SLIDE * 1000:.0f} mm")
    v.check("Stopping", "One tripod lifted at a time", all(r["tripod_ok"] for _, r in stop_results),
            f"{sum(r['tripod_ok'] for _, r in stop_results)}/{len(stop_results)} stops", "all")

    # Command-loss watchdog: the last /cmd_vel (non-zero) was sent at t_last_cmd.
    wd = v.check_stop(silent, t_last_cmd=t_last_cmd)
    v.check("Command-loss watchdog", "Stops and stands when /cmd_vel stops",
            wd["settle"] <= CMD_TIMEOUT + 1.0,
            f"standing {wd['settle']:.2f} s after the last message",
            f"≤ {CMD_TIMEOUT + 1.0:.1f} s ({CMD_TIMEOUT:.1f} s timeout + ≤ 1.0 s stop)")
    v.check("Command-loss watchdog", "At rest, feet down, no dragging",
            wd["feet_down"] and wd["rest_mm"] <= 5.0 and wd["slide_mm"] <= MAX_SLIDE * 1000,
            f"drift {wd['rest_mm']:.1f} mm (last 1 s), slide {wd['slide_mm']:.1f} mm",
            f"feet down, ≤ 5 mm, slide ≤ {MAX_SLIDE * 1000:.0f} mm")
    stop_results.append(("watchdog", wd))

    dm = deadband["metrics"]
    fc = in_window(probe.contacts, deadband["t0"], deadband["t1"])
    v.check("/cmd_vel handling", "Tiny command ignored (deadband)",
            all(all(c) for _, c in fc) and dm["distance"] <= 0.005,
            f"{sum(not all(c) for _, c in fc)} stepping msgs, drift {dm['distance'] * 1000:.1f} mm",
            "no steps, ≤ 5 mm")
    sm = saturate["metrics"]
    sat_logs = in_window(probe.gait_logs, saturate["t0"], saturate["t1"])
    v.check("/cmd_vel handling", "Oversized command handled safely",
            not sat_logs and sm["tilt_max_deg"] <= 8.0 and sm["tripod_ok"] == 1.0
            and (sm["distance"] >= 0.05 or abs(sm["yaw_deg"]) >= 5.0),
            f"{len(sat_logs)} gait warnings, tilt {sm['tilt_max_deg']:.1f}°, moved "
            f"{sm['distance'] * 100:.1f} cm / {sm['yaw_deg']:+.0f}°",
            "no warnings, tilt ≤ 8°, tripod OK, still moves")
    v.check("Overall", "No gait warnings or errors", not probe.gait_logs,
            f"{len(probe.gait_logs)} WARN/ERROR from {GAIT_NODE}", "0")
    v.check_stand("Stand (final)", stand1)
    dz = abs(stand1["metrics"]["z_mean"] - stand0["metrics"]["z_mean"])
    v.check("Overall", "Same stand after all manoeuvres", dz <= 0.003,
            f"height changed {dz * 1000:.1f} mm", "≤ 3 mm")

    # ---- report ----
    print()
    group = None
    for c in v.checks:
        if c["group"] != group:
            group = c["group"]
            print(f"{group}")
        mark = ("PASS" if c["passed"] else "FAIL") if c["required"] else ("info" if c["passed"] else "NOTE")
        print(f"  [{mark}] {c['name']:<38} {c['measured']:<52} ({c['criterion']})")
    required = [c for c in v.checks if c["required"]]
    failed = [c for c in required if not c["passed"]]
    rtf_all = [r for _, r in in_window(gt.rtf, v.phases[0]["t0"], v.phases[-1]["t1"])]
    rtf_mean, rtf_min = float(np.mean(rtf_all)), float(min(rtf_all))
    sim_span = v.phases[-1]["t1"] - v.phases[0]["t0"]
    print(f"\nTiming: {sim_span:.1f} s of simulation time; Gazebo real-time factor "
          f"mean {rtf_mean:.2f}, min {rtf_min:.2f}")
    print(f"RESULT: {len(required) - len(failed)}/{len(required)} required checks passed"
          f" -> Stage 1 {'PASSED' if not failed else 'FAILED'}")

    results = dict(
        stage="1 - stable basic gait", date=datetime.datetime.now().isoformat(timespec="seconds"),
        passed=not failed, required_passed=len(required) - len(failed), required_total=len(required),
        timing=dict(clock="simulation", sim_seconds=sim_span, rtf_mean=rtf_mean, rtf_min=rtf_min),
        checks=v.checks,
        phases=[{k: ph[k] for k in ("name", "cmd", "publish", "t0", "t1", "metrics", "traj_hz",
                                    "rtf_mean", "rtf_min")}
                for ph in v.phases],
        stops={name: r for name, r in stop_results},
        gait_logs=probe.gait_logs,
        bag=os.path.relpath(os.path.join(run_dir, "bag"), WS) if args.bag else None,
    )
    with open(os.path.join(run_dir, "results.json"), "w") as f:
        json.dump(results, f, indent=2, default=float)
    print(f"Results: {os.path.relpath(os.path.join(run_dir, 'results.json'), WS)}")

    probe.send()                         # leave the robot standing still
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if not failed else 1)


if __name__ == "__main__":
    main()
