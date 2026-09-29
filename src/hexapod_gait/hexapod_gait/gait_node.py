#!/usr/bin/env python3
"""Tripod walking gait for the hexapod.

Subscribes /cmd_vel (geometry_msgs/Twist) and streams JointTrajectory commands
to /leg_controller so the robot walks. Uses an alternating tripod:

    Tripod A: leg_l1, leg_l3, leg_r2      (swing while B supports)
    Tripod B: leg_r1, leg_r3, leg_l2

Each foot follows a D-shaped cycle: a straight ground stroke (stance, propels
the body) and a raised arc (swing, returns the foot). Foot targets are turned
into joint angles by per-leg IK (see kinematics.py).

Gait states:  STANDING -> STARTING -> WALKING -> STOPPING -> STANDING
Standing and walking use different footprints. STARTING and STOPPING move the
feet between them one tripod at a time: a foot is only repositioned while it is
lifted and the other tripod stays planted, so no foot is dragged.

Command watchdog: if no /cmd_vel arrives for cmd_vel_timeout seconds, the
command is dropped and the robot stops through the normal STOPPING transition.

Timing: the control timer, gait phase, transitions and watchdog all run on the
node's ROS clock. In simulation the node uses use_sim_time (Gazebo's /clock), so
the gait stays locked to the physics even when Gazebo runs slower than real time.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from hexapod_gait.kinematics import HexapodKinematics, LEGS, JOINTS_PER_LEG

# Both poses share ONE mirror-symmetric radial footprint (symmetric left/right and
# front/back about the body centre x=0.115, y=0.115):
#   front legs splayed 35 deg forward, middle legs straight out, rear legs 35 deg back.
# The coxa joints are re-zeroed in the URDF to exactly these directions, so every
# coxa is 0 in both poses and keeps its full +-30 deg for striding.
#   * STAND — feet 150 mm from each hip, body 143 mm high. Used when idle.
#             Tripod stability margin 89 mm, joint-limit margin 16.6 deg.
#   * WALK  — feet 120 mm from each hip, same 143 mm height. The legs pull in to
#             give room for full 0.10 m strides (forward, sideways and turning).
# Starting/stopping only moves the feet radially in or out (no height change, no
# twisting), and only while they are lifted: one tripod steps at a time (STARTING /
# STOPPING states). Foot positions are in the base_footprint frame.

# WALK pose — gait offsets are added to these.
WALK_HOME = {
    "leg_l1": (0.3213, -0.0838, -0.1300),
    "leg_l2": (0.3510,  0.1150, -0.1300),
    "leg_l3": (0.3213,  0.3138, -0.1300),
    "leg_r1": (-0.0913, -0.0838, -0.1300),
    "leg_r2": (-0.1210,  0.1150, -0.1300),
    "leg_r3": (-0.0913,  0.3138, -0.1300),
}
# STAND pose.
STAND_HOME = {
    "leg_l1": (0.3459, -0.1010, -0.1300),
    "leg_l2": (0.3810,  0.1150, -0.1300),
    "leg_l3": (0.3459,  0.3310, -0.1300),
    "leg_r1": (-0.1159, -0.1010, -0.1300),
    "leg_r2": (-0.1510,  0.1150, -0.1300),
    "leg_r3": (-0.1159,  0.3310, -0.1300),
}
# STAND joint angles (coxa, femur, tibia). Also the IK warm-start seed.
# Femur/tibia differ slightly between legs because the CAD femur/tibia zeros are
# not identical; the resulting leg SHAPES are mirror-symmetric.
STAND_Q = {
    "leg_l1": (0.0, -0.5920, 0.2684),
    "leg_l2": (0.0, -0.5921, 0.2687),
    "leg_l3": (0.0, -0.4538, 0.2681),
    "leg_r1": (0.0, 0.5557, -0.4075),
    "leg_r2": (0.0, 0.5557, -0.4075),
    "leg_r3": (0.0, 0.5556, -0.4088),
}

# Alternating tripod grouping (B is a half-cycle out of phase with A).
TRIPOD_B = {"leg_r1", "leg_r3", "leg_l2"}
TRIPOD_A = set(LEGS) - TRIPOD_B

# Gait states (see module docstring).
STANDING, STARTING, WALKING, STOPPING = "standing", "starting", "walking", "stopping"

JOINT_ORDER = [f"{leg}_{j}" for leg in LEGS for j in JOINTS_PER_LEG]


class GaitNode(Node):
    def __init__(self):
        super().__init__("hexapod_gait")

        # ---- parameters ----
        self.declare_parameter("cycle_time", 1.0)      # s per full gait cycle (tuned)
        self.declare_parameter("step_height", 0.035)   # m foot lift in swing (femur limit caps ~0.035)
        self.declare_parameter("max_step", 0.10)       # m clamp on stride (tuned)
        self.declare_parameter("update_rate", 50.0)    # Hz
        self.declare_parameter("deadband", 0.01)       # min |cmd| to walk
        self.declare_parameter("cmd_vel_timeout", 0.5)  # s without /cmd_vel -> stop (watchdog)
        self.declare_parameter("transition_step_time", 0.3)  # s per tripod step when starting/stopping
        self.declare_parameter("transition_settle_time", 0.075)  # s pause after a step before the next lifts

        self.cycle_time = self.get_parameter("cycle_time").value
        self.step_height = self.get_parameter("step_height").value
        self.max_step = self.get_parameter("max_step").value
        self.rate = self.get_parameter("update_rate").value
        self.deadband = self.get_parameter("deadband").value
        self.cmd_timeout = self.get_parameter("cmd_vel_timeout").value
        self.transition_time = self.get_parameter("transition_step_time").value
        self.transition_settle = self.get_parameter("transition_settle_time").value

        # body-center (mean of walk foot homes) for turning
        hs = np.array([WALK_HOME[l][:2] for l in LEGS])
        self.center = hs.mean(axis=0)

        self.cmd = np.zeros(3)          # vx, vy, wz
        self.last_cmd_time = 0.0        # ROS-clock time (s) of the last /cmd_vel (watchdog)
        self.phase = 0.0
        self.state = STANDING
        self.feet = {l: np.array(STAND_HOME[l], dtype=float) for l in LEGS}  # last foot targets
        self.steps = []                 # pending transition steps: (legs, target(leg))
        self.step_from = None           # foot positions when the current step began
        self.step_t = 0.0
        self.settle_left = 0.0          # s left in the pause between two tripod steps
        self.swing_from = {}            # leg -> where its first swing after STARTING begins
        self.kin = None
        # warm-start every leg's IK from the idle stand pose (never from 0,0,0)
        self.q_prev = {l: np.array(STAND_Q[l], dtype=float) for l in LEGS}

        # ---- IO ----
        self.pub = self.create_publisher(JointTrajectory, "/leg_controller/joint_trajectory", 10)
        # per-leg foot contact state (1 = foot on ground / stance, 0 = swing),
        # derived from the gait phase. Order = LEGS. Matches the real robot,
        # which has no physical foot sensors.
        self.contact_pub = self.create_publisher(Int8MultiArray, "/foot_contacts", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        # robot_description is latched (transient_local)
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(String, "/robot_description", self.on_urdf, qos)

        # Create the timer HERE (in __init__), not inside on_urdf. A timer created
        # inside a subscription callback under rclpy.spin() is not reliably picked
        # up by the executor, so tick() never fired. tick() early-returns until the
        # kinematics are built from /robot_description.
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info("Waiting for /robot_description to build kinematics...")

    # ---- callbacks ----
    def on_urdf(self, msg):
        if self.kin is None:
            self.kin = HexapodKinematics(msg.data)
            self.get_logger().info("Kinematics ready. Gait running. Publish /cmd_vel to walk.")

    def on_cmd(self, msg):
        # Map standard ROS Twist convention -> this robot's body frame.
        # The robot faces -Y (the face), and its left is +X, so:
        #   linear.x (forward) -> -Y stride   |   linear.y (left) -> +X stride
        # This lets forward = linear.x, so teleop_twist_keyboard works normally.
        vx = msg.linear.y          # strafe left/right  -> body +/-X
        vy = -msg.linear.x         # forward/back        -> body -/+Y (forward = +linear.x)
        wz = msg.angular.z         # yaw (CCW positive)
        self.cmd = np.array([vx, vy, wz])
        self.last_cmd_time = self.now_s()

    def now_s(self):
        """Current ROS time in seconds (simulation time when use_sim_time is set)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def walk_requested(self):
        """True while a non-zero /cmd_vel is fresh. Watchdog: if no message has
        arrived for cmd_vel_timeout, the command is dropped and the robot stops."""
        if np.linalg.norm(self.cmd) <= self.deadband:
            return False
        age = self.now_s() - self.last_cmd_time
        if age > self.cmd_timeout or age < 0.0:     # stale, or the clock jumped back (sim reset)
            self.get_logger().info(
                f"No /cmd_vel for {self.cmd_timeout:.2f} s: stopping (command watchdog)")
            self.cmd = np.zeros(3)
            return False
        return True

    # ---- gait ----
    def step_vector(self, leg):
        """Per-leg stride vector (m) for the current command: translation +
        rotational tangential component, clamped to max_step."""
        vx, vy, wz = self.cmd
        rx, ry = np.array(WALK_HOME[leg][:2]) - self.center
        d = np.array([vx - wz * ry, vy + wz * rx]) * (self.cycle_time / 2.0)
        n = np.linalg.norm(d)
        if n > self.max_step:
            d *= self.max_step / n
        return d

    def foot_target(self, leg):
        p = (self.phase + (0.5 if leg in TRIPOD_B else 0.0)) % 1.0
        d = self.step_vector(leg)
        hx, hy, hz = WALK_HOME[leg]
        if p < 0.5:                     # stance: +0.5d -> -0.5d on the ground
            frac = p / 0.5
            off = d * (0.5 - frac)
            oz = 0.0
        else:                           # swing: -0.5d -> +0.5d, lifted in an arc
            frac = (p - 0.5) / 0.5
            off = d * (frac - 0.5)
            oz = self.step_height * math.sin(math.pi * frac)
            start = self.swing_from.get(leg)
            if start is not None:       # first swing after STARTING: leave from where the foot is
                lift_off = start[:2] - np.array([hx, hy])
                off = lift_off + (0.5 * d - lift_off) * frac
        return np.array([hx + off[0], hy + off[1], hz + oz])

    def in_stance(self, leg):
        """While walking: True in the first half of the leg's cycle (foot down)."""
        pp = (self.phase + (0.5 if leg in TRIPOD_B else 0.0)) % 1.0
        return pp < 0.5

    # ---- start/stop transitions ----
    def begin_transition(self, state, steps):
        """Queue tripod steps. Each step lifts its legs, carries them to target(leg)
        and sets them down while every other leg stays planted."""
        self.state = state
        self.steps = [(legs, target) for legs, target in steps if legs]
        self.step_from = None
        self.settle_left = 0.0

    def begin_start(self):
        """Tripod A steps to the start of its walking stroke while B stays planted;
        B then leaves the stand footprint directly on its first swing."""
        self.begin_transition(STARTING, [(TRIPOD_A, self.stroke_start)])

    def begin_walking(self):
        self.state = WALKING
        self.phase = 0.0                # A starts its stroke, B starts its swing
        self.swing_from = {l: self.feet[l].copy() for l in TRIPOD_B}

    def begin_stop(self):
        """Step every foot to the stand footprint: first the tripod that is
        already in the air, then the other one. Feet already there stay put."""
        first = self.lifted_legs() or TRIPOD_A
        second = set(LEGS) - first
        self.begin_transition(STOPPING, [
            ({l for l in first if not self.at_stand(l)}, self.stand_target),
            ({l for l in second if not self.at_stand(l)}, self.stand_target),
        ])

    def lifted_legs(self):
        """Legs currently in the air: the walking swing tripod or the active step."""
        if self.state == WALKING:
            return {l for l in LEGS if not self.in_stance(l)}
        if self.state in (STARTING, STOPPING) and self.steps and self.step_from is not None:
            return set(self.steps[0][0])
        return set()

    def stroke_start(self, leg):
        return np.array(WALK_HOME[leg]) + np.append(0.5 * self.step_vector(leg), 0.0)

    def stand_target(self, leg):
        return np.array(STAND_HOME[leg])

    def at_stand(self, leg):
        return np.linalg.norm(self.feet[leg] - self.stand_target(leg)) < 1e-3

    def transition_targets(self):
        """Foot targets for this tick of the current step -> (targets, lifted legs)."""
        if self.settle_left > 1e-9:     # the tripod that just landed takes the load first
            self.settle_left -= self.dt
            return dict(self.feet), set()
        legs, target = self.steps[0]
        if self.step_from is None:
            self.step_from = {l: self.feet[l].copy() for l in legs}
            self.step_t = 0.0
        self.step_t += self.dt
        u = min(1.0, self.step_t / self.transition_time)
        targets = dict(self.feet)       # every other leg stays planted
        for leg in legs:
            targets[leg] = self.step_point(self.step_from[leg], target(leg), u)
        if u < 1.0:
            return targets, set(legs)
        self.steps.pop(0)               # step done: feet are down
        self.step_from = None
        if self.steps:
            self.settle_left = self.transition_settle
        return targets, set()

    def step_point(self, p0, p1, u):
        """Point u (0..1) along a lifted step from p0 to p1.

        Horizontal motion eases in and out over the whole step (a compressed carry
        would need foot speeds these servos cannot follow), and the arc peaks near
        step_height even if the foot starts in the air. The sine arc is deliberate:
        it climbs and descends fastest where the horizontal motion is slowest, which
        keeps the foot's overall speed - and therefore the servo lag - low.
        """
        s = u * u * (3.0 - 2.0 * u)
        xy = p0[:2] + (p1[:2] - p0[:2]) * s
        lift = max(0.0, p1[2] + self.step_height - p0[2])
        z = p0[2] + (p1[2] - p0[2]) * u + lift * math.sin(math.pi * u)
        return np.array([xy[0], xy[1], z])

    def publish_contacts(self, lifted):
        msg = Int8MultiArray()
        dim = MultiArrayDimension()
        dim.label = ",".join(LEGS)      # documents the order for consumers
        dim.size = len(LEGS)
        dim.stride = len(LEGS)
        msg.layout.dim = [dim]
        msg.data = [0 if leg in lifted else 1 for leg in LEGS]
        self.contact_pub.publish(msg)

    def tick(self):
        if self.kin is None:
            return                      # kinematics not built yet (no URDF)
        walk = self.walk_requested()
        if self.state == STANDING and walk:
            self.begin_start()
        elif self.state in (STARTING, WALKING) and not walk:
            self.begin_stop()           # also cancels a start step: re-plan from where the feet are

        if self.state == WALKING:
            for leg in LEGS:
                if self.in_stance(leg):
                    self.swing_from.pop(leg, None)
            targets = {leg: self.foot_target(leg) for leg in LEGS}
            lifted = {leg for leg in LEGS if not self.in_stance(leg)}
            self.phase = (self.phase + self.dt / self.cycle_time) % 1.0
        elif self.state in (STARTING, STOPPING):
            targets, lifted = self.transition_targets() if self.steps else (dict(self.feet), set())
            if not self.steps:          # transition finished
                if self.state == STOPPING:
                    self.state = STANDING
                elif walk:
                    self.begin_walking()
                else:
                    self.begin_stop()
        else:                           # STANDING
            targets = {leg: np.array(STAND_HOME[leg]) for leg in LEGS}
            lifted = set()

        self.publish_contacts(lifted)

        positions = []
        for leg in LEGS:
            target = targets[leg]
            self.feet[leg] = target
            try:
                q, rc = self.kin.solve(leg, target, seed=self.q_prev[leg])
                if rc >= 0:
                    self.q_prev[leg] = q      # good solution: warm-start next tick
                else:
                    q = self.q_prev[leg]      # IK did not converge: hold last pose
            except Exception as exc:          # never let one bad solve kill the node
                self.get_logger().warn(
                    f"IK error on {leg} (target={np.round(target, 3)}): {exc}; holding pose",
                    throttle_duration_sec=2.0)
                q = self.q_prev[leg]
            positions.extend(float(v) for v in q)

        traj = JointTrajectory()
        traj.joint_names = JOINT_ORDER
        pt = JointTrajectoryPoint()
        pt.positions = positions
        # aim a few control periods ahead for smooth streaming
        t = 3.0 * self.dt
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        traj.points = [pt]
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
