#!/usr/bin/env python3
"""Tripod walking gait for the hexapod.

Subscribes /cmd_vel (geometry_msgs/Twist) and streams JointTrajectory commands
to /leg_controller so the robot walks. Uses an alternating tripod:

    Tripod A: leg_l1, leg_l3, leg_r2      (swing while B supports)
    Tripod B: leg_r1, leg_r3, leg_l2

Each foot follows a D-shaped cycle: a straight ground stroke (stance, propels
the body) and a raised arc (swing, returns the foot). Foot targets are turned
into joint angles by per-leg IK (see kinematics.py). When the command is ~0 the
robot holds its neutral stance.
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
# Starting/stopping therefore only moves the feet radially in or out — no height
# change and no twisting. Foot positions are in the base_footprint frame.

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

        self.cycle_time = self.get_parameter("cycle_time").value
        self.step_height = self.get_parameter("step_height").value
        self.max_step = self.get_parameter("max_step").value
        self.rate = self.get_parameter("update_rate").value
        self.deadband = self.get_parameter("deadband").value

        # body-center (mean of walk foot homes) for turning
        hs = np.array([WALK_HOME[l][:2] for l in LEGS])
        self.center = hs.mean(axis=0)

        self.cmd = np.zeros(3)          # vx, vy, wz
        self.phase = 0.0
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
        return np.array([hx + off[0], hy + off[1], hz + oz])

    def in_stance(self, leg, moving):
        """True if the foot is on the ground (stance), False if swinging.
        Idle -> all feet down; moving -> first half of each leg's cycle."""
        if not moving:
            return True
        pp = (self.phase + (0.5 if leg in TRIPOD_B else 0.0)) % 1.0
        return pp < 0.5

    def publish_contacts(self, moving):
        msg = Int8MultiArray()
        dim = MultiArrayDimension()
        dim.label = ",".join(LEGS)      # documents the order for consumers
        dim.size = len(LEGS)
        dim.stride = len(LEGS)
        msg.layout.dim = [dim]
        msg.data = [1 if self.in_stance(leg, moving) else 0 for leg in LEGS]
        self.contact_pub.publish(msg)

    def tick(self):
        if self.kin is None:
            return                      # kinematics not built yet (no URDF)
        moving = np.linalg.norm(self.cmd) > self.deadband
        if moving:
            self.phase = (self.phase + self.dt / self.cycle_time) % 1.0

        self.publish_contacts(moving)

        positions = []
        for leg in LEGS:
            target = self.foot_target(leg) if moving else np.array(STAND_HOME[leg])
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
