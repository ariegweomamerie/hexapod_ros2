#!/usr/bin/env python3
"""Pan/tilt head command interface for the hexapod's face.

Gives the head a simple "look here" command instead of raw trajectories:

    ros2 topic pub --once /head/cmd std_msgs/msg/Float64MultiArray "{data: [0.25, -0.2]}"

ROS interface
  Subscribes  /head/cmd             std_msgs/Float64MultiArray  [pan, tilt] (rad)
              /joint_states         sensor_msgs/JointState      current head angles
              /robot_description    std_msgs/String (latched)   joint limits
  Publishes   /face_controller/joint_trajectory  trajectory_msgs/JointTrajectory

(0, 0) looks straight ahead; pan + = left, tilt + = up. Targets outside the
URDF limits are clamped. Each move is timed from the head's current angles so
it turns at a steady, limited speed; a new command smoothly replaces the
previous one mid-move.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hexapod_head.motion import HEAD_JOINTS, JointLimits, plan_move, read_joint_limits

# Commands are kept this far inside the URDF limits. Driving a loaded joint exactly
# onto its hard stop can lock it up in the physics engine (face_tilt did), and on a
# real servo it means stalling against the end stop.
SOFT_LIMIT_MARGIN = 0.02        # rad (~1.15 deg)


class HeadNode(Node):
    def __init__(self):
        super().__init__("hexapod_head")

        # ---- parameters ----
        self.declare_parameter("max_velocity", 1.0)   # rad/s, peak speed of either joint
        self.declare_parameter("min_duration", 0.2)   # s, shortest move
        self.max_velocity = self.get_parameter("max_velocity").value
        self.min_duration = self.get_parameter("min_duration").value

        self.limits = None               # (pan, tilt) JointLimits, from the URDF
        self.current = {}                # joint name -> measured position

        # ---- IO ----
        self.pub = self.create_publisher(JointTrajectory, "/face_controller/joint_trajectory", 10)
        self.create_subscription(Float64MultiArray, "/head/cmd", self.on_cmd, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states,
                                 qos_profile_sensor_data)
        # robot_description is latched (transient_local)
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(String, "/robot_description", self.on_urdf, qos)
        self.get_logger().info("Waiting for /robot_description to read head limits...")

    # ---- callbacks ----
    def on_urdf(self, msg):
        if self.limits is not None:
            return
        limits = read_joint_limits(msg.data, HEAD_JOINTS)
        missing = [j for j in HEAD_JOINTS if j not in limits]
        if missing:
            self.get_logger().error(f"URDF has no limits for {missing}; head disabled")
            return
        self.limits = tuple(JointLimits(limits[j].lower + SOFT_LIMIT_MARGIN,
                                        limits[j].upper - SOFT_LIMIT_MARGIN)
                            for j in HEAD_JOINTS)
        pan, tilt = self.limits
        self.get_logger().info(
            f"Head ready. pan {pan.lower:+.3f}..{pan.upper:+.3f} rad (left +), "
            f"tilt {tilt.lower:+.3f}..{tilt.upper:+.3f} rad (up +); "
            f"kept {SOFT_LIMIT_MARGIN:.2f} rad inside the URDF limits. "
            "Publish [pan, tilt] to /head/cmd.")

    def on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in HEAD_JOINTS:
                self.current[name] = pos

    def on_cmd(self, msg):
        if self.limits is None:
            self.get_logger().warn("No head limits yet (waiting for /robot_description); "
                                   "ignoring /head/cmd", throttle_duration_sec=2.0)
            return
        current = None
        if all(j in self.current for j in HEAD_JOINTS):
            current = tuple(self.current[j] for j in HEAD_JOINTS)
        try:
            goal, duration, clamped = plan_move(
                tuple(msg.data), self.limits, current,
                self.max_velocity, self.min_duration)
        except ValueError as exc:
            self.get_logger().warn(f"Rejected /head/cmd: {exc}", throttle_duration_sec=2.0)
            return
        if clamped:
            self.get_logger().warn(
                f"/head/cmd {[round(v, 3) for v in msg.data]} is outside the head limits; "
                f"clamped to {[round(v, 3) for v in goal]}", throttle_duration_sec=2.0)

        traj = JointTrajectory()
        traj.joint_names = list(HEAD_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in goal]
        # zero end velocity -> the controller eases in and out (cubic), and
        # blends smoothly from the current motion if a move is interrupted
        pt.velocities = [0.0] * len(HEAD_JOINTS)
        pt.time_from_start = Duration(sec=int(duration),
                                      nanosec=int((duration % 1.0) * 1e9))
        traj.points = [pt]
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = HeadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
