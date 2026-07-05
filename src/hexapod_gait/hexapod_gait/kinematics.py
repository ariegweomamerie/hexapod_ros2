#!/usr/bin/env python3
"""Per-leg forward/inverse kinematics for the hexapod using PyKDL.

kdl_parser_py is not packaged for ROS 2 Jazzy, so we replicate its urdf->KDL
conversion with urdf_parser_py, then expose FK/IK for each
base_footprint -> leg_*_foot chain. Legs are 3-DOF so IK is position-only.
"""
import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

LEGS = ["leg_l1", "leg_l2", "leg_l3", "leg_r1", "leg_r2", "leg_r3"]
JOINTS_PER_LEG = ("coxa", "femur", "tibia")
BASE = "base_footprint"


def _pose_to_frame(pose):
    if pose is None:
        return kdl.Frame()
    xyz = pose.xyz or [0, 0, 0]
    rpy = pose.rpy or [0, 0, 0]
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def _joint_to_kdl(jnt):
    origin = _pose_to_frame(jnt.origin)
    if jnt.type == "fixed" or jnt.axis is None:
        return kdl.Joint(jnt.name, kdl.Joint.Fixed)
    axis = kdl.Vector(*jnt.axis)
    if jnt.type in ("revolute", "continuous"):
        return kdl.Joint(jnt.name, origin.p, origin.M * axis, kdl.Joint.RotAxis)
    if jnt.type == "prismatic":
        return kdl.Joint(jnt.name, origin.p, origin.M * axis, kdl.Joint.TransAxis)
    return kdl.Joint(jnt.name, kdl.Joint.Fixed)


def urdf_to_kdl_tree(robot):
    tree = kdl.Tree(robot.get_root())

    def add(parent):
        if parent in robot.child_map:
            for joint_name, child in robot.child_map[parent]:
                jnt = robot.joint_map[joint_name]
                seg = kdl.Segment(child, _joint_to_kdl(jnt), _pose_to_frame(jnt.origin))
                tree.addSegment(seg, parent)
                add(child)

    add(robot.get_root())
    return tree


class HexapodKinematics:
    """FK/IK for all six base_footprint -> leg_*_foot chains.

    IK is a damped least-squares (Levenberg-Marquardt style) Jacobian iteration
    on the position error only (legs are 3-DOF), with joint-limit clipping. This
    is far more robust here than KDL's ChainIkSolverPos_LMA, which diverges for
    several legs under position-only weighting.
    """

    def __init__(self, urdf_string):
        robot = URDF.from_xml_string(urdf_string)
        tree = urdf_to_kdl_tree(robot)
        self.nj, self.fk, self.jac = {}, {}, {}
        self.qlow, self.qhigh = {}, {}
        self.chains = {}   # keep Chain objects alive: solvers only hold C++ refs
        for leg in LEGS:
            ch = tree.getChain(BASE, f"{leg}_foot")
            self.chains[leg] = ch
            self.nj[leg] = ch.getNrOfJoints()
            self.fk[leg] = kdl.ChainFkSolverPos_recursive(ch)
            self.jac[leg] = kdl.ChainJntToJacSolver(ch)
            lo, hi = [], []
            for j in JOINTS_PER_LEG:
                lim = robot.joint_map[f"{leg}_{j}"].limit
                lo.append(lim.lower)
                hi.append(lim.upper)
            self.qlow[leg] = np.array(lo)
            self.qhigh[leg] = np.array(hi)

    def _arr(self, leg, q):
        a = kdl.JntArray(self.nj[leg])
        for i, v in enumerate(q):
            a[i] = float(v)
        return a

    def foot_position(self, leg, q):
        """FK: joint angles -> foot (x, y, z) in base_footprint frame."""
        f = kdl.Frame()
        self.fk[leg].JntToCart(self._arr(leg, q), f)
        return np.array([f.p.x(), f.p.y(), f.p.z()])

    def _jacobian(self, leg, q):
        J = kdl.Jacobian(self.nj[leg])
        self.jac[leg].JntToJac(self._arr(leg, q), J)
        return np.array([[J[r, c] for c in range(self.nj[leg])] for r in range(3)])

    def solve(self, leg, target_xyz, seed=(0.0, 0.0, 0.0),
              tol=1e-5, max_iter=300, damping=0.05, max_step=0.2):
        """IK via damped least squares with step clamping. Returns (q, rc);
        rc=0 on convergence. Step clamping keeps it stable near the
        near-singular (nearly extended) stance configuration."""
        q = np.array(seed, dtype=float)
        tgt = np.array([float(v) for v in target_xyz])
        for _ in range(max_iter):
            e = tgt - self.foot_position(leg, q)
            if np.linalg.norm(e) < tol:
                return q, 0
            J = self._jacobian(leg, q)
            dq = J.T @ np.linalg.solve(J @ J.T + (damping ** 2) * np.eye(3), e)
            n = np.linalg.norm(dq)
            if n > max_step:
                dq *= max_step / n          # damp overshoot near singularities
            q = np.clip(q + dq, self.qlow[leg], self.qhigh[leg])
        return q, -1
