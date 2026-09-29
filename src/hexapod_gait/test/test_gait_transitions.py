"""Gait state-machine tests (no simulator needed).

Drives GaitNode.tick() directly with the real robot kinematics and a fake clock,
and checks the guarantees the start/stop transitions and the watchdog give:
  * planted feet never move during STARTING / STOPPING (no dragging)
  * at most one tripod is lifted at any time
  * foot targets are continuous (no jumps) and IK always succeeds
  * a stop always completes within two tripod steps (plus the settle pause)
  * steady walking is exactly the original D-shaped stride (walking unchanged)
  * the watchdog stops the robot after cmd_vel_timeout; tiny commands are ignored
"""
import math
import os

import numpy as np
import pytest
import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory

import hexapod_gait.gait_node as gait
from hexapod_gait.kinematics import HexapodKinematics, LEGS

COMMANDS = {
    "forward": (0.10, 0.0, 0.0),
    "backward": (-0.10, 0.0, 0.0),
    "turn": (0.0, 0.0, 0.4),
    "strafe": (0.0, 0.10, 0.0),
    "arc": (0.08, 0.0, 0.3),
    "oversized": (1.0, 0.0, 1.0),
}
TRIPODS = (gait.TRIPOD_A, gait.TRIPOD_B)


@pytest.fixture(scope="module")
def urdf():
    path = os.path.join(get_package_share_directory("Hexapod_Robot_description"),
                        "urdf", "Hexapod_Robot.xacro")
    return xacro.process_file(path).toxml()


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    """A GaitNode with a fake clock that records every tick."""

    def __init__(self, urdf, monkeypatch):
        self.now = 0.0
        self.node = gait.GaitNode()
        monkeypatch.setattr(self.node, "now_s", lambda: self.now)
        self.node.kin = HexapodKinematics(urdf)
        self.ik_failures = []
        solve = self.node.kin.solve

        def checked_solve(leg, target, seed=None):
            q, rc = solve(leg, target, seed=seed)
            if rc < 0:
                self.ik_failures.append((self.node.state, leg, target))
            return q, rc

        self.node.kin.solve = checked_solve
        self.lifted = set()
        self.node.publish_contacts = lambda lifted: setattr(self, "lifted", set(lifted))
        self.node.pub.publish = lambda msg: None

    def tick(self, cmd=None):
        """One control period. cmd=(linear.x, linear.y, angular.z) publishes a
        /cmd_vel first; None means no message this period."""
        self.now += self.node.dt
        if cmd is not None:
            self.node.on_cmd(_twist(*cmd))
        state_before = self.node.state
        feet_before = {leg: self.node.feet[leg].copy() for leg in LEGS}
        self.node.tick()
        return state_before, feet_before, self.lifted

    def run(self, ticks, cmd=None):
        return [self.tick(cmd) for _ in range(ticks)]

    def destroy(self):
        self.node.destroy_node()


def _twist(vx, vy, wz):
    from geometry_msgs.msg import Twist
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
    return t


@pytest.fixture
def robot(ros, urdf, monkeypatch):
    h = Harness(urdf, monkeypatch)
    yield h
    h.destroy()


def _ticks(seconds, node):
    return int(round(seconds / node.dt))


@pytest.mark.parametrize("name", COMMANDS)
def test_start_and_stop_never_drag_feet(robot, name):
    """Stop at many points of the gait cycle: planted feet stay still during every
    transition, one tripod at most is lifted, targets are continuous, IK succeeds,
    and the robot is standing within two tripod steps."""
    node = robot.node
    stop_limit = 2 * node.transition_time + node.transition_settle + 2 * node.dt
    for walk_ticks in range(10, 10 + _ticks(node.cycle_time, node) * 2, 3):
        robot.run(_ticks(0.5, node), (0.0, 0.0, 0.0))
        assert node.state == gait.STANDING
        records = robot.run(walk_ticks, COMMANDS[name])
        stop = robot.run(_ticks(1.5, node), (0.0, 0.0, 0.0))
        records += stop

        # stop[i][0] is the state at the start of tick i; the zero command came at tick 0
        standing_after = next(i for i, rec in enumerate(stop) if rec[0] == gait.STANDING) * node.dt
        assert standing_after <= stop_limit, f"stop took {standing_after:.2f} s"

        for i, (state, before, lifted) in enumerate(records):
            after = records[i + 1][1] if i + 1 < len(records) else node.feet
            assert any(lifted <= tri for tri in TRIPODS), f"lifted legs {lifted} span both tripods"
            for leg in LEGS:
                move = np.linalg.norm(after[leg] - before[leg])
                assert move < 0.010, f"{leg} target jumped {move * 1000:.1f} mm"
                planted = (state in (gait.STARTING, gait.STOPPING) and leg not in lifted
                           and abs(before[leg][2] - gait.STAND_HOME[leg][2]) < 1e-9
                           and abs(after[leg][2] - gait.STAND_HOME[leg][2]) < 1e-9)
                if planted:
                    assert move < 1e-9, f"planted {leg} moved {move * 1000:.2f} mm in {state}"
    assert not robot.ik_failures, robot.ik_failures[:3]


@pytest.mark.parametrize("name", ["forward", "turn", "arc"])
def test_steady_walking_is_the_original_stride(robot, name):
    """After the first cycle, every foot follows the original D-shaped stride:
    stance home + d(0.5 - f), swing home + d(f - 0.5) lifted by h*sin(pi f)."""
    node = robot.node
    cmd = COMMANDS[name]
    robot.run(_ticks(0.3, node) + 1 + _ticks(node.cycle_time, node) + 5, cmd)
    for _ in range(_ticks(node.cycle_time, node)):
        phase = node.phase
        robot.tick(cmd)
        for leg in LEGS:
            p = (phase + (0.5 if leg in gait.TRIPOD_B else 0.0)) % 1.0
            d = node.step_vector(leg)
            hx, hy, hz = gait.WALK_HOME[leg]
            if p < 0.5:
                off, oz = d * (0.5 - p / 0.5), 0.0
            else:
                f = (p - 0.5) / 0.5
                off, oz = d * (f - 0.5), node.step_height * math.sin(math.pi * f)
            expected = np.array([hx + off[0], hy + off[1], hz + oz])
            assert np.allclose(node.feet[leg], expected, atol=1e-9), leg


def test_watchdog_stops_after_timeout(robot):
    node = robot.node
    robot.run(_ticks(2.0, node), COMMANDS["forward"])
    assert node.state == gait.WALKING
    silent = robot.run(_ticks(2.0, node))                 # publisher goes quiet
    walking = [i for i, rec in enumerate(silent) if rec[0] == gait.WALKING]
    stopped_after = (max(walking) + 1) * node.dt
    assert node.cmd_timeout - node.dt <= stopped_after <= node.cmd_timeout + 2 * node.dt
    assert node.state == gait.STANDING
    assert not robot.ik_failures


def test_watchdog_does_not_stop_a_live_command(robot):
    node = robot.node
    records = robot.run(_ticks(5.0, node), COMMANDS["forward"])
    states = [rec[0] for rec in records[_ticks(0.5, node):]]
    assert set(states) == {gait.WALKING}


def test_tiny_command_is_ignored(robot):
    node = robot.node
    records = robot.run(_ticks(2.0, node), (0.005, 0.0, 0.0))
    assert {rec[0] for rec in records} == {gait.STANDING}
    assert all(not rec[2] for rec in records)
