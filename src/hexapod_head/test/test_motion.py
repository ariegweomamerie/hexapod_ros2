"""Unit tests for head motion planning (no ROS needed)."""
import math

import pytest

from hexapod_head.motion import (CUBIC_PEAK_FACTOR, HEAD_JOINTS, JointLimits, plan_move,
                                 read_joint_limits)

LIMITS = (JointLimits(-0.378190, 0.319942), JointLimits(-0.423252, 0.623945))


def test_read_joint_limits_from_urdf():
    urdf = """<robot name="r">
      <joint name="face_pan" type="revolute"><limit upper="0.319942" lower="-0.378190"/></joint>
      <joint name="face_tilt" type="revolute"><limit upper="0.623945" lower="-0.423252"/></joint>
      <joint name="leg_l1_coxa" type="revolute"><limit upper="0.5" lower="-0.5"/></joint>
      <joint name="base_joint" type="fixed"/>
    </robot>"""
    limits = read_joint_limits(urdf, HEAD_JOINTS)
    assert set(limits) == set(HEAD_JOINTS)
    assert (limits["face_pan"], limits["face_tilt"]) == LIMITS


def test_in_range_target_is_unchanged():
    goal, _, clamped = plan_move((0.25, -0.2), LIMITS, current=(0.0, 0.0))
    assert goal == (0.25, -0.2)
    assert not clamped


def test_out_of_range_target_is_clamped():
    goal, _, clamped = plan_move((1.0, -5.0), LIMITS, current=(0.0, 0.0))
    assert goal == (0.319942, -0.423252)
    assert clamped


def test_duration_keeps_peak_speed_at_limit():
    # largest move is tilt 0.5 rad at 1 rad/s peak
    _, duration, _ = plan_move((0.1, 0.5), LIMITS, current=(0.0, 0.0), max_velocity=1.0)
    assert duration == pytest.approx(CUBIC_PEAK_FACTOR * 0.5)


def test_tiny_move_uses_min_duration():
    _, duration, _ = plan_move((0.001, 0.0), LIMITS, current=(0.0, 0.0), min_duration=0.2)
    assert duration == pytest.approx(0.2)


def test_unknown_current_assumes_worst_case_travel():
    # from goal (0, 0), the farthest possible start is tilt upper (0.623945)
    _, duration, _ = plan_move((0.0, 0.0), LIMITS, current=None, max_velocity=1.0)
    assert duration == pytest.approx(CUBIC_PEAK_FACTOR * 0.623945)


@pytest.mark.parametrize("target", [(0.1,), (0.1, 0.2, 0.3), (math.nan, 0.0), (0.0, math.inf)])
def test_malformed_target_is_rejected(target):
    with pytest.raises(ValueError):
        plan_move(target, LIMITS, current=(0.0, 0.0))


def test_non_positive_velocity_is_rejected():
    with pytest.raises(ValueError):
        plan_move((0.0, 0.0), LIMITS, current=(0.0, 0.0), max_velocity=0.0)
