"""Head motion planning for the pan/tilt face (pure Python, no ROS).

Kept free of ROS so it can be unit tested and reused by a real servo driver.
Angles are radians. With the re-zeroed URDF, (0, 0) looks straight ahead and
level; pan is positive to the LEFT, tilt is positive UP.
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

HEAD_JOINTS = ("face_pan", "face_tilt")

# The controller moves along a cubic that starts and stops at rest. Its peak
# speed is 1.5x the average speed, so durations include this factor to keep
# the PEAK speed at or below max_velocity.
CUBIC_PEAK_FACTOR = 1.5


@dataclass(frozen=True)
class JointLimits:
    lower: float
    upper: float

    def clamp(self, value):
        return min(max(value, self.lower), self.upper)


def read_joint_limits(urdf_xml, joint_names):
    """Return {joint: JointLimits} for the named joints that have a <limit>."""
    limits = {}
    for joint in ET.fromstring(urdf_xml).iter("joint"):
        limit = joint.find("limit")
        if joint.get("name") in joint_names and limit is not None:
            limits[joint.get("name")] = JointLimits(float(limit.get("lower")),
                                                    float(limit.get("upper")))
    return limits


def plan_move(target, limits, current=None, max_velocity=1.0, min_duration=0.2):
    """Plan one head move.

    target:  (pan, tilt) requested angles.
    limits:  (pan JointLimits, tilt JointLimits).
    current: (pan, tilt) measured angles, or None if not known yet. When
             unknown, the move is timed for the worst case (the far end of
             each joint's range), so it is never faster than allowed.

    Returns (goal, duration, clamped):
      goal     - target clamped into the joint limits
      duration - seconds for the move, so no joint's peak speed exceeds
                 max_velocity (never shorter than min_duration)
      clamped  - True if any requested angle was outside its limits
    """
    if len(target) != len(HEAD_JOINTS):
        raise ValueError(f"expected [pan, tilt] (2 values), got {len(target)}")
    if not all(math.isfinite(v) for v in target):
        raise ValueError(f"angles must be finite numbers, got {list(target)}")
    if max_velocity <= 0.0:
        raise ValueError(f"max_velocity must be > 0, got {max_velocity}")

    goal = tuple(lim.clamp(v) for lim, v in zip(limits, target))
    clamped = any(g != v for g, v in zip(goal, target))

    if current is None:
        travel = max(max(g - lim.lower, lim.upper - g) for g, lim in zip(goal, limits))
    else:
        travel = max(abs(g - c) for g, c in zip(goal, current))

    duration = max(min_duration, CUBIC_PEAK_FACTOR * travel / max_velocity)
    return goal, duration, clamped
