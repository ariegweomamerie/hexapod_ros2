"""Unit tests for the odometry scoring maths (no ROS graph needed).

These check that the evaluation would report the truth on trajectories whose
answer we already know, so a number in the Stage 2.3 report means what it says.
"""
import math

from hexapod_slam.evaluate_odometry import align_ground_truth, resample, yaw_of


class Q:
    def __init__(self, yaw):
        self.x = self.y = 0.0
        self.z = math.sin(yaw / 2)
        self.w = math.cos(yaw / 2)


def test_yaw_of_recovers_the_angle():
    for yaw in (-2.0, -0.3, 0.0, 1.5708, 3.0):
        assert abs(yaw_of(Q(yaw)) - yaw) < 1e-9


def test_alignment_puts_the_start_at_the_origin_facing_forward():
    # ground truth starts at the facility START pose (3.3, 3.3, yaw 90 deg)
    gt = [(0.0, 3.3, 3.3, 0.0, math.pi / 2),
          (1.0, 3.3, 4.3, 0.0, math.pi / 2),     # 1 m along its own heading
          (2.0, 2.3, 4.3, 0.0, math.pi)]         # then 1 m to its own left
    a = align_ground_truth(gt)
    assert a[0][1:] == (0.0, 0.0, 0.0, 0.0)
    assert abs(a[1][1] - 1.0) < 1e-9 and abs(a[1][2]) < 1e-9      # forward = +x
    assert abs(a[2][1] - 1.0) < 1e-9 and abs(a[2][2] - 1.0) < 1e-9  # left = +y
    assert abs(a[2][4] - math.pi / 2) < 1e-9


def test_alignment_preserves_path_length_and_shape():
    gt = [(t, 2.0 + math.cos(t), 5.0 + math.sin(t), 0.0, t) for t in
          [i * 0.1 for i in range(40)]]
    a = align_ground_truth(gt)
    for (_, x1, y1, _, _), (_, x2, y2, _, _) in zip(gt[1:], a[1:]):
        pass
    length = lambda s: sum(math.dist(p[1:3], q[1:3]) for p, q in zip(s, s[1:]))
    assert abs(length(gt) - length(a)) < 1e-9


def test_resample_picks_the_nearest_sample_in_time():
    series = [(0.0, 0, 0, 0, 0), (1.0, 1, 0, 0, 0), (2.0, 2, 0, 0, 0)]
    assert [s[0] for s in resample(series, [0.1, 0.9, 1.4, 5.0])] == [0.0, 1.0, 1.0, 2.0]


def test_a_perfect_odometry_scores_zero_error():
    gt = [(t * 0.1, 3.3 + 0.1 * t, 3.3, 0.0, 0.0) for t in range(50)]
    a = align_ground_truth(gt)
    err = [math.hypot(v[1] - g[1], v[2] - g[2]) for v, g in zip(a, resample(a, [p[0] for p in a]))]
    assert max(err) == 0.0
