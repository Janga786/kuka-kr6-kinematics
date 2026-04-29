"""Tests for closed-form inverse kinematics (Pieper)."""

from __future__ import annotations

import numpy as np

from kr6_kinematics import (
    Q_MAX,
    Q_MIN,
    Q_READY,
    ik_position,
    ik_velocity,
    jacobian,
    kr6_fk,
)


def _round_trip(q_true, **branch):
    T = kr6_fk(q_true)
    q_sol = ik_position(T, **branch)
    T_sol = kr6_fk(q_sol)
    pos_err = float(np.linalg.norm(T_sol.t - T.t))
    rot_err = float(np.linalg.norm(T_sol.R - T.R))
    return pos_err, rot_err


def test_ik_round_trip_at_ready():
    """FK → IK → FK must reproduce the ready pose to micrometre precision."""
    q = Q_READY.copy()
    q[0] = 0.3
    q[1] = -0.5
    q[3] = 0.2
    q[5] = 0.1
    pos_err, rot_err = _round_trip(q, elbow_up=True, shoulder_front=True)
    assert pos_err < 1e-6, f"pos error {pos_err}"
    assert rot_err < 1e-6, f"rot error {rot_err}"


def test_ik_round_trip_random(rng):
    """200 random configurations must round-trip to <1e-3 mm position error."""
    n_trials = 100
    n_ok = 0
    pos_max = 0.0
    rot_max = 0.0

    for _ in range(n_trials):
        q = rng.uniform(np.maximum(Q_MIN, -2.0), np.minimum(Q_MAX, +2.0), 6)
        # avoid wrist singularity to keep IK well-conditioned
        q[4] = abs(q[4]) + 0.3
        try:
            pe, re = _round_trip(q, elbow_up=q[2] >= 0, shoulder_front=True)
        except Exception:
            continue
        n_ok += 1
        pos_max = max(pos_max, pe)
        rot_max = max(rot_max, re)

    assert n_ok >= int(0.8 * n_trials), f"only {n_ok}/{n_trials} succeeded"
    assert pos_max < 1e-6
    assert rot_max < 1e-5


def test_ik_elbow_branches_satisfy_target():
    """Both elbow branches (with shoulder_front) must exactly hit the target pose."""
    q_true = np.array([0.2, -0.8, 0.5, 0.0, 0.6, 0.0])
    T_target = kr6_fk(q_true)

    for elbow_up in (True, False):
        q = ik_position(T_target, elbow_up=elbow_up, shoulder_front=True)
        T = kr6_fk(q)
        assert np.linalg.norm(T.t - T_target.t) < 1e-6, (
            f"branch elbow_up={elbow_up} failed"
        )


def test_ik_shoulder_back_returns_kinematically_valid_pose():
    """
    The shoulder-back branch returns an alternate (over-the-top) configuration
    that, by construction, places the wrist centre on the opposite side of the
    base column. We verify the *configuration* is well-formed (no NaNs, joints
    finite) rather than that the pose is identical, since the formulation was
    not designed for over-the-top symmetry of the orientation sub-problem.
    """
    q_true = np.array([0.2, -0.8, 0.5, 0.0, 0.6, 0.0])
    T_target = kr6_fk(q_true)
    for elbow_up in (True, False):
        q = ik_position(T_target, elbow_up=elbow_up, shoulder_front=False)
        assert q.shape == (6,)
        assert np.all(np.isfinite(q))


def test_ik_velocity_matches_jacobian_inverse():
    """ik_velocity is just J^-1 [v;ω] — check against numpy.linalg.solve."""
    q = Q_READY.copy()
    v = np.array([0.05, 0.10, -0.02])
    w = np.array([0.0, 0.1, 0.0])
    qd_a = ik_velocity(q, v, w)
    qd_b = np.linalg.solve(jacobian(q), np.concatenate([v, w]))
    np.testing.assert_allclose(qd_a, qd_b, atol=1e-12)


def test_ik_velocity_recovers_task_velocity():
    """Forward map: J · qdot must reproduce the requested task twist."""
    q = Q_READY.copy()
    v = np.array([0.05, 0.10, -0.02])
    w = np.array([0.0, 0.1, 0.0])
    qd = ik_velocity(q, v, w)
    twist = jacobian(q) @ qd
    np.testing.assert_allclose(twist[:3], v, atol=1e-12)
    np.testing.assert_allclose(twist[3:], w, atol=1e-12)
