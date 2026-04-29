"""Tests for the geometric Jacobian and singularity diagnostics."""

from __future__ import annotations

import numpy as np
import pytest

from kr6_kinematics import (
    Q_READY,
    is_singular,
    jacobian,
    kr6_fk,
    manipulability,
)


def _fd_jacobian(q: np.ndarray, h: float = 1e-6) -> np.ndarray:
    """Central-difference Jacobian (used as ground truth in tests)."""
    J_fd = np.zeros((6, 6))
    for j in range(6):
        qp = q.copy()
        qm = q.copy()
        qp[j] += h
        qm[j] -= h
        Tp = kr6_fk(qp)
        Tm = kr6_fk(qm)
        dp = (Tp.t - Tm.t) / (2 * h)
        R_err = Tp.R @ Tm.R.T
        w = 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ]) / (2 * h)
        J_fd[:, j] = np.concatenate([dp, w])
    return J_fd


def test_jacobian_shape():
    assert jacobian(Q_READY).shape == (6, 6)


def test_jacobian_matches_finite_difference(rng):
    """Analytic J(q) must agree with central-difference J(q) to ~1e-6."""
    worst = 0.0
    for _ in range(30):
        q = rng.uniform(-1.0, 1.0, 6)
        err = float(np.max(np.abs(jacobian(q) - _fd_jacobian(q))))
        worst = max(worst, err)
    assert worst < 1e-5


def test_manipulability_positive_at_ready():
    assert manipulability(Q_READY) > 0


def test_wrist_singularity_detected():
    """A q5 = 0 configuration must be flagged as singular."""
    q = Q_READY.copy()
    q[4] = 0.0
    assert is_singular(q) is True


def test_ready_pose_is_not_singular():
    assert is_singular(Q_READY) is False


@pytest.mark.parametrize("q5", [0.0, 1e-4, -1e-4])
def test_manipulability_collapses_near_wrist_singularity(q5):
    q = Q_READY.copy()
    q[4] = q5
    w_sing = manipulability(q)
    w_ready = manipulability(Q_READY)
    assert w_sing < 0.05 * w_ready
