"""Tests for trajectory primitives."""

from __future__ import annotations

import numpy as np
import pytest

from kr6_kinematics import lspb, quintic, quintic_vec


# ---------------------------------------------------------------------------
# Quintic
# ---------------------------------------------------------------------------

def test_quintic_boundary_conditions():
    """Position, velocity and acceleration must equal the prescribed endpoints."""
    t, q, qd, qdd = quintic(0, 1, 0.0, 1.0, 0.1, -0.2, 0.3, -0.4, num_points=200)
    assert np.isclose(q[0],  0.0)
    assert np.isclose(q[-1], 1.0)
    assert np.isclose(qd[0],   0.1)
    assert np.isclose(qd[-1], -0.2)
    assert np.isclose(qdd[0],   0.3)
    assert np.isclose(qdd[-1], -0.4)


def test_quintic_velocity_is_derivative_of_position():
    t, q, qd, _ = quintic(0, 1, 0, 1, num_points=2000)
    qd_fd = np.gradient(q, t)
    assert np.max(np.abs(qd - qd_fd)) < 5e-3


def test_quintic_acceleration_is_derivative_of_velocity():
    t, _, qd, qdd = quintic(0, 1, 0, 1, num_points=2000)
    qdd_fd = np.gradient(qd, t)
    assert np.max(np.abs(qdd - qdd_fd)) < 5e-2


def test_quintic_time_grid_endpoints():
    t, _, _, _ = quintic(0.5, 2.5, 0, 1, num_points=10)
    assert np.isclose(t[0], 0.5)
    assert np.isclose(t[-1], 2.5)


# ---------------------------------------------------------------------------
# Quintic-vec
# ---------------------------------------------------------------------------

def test_quintic_vec_shape_and_endpoints():
    q0 = np.zeros(6)
    qf = np.array([0.5, -0.2, 0.3, 0.0, -0.4, 0.7])
    t, Q, Qd, Qdd = quintic_vec(0, 2.0, q0, qf, num_points=50)

    assert Q.shape == (50, 6)
    np.testing.assert_allclose(Q[0],  q0, atol=1e-10)
    np.testing.assert_allclose(Q[-1], qf, atol=1e-10)
    np.testing.assert_allclose(Qd[0],  np.zeros(6), atol=1e-10)
    np.testing.assert_allclose(Qd[-1], np.zeros(6), atol=1e-10)
    np.testing.assert_allclose(Qdd[0],  np.zeros(6), atol=1e-10)
    np.testing.assert_allclose(Qdd[-1], np.zeros(6), atol=1e-10)


# ---------------------------------------------------------------------------
# LSPB
# ---------------------------------------------------------------------------

def test_lspb_endpoints():
    t, q, qd, qdd = lspb(0, 1, 0, 1, 1.25, num_points=200)
    assert np.isclose(q[0],  0.0)
    assert np.isclose(q[-1], 1.0)
    assert np.isclose(qd[0],  0.0, atol=1e-2)
    assert np.isclose(qd[-1], 0.0, atol=1e-2)


def test_lspb_cruise_velocity_is_attained():
    """During the linear segment, velocity must equal the requested cruise speed."""
    t, q, qd, qdd = lspb(0, 1, 0, 1, 1.25, num_points=400)
    # midway through: definitely on the cruise segment
    mid = qd[200]
    assert np.isclose(mid, 1.25, atol=1e-3)


@pytest.mark.parametrize("v_bad", [0.5, 5.0])
def test_lspb_rejects_infeasible_velocity(v_bad):
    """v must lie in (|h|/T, 2|h|/T] - other values raise ValueError."""
    with pytest.raises(ValueError):
        lspb(0, 1, 0, 1, v_bad)
