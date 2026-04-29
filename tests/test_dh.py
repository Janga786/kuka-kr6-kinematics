"""Tests for DH transforms and forward kinematics."""

from __future__ import annotations

import numpy as np
import pytest

from kr6_kinematics import (
    KR6_SDH,
    Q_MAX,
    Q_MIN,
    Q_READY,
    Q_ZERO,
    clip_to_limits,
    dh_link,
    in_limits,
    kr6_fk,
    kr6_frames,
)


def test_dh_table_shape_and_dtype():
    assert KR6_SDH.shape == (6, 4)
    assert KR6_SDH.dtype == np.float64


def test_joint_limit_ordering():
    assert np.all(Q_MIN < Q_MAX)


def test_dh_link_is_a_proper_homogeneous_transform():
    """Each row must be a valid SE(3) transform (rotation orthonormal, last row [0,0,0,1])."""
    for i, (a, alpha, d, theta_off) in enumerate(KR6_SDH):
        T = dh_link(a, alpha, d, theta_off)
        R = T[:3, :3]

        assert T.shape == (4, 4), f"row {i}: bad shape"
        np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-10)


def test_kr6_frames_returns_seven_se3():
    frames = kr6_frames(Q_ZERO)
    assert len(frames) == 7
    # frame 0 is the identity base frame
    np.testing.assert_allclose(frames[0].t, np.zeros(3), atol=1e-12)


def test_fk_at_zero_matches_kuka_reach_spec():
    """At q = 0 the wrist-centre xy-reach must be ~0.901 m (KUKA spec)."""
    p_wc = kr6_frames(Q_ZERO)[5].t
    reach = float(np.linalg.norm(p_wc[:2]))
    assert abs(reach - 0.901) < 5e-3   # within 5 mm of the published value


def test_fk_consistency_between_frames_and_fk():
    """kr6_fk and kr6_frames[-1] must agree to floating-point precision."""
    T_a = kr6_fk(Q_READY).A
    T_b = kr6_frames(Q_READY)[-1].A
    np.testing.assert_allclose(T_a, T_b, atol=1e-12)


def test_fk_continuity_under_small_perturbation(rng):
    """Forward kinematics is smooth: a 1e-6 perturbation should move the flange < 5 mm."""
    q = rng.uniform(-1.0, 1.0, 6)
    p = kr6_fk(q).t
    for _ in range(5):
        dq = rng.uniform(-1, 1, 6) * 1e-6
        p2 = kr6_fk(q + dq).t
        assert np.linalg.norm(p2 - p) < 5e-3


@pytest.mark.parametrize(
    "q, expect",
    [
        (Q_ZERO, True),
        (Q_READY, True),
        (Q_MAX + 0.1, False),
        (Q_MIN - 0.1, False),
    ],
)
def test_in_limits(q, expect):
    assert in_limits(q) is expect


def test_clip_to_limits_preserves_shape_and_clips():
    q = np.full(6, 100.0)
    qc = clip_to_limits(q)
    assert qc.shape == (6,)
    assert np.all(qc <= Q_MAX + 1e-12)
    assert np.all(qc >= Q_MIN - 1e-12)


def test_kr6_frames_rejects_wrong_shape():
    with pytest.raises(ValueError):
        kr6_frames(np.zeros(5))
