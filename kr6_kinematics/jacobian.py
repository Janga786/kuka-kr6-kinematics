"""
Velocity kinematics — geometric Jacobian and singularity diagnostics
====================================================================

The 6×6 geometric Jacobian relates joint velocities to the spatial
twist of the end-effector:

.. math::

    \\begin{bmatrix} \\mathbf{v} \\\\ \\boldsymbol{\\omega} \\end{bmatrix}
    = J(q)\\, \\dot q
    \\qquad
    J(q) = \\begin{bmatrix} J_v(q) \\\\ J_\\omega(q) \\end{bmatrix}
    \\in \\mathbb{R}^{6\\times 6}.

For a serial chain of revolute joints, the *i*-th column of :math:`J`
takes the well-known form

.. math::

    J_i(q) = \\begin{bmatrix}
        \\hat z_i \\times (p_e - p_i) \\\\
        \\hat z_i
    \\end{bmatrix},

where :math:`\\hat z_i` is the joint axis of joint *i* and
:math:`p_i, p_e` are positions of the joint and the end-effector,
both expressed in the base frame.

Because the KR 6 R900 sixx is non-redundant (6 DoF, 6 task
coordinates), :math:`J(q)` is square and the inverse velocity
problem reduces to ``np.linalg.solve(J, twist)`` — provided we are
not at a singularity.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .dh import kr6_frames


def jacobian(q: ArrayLike) -> NDArray[np.float64]:
    """
    Compute the 6×6 geometric Jacobian in the base frame.

    Parameters
    ----------
    q
        A length-6 joint vector (radians).

    Returns
    -------
    J
        The Jacobian matrix.  Rows 0–2 form the linear-velocity block
        :math:`J_v`; rows 3–5 form the angular-velocity block
        :math:`J_\\omega`.
    """
    frames = kr6_frames(q)
    origins = [T.t for T in frames]
    z_axes = [T.R[:, 2] for T in frames]
    p_ee = origins[6]

    J = np.zeros((6, 6))
    for i in range(6):
        z_i = z_axes[i]
        p_i = origins[i]
        J[:3, i] = np.cross(z_i, p_ee - p_i)
        J[3:, i] = z_i
    return J


def manipulability(q: ArrayLike) -> float:
    """
    Return Yoshikawa's manipulability index :math:`w(q) = \\sqrt{\\det(J J^\\top)}`.

    For a non-redundant manipulator this reduces to :math:`|\\det J|`,
    but the SVD-based form generalises naturally to redundant cases and
    is numerically equivalent here.
    """
    J = jacobian(q)
    return float(np.sqrt(np.abs(np.linalg.det(J @ J.T))))


def is_singular(q: ArrayLike, sigma_min_threshold: float = 1e-3) -> bool:
    """
    Return ``True`` if the Jacobian is "close" to singular.

    A configuration is flagged as singular when the smallest singular
    value of :math:`J(q)` falls below ``sigma_min_threshold``. The
    default cut-off (1e-3) is chosen empirically from the wrist
    singularity sweep — see :mod:`scripts.05_jacobian_singularities`.
    """
    sv_min = np.linalg.svd(jacobian(q), compute_uv=False)[-1]
    return bool(sv_min < sigma_min_threshold)
