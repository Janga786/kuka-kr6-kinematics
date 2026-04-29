"""
Closed-form inverse kinematics via Pieper's spherical-wrist decoupling
======================================================================

The KR 6 R900 sixx satisfies *Pieper's criterion* — its three wrist
axes (A4, A5, A6) intersect at a single point, the *wrist centre*. Any
target flange pose :math:`(R_d, p_d)` therefore decouples into:

1. **Position sub-problem.** Compute the wrist-centre location
   :math:`p_{wc} = p_d - d_6\\, R_d \\hat z` and solve the resulting
   2-link planar geometry for :math:`q_1, q_2, q_3`.
2. **Orientation sub-problem.** Form
   :math:`R_3^6 = (R_0^3)^{\\!\\top} R_d` and recover
   :math:`q_4, q_5, q_6` via a ZYZ Euler-angle decomposition.

Each sub-problem admits two geometric branches (``shoulder front/back``
and ``elbow up/down``), which combine to four position branches.
Together with the two orientation branches obtained by flipping
:math:`q_5 \\to -q_5`, this yields up to eight IK solutions, of which
this implementation returns one (selected by the caller).

For inverse *velocity* kinematics the manipulator is non-redundant, so
``qdot = J^{-1} \\xi`` is computed directly via :func:`numpy.linalg.solve`.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .dh import KR6_SDH, dh_link
from .jacobian import jacobian


# ---------------------------------------------------------------------------
# Geometry constants (extracted once from the DH table for clarity)
# ---------------------------------------------------------------------------

A1 = KR6_SDH[0, 0]   # shoulder horizontal offset
D1 = KR6_SDH[0, 2]   # shoulder height
A2 = KR6_SDH[1, 0]   # upper-arm length
A3 = KR6_SDH[2, 0]   # elbow horizontal offset
D4 = KR6_SDH[3, 2]   # forearm length
D6 = KR6_SDH[5, 2]   # wrist-to-flange distance

# Effective forearm length and "tilt" angle when collapsing the elbow
# offset (a3, d4) into a single planar 2R link. See accompanying report.
L_FORE: float = float(np.sqrt(A3 ** 2 + D4 ** 2))
BETA_OFF: float = float(np.arctan2(A3, D4))


# ---------------------------------------------------------------------------
# Position IK (Pieper)
# ---------------------------------------------------------------------------

def ik_position(
    T_target,
    elbow_up: bool = True,
    shoulder_front: bool = True,
) -> NDArray[np.float64]:
    """
    Solve the inverse-kinematics problem for a target flange pose.

    Parameters
    ----------
    T_target
        The desired flange pose. May be any object exposing an ``A``
        attribute that returns a :math:`4\\times4` homogeneous matrix
        (e.g. :class:`spatialmath.SE3`) or a raw :math:`4\\times4`
        ``ndarray``.
    elbow_up
        Selects the elbow-up branch of the position sub-problem when
        ``True``, elbow-down otherwise.
    shoulder_front
        Selects the shoulder-front branch when ``True``, shoulder-back
        (over-the-top) otherwise.

    Returns
    -------
    q
        Length-6 array of joint angles, wrapped to :math:`(-\\pi, \\pi]`.

    Notes
    -----
    The implementation does not check joint limits; the caller is
    expected to compose this with :func:`kr6_kinematics.in_limits` /
    :func:`kr6_kinematics.clip_to_limits` if hard-stop avoidance is
    required.
    """
    T = np.asarray(T_target.A) if hasattr(T_target, "A") else np.asarray(T_target)
    R_d = T[:3, :3]
    p_d = T[:3, 3]

    # ---- Position sub-problem ------------------------------------------------

    # 1. Wrist-centre location: subtract the d6 offset along the flange z-axis.
    p_wc = p_d - D6 * R_d[:, 2]
    x_wc, y_wc, z_wc = p_wc

    # 2. q1 from the planar wrist-centre projection.
    base_angle = np.arctan2(y_wc, x_wc)
    q_1 = base_angle if shoulder_front else base_angle + np.pi

    # 3. q2, q3 from the planar 2R triangle (a2 — L_FORE).
    r_xy = np.sqrt(x_wc ** 2 + y_wc ** 2)
    sx = (r_xy if shoulder_front else -r_xy) - A1
    sz = z_wc - D1
    d_planar = np.sqrt(sx ** 2 + sz ** 2)

    cos_bend = (d_planar ** 2 - A2 ** 2 - L_FORE ** 2) / (2 * A2 * L_FORE)
    cos_bend = np.clip(cos_bend, -1.0, 1.0)
    bend = np.arccos(cos_bend) if elbow_up else -np.arccos(cos_bend)

    q_3 = bend + BETA_OFF

    psi = np.arctan2(sz, sx)
    phi = np.arctan2(L_FORE * np.sin(bend), A2 + L_FORE * np.cos(bend))
    q_2 = -(psi + phi)

    # ---- Orientation sub-problem --------------------------------------------

    # Build R_0^3 from the position joints we just solved.
    T_0_3 = np.eye(4)
    for i, theta_i in enumerate([q_1, q_2, q_3]):
        a, alpha, d, theta_off = KR6_SDH[i]
        T_0_3 = T_0_3 @ dh_link(a, alpha, d, theta_off + theta_i)
    R_0_3 = T_0_3[:3, :3]

    # The wrist-only rotation expressed in the {3} frame.
    R_3_6 = R_0_3.T @ R_d

    # ZYZ Euler decomposition.
    sin_b = np.sqrt(R_3_6[0, 2] ** 2 + R_3_6[1, 2] ** 2)
    cos_b = R_3_6[2, 2]
    beta = np.arctan2(sin_b, cos_b)

    if sin_b > 1e-6:
        alpha_e = np.arctan2(R_3_6[1, 2], R_3_6[0, 2])
        gamma_e = np.arctan2(R_3_6[2, 1], -R_3_6[2, 0])
    else:
        # Wrist singularity: q4 + q6 is determined but not q4 and q6 separately.
        # By convention we put all the "spin" into q6.
        alpha_e = 0.0
        gamma_e = np.arctan2(R_3_6[1, 0], R_3_6[0, 0])

    q_4, q_5, q_6 = alpha_e, beta, gamma_e

    q = np.array([q_1, q_2, q_3, q_4, q_5, q_6])
    # Wrap each angle to (-pi, pi].
    return ((q + np.pi) % (2 * np.pi)) - np.pi


# ---------------------------------------------------------------------------
# Velocity IK
# ---------------------------------------------------------------------------

def ik_velocity(
    q: ArrayLike,
    v: ArrayLike,
    omega: ArrayLike,
) -> NDArray[np.float64]:
    """
    Map a desired spatial twist to joint velocities, :math:`\\dot q = J^{-1}\\xi`.

    Parameters
    ----------
    q
        Current joint configuration (radians).
    v
        Desired linear velocity of the flange in the base frame
        (length-3, m/s).
    omega
        Desired angular velocity of the flange in the base frame
        (length-3, rad/s).

    Returns
    -------
    qdot
        Length-6 vector of joint velocities. A :class:`numpy.linalg.LinAlgError`
        is raised if the Jacobian is singular at ``q``.
    """
    J = jacobian(q)
    twist = np.concatenate([np.asarray(v, dtype=float), np.asarray(omega, dtype=float)])
    return np.linalg.solve(J, twist)
