"""
Denavit-Hartenberg parameterisation and forward kinematics
==========================================================

The KUKA KR 6 R900 sixx is a 6-DoF serial chain whose three wrist axes
(A4, A5, A6) intersect at a common point — a so-called "in-line" or
"spherical" wrist. This makes the manipulator amenable to Pieper's
kinematic decoupling: the position of the wrist centre is determined
entirely by the first three joints, and the orientation of the flange
is determined entirely by the last three.

The standard Denavit-Hartenberg convention is used throughout:

    .. math::

        T_{i-1}^{\\,i}(\\theta_i) = R_z(\\theta_i)\\, T_z(d_i)\\,
                                    T_x(a_i)\\, R_x(\\alpha_i)

where :math:`a_i, \\alpha_i, d_i` are link constants and :math:`\\theta_i`
is the joint variable for revolute joint *i*.

The link constants below are taken from the ROS-Industrial URDF for the
KR 6 R900 sixx, cross-checked against the KUKA operating instructions
manual (2015) and against the parameters reported by Turgut & Kaleli
("Kinematic and dynamic analysis of a 6-DoF KUKA KR 6 R900 sixx
industrial robot", 2022). All units are SI (metres and radians).
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray
from spatialmath import SE3

# ---------------------------------------------------------------------------
# DH parameter table
# ---------------------------------------------------------------------------
#
# Rows are ordered (a, alpha, d, theta_offset). The configuration q = 0
# corresponds to the URDF "zero" pose (arm fully extended along +x).

KR6_SDH: NDArray[np.float64] = np.array([
    # a [m]     alpha [rad]   d [m]      theta_offset [rad]
    [ 0.025,  -np.pi / 2,    0.400,     0.0          ],   # A1 — shoulder rotation
    [ 0.455,   0.0,          0.000,     0.0          ],   # A2 — upper arm
    [ 0.035,  -np.pi / 2,    0.000,    -np.pi / 2    ],   # A3 — elbow
    [ 0.000,   np.pi / 2,    0.420,    -np.pi        ],   # A4 — forearm roll
    [ 0.000,  -np.pi / 2,    0.000,     0.0          ],   # A5 — wrist pitch
    [ 0.000,   0.0,          0.080,    -np.pi        ],   # A6 — flange roll
], dtype=np.float64)


# ---------------------------------------------------------------------------
# Joint limits and velocity limits (KUKA KR 6 R900 sixx data sheet)
# ---------------------------------------------------------------------------

Q_MIN: NDArray[np.float64] = np.array(
    [-2.9671, -3.3161, -2.0944, -3.2289, -2.0944, -6.1087]
)
Q_MAX: NDArray[np.float64] = np.array(
    [ 2.9671,  0.7854,  2.7227,  3.2289,  2.0944,  6.1087]
)

# Maximum joint velocities (rad/s) per axis from KUKA spec
QD_MAX: NDArray[np.float64] = np.array(
    [6.2832, 5.2360, 6.2832, 6.6497, 6.7718, 10.7338]
)


# ---------------------------------------------------------------------------
# Useful canonical poses
# ---------------------------------------------------------------------------

#: URDF zero configuration (arm fully extended).
Q_ZERO: NDArray[np.float64] = np.zeros(6)

#: A non-singular "ready" pose used as a benchmark throughout the project.
#: Roughly an L-shape, comfortably away from all three singularity types.
Q_READY: NDArray[np.float64] = np.array(
    [0.0, -np.pi / 3, np.pi / 3, 0.0, np.pi / 4, 0.0]
)

#: Canonical KUKA joint names (A1 … A6).
JOINT_NAMES: list[str] = [f"A{i + 1}" for i in range(6)]


# ---------------------------------------------------------------------------
# Single-link DH transform
# ---------------------------------------------------------------------------

def dh_link(a: float, alpha: float, d: float, theta: float) -> NDArray[np.float64]:
    """
    Return the homogeneous transform for a single standard-DH link.

    Implements

    .. math::

        T = \\begin{bmatrix}
            c\\theta & -s\\theta\\,c\\alpha &  s\\theta\\,s\\alpha & a\\,c\\theta \\\\
            s\\theta &  c\\theta\\,c\\alpha & -c\\theta\\,s\\alpha & a\\,s\\theta \\\\
                  0  &           s\\alpha   &           c\\alpha   &       d      \\\\
                  0  &              0       &              0       &       1      \\\\
            \\end{bmatrix}

    Parameters
    ----------
    a, alpha, d, theta
        The four DH parameters (link length, link twist, link offset,
        joint angle).

    Returns
    -------
    T
        The :math:`4 \\times 4` homogeneous transform expressed as a
        plain ``numpy`` array (no ``spatialmath`` overhead).
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [ 0,       sa,       ca,      d],
        [ 0,        0,        0,      1],
    ])


def dh_link_SE3(a: float, alpha: float, d: float, theta: float) -> SE3:
    """Return :func:`dh_link` wrapped in an :class:`spatialmath.SE3` object."""
    return SE3(dh_link(a, alpha, d, theta), check=False)


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def kr6_frames(q: ArrayLike) -> list[SE3]:
    """
    Compute every DH frame along the kinematic chain.

    Parameters
    ----------
    q
        A length-6 vector of joint angles, in radians.

    Returns
    -------
    frames
        A list of seven :class:`SE3` objects, ``[T_0, T_0^1, …, T_0^6]``,
        representing the base frame followed by each of the six joint
        frames expressed in the base frame.
    """
    q = np.asarray(q, dtype=float).flatten()
    if q.shape != (6,):
        raise ValueError(f"expected length-6 joint vector, got shape {q.shape}")

    T = SE3()
    frames: list[SE3] = [T]
    for i in range(6):
        a, alpha, d, theta_off = KR6_SDH[i]
        T = T * dh_link_SE3(a, alpha, d, theta_off + q[i])
        frames.append(T)
    return frames


def kr6_fk(q: ArrayLike) -> SE3:
    """
    Compute the flange pose :math:`T_0^6` for a given joint vector.

    This is a thin convenience wrapper around :func:`kr6_frames` that
    returns only the final transform.
    """
    return kr6_frames(q)[-1]


# ---------------------------------------------------------------------------
# Joint-limit utilities
# ---------------------------------------------------------------------------

def clip_to_limits(q: ArrayLike) -> NDArray[np.float64]:
    """Saturate ``q`` element-wise to ``[Q_MIN, Q_MAX]``."""
    return np.clip(np.asarray(q, dtype=float), Q_MIN, Q_MAX)


def in_limits(q: ArrayLike, tol: float = 1e-6) -> bool:
    """Return ``True`` if every joint of ``q`` lies within (limits ± ``tol``)."""
    q = np.asarray(q, dtype=float)
    return bool(np.all(q >= Q_MIN - tol) and np.all(q <= Q_MAX + tol))


# ---------------------------------------------------------------------------
# Self-check
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("DH table:")
    print(f"{'i':>3} {'a [m]':>10} {'alpha [rad]':>14} {'d [m]':>10} {'theta_off [rad]':>18}")
    for i, row in enumerate(KR6_SDH):
        a, al, d, th = row
        print(f"{i + 1:>3} {a:>+10.4f} {al:>+14.4f} {d:>+10.4f} {th:>+18.4f}")

    print(f"\nFK at q = 0          : flange at {np.round(kr6_fk(Q_ZERO).t, 4)}")
    print(f"FK at q = q_ready    : flange at {np.round(kr6_fk(Q_READY).t, 4)}")

    p_wc = kr6_frames(Q_ZERO)[5].t
    print(
        f"\nWrist-centre reach at q = 0: "
        f"{np.linalg.norm(p_wc[:2]):.4f} m  (manufacturer spec: 0.901 m)"
    )
