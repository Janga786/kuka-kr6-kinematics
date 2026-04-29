"""
Joint-space and task-space trajectory primitives
================================================

This module implements two staple polynomial / piece-wise blends used
throughout the project:

- **Quintic** (5th-order polynomial) interpolation that satisfies
  arbitrary boundary position, velocity *and* acceleration. Coefficients
  are obtained by solving the 6×6 boundary-condition system in closed
  form.
- **LSPB** (linear segment with parabolic blends) — the classical
  trapezoidal velocity profile commonly used as a baseline against
  quintics.

A vector helper :func:`quintic_vec` applies an independent quintic to
each component of a multidimensional waypoint, with zero boundary
velocity and acceleration.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

# ---------------------------------------------------------------------------
# Quintic (5th-order) polynomial trajectory
# ---------------------------------------------------------------------------

def quintic(
    t0: float,
    tf: float,
    q0: float,
    qf: float,
    qd0: float = 0.0,
    qdf: float = 0.0,
    qdd0: float = 0.0,
    qddf: float = 0.0,
    num_points: int = 200,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """
    Generate a quintic polynomial scalar trajectory.

    Solves the linear system

    .. math::

        \\begin{bmatrix}
        1 & t_0 & t_0^2 & t_0^3 & t_0^4 & t_0^5 \\\\
        0 & 1   & 2t_0  & 3t_0^2 & 4t_0^3 & 5t_0^4 \\\\
        0 & 0   & 2     & 6t_0   & 12t_0^2 & 20t_0^3 \\\\
        1 & t_f & t_f^2 & t_f^3 & t_f^4 & t_f^5 \\\\
        0 & 1   & 2t_f  & 3t_f^2 & 4t_f^3 & 5t_f^4 \\\\
        0 & 0   & 2     & 6t_f   & 12t_f^2 & 20t_f^3 \\\\
        \\end{bmatrix}
        \\begin{bmatrix}a_0\\\\a_1\\\\a_2\\\\a_3\\\\a_4\\\\a_5\\end{bmatrix}
        =
        \\begin{bmatrix}q_0\\\\\\dot q_0\\\\\\ddot q_0\\\\q_f\\\\\\dot q_f\\\\\\ddot q_f
        \\end{bmatrix}

    for the polynomial coefficients :math:`a_k`, then evaluates
    :math:`q,\\dot q,\\ddot q` on a uniform time grid.

    Returns
    -------
    t, q, qd, qdd
        Time grid and the position, velocity and acceleration profiles.
    """
    M = np.array([
        [1, t0, t0 ** 2,   t0 ** 3,    t0 ** 4,    t0 ** 5   ],
        [0,  1, 2 * t0,    3 * t0 ** 2, 4 * t0 ** 3, 5 * t0 ** 4],
        [0,  0,  2,         6 * t0,     12 * t0 ** 2, 20 * t0 ** 3],
        [1, tf, tf ** 2,   tf ** 3,    tf ** 4,    tf ** 5   ],
        [0,  1, 2 * tf,    3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
        [0,  0,  2,         6 * tf,     12 * tf ** 2, 20 * tf ** 3],
    ])
    b = np.array([q0, qd0, qdd0, qf, qdf, qddf])
    a0, a1, a2, a3, a4, a5 = np.linalg.solve(M, b)

    t = np.linspace(t0, tf, num_points)
    q   = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5
    qd  = a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4
    qdd = 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3
    return t, q, qd, qdd


def quintic_vec(
    t0: float,
    tf: float,
    q0: ArrayLike,
    qf: ArrayLike,
    num_points: int = 200,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """
    Component-wise quintic interpolation between two vector waypoints.

    Boundary velocities and accelerations are zero on every channel.

    Returns
    -------
    t, Q, Qd, Qdd
        Time grid (length ``num_points``) and three (``num_points`` × *n*)
        arrays of position, velocity and acceleration, where *n* is the
        dimensionality of ``q0``.
    """
    q0 = np.atleast_1d(q0).astype(float)
    qf = np.atleast_1d(qf).astype(float)
    n = q0.size

    t = np.linspace(t0, tf, num_points)
    Q   = np.zeros((num_points, n))
    Qd  = np.zeros((num_points, n))
    Qdd = np.zeros((num_points, n))
    for j in range(n):
        _, qj, qdj, qddj = quintic(t0, tf, q0[j], qf[j], 0, 0, 0, 0, num_points)
        Q[:, j]   = qj
        Qd[:, j]  = qdj
        Qdd[:, j] = qddj
    return t, Q, Qd, Qdd


# ---------------------------------------------------------------------------
# LSPB (linear segment with parabolic blends)
# ---------------------------------------------------------------------------

def lspb(
    t0: float,
    tf: float,
    q0: float,
    qf: float,
    v: float,
    num_points: int = 200,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """
    Generate an LSPB (trapezoidal-velocity) trajectory.

    The cruise velocity ``v`` must satisfy
    :math:`|h|/T < |v| \\le 2|h|/T` where :math:`h = q_f - q_0` and
    :math:`T = t_f - t_0`; otherwise no feasible blend time exists and a
    :class:`ValueError` is raised.

    Returns
    -------
    t, q, qd, qdd
        Time grid and position, velocity and acceleration profiles.
    """
    T = tf - t0
    h = qf - q0
    tb = T - abs(h) / abs(v)
    if tb <= 0 or tb >= T / 2:
        raise ValueError(
            f"infeasible blend time tb={tb:.3f}; "
            f"choose v in ({abs(h) / T:.3f}, {2 * abs(h) / T:.3f}] for these endpoints"
        )
    a = v / tb

    t = np.linspace(t0, tf, num_points)
    q   = np.zeros_like(t)
    qd  = np.zeros_like(t)
    qdd = np.zeros_like(t)
    for i, ti in enumerate(t):
        tp = ti - t0
        if tp <= tb:
            # Acceleration ramp-up.
            q[i]   = q0 + 0.5 * a * tp ** 2
            qd[i]  = a * tp
            qdd[i] = a
        elif tp <= T - tb:
            # Cruise at constant velocity.
            q[i]   = q0 + 0.5 * a * tb ** 2 + v * (tp - tb)
            qd[i]  = v
            qdd[i] = 0.0
        else:
            # Deceleration ramp-down.
            td = T - tp
            q[i]   = qf - 0.5 * a * td ** 2
            qd[i]  = a * td
            qdd[i] = -a
    return t, q, qd, qdd


# ---------------------------------------------------------------------------
# Self-check
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    t, q, qd, qdd = quintic(0, 1, 0, 1)
    print(f"quintic: q(0)={q[0]:+.3f}  q(1)={q[-1]:+.3f}  "
          f"max|qdd|={np.max(np.abs(qdd)):.3f}")

    t, q, qd, qdd = lspb(0, 1, 0, 1, 1.25)
    print(f"lspb   : q(0)={q[0]:+.3f}  q(1)={q[-1]:+.3f}  "
          f"max|qdd|={np.max(np.abs(qdd)):.3f}")
