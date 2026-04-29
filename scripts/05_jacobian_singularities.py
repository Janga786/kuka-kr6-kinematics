"""
Problem 5 — Jacobian and singularity analysis
=============================================

Compute and verify the 6×6 geometric Jacobian, sweep the wrist pitch
:math:`q_5` through zero to expose the *wrist singularity*, and render
an animation that overlays linear- and angular-velocity arrows on the
moving flange.

Verification
------------
The analytic Jacobian is checked column-by-column against a central-
finite-difference approximation of the same quantity, evaluated at 50
random configurations.  The reported max-norm error should be of order
``1e-7`` on a typical CPU.

Outputs
-------
- ``figures/problem5_singular_sweep.png``
- ``animations/problem5_velocity.gif``
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _paths import ANIM_DIR, FIG_DIR

from kr6_kinematics import (
    Q_READY,
    animate_joint_trajectory,
    jacobian,
    kr6_fk,
    quintic_vec,
)

# ---------------------------------------------------------------------------
# Analytic vs. finite-difference Jacobian verification
# ---------------------------------------------------------------------------

def fd_jacobian_check(num_samples: int = 50, seed: int = 7) -> float:
    """Return the maximum element-wise error between J(q) and FD J over random q."""
    rng = np.random.default_rng(seed)
    worst = 0.0
    h = 1e-6

    for _ in range(num_samples):
        q = rng.uniform(-1.2, 1.2, 6)
        Ja = jacobian(q)
        J_fd = np.zeros((6, 6))

        for j in range(6):
            qp = q.copy()
            qm = q.copy()
            qp[j] += h
            qm[j] -= h
            Tp = kr6_fk(qp)
            Tm = kr6_fk(qm)

            dp = (Tp.t - Tm.t) / (2 * h)

            # Skew-symmetric part of (Tp.R @ Tm.R.T) approximates dR/dq.
            R_err = Tp.R @ Tm.R.T
            w = 0.5 * np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ]) / (2 * h)
            J_fd[:, j] = np.concatenate([dp, w])

        worst = max(worst, float(np.max(np.abs(Ja - J_fd))))
    return worst


# ---------------------------------------------------------------------------
# Static reports
# ---------------------------------------------------------------------------

def print_jacobian_at_ready() -> None:
    J = jacobian(Q_READY)
    print("J(q_ready):")
    print(np.array2string(J, precision=4, suppress_small=True, prefix="  "))
    sv = np.linalg.svd(J, compute_uv=False)
    print(f"\n  singular values   = {np.round(sv, 4).tolist()}")
    print(f"  det(J)            = {np.linalg.det(J):+.6f}")
    print(f"  manipulability w  = {np.sqrt(np.abs(np.linalg.det(J @ J.T))):.6f}")


def singular_sweep_figure() -> None:
    """Sweep q5 toward 0 and plot the smallest singular value vs manipulability."""
    q5_vals = np.linspace(Q_READY[4], 0.0, 120)
    sigmin, manip, dets = [], [], []
    for v in q5_vals:
        q = Q_READY.copy()
        q[4] = v
        J = jacobian(q)
        sv = np.linalg.svd(J, compute_uv=False)
        sigmin.append(sv[-1])
        manip.append(np.sqrt(abs(np.linalg.det(J @ J.T))))
        dets.append(abs(np.linalg.det(J)))

    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axes[0].plot(q5_vals, sigmin, "b-", linewidth=2)
    axes[0].set_ylabel(r"$\sigma_{\min}(J)$")
    axes[0].set_title(r"Wrist singularity:  $\sigma_{\min}, w \to 0$ as $q_5 \to 0$")
    axes[0].grid(alpha=0.3)

    axes[1].plot(q5_vals, manip, "g-",  linewidth=2.0, label=r"$\sqrt{|\det J J^\top|}$")
    axes[1].plot(q5_vals, dets,  "r--", linewidth=1.3, label=r"$|\det J|$")
    axes[1].set_ylabel("manipulability")
    axes[1].set_xlabel(r"$q_5$  [rad]")
    axes[1].grid(alpha=0.3)
    axes[1].legend(loc="upper right")

    plt.tight_layout()
    out = FIG_DIR / "problem5_singular_sweep.png"
    plt.savefig(out, dpi=140)
    plt.close()
    print(f"saved {out}")


# ---------------------------------------------------------------------------
# Velocity-arrow animation
# ---------------------------------------------------------------------------

def velocity_arrows_animation() -> None:
    """Animate a small motion with the flange linear- and angular-velocity arrows."""
    q0 = Q_READY.copy()
    qf = Q_READY.copy()
    qf[0] += 0.4
    qf[1] += 0.2
    qf[2] -= 0.2
    qf[4] += 0.3
    _, Q, Qd, _ = quintic_vec(0, 2.0, q0, qf, num_points=80)

    def draw_velocity(ax, q, idx):
        J = jacobian(q)
        v = J[:3] @ Qd[idx]
        w = J[3:] @ Qd[idx]
        p = kr6_fk(q).t

        ax.quiver(p[0], p[1], p[2], v[0], v[1], v[2],
                  color="magenta", linewidth=2, length=0.4, normalize=False)
        ax.quiver(p[0], p[1], p[2], 0.2 * w[0], 0.2 * w[1], 0.2 * w[2],
                  color="cyan", linewidth=2)

        ax.text2D(
            0.02, 0.96,
            f"|v| = {np.linalg.norm(v):.2f} m/s\n"
            f"|ω| = {np.linalg.norm(w):.2f} rad/s",
            transform=ax.transAxes, fontsize=9, verticalalignment="top",
            bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"),
        )

    gif = ANIM_DIR / "problem5_velocity.gif"
    animate_joint_trajectory(
        Q,
        title="Flange v (magenta) and ω (cyan)",
        filename=str(gif),
        fps=18,
        lim=1.1,
        extra_draw=draw_velocity,
    )


def main() -> None:
    print("=" * 68)
    print("  Problem 5 — Jacobian and singularity analysis")
    print("=" * 68)

    print_jacobian_at_ready()
    print()

    err = fd_jacobian_check()
    print(f"max |J_analytic − J_fd| over 50 random configs:  {err:.3e}")
    print()

    singular_sweep_figure()
    velocity_arrows_animation()
    print("done.")


if __name__ == "__main__":
    main()
