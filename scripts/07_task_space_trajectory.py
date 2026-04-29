"""
Problem 7 — Task-space trajectory with position + velocity IK
=============================================================

Generate a straight-line task-space path (with quintic time scaling on
each Cartesian component) starting near the ready pose, then for every
sampled task pose:

* solve **inverse position kinematics** to obtain :math:`q_k`, and
* solve **inverse velocity kinematics**
  :math:`\\dot q_k = J(q_k)^{-1} \\xi_k` to obtain joint velocities.

The recovered joint velocities are validated against a simple
finite-difference estimate :math:`\\dot q_k \\approx \\Delta q / \\Delta t`,
and an animated GIF visualises the resulting motion.

Outputs
-------
- ``figures/problem7_joint_trajectories.png``
- ``figures/problem7_joint_velocities.png``
- ``figures/problem7_ee_path.png``
- ``figures/problem7_Q.npy`` ``problem7_Qd.npy``  (numerical artefacts
  consumed by Problem 8)
- ``animations/problem7_animation.gif``
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _paths import ANIM_DIR, FIG_DIR
from spatialmath import SE3

from kr6_kinematics import (
    Q_READY,
    animate_joint_trajectory,
    draw_stick_figure,
    ik_position,
    jacobian,
    kr6_fk,
    quintic,
    setup_3d_axes,
)

N_POINTS = 80
DURATION = 2.0


# ---------------------------------------------------------------------------
# Path generation and IK tracking
# ---------------------------------------------------------------------------

def task_trajectory() -> tuple[np.ndarray, list[SE3], np.ndarray, np.ndarray, np.ndarray]:
    """Generate a straight-line Cartesian trajectory with quintic time scaling."""
    T_ready = kr6_fk(Q_READY)
    R_fixed = T_ready.R

    p_start = T_ready.t + np.array([-0.15, +0.05, +0.05])
    p_end   = T_ready.t + np.array([-0.05, -0.08, -0.05])

    t,  xs, vxs, _ = quintic(0, DURATION, p_start[0], p_end[0], 0, 0, 0, 0, N_POINTS)
    _,  ys, vys, _ = quintic(0, DURATION, p_start[1], p_end[1], 0, 0, 0, 0, N_POINTS)
    _,  zs, vzs, _ = quintic(0, DURATION, p_start[2], p_end[2], 0, 0, 0, 0, N_POINTS)

    poses = [SE3.Rt(R_fixed, np.array([xs[k], ys[k], zs[k]])) for k in range(N_POINTS)]
    V = np.zeros((N_POINTS, 6))
    V[:, 0], V[:, 1], V[:, 2] = vxs, vys, vzs   # angular velocity is zero
    return t, poses, V, p_start, p_end


def ik_track(poses: list[SE3], V: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Run position IK and velocity IK at every sample of the path."""
    N = len(poses)
    Q = np.zeros((N, 6))
    Qd = np.zeros((N, 6))
    for k, T in enumerate(poses):
        Q[k] = ik_position(T, elbow_up=True, shoulder_front=True)
        J = jacobian(Q[k])
        Qd[k] = np.linalg.solve(J, V[k])
    return Q, Qd


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_joint_trajectories(t: np.ndarray, Q: np.ndarray) -> None:
    fig, axes = plt.subplots(6, 1, figsize=(8, 10), sharex=True)
    fig.suptitle("Problem 7: joint positions reconstructed from position IK")
    for j in range(6):
        axes[j].plot(t, Q[:, j], "-", linewidth=2)
        axes[j].set_ylabel(f"q{j + 1} [rad]")
        axes[j].grid(alpha=0.3)
    axes[-1].set_xlabel("t [s]")
    plt.tight_layout()
    out = FIG_DIR / "problem7_joint_trajectories.png"
    plt.savefig(out, dpi=140)
    plt.close()
    print(f"saved {out}")


def plot_joint_velocities(t: np.ndarray, Qd: np.ndarray, Q: np.ndarray) -> None:
    """Compare J⁻¹·v against a finite-difference estimate of dq/dt."""
    dt = t[1] - t[0]
    Qd_fd = np.gradient(Q, dt, axis=0)

    fig, axes = plt.subplots(6, 1, figsize=(8, 10), sharex=True)
    fig.suptitle(r"Problem 7: joint velocities  $\dot q = J^{-1} v_{\mathrm{task}}$")
    for j in range(6):
        axes[j].plot(t, Qd[:, j], "-", linewidth=2, label=r"$J^{-1} v$")
        axes[j].plot(t, Qd_fd[:, j], "--", linewidth=1, label="finite difference")
        axes[j].set_ylabel(f"qd{j + 1}")
        axes[j].grid(alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=9)
    axes[-1].set_xlabel("t [s]")
    plt.tight_layout()
    out = FIG_DIR / "problem7_joint_velocities.png"
    plt.savefig(out, dpi=140)
    plt.close()
    print(f"saved {out}")

    worst = float(np.max(np.abs(Qd - Qd_fd)))
    rms = float(np.sqrt(np.mean((Qd - Qd_fd) ** 2)))
    print(f"  velocity-IK self-check  : max err = {worst:.3e},  rms = {rms:.3e} rad/s")


def plot_ee_path(Q: np.ndarray, p_start: np.ndarray, p_end: np.ndarray) -> None:
    ee = np.array([kr6_fk(Q[k]).t for k in range(Q.shape[0])])

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    setup_3d_axes(ax, lim=1.1, title="Problem 7 — end-effector path")
    draw_stick_figure(ax, Q[0], show_frames=False)
    draw_stick_figure(
        ax, Q[-1], show_frames=False,
        link_color="tab:orange", joint_color="tab:orange", lw=1.5,
    )
    ax.plot(ee[:, 0], ee[:, 1], ee[:, 2], "r-", linewidth=2, label="ee path")
    ax.scatter(*p_start, color="green", s=80, label="start")
    ax.scatter(*p_end,   color="red",   s=80, label="end")
    ax.legend()
    plt.tight_layout()
    out = FIG_DIR / "problem7_ee_path.png"
    plt.savefig(out, dpi=140)
    plt.close()
    print(f"saved {out}")


def main() -> None:
    print("=" * 68)
    print("  Problem 7 — Task-space trajectory + IK tracking")
    print("=" * 68)
    t, poses, V, p_start, p_end = task_trajectory()
    print(f"  {len(poses)} samples,  {np.round(p_start, 3)} → {np.round(p_end, 3)}")
    print(f"  peak |v_task| = {np.linalg.norm(V[:, :3], axis=1).max():.3f} m/s")

    Q, Qd = ik_track(poses, V)
    np.save(FIG_DIR / "problem7_Q.npy", Q)
    np.save(FIG_DIR / "problem7_Qd.npy", Qd)
    print(f"  peak |q̇|     = {np.linalg.norm(Qd, axis=1).max():.3f} rad/s")

    plot_joint_trajectories(t, Q)
    plot_joint_velocities(t, Qd, Q)
    plot_ee_path(Q, p_start, p_end)

    gif = ANIM_DIR / "problem7_animation.gif"
    animate_joint_trajectory(
        Q,
        title="Problem 7: task-space quintic trajectory",
        filename=str(gif),
        fps=20,
        lim=1.1,
        ee_trail=True,
    )
    print("done.")


if __name__ == "__main__":
    main()
