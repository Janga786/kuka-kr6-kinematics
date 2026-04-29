"""
Problem 4 — Joint-space quintic trajectories and stick-figure animation
=======================================================================

Demonstrate the quintic primitive on a single joint, render a six-panel
"shifted pose" figure that visualises the workspace effect of each
joint independently, and produce an out-and-back animation that sweeps
each joint through ``DQ`` radians from the ready pose.

Output
------
- ``figures/problem4_quintic_q1.png``
- ``figures/problem4_joints_panel.png``
- ``animations/problem4_motion.gif``
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from kr6_kinematics import (
    Q_READY,
    animate_joint_trajectory,
    draw_stick_figure,
    quintic,
    quintic_vec,
    setup_3d_axes,
)
from _paths import ANIM_DIR, FIG_DIR


# Per-joint sweep amplitude (radians) for the joint-isolation panel.
DQ = np.array([0.8, 0.6, 0.8, 0.6, 0.8, 0.6])


def quintic_profile_figure() -> None:
    """Plot q, q-dot, q-double-dot for a single quintic."""
    t, q, qd, qdd = quintic(0, 2.0, 0.0, 0.8, 0, 0, 0, 0, num_points=200)

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axes[0].plot(t, q,  linewidth=2)
    axes[0].set_ylabel(r"$q(t)$  [rad]")
    axes[0].set_title(r"Quintic profile for joint 1   (0 $\to$ 0.8 rad)")
    axes[1].plot(t, qd, linewidth=2)
    axes[1].set_ylabel(r"$\dot q(t)$  [rad/s]")
    axes[2].plot(t, qdd, linewidth=2)
    axes[2].set_ylabel(r"$\ddot q(t)$  [rad/s²]")
    axes[2].set_xlabel("t [s]")
    for a in axes:
        a.grid(alpha=0.3)
    plt.tight_layout()
    out = FIG_DIR / "problem4_quintic_q1.png"
    plt.savefig(out, dpi=140)
    plt.close()
    print(f"saved {out}")


def joints_panel_figure() -> None:
    """Six-up panel: each subplot shows q_ready ghosted alongside a single-joint sweep."""
    fig = plt.figure(figsize=(14, 7))
    for j in range(6):
        ax = fig.add_subplot(2, 3, j + 1, projection="3d")
        setup_3d_axes(ax, lim=1.1, title=f"sweep A{j + 1} by {DQ[j]:.2f} rad")

        # ghosted ready pose for reference
        draw_stick_figure(
            ax, Q_READY,
            show_frames=False,
            link_color="0.7", joint_color="0.7", lw=1,
        )
        # shifted pose
        q_shift = Q_READY.copy()
        q_shift[j] += DQ[j]
        draw_stick_figure(
            ax, q_shift,
            show_frames=False,
            link_color="tab:blue", joint_color="tab:orange", lw=2,
        )

    plt.tight_layout()
    out = FIG_DIR / "problem4_joints_panel.png"
    plt.savefig(out, dpi=140, bbox_inches="tight")
    plt.close()
    print(f"saved {out}")


def out_and_back_animation() -> None:
    """Sweep each joint out and back one at a time using quintics."""
    n_per = 50
    Q_list = []
    q_cur = Q_READY.copy()
    for j in range(6):
        qf = q_cur.copy()
        qf[j] += DQ[j]
        _, Qout, _, _ = quintic_vec(0, 1.0, q_cur, qf, num_points=n_per)
        _, Qback, _, _ = quintic_vec(0, 1.0, qf, q_cur, num_points=n_per)
        Q_list.append(Qout)
        Q_list.append(Qback)
    Q_full = np.concatenate(Q_list, axis=0)
    print(f"  animation frames: {Q_full.shape[0]}")

    gif = ANIM_DIR / "problem4_motion.gif"
    animate_joint_trajectory(
        Q_full,
        title="KR 6 — joint-at-a-time quintic sweep",
        filename=str(gif),
        fps=18,
        lim=1.1,
        ee_trail=True,
    )


def main() -> None:
    print("=" * 68)
    print("  Problem 4 — Joint-space quintic trajectories")
    print("=" * 68)
    quintic_profile_figure()
    joints_panel_figure()
    out_and_back_animation()
    print("done.")


if __name__ == "__main__":
    main()
