"""
Problem 2 — DH parameter table and link frames
==============================================

Print the standard-DH table used throughout the project and render a
3-D figure showing every DH frame at the URDF zero configuration.

Output
------
- ``figures/problem2_frames.png``

Run with::

    python scripts/02_dh_frames.py
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from kr6_kinematics import KR6_SDH, Q_ZERO, kr6_frames, setup_3d_axes, draw_stick_figure
from _paths import FIG_DIR


def print_dh_table() -> None:
    print("  i   a_i [m]   alpha_i [rad]    d_i [m]   theta_offset [rad]")
    print("  --  --------  --------------  --------  ---------------------")
    for i, row in enumerate(KR6_SDH):
        a, al, d, th = row
        print(f"  {i + 1:>2}  {a:+8.4f}    {al:+10.4f}     {d:+8.4f}        {th:+8.4f}")


def plot_frames_at_zero() -> None:
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    setup_3d_axes(ax, lim=1.1, title="Problem 2: DH frames at q = 0")
    draw_stick_figure(ax, Q_ZERO, show_frames=True, frame_len=0.10)

    for i, T in enumerate(kr6_frames(Q_ZERO)):
        p = T.t
        ax.text(
            p[0] + 0.02, p[1] + 0.02, p[2] + 0.02,
            f"frame {i}", fontsize=8, color="navy",
        )

    out = FIG_DIR / "problem2_frames.png"
    plt.tight_layout()
    plt.savefig(out, dpi=140, bbox_inches="tight")
    plt.close()
    print(f"saved {out}")


def main() -> None:
    print("=" * 68)
    print("  Problem 2 — DH parameter table and link frames")
    print("=" * 68)
    print()
    print_dh_table()
    print()
    plot_frames_at_zero()


if __name__ == "__main__":
    main()
