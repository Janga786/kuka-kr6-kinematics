"""
Forward kinematics demo
=======================

Print every per-link transform :math:`T_{i-1}^{\\,i}` and the composite
:math:`T_0^6` for two reference configurations, and save a comparison
figure of the corresponding stick-figure poses.

Output
------
- ``figures/forward_kinematics_configs.png``

Run with::

    python scripts/forward_kinematics.py
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _paths import FIG_DIR

from kr6_kinematics import (
    KR6_SDH,
    Q_READY,
    Q_ZERO,
    dh_link,
    draw_stick_figure,
    kr6_fk,
    setup_3d_axes,
)


def print_link_transforms(q, label: str) -> None:
    print(f"\nLink transforms at {label}:")
    for i in range(6):
        a, alpha, d, theta_off = KR6_SDH[i]
        theta = theta_off + q[i]
        T = dh_link(a, alpha, d, theta)
        print(f"\n  T_{i}^{i + 1}    (q{i + 1} = {q[i]:+.4f}):")
        print(np.array2string(T, precision=4, suppress_small=True, prefix="  "))


def print_fk(q, label: str) -> None:
    T = kr6_fk(q)
    print(f"\n  T_0^6 at {label}:")
    print(np.array2string(T.A, precision=4, suppress_small=True, prefix="    "))
    print(f"    flange position: {np.round(T.t, 4)}")


def plot_fk_configs() -> None:
    fig = plt.figure(figsize=(13, 6))
    for idx, (q, title) in enumerate([
        (Q_ZERO,  "q = 0  (URDF zero)"),
        (Q_READY, "q = q_ready"),
    ]):
        ax = fig.add_subplot(1, 2, idx + 1, projection="3d")
        setup_3d_axes(ax, lim=1.1, title=title)
        draw_stick_figure(ax, q, show_frames=False)
    plt.tight_layout()
    out = FIG_DIR / "forward_kinematics_configs.png"
    plt.savefig(out, dpi=140, bbox_inches="tight")
    plt.close()
    print(f"\nsaved {out}")


def main() -> None:
    print("=" * 68)
    print("  Forward kinematics demo")
    print("=" * 68)

    print_link_transforms(Q_ZERO, "q_zero")
    print_fk(Q_ZERO, "q_zero")

    print_link_transforms(Q_READY, "q_ready")
    print_fk(Q_READY, "q_ready")

    plot_fk_configs()


if __name__ == "__main__":
    main()
