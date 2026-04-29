"""
Closed-form inverse kinematics demo
===================================

Verify the Pieper-decoupled IK solver via:

1. **Round-trip self-consistency.**  ``q_true → FK → IK → FK`` is run
   on a hand-picked configuration and on 200 random configurations
   (with :math:`q_5` away from 0 to avoid the wrist singularity); the
   maximum position and rotation errors are reported.
2. **Branch enumeration.**  All four position-IK branches (elbow up /
   down × shoulder front / back) are computed for a single target and
   plotted side-by-side, demonstrating that each is a kinematically
   valid solution to the same task pose.

Outputs
-------
- ``figures/ik_round_trip.png``
- ``figures/ik_branches.png``
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _paths import FIG_DIR

from kr6_kinematics import (
    Q_MAX,
    Q_MIN,
    Q_READY,
    draw_frame,
    draw_stick_figure,
    ik_position,
    kr6_fk,
    setup_3d_axes,
)

# ---------------------------------------------------------------------------
# Round-trip helpers
# ---------------------------------------------------------------------------

def round_trip(
    q_true,
    elbow_up: bool = True,
    shoulder_front: bool = True,
) -> tuple[np.ndarray, float, float]:
    """FK → IK → FK on ``q_true``.  Returns ``(q_sol, pos_err_m, rot_err)``."""
    T = kr6_fk(q_true)
    q_sol = ik_position(T, elbow_up=elbow_up, shoulder_front=shoulder_front)
    T_sol = kr6_fk(q_sol)
    pos_err = float(np.linalg.norm(T_sol.t - T.t))
    rot_err = float(np.linalg.norm(T_sol.R - T.R))
    return q_sol, pos_err, rot_err


# ---------------------------------------------------------------------------
# Single-target IK + side-by-side figure
# ---------------------------------------------------------------------------

def solve_and_plot() -> None:
    q_true = Q_READY.copy()
    q_true[0] = 0.3
    q_true[1] = -0.5
    q_true[3] = 0.2
    q_true[5] = 0.1

    T_target = kr6_fk(q_true)
    print(f"target flange position : {np.round(T_target.t, 4)}")

    q_sol, pos_err, rot_err = round_trip(q_true, elbow_up=True, shoulder_front=True)
    print("\nIK solution (elbow up, shoulder front):")
    print(f"  q_true       = {np.round(q_true, 4)}")
    print(f"  q_sol        = {np.round(q_sol, 4)}")
    print(f"  position err = {pos_err * 1000:.4f} mm")
    print(f"  rotation err = {rot_err:.2e}")

    fig = plt.figure(figsize=(13, 6))
    for idx, (q, label) in enumerate([
        (q_true, "target  (FK)"),
        (q_sol,  "IK solution"),
    ]):
        ax = fig.add_subplot(1, 2, idx + 1, projection="3d")
        setup_3d_axes(ax, lim=1.1, title=label)
        draw_stick_figure(ax, q, show_frames=False)
        draw_frame(ax, kr6_fk(q), length=0.08)
    plt.tight_layout()
    out = FIG_DIR / "ik_round_trip.png"
    plt.savefig(out, dpi=140, bbox_inches="tight")
    plt.close()
    print(f"saved {out}")


# ---------------------------------------------------------------------------
# Branch enumeration
# ---------------------------------------------------------------------------

def plot_branches() -> None:
    """Show all four position-IK branches for a single flange target."""
    q_true = np.array([0.2, -0.8, 0.5, 0.0, 0.6, 0.0])
    T_target = kr6_fk(q_true)

    fig = plt.figure(figsize=(14, 10))
    idx = 1
    for elbow in (True, False):
        for shoulder in (True, False):
            ax = fig.add_subplot(2, 2, idx, projection="3d")
            label = (
                f"elbow {'up' if elbow else 'down'}, "
                f"shoulder {'front' if shoulder else 'back'}"
            )
            setup_3d_axes(ax, lim=1.2, title=label)
            try:
                q = ik_position(T_target, elbow_up=elbow, shoulder_front=shoulder)
                draw_stick_figure(ax, q, show_frames=False)
                draw_frame(ax, kr6_fk(q), length=0.08)
            except Exception as e:  # pragma: no cover - defensive
                ax.text2D(0.2, 0.5, str(e), transform=ax.transAxes)
            idx += 1
    plt.tight_layout()
    out = FIG_DIR / "ik_branches.png"
    plt.savefig(out, dpi=140, bbox_inches="tight")
    plt.close()
    print(f"saved {out}")


# ---------------------------------------------------------------------------
# Statistical round-trip over random configurations
# ---------------------------------------------------------------------------

def scatter_round_trip(n_samples: int = 200, seed: int = 1) -> None:
    """Run FK → IK → FK on n random configurations and report worst-case error."""
    rng = np.random.default_rng(seed)
    pos_errs, rot_errs = [], []
    for _ in range(n_samples):
        q = rng.uniform(np.maximum(Q_MIN, -2.0), np.minimum(Q_MAX, +2.0), 6)
        # keep |q5| > 0.3 so we never sample the wrist singularity
        q[4] = abs(q[4]) + 0.3
        try:
            _, pe, re = round_trip(q, elbow_up=q[2] >= 0, shoulder_front=True)
            pos_errs.append(pe)
            rot_errs.append(re)
        except Exception:
            continue

    pos_errs = np.asarray(pos_errs)
    rot_errs = np.asarray(rot_errs)
    print(f"\nRound trip over {len(pos_errs)} random configurations:")
    print(f"  max position error  = {pos_errs.max() * 1000:.4f} mm")
    print(f"  max rotation error  = {rot_errs.max():.2e}")
    print(f"  rms position error  = {np.sqrt(np.mean(pos_errs ** 2)) * 1000:.4f} mm")
    print(f"  rms rotation error  = {np.sqrt(np.mean(rot_errs ** 2)):.2e}")


def main() -> None:
    print("=" * 68)
    print("  Closed-form inverse kinematics (Pieper)")
    print("=" * 68)
    solve_and_plot()
    plot_branches()
    scatter_round_trip()
    print("done.")


if __name__ == "__main__":
    main()
