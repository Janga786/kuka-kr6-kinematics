"""
Run every demo driver in order, regenerating the full set of figures
and animations under :file:`figures/` and :file:`animations/`.

Usage::

    python scripts/run_all.py

Each driver is executed in its own ``runpy`` namespace so that failures
in one driver don't pollute global state for the next.
"""

from __future__ import annotations

import runpy
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DRIVERS: list[str] = [
    "robot_spec.py",
    "dh_frames.py",
    "forward_kinematics.py",
    "joint_trajectories.py",
    "jacobian_singularities.py",
    "inverse_kinematics.py",
    "task_space_trajectory.py",
    "urdf_demo.py",
]


def main() -> None:
    overall_t0 = time.perf_counter()
    for filename in DRIVERS:
        print()
        print("#" * 76)
        print(f"# Running scripts/{filename}")
        print("#" * 76)
        t0 = time.perf_counter()
        runpy.run_path(str(HERE / filename), run_name="__main__")
        print(f"\n[ scripts/{filename} : {time.perf_counter() - t0:.1f} s ]")

    print()
    print("=" * 76)
    print(f"all drivers regenerated in {time.perf_counter() - overall_t0:.1f} s")
    print("=" * 76)


if __name__ == "__main__":
    main()
