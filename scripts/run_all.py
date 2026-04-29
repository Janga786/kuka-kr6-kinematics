"""
Run every problem driver in order, regenerating the full set of
figures and animations under :file:`figures/` and :file:`animations/`.

Usage::

    python scripts/run_all.py

Each driver is executed in its own ``runpy`` namespace so that failures
in one driver don't pollute global state for the next.
"""

from __future__ import annotations

import runpy
import time
from pathlib import Path
from typing import List


HERE = Path(__file__).resolve().parent
PROBLEMS: List[str] = [
    "01_robot_id.py",
    "02_dh_frames.py",
    "03_forward_kinematics.py",
    "04_joint_trajectories.py",
    "05_jacobian_singularities.py",
    "06_inverse_kinematics.py",
    "07_task_space_trajectory.py",
    "08_urdf_visualization.py",
]


def main() -> None:
    overall_t0 = time.perf_counter()
    for filename in PROBLEMS:
        print()
        print("#" * 76)
        print(f"# Running scripts/{filename}")
        print("#" * 76)
        t0 = time.perf_counter()
        runpy.run_path(str(HERE / filename), run_name="__main__")
        print(f"\n[ scripts/{filename} : {time.perf_counter() - t0:.1f} s ]")

    print()
    print("=" * 76)
    print(f"all problems regenerated in {time.perf_counter() - overall_t0:.1f} s")
    print("=" * 76)


if __name__ == "__main__":
    main()
