"""
Resolve the canonical ``figures/``, ``animations/`` and ``urdf/``
directories relative to the repository root, and ensure they exist.

Every driver script imports from this module instead of hard-coding
its own ``..`` path arithmetic.
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT: Path = Path(__file__).resolve().parents[1]
FIG_DIR:    Path = REPO_ROOT / "figures"
ANIM_DIR:   Path = REPO_ROOT / "animations"
URDF_DIR:   Path = REPO_ROOT / "urdf"

for _d in (FIG_DIR, ANIM_DIR, URDF_DIR):
    _d.mkdir(parents=True, exist_ok=True)
