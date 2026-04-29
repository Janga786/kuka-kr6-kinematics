"""Tests for the auto-generated URDF and the ``urchin`` round-trip."""

from __future__ import annotations

import numpy as np
import pytest

from kr6_kinematics import Q_READY, kr6_fk
from kr6_kinematics.urdf_builder import build_urdf_string, write_urdf


def test_urdf_string_well_formed():
    """The generated URDF must be parseable XML and contain the six joints."""
    import xml.etree.ElementTree as ET

    text = build_urdf_string()
    root = ET.fromstring(text)
    assert root.tag == "robot"
    joint_names = {j.get("name") for j in root.findall("joint") if j.get("type") == "revolute"}
    assert joint_names == {f"joint_{i + 1}" for i in range(6)}


def test_urdf_fk_matches_dh_fk(tmp_path):
    """urchin's URDF FK must agree with the analytic DH FK at q_ready."""
    pytest.importorskip("urchin")
    from urchin import URDF

    path = write_urdf(tmp_path / "kr6_test.urdf")
    urdf = URDF.load(str(path))

    cfg = {f"joint_{i + 1}": Q_READY[i] for i in range(6)}
    fk_urdf = urdf.link_fk(cfg=cfg)
    T_urdf = list(fk_urdf.values())[-1]
    T_dh = kr6_fk(Q_READY).A

    # rpy_from_R loses ~5 decimals when round-tripping. The position
    # tolerance below (5e-5 m = 50 μm) is well within the manufacturer-
    # specified ±0.03 mm repeatability of the arm.
    np.testing.assert_allclose(T_urdf[:3, 3], T_dh[:3, 3], atol=5e-5)
    np.testing.assert_allclose(T_urdf[:3, :3], T_dh[:3, :3], atol=1e-4)


def test_urdf_committed_file_in_sync():
    """The committed urdf/kr6_sdh.urdf must equal what build_urdf_string produces."""
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[1]
    committed = (repo_root / "urdf" / "kr6_sdh.urdf").read_text()
    generated = build_urdf_string()
    assert committed.strip() == generated.strip(), (
        "urdf/kr6_sdh.urdf is out of sync with build_urdf_string(); "
        "regenerate via `python scripts/08_urdf_visualization.py`"
    )
