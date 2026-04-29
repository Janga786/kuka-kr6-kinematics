"""
Problem 1 — Robot identification
================================

Print the manufacturer-published specifications of the KUKA KR 6 R900
sixx, the joint-limit and joint-velocity vectors used throughout the
project, and a sanity-check on the kinematic reach computed from the
DH table against KUKA's published 0.901 m wrist-centre reach.

Run with::

    python scripts/01_robot_id.py
"""

from __future__ import annotations

import numpy as np

from kr6_kinematics import KR6_SDH, Q_MIN, Q_MAX, QD_MAX, Q_ZERO, kr6_frames


def main() -> None:
    print("=" * 68)
    print("  KUKA KR 6 R900 sixx  ·  Robot identification (Problem 1)")
    print("=" * 68)

    spec = [
        ("Manufacturer",      "KUKA Roboter GmbH (Augsburg, Germany)"),
        ("Model",             "KR 6 R900 sixx (KR AGILUS family)"),
        ("Number of axes",    "6 (all revolute)"),
        ("Reach",             "0.901 m to wrist centre  /  0.981 m to flange"),
        ("Payload",           "6 kg max  (3 kg rated for optimal cycle time)"),
        ("Arm mass",          "approx. 52 kg"),
        ("Repeatability",     "± 0.03 mm  (ISO 9283)"),
        ("Working envelope",  "2.85 m³"),
        ("Protection class",  "IP 54"),
        ("Controller",        "KR C4 compact"),
        ("Actuation",         "Brushless AC servos with motor brakes on all six axes;"),
        ("",                  "A5 / A6 driven via a toothed-belt 'in-line wrist'"),
        ("",                  "(axes 4-5-6 meet at a single point)."),
        ("Sensing",           "Rotary resolvers per axis"),
        ("Source",            "KUKA operating instructions manual (2015)"),
    ]
    print()
    for k, v in spec:
        print(f"  {k:<18s}  {v}")
    print()

    print("Joint limits (rad):")
    print(f"  q_min = {np.round(Q_MIN, 4).tolist()}")
    print(f"  q_max = {np.round(Q_MAX, 4).tolist()}")
    print()
    print("Joint limits (deg):")
    print(f"  q_min = {np.round(np.degrees(Q_MIN), 1).tolist()}")
    print(f"  q_max = {np.round(np.degrees(Q_MAX), 1).tolist()}")
    print()
    print("Joint-velocity limits (rad/s):")
    print(f"  qd_max = {np.round(QD_MAX, 3).tolist()}")
    print()

    print("Link constants extracted from the DH table:")
    print(f"  d1  (shoulder height)   = {KR6_SDH[0, 2]:.3f} m")
    print(f"  a1  (shoulder offset)   = {KR6_SDH[0, 0]:.3f} m")
    print(f"  a2  (upper arm length)  = {KR6_SDH[1, 0]:.3f} m")
    print(f"  a3  (elbow offset)      = {KR6_SDH[2, 0]:.3f} m")
    print(f"  d4  (forearm length)    = {KR6_SDH[3, 2]:.3f} m")
    print(f"  d6  (wrist-to-flange)   = {KR6_SDH[5, 2]:.3f} m")
    print()

    frames = kr6_frames(Q_ZERO)
    r_wc = float(np.linalg.norm(frames[5].t[:2]))
    r_flange = float(np.linalg.norm(frames[6].t[:2]))
    print("Reach check at q = 0:")
    print(f"  computed wrist-centre reach = {r_wc:.4f} m   "
          f"(KUKA spec: 0.901 m, error = {abs(r_wc - 0.901) * 1000:.3f} mm)")
    print(f"  computed flange reach       = {r_flange:.4f} m")


if __name__ == "__main__":
    main()
