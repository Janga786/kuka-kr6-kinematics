"""
URDF generator
==============

Construct a URDF whose link frames are *exactly* the DH frames defined
in :mod:`kr6_kinematics.dh`.

The trick is that a URDF revolute joint contributes the rotation
:math:`R_z(q)` *after* its ``<origin>`` transform — but a standard-DH
link transform interleaves the joint rotation with the link constants:

.. math::

    T_{i-1}^{\\,i}(\\theta_i) = R_z(\\theta_i)\\, T_z(d_i)\\,
                                T_x(a_i)\\, R_x(\\alpha_i)
                              \\;=\\;
                                \\underbrace{R_z(\\theta_{0,i})\\,T_z(d_i)}_{\\text{pre}}
                                \\; R_z(q_i) \\;
                                \\underbrace{T_x(a_i)\\, R_x(\\alpha_i)}_{\\text{post}}.

We therefore split each DH row into a *pre* factor (pushed onto the
``<origin>`` of the revolute joint) and a *post* factor (placed on a
trailing fixed joint). The resulting URDF is consumed by
``urchin.URDF`` / ``rviz`` / Gazebo with link frames that coincide
with the DH frames.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from .dh import KR6_SDH, Q_MAX, Q_MIN

# ---------------------------------------------------------------------------
# Elementary transforms used to build pre/post factors
# ---------------------------------------------------------------------------

def _Rz(angle: float) -> NDArray[np.float64]:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]])


def _Rx(angle: float) -> NDArray[np.float64]:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0,  0, 0],
                     [0, c, -s, 0],
                     [0, s,  c, 0],
                     [0, 0,  0, 1]])


def _Tz(d: float) -> NDArray[np.float64]:
    M = np.eye(4)
    M[2, 3] = d
    return M


def _Tx(a: float) -> NDArray[np.float64]:
    M = np.eye(4)
    M[0, 3] = a
    return M


def rpy_from_R(R: NDArray[np.float64]) -> tuple[float, float, float]:
    """Extract URDF-convention roll-pitch-yaw from a rotation matrix."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return float(roll), float(pitch), float(yaw)


# ---------------------------------------------------------------------------
# Visual catalogue — sized so each link looks like the real KR 6 arm
# ---------------------------------------------------------------------------

_LINK_VISUALS = {
    "base_link": [
        # (xyz, rpy, length, radius, material)
        ("0 0 0.025", "0 0 0", 0.05, 0.13, "kr6_grey"),
        ("0 0 0.22",  "0 0 0", 0.34, 0.075, "kr6_grey"),
    ],
    "link_1": [
        ("0 0 0", "0 0 0", 0.10, 0.065, "kr6_orange"),
    ],
    "link_2": [
        ("-0.2275 0 0", "0 1.5708 0", 0.45, 0.045, "kr6_orange"),
    ],
    "link_3": [
        ("0 0 0.01", "0 0 0", 0.08, 0.05, "kr6_orange"),
        ("0 0 0.22", "0 0 0", 0.38, 0.035, "kr6_orange"),
    ],
    "link_4": [
        ("0 0 0", "0 0 0", 0.05, 0.035, "kr6_grey"),
    ],
    "link_5": [
        ("0 0 0.04", "0 0 0", 0.08, 0.028, "kr6_grey"),
    ],
    "link_6": [
        ("0 0 0.005", "0 0 0", 0.012, 0.045, "kr6_orange"),
    ],
}


def _emit_visuals(name: str, lines: list) -> None:
    for vxyz, vrpy, vlen, vrad, vmat in _LINK_VISUALS[name]:
        lines.append(f'    <visual><origin xyz="{vxyz}" rpy="{vrpy}"/>')
        lines.append(
            f'            <geometry><cylinder length="{vlen}" radius="{vrad}"/></geometry>'
        )
        lines.append(f'            <material name="{vmat}"/></visual>')


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def build_urdf_string() -> str:
    """Return the full URDF document as a single string."""
    pres = []
    posts = []
    for i in range(6):
        a_i, alpha_i, d_i, theta_off_i = KR6_SDH[i]
        pres.append(_Rz(theta_off_i) @ _Tz(d_i))
        posts.append(_Tx(a_i) @ _Rx(alpha_i))

    lines: list[str] = []
    lines.append('<?xml version="1.0"?>')
    lines.append("<!-- kr6_sdh.urdf - KUKA KR 6 R900 sixx -->")
    lines.append("<!-- generated from the DH table in kr6_kinematics.dh -->")
    lines.append('<robot name="kr6_sdh">')
    lines.append('  <material name="kr6_orange"><color rgba="0.95 0.45 0.15 1"/></material>')
    lines.append('  <material name="kr6_grey"><color rgba="0.30 0.30 0.30 1"/></material>')

    # base link
    lines.append('  <link name="base_link">')
    _emit_visuals("base_link", lines)
    lines.append(
        '    <inertial><mass value="2.0"/><inertia ixx="0.01" ixy="0" ixz="0" '
        'iyy="0.01" iyz="0" izz="0.01"/></inertial>'
    )
    lines.append('  </link>')

    for i in range(6):
        pre_i = pres[i]
        post_i = posts[i]
        pxyz = pre_i[:3, 3]
        pr, pp, py = rpy_from_R(pre_i[:3, :3])
        qxyz = post_i[:3, 3]
        qr, qp, qy = rpy_from_R(post_i[:3, :3])

        parent = "base_link" if i == 0 else f"link_{i}"
        pivot = f"pivot_{i + 1}"
        child = f"link_{i + 1}"
        qmin, qmax = Q_MIN[i], Q_MAX[i]

        # revolute joint, with pre factor on the origin
        lines.append("")
        lines.append(f'  <joint name="joint_{i + 1}" type="revolute">')
        lines.append(f'    <parent link="{parent}"/>')
        lines.append(f'    <child  link="{pivot}"/>')
        lines.append(
            f'    <origin xyz="{pxyz[0]:.4f} {pxyz[1]:.4f} {pxyz[2]:.4f}" '
            f'rpy="{pr:.4f} {pp:.4f} {py:.4f}"/>'
        )
        lines.append('    <axis xyz="0 0 1"/>')
        lines.append(
            f'    <limit lower="{qmin:.4f}" upper="{qmax:.4f}" effort="50" velocity="6"/>'
        )
        lines.append('  </joint>')
        lines.append(f'  <link name="{pivot}"/>')

        # fixed joint, with post factor on the origin
        lines.append(f'  <joint name="FJ_{i + 1}" type="fixed">')
        lines.append(f'    <parent link="{pivot}"/>')
        lines.append(f'    <child  link="{child}"/>')
        lines.append(
            f'    <origin xyz="{qxyz[0]:.4f} {qxyz[1]:.4f} {qxyz[2]:.4f}" '
            f'rpy="{qr:.4f} {qp:.4f} {qy:.4f}"/>'
        )
        lines.append('  </joint>')

        # link with sized visuals
        lines.append(f'  <link name="{child}">')
        _emit_visuals(child, lines)
        lines.append(
            '    <inertial><mass value="0.5"/><inertia ixx="0.01" ixy="0" ixz="0" '
            'iyy="0.01" iyz="0" izz="0.01"/></inertial>'
        )
        lines.append('  </link>')

    # tool frame at the flange
    lines.append("")
    lines.append('  <joint name="flange-tool0" type="fixed">')
    lines.append('    <parent link="link_6"/><child link="tool0"/>')
    lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
    lines.append('  </joint>')
    lines.append('  <link name="tool0"/>')
    lines.append('</robot>')
    return "\n".join(lines) + "\n"


def write_urdf(path: str | Path) -> Path:
    """Write the generated URDF to ``path`` and return the resolved path."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(build_urdf_string())
    return path
