"""
kr6_kinematics
==============

A self-contained kinematics, dynamics-adjacent and trajectory-planning library
for the KUKA KR 6 R900 sixx industrial manipulator.

The package implements, from first principles:

- Standard Denavit-Hartenberg parameterisation and forward kinematics
- The geometric (body-frame agnostic) 6x6 manipulator Jacobian
- Closed-form inverse position kinematics via Pieper's spherical-wrist
  decoupling (four geometric IK branches)
- Inverse velocity kinematics via direct Jacobian inversion
- Quintic and LSPB joint-space trajectory primitives
- 3-D stick-figure visualisation and animation utilities
- A programmatically generated URDF whose link frames coincide with the
  DH frames for cross-validation against ROS / `urchin` based forward
  kinematics

The implementation is verified against the manufacturer's data sheet
(KUKA Roboter GmbH, 2015) and the ROS-Industrial KR 6 R900 URDF.

Public API
----------

``DH parameters & forward kinematics``
    :data:`KR6_SDH`, :data:`Q_ZERO`, :data:`Q_READY`,
    :func:`kr6_fk`, :func:`kr6_frames`, :func:`dh_link`

``Joint limits``
    :data:`Q_MIN`, :data:`Q_MAX`, :data:`QD_MAX`,
    :func:`clip_to_limits`, :func:`in_limits`

``Velocity kinematics``
    :func:`jacobian`, :func:`manipulability`, :func:`is_singular`

``Inverse kinematics``
    :func:`ik_position`, :func:`ik_velocity`

``Trajectory generation``
    :func:`quintic`, :func:`quintic_vec`, :func:`lspb`

``Visualisation``
    :func:`setup_3d_axes`, :func:`draw_stick_figure`, :func:`draw_frame`,
    :func:`animate_joint_trajectory`
"""

from .dh import (
    JOINT_NAMES,
    KR6_SDH,
    Q_MAX,
    Q_MIN,
    Q_READY,
    Q_ZERO,
    QD_MAX,
    clip_to_limits,
    dh_link,
    dh_link_SE3,
    in_limits,
    kr6_fk,
    kr6_frames,
)
from .ik import ik_position, ik_velocity
from .jacobian import is_singular, jacobian, manipulability
from .trajectories import lspb, quintic, quintic_vec
from .viz import (
    animate_joint_trajectory,
    draw_frame,
    draw_stick_figure,
    setup_3d_axes,
)

__all__ = [
    # DH & FK
    "KR6_SDH",
    "Q_MIN",
    "Q_MAX",
    "QD_MAX",
    "Q_ZERO",
    "Q_READY",
    "JOINT_NAMES",
    "dh_link",
    "dh_link_SE3",
    "kr6_frames",
    "kr6_fk",
    "clip_to_limits",
    "in_limits",
    # Velocity
    "jacobian",
    "manipulability",
    "is_singular",
    # IK
    "ik_position",
    "ik_velocity",
    # Trajectories
    "quintic",
    "quintic_vec",
    "lspb",
    # Visualisation
    "setup_3d_axes",
    "draw_frame",
    "draw_stick_figure",
    "animate_joint_trajectory",
]

__version__ = "1.0.0"
__author__ = "Bliss Janga"
__license__ = "MIT"
