"""
3-D visualisation utilities
===========================

Lightweight helpers built on top of ``matplotlib`` for drawing the
KR 6 R900 sixx as a stick-figure (link polyline + joint markers + RGB
coordinate frames) and for rendering joint-trajectory animations to
GIF/MP4.

These are intentionally written in pure ``matplotlib`` rather than
e.g. ``trimesh`` or ``pybullet`` so that the project remains easy to
reproduce on a stock scientific Python install.
"""

from __future__ import annotations

from typing import Callable

import numpy as np
from matplotlib.animation import FuncAnimation
from spatialmath import SE3

from .dh import kr6_frames

# ---------------------------------------------------------------------------
# Axis decoration
# ---------------------------------------------------------------------------

def setup_3d_axes(ax, lim: float = 1.0, title: str = "") -> None:
    """Set common limits, labels, equal aspect ratio and viewing angle."""
    ax.set_xlim([-lim, lim])
    ax.set_ylim([-lim, lim])
    ax.set_zlim([0, 2 * lim])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=20, azim=45)
    if title:
        ax.set_title(title)


def draw_frame(
    ax,
    T,
    length: float = 0.08,
    alpha: float = 1.0,
    linewidth: float = 1.5,
) -> list:
    """Draw the three RGB axes of a homogeneous frame ``T`` on ``ax``."""
    if isinstance(T, SE3):
        origin = T.t
        R = T.R
    else:
        T = np.asarray(T)
        origin = T[:3, 3]
        R = T[:3, :3]

    colors = ("r", "g", "b")
    artists = []
    for i in range(3):
        d = R[:, i] * length
        q = ax.quiver(
            origin[0], origin[1], origin[2],
            d[0], d[1], d[2],
            color=colors[i],
            linewidth=linewidth,
            alpha=alpha,
            arrow_length_ratio=0.25,
        )
        artists.append(q)
    return artists


# ---------------------------------------------------------------------------
# Stick-figure renderer
# ---------------------------------------------------------------------------

def draw_stick_figure(
    ax,
    q,
    show_frames: bool = True,
    frame_len: float = 0.06,
    link_color: str = "k",
    joint_color: str = "tab:blue",
    lw: float = 2,
) -> list:
    """
    Draw the manipulator as a polyline through DH frame origins.

    Frame origins are connected by black line segments; intermediate
    joints are drawn as small dots, the base as a grey square and the
    flange as a green triangle. If ``show_frames`` is ``True``, the RGB
    coordinate triad of every frame is overlaid.
    """
    frames = kr6_frames(q)
    origins = np.array([T.t for T in frames])  # shape (7, 3)

    artists = []
    line, = ax.plot(
        origins[:, 0], origins[:, 1], origins[:, 2],
        color=link_color, linewidth=lw,
    )
    artists.append(line)

    joints = origins[1:-1]
    sc = ax.scatter(
        joints[:, 0], joints[:, 1], joints[:, 2],
        color=joint_color, s=30, zorder=5,
    )
    artists.append(sc)

    # Base + flange markers.
    ax.scatter(
        [origins[0, 0]], [origins[0, 1]], [origins[0, 2]],
        color="grey", s=80, marker="s", zorder=5,
    )
    ax.scatter(
        [origins[-1, 0]], [origins[-1, 1]], [origins[-1, 2]],
        color="green", s=60, marker="^", zorder=5,
    )

    if show_frames:
        for i, T in enumerate(frames):
            alpha = 0.5 if 0 < i < 6 else 1.0
            draw_frame(ax, T, length=frame_len, alpha=alpha, linewidth=1.0)

    return artists


# ---------------------------------------------------------------------------
# Animation
# ---------------------------------------------------------------------------

def animate_joint_trajectory(
    Q,
    title: str = "",
    filename: str | None = None,
    fps: int = 20,
    show_frames: bool = False,
    ee_trail: bool = True,
    lim: float = 1.0,
    extra_draw: Callable | None = None,
) -> FuncAnimation:
    """
    Animate the manipulator along a sequence of joint configurations.

    Parameters
    ----------
    Q
        ``(N, 6)`` array of joint configurations.
    title
        Plot title (the frame index ``k/N`` is appended automatically).
    filename
        Optional output path. If supplied, the animation is saved using
        the ``pillow`` writer (GIF) at the given ``fps``.
    show_frames
        Forward to :func:`draw_stick_figure` to overlay coordinate
        frames at each joint.
    ee_trail
        If ``True``, the end-effector path traversed so far is drawn in
        orange.
    lim
        Symmetric workspace limit passed to :func:`setup_3d_axes`.
    extra_draw
        Optional callback ``f(ax, q, k)`` used to overlay extra
        information (e.g. velocity arrows in the singularity demos).

    Returns
    -------
    animation
        The :class:`matplotlib.animation.FuncAnimation` object. The
        figure is closed before returning, so callers should only keep
        the animation if they wish to display it inline (e.g. inside a
        Jupyter notebook).
    """
    import matplotlib.pyplot as plt

    Q = np.asarray(Q)
    N = Q.shape[0]

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    ee_path = np.array([kr6_frames(Q[i])[-1].t for i in range(N)])

    def update(frame_idx: int):
        ax.cla()
        setup_3d_axes(ax, lim=lim, title=f"{title}   (frame {frame_idx + 1}/{N})")
        draw_stick_figure(ax, Q[frame_idx], show_frames=show_frames)
        if ee_trail and frame_idx > 0:
            ax.plot(
                ee_path[: frame_idx + 1, 0],
                ee_path[: frame_idx + 1, 1],
                ee_path[: frame_idx + 1, 2],
                color="tab:orange",
                linewidth=1.2,
                alpha=0.8,
            )
        if extra_draw is not None:
            extra_draw(ax, Q[frame_idx], frame_idx)
        return []

    anim = FuncAnimation(fig, update, frames=N, interval=1000 / fps, blit=False)
    if filename is not None:
        anim.save(filename, writer="pillow", fps=fps)
        print(f"saved {filename}")
    plt.close(fig)
    return anim
