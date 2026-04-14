from __future__ import annotations

from typing import Dict, List

import numpy as np
from matplotlib.axes import Axes

from app.math_core import moved_anchors
from app.models import CABLES, RigConfig


def _coord_map(cfg: RigConfig, moved: Dict[str, np.ndarray], key: str) -> tuple[np.ndarray, np.ndarray]:
    f = np.array([cfg.fixed_points[key].x, cfg.fixed_points[key].y, cfg.fixed_points[key].z], dtype=float)
    a = moved[key]
    return f, a


def plot_top_view(ax: Axes, cfg: RigConfig, roll_rad: float, pitch_rad: float, lengths_ok: Dict[str, bool]) -> None:
    ax.clear()
    m = moved_anchors(cfg, roll_rad, pitch_rad)
    for k in CABLES:
        f, a = _coord_map(cfg, m, k)
        color = "tab:green" if lengths_ok.get(k, True) else "tab:red"
        ax.plot([f[0], a[0]], [f[1], a[1]], color=color, linewidth=1.8)
        ax.scatter([f[0]], [f[1]], marker="s", color="tab:blue")
        ax.scatter([a[0]], [a[1]], marker="o", color="tab:orange")
        ax.text(a[0], a[1], k)
    ax.set_title("Top View (x-y)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)


def plot_side_view(ax: Axes, cfg: RigConfig, roll_rad: float, pitch_rad: float, lengths_ok: Dict[str, bool]) -> None:
    ax.clear()
    m = moved_anchors(cfg, roll_rad, pitch_rad)
    for k in CABLES:
        f, a = _coord_map(cfg, m, k)
        color = "tab:green" if lengths_ok.get(k, True) else "tab:red"
        ax.plot([f[0], a[0]], [f[2], a[2]], color=color, linewidth=1.8)
        ax.scatter([f[0]], [f[2]], marker="s", color="tab:blue")
        ax.scatter([a[0]], [a[2]], marker="o", color="tab:orange")
        ax.text(a[0], a[2], k)
    ax.set_title("Side View (x-z)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)


def plot_heatmap(ax: Axes, heatmap: List[List[float]], pitch_min: float, pitch_max: float, roll_min: float, roll_max: float) -> None:
    ax.clear()
    arr = np.array(heatmap, dtype=float)
    im = ax.imshow(
        arr,
        origin="lower",
        extent=[pitch_min, pitch_max, roll_min, roll_max],
        aspect="auto",
        cmap="viridis",
    )
    ax.figure.colorbar(im, ax=ax, label="RMS error [m]")
    ax.set_xlabel("Pitch [deg]")
    ax.set_ylabel("Roll [deg]")
    ax.set_title("Workspace RMS Heatmap")
