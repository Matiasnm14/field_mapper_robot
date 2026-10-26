"""Figuras estáticas renderizadas con Agg, sin abrir ventanas."""

from pathlib import Path

import numpy as np

from .config import PlanConfig
from .planner import Trajectory


def save_trajectory_figure(trajectory: Trajectory, config: PlanConfig, destination: str | Path) -> None:
    # Las dependencias de dibujo se cargan solo al solicitar una figura.
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.colors import Normalize
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    figure = Figure(figsize=(15, 7), layout="constrained")
    FigureCanvasAgg(figure)
    overview = figure.add_subplot(121, projection="3d")
    detail = figure.add_subplot(122, projection="3d", computed_zorder=False)
    geometry = config.geometry
    radius = geometry.radius_mm
    points = trajectory.positions_mm
    # Permutación cíclica (y,z,x): x se dibuja vertical y se conserva la mano derecha.
    drawing_points = points[:, [1, 2, 0]]
    azimuth = np.linspace(0, 2 * np.pi, 37)
    polar = np.linspace(0, np.pi, 19)
    sphere_x = radius * np.outer(np.cos(azimuth), np.sin(polar))
    sphere_y = radius * np.outer(np.sin(azimuth), np.sin(polar))
    sphere_z = radius * np.outer(np.ones_like(azimuth), np.cos(polar))
    for axis in (overview, detail):
        axis.plot_wireframe(sphere_y, sphere_z, sphere_x, color="darkorange", alpha=0.25, linewidth=0.5)
        axis.set_xlabel("y [mm]")
        axis.set_ylabel("z [mm] — eje del BORE")
        axis.set_zlabel("x [mm] — vertical")
        axis.view_init(elev=22, azim=-55)

    bore_z, bore_angle = np.meshgrid(
        np.linspace(-geometry.bore_length_mm / 2, geometry.bore_length_mm / 2, 9), azimuth
    )
    bore_x = geometry.bore_radius_mm * np.cos(bore_angle)
    bore_y = geometry.bore_radius_mm * np.sin(bore_angle)
    overview.plot_wireframe(bore_y, bore_z, bore_x, color="slategray", alpha=0.3, linewidth=0.6)
    overview.scatter(*drawing_points.T, s=3, color="steelblue", alpha=0.5)
    overview.set_box_aspect((2 * geometry.bore_radius_mm, geometry.bore_length_mm, 2 * geometry.bore_radius_mm))
    overview.set_title("ROI y BORE de referencia")
    overview.text2D(0.02, 0.03, "Geometría del punto sensible; no valida colisiones.", transform=overview.transAxes, fontsize=8)

    visit_indices = np.arange(len(points))
    normalization = Normalize(vmin=0, vmax=max(len(points) - 1, 1))
    segments = np.stack((drawing_points[:-1], drawing_points[1:]), axis=1)
    lines = Line3DCollection(segments, cmap="viridis", norm=normalization, linewidths=0.8, alpha=0.6)
    lines.set_array(visit_indices[:-1])
    detail.add_collection3d(lines)
    scatter = detail.scatter(
        *drawing_points.T, c=visit_indices, cmap="viridis", norm=normalization, s=9
    )
    detail.scatter(
        *drawing_points[0], color="limegreen", edgecolor="black", s=70,
        marker="o", label="Inicio", depthshade=False, zorder=10,
    )
    detail.scatter(
        *drawing_points[-1], color="crimson", s=100, marker="*", label="Fin",
        depthshade=False, zorder=11,
    )
    for axis_limit in (detail.set_xlim, detail.set_ylim, detail.set_zlim):
        axis_limit(-radius * 1.15, radius * 1.15)
    detail.set_box_aspect((1, 1, 1))
    detail.set_title("Orden serpenteante — ampliación de la ROI")
    detail.legend(loc="upper left")
    figure.colorbar(
        scatter, ax=detail, location="bottom", shrink=0.75, pad=0.12,
        fraction=0.04, label="Índice de visita (desde 0)",
    )
    figure.suptitle(
        f"PLAN SIMULADO · DSV {2 * radius:g} mm · paso {geometry.step_mm:g} mm · "
        f"{len(points):,} puntos · {trajectory.layer_count} capas"
    )
    figure.savefig(destination, dpi=160, bbox_inches="tight", pad_inches=0.2)
    figure.clear()
