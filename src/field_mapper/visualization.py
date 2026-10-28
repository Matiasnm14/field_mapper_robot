"""Figuras del barrido horizontal ascendente, con Agg y sin ventanas."""

from pathlib import Path

import numpy as np

from .config import PlanConfig
from .planner import Trajectory


def save_trajectory_figure(trajectory: Trajectory, config: PlanConfig, destination: str | Path) -> None:
    # Dibujar en el marco físico de barrido mantiene +h vertical incluso si el
    # imán está girado. El BORE se convierte desde su marco longitudinal propio.
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.colors import Normalize
    from matplotlib import colormaps
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    figure = Figure(figsize=(16, 8), layout="constrained")
    FigureCanvasAgg(figure)
    try:
        overview = figure.add_subplot(121, projection="3d")
        detail = figure.add_subplot(122, projection="3d", computed_zorder=False)
        geometry = config.geometry
        radius = geometry.radius_mm
        points = trajectory.positions_scan_mm
        magnet_from_scan = np.asarray(trajectory.rotation_magnet_from_scan)
        azimuth = np.linspace(0, 2 * np.pi, 65)
        polar = np.linspace(0, np.pi, 19)
        sphere_u = radius * np.outer(np.cos(azimuth), np.sin(polar))
        sphere_v = radius * np.outer(np.sin(azimuth), np.sin(polar))
        sphere_h = radius * np.outer(np.ones_like(azimuth), np.cos(polar))
        for axis in (overview, detail):
            axis.plot_wireframe(sphere_u, sphere_v, sphere_h, color="darkorange", alpha=0.18, linewidth=0.4)
            axis.set_xlabel("u de barrido [mm]")
            axis.set_ylabel("v de barrido [mm]")
            axis.set_zlabel("h [mm] · altura física")
            axis.view_init(elev=22, azim=-55)

        bore_z, bore_angle = np.meshgrid(
            np.linspace(-geometry.bore_length_mm / 2, geometry.bore_length_mm / 2, 9), azimuth
        )
        bore_x = geometry.bore_radius_mm * np.cos(bore_angle)
        bore_y = geometry.bore_radius_mm * np.sin(bore_angle)
        bore_scan = np.stack((bore_x, bore_y, bore_z), axis=-1) @ magnet_from_scan
        overview.plot_wireframe(*np.moveaxis(bore_scan, -1, 0), color="slategray", alpha=0.25, linewidth=0.6)
        overview.scatter(*points.T, s=3, color="steelblue", alpha=0.5)
        bore_axis = np.array([[0, 0, -geometry.bore_length_mm / 2], [0, 0, geometry.bore_length_mm / 2]]) @ magnet_from_scan
        overview.plot(*bore_axis.T, linestyle="--", color="dimgray", linewidth=1)
        overview.text(*bore_axis[-1], " +z imán\n eje longitudinal BORE", fontsize=8)
        overview.set_box_aspect(np.maximum(np.ptp(bore_scan.reshape(-1, 3), axis=0), 2 * radius))
        overview.set_title("ROI y BORE de referencia · marco de barrido (u, v, h)")
        overview.text2D(0.02, 0.02, "Punto sensible y geometría de referencia; no valida colisiones.", transform=overview.transAxes, fontsize=8)

        visit_indices = np.arange(len(points))
        normalization = Normalize(vmin=0, vmax=max(len(points) - 1, 1))
        segments = np.stack((points[:-1], points[1:]), axis=1)
        lines = Line3DCollection(segments, cmap="viridis", norm=normalization, linewidths=0.8, alpha=0.6)
        lines.set_array(visit_indices[:-1])
        detail.add_collection3d(lines)
        scatter = detail.scatter(*points.T, c=visit_indices, cmap="viridis", norm=normalization, s=9)
        layers = sorted({point.indices[2] for point in trajectory.points})
        height_normalization = Normalize(vmin=-radius, vmax=radius)
        for layer_index in layers:
            height = layer_index * trajectory.step_mm
            layer_radius = np.sqrt(max(0.0, radius * radius - height * height))
            color = colormaps["viridis"](height_normalization(height))
            detail.plot(layer_radius * np.cos(azimuth), layer_radius * np.sin(azimuth),
                        np.full_like(azimuth, height), color=color, alpha=0.55, linewidth=0.7)
            detail.text(radius * 1.05, 0, height, f"{height:g}", fontsize=6, color="dimgray")
        detail.text(radius * 1.05, 0, radius * 1.15, "h [mm]", fontsize=7)
        for axis in (overview, detail):
            axis.quiver(-radius * 1.12, -radius * 1.12, -radius, 0, 0, 2 * radius,
                        color="firebrick", arrow_length_ratio=0.08, linewidth=1.7)
            axis.text(-radius * 1.12, -radius * 1.12, radius * 1.08, "+h · arriba", color="firebrick", fontsize=8)
        detail.scatter(*points[0], color="limegreen", edgecolor="black", s=75,
                       marker="o", label="Inicio: extremo inferior", depthshade=False, zorder=10)
        detail.scatter(*points[-1], color="crimson", s=110, marker="*", label="Fin: extremo superior",
                       depthshade=False, zorder=11)
        for axis_limit in (detail.set_xlim, detail.set_ylim, detail.set_zlim):
            axis_limit(-radius * 1.3, radius * 1.3)
        detail.set_box_aspect((1, 1, 1))
        detail.set_title(f"{trajectory.layer_count} capas horizontales completas · de abajo hacia arriba")
        detail.legend(loc="upper left", fontsize=8)
        figure.colorbar(scatter, ax=detail, location="bottom", shrink=0.75, pad=0.12,
                        fraction=0.04, label="Índice de visita de malla (desde 0)")
        figure.suptitle(
            f"PLAN SIMULADO · DSV {2 * radius:g} mm · paso {geometry.step_mm:g} mm · "
            f"{len(points):,} puntos · barrido serpenteante por altura física"
        )
        figure.savefig(destination, dpi=160, bbox_inches="tight", pad_inches=0.2)
    finally:
        figure.clear()
