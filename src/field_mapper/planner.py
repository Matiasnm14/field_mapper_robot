"""Retícula global recortada por una esfera y recorrido serpenteante."""

from collections import defaultdict
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .config import PlanConfig
from .geometry import grid_extent


@dataclass(frozen=True)
class GridPoint:
    indices: tuple[int, int, int]
    step_mm: float

    @property
    def point_id(self) -> str:
        """Identificador relativo a la retícula, independiente del orden de visita."""
        return "p:" + ":".join(str(index) for index in self.indices)

    @property
    def position_mm(self) -> tuple[float, float, float]:
        return tuple(index * self.step_mm for index in self.indices)


@dataclass(frozen=True)
class Trajectory:
    radius_mm: float
    step_mm: float
    points: tuple[GridPoint, ...]

    @property
    def positions_mm(self) -> NDArray[np.float64]:
        return np.array([point.position_mm for point in self.points], dtype=float).reshape(-1, 3)

    @property
    def layer_count(self) -> int:
        return len({point.indices[2] for point in self.points})

    @property
    def length_mm(self) -> float:
        return float(np.linalg.norm(np.diff(self.positions_mm, axis=0), axis=1).sum())


def generate_grid(radius_mm: float, step_mm: float) -> tuple[GridPoint, ...]:
    """Centro y frontera incluidos; pertenencia calculada con enteros."""
    extent = grid_extent(radius_mm, step_mm)
    axis = range(-extent, extent + 1)
    return tuple(
        GridPoint((ix, iy, iz), float(step_mm))
        for iz in axis for ix in axis for iy in axis
        if ix * ix + iy * iy + iz * iz <= extent * extent
    )


def _layer_order(rows: dict[int, list[GridPoint]], reverse_x: bool, reverse_y: bool) -> tuple[GridPoint, ...]:
    ordered = []
    for row_index, ix in enumerate(sorted(rows, reverse=reverse_x)):
        descending_y = reverse_y ^ bool(row_index % 2)
        ordered.extend(sorted(rows[ix], key=lambda point: point.indices[1], reverse=descending_y))
    return tuple(ordered)


def serpentine_order(points: tuple[GridPoint, ...]) -> tuple[GridPoint, ...]:
    """Capas z ascendentes; cuatro candidatos por capa, con entrada más cercana.

    Los empates prefieren x ascendente e y inicial ascendente. La decisión
    minimiza solo la transición de entrada de cada capa, no la ruta completa.
    """
    layers = defaultdict(lambda: defaultdict(list))
    for point in points:
        layers[point.indices[2]][point.indices[0]].append(point)
    ordered = []
    for iz in sorted(layers):
        candidates = [
            _layer_order(layers[iz], reverse_x, reverse_y)
            for reverse_x in (False, True) for reverse_y in (False, True)
        ]
        if not ordered:
            chosen = candidates[0]
        else:
            previous = ordered[-1].indices
            chosen = min(
                candidates,
                key=lambda candidate: sum(
                    (a - b) ** 2 for a, b in zip(previous, candidate[0].indices)
                ),
            )
        ordered.extend(chosen)
    return tuple(ordered)


def plan_trajectory(config: PlanConfig) -> Trajectory:
    geometry = config.geometry
    trajectory = Trajectory(
        geometry.radius_mm, geometry.step_mm,
        serpentine_order(generate_grid(geometry.radius_mm, geometry.step_mm)),
    )
    validate_trajectory(trajectory, config)
    return trajectory


def validate_trajectory(trajectory: Trajectory, config: PlanConfig) -> None:
    """Comprueba compatibilidad y cobertura antes de exportar o utilizar un plan."""
    geometry = config.geometry
    if (trajectory.radius_mm, trajectory.step_mm) != (geometry.radius_mm, geometry.step_mm):
        raise ValueError("La trayectoria y la configuración tienen radio/paso incompatibles.")
    expected = {point.indices for point in generate_grid(geometry.radius_mm, geometry.step_mm)}
    indices = [point.indices for point in trajectory.points]
    if (
        len(indices) != len(expected)
        or set(indices) != expected
        or any(
            len(point.indices) != 3
            or any(type(index) is not int for index in point.indices)
            or point.step_mm != geometry.step_mm
            for point in trajectory.points
        )
    ):
        raise ValueError("La trayectoria no contiene exactamente la malla configurada, sin duplicados.")
    positions = trajectory.positions_mm
    if not geometry.sphere.contains(positions).all() or not geometry.bore.contains(positions).all():
        raise ValueError("Hay puntos fuera de la ROI o del BORE de referencia.")
