"""Retícula esférica en (u,v,h), con capas de altura física creciente."""

from collections import defaultdict
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .config import PlanConfig
from .geometry import RigidTransform, grid_extent


DEFAULT_SCAN_ROTATION = ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def _rotation_tuple(rotation) -> tuple[tuple[float, ...], ...]:
    validated = RigidTransform(rotation, (0, 0, 0)).rotation
    return tuple(tuple(float(value) for value in row) for row in validated)


@dataclass(frozen=True)
class GridPoint:
    indices: tuple[int, int, int]
    step_mm: float
    rotation_magnet_from_scan: tuple[tuple[float, ...], ...] = DEFAULT_SCAN_ROTATION

    def __post_init__(self) -> None:
        object.__setattr__(self, "rotation_magnet_from_scan", _rotation_tuple(self.rotation_magnet_from_scan))

    @property
    def point_id(self) -> str:
        """Índices enteros (iu,iv,ih), interpretados con el marco del perfil."""
        return "p:" + ":".join(str(index) for index in self.indices)

    @property
    def position_scan_mm(self) -> tuple[float, float, float]:
        return tuple(index * self.step_mm for index in self.indices)

    @property
    def position_mm(self) -> tuple[float, float, float]:
        """Posición sensible en el marco del imán, no en la base del robot."""
        return tuple(np.asarray(self.rotation_magnet_from_scan) @ self.position_scan_mm)


@dataclass(frozen=True)
class Trajectory:
    radius_mm: float
    step_mm: float
    points: tuple[GridPoint, ...]
    rotation_magnet_from_scan: tuple[tuple[float, ...], ...] = DEFAULT_SCAN_ROTATION

    def __post_init__(self) -> None:
        object.__setattr__(self, "rotation_magnet_from_scan", _rotation_tuple(self.rotation_magnet_from_scan))

    @property
    def positions_mm(self) -> NDArray[np.float64]:
        return np.array([point.position_mm for point in self.points], dtype=float).reshape(-1, 3)

    @property
    def positions_scan_mm(self) -> NDArray[np.float64]:
        return np.array([point.position_scan_mm for point in self.points], dtype=float).reshape(-1, 3)

    @property
    def layer_count(self) -> int:
        return len({point.indices[2] for point in self.points})

    @property
    def length_mm(self) -> float:
        return float(np.linalg.norm(np.diff(self.positions_mm, axis=0), axis=1).sum())


def generate_grid(radius_mm: float, step_mm: float, rotation=DEFAULT_SCAN_ROTATION) -> tuple[GridPoint, ...]:
    """Retícula común (u,v,h), recortada con enteros y expresada en el imán."""
    extent = grid_extent(radius_mm, step_mm)
    rotation = _rotation_tuple(rotation)
    axis = range(-extent, extent + 1)
    return tuple(
        GridPoint((iu, iv, ih), float(step_mm), rotation)
        for ih in axis for iv in axis for iu in axis
        if iu * iu + iv * iv + ih * ih <= extent * extent
    )


def _layer_order(rows: dict[int, list[GridPoint]], reverse_v: bool, reverse_u: bool) -> tuple[GridPoint, ...]:
    ordered = []
    for row_index, iv in enumerate(sorted(rows, reverse=reverse_v)):
        descending_u = reverse_u ^ bool(row_index % 2)
        ordered.extend(sorted(rows[iv], key=lambda point: point.indices[0], reverse=descending_u))
    return tuple(ordered)


def serpentine_order(points: tuple[GridPoint, ...]) -> tuple[GridPoint, ...]:
    """Capas h ascendentes; cuatro candidatos por capa, con entrada más cercana.

    Los empates prefieren filas v ascendentes y u inicial ascendente. La decisión
    minimiza solo la transición de entrada de cada capa, no la ruta completa.
    """
    layers = defaultdict(lambda: defaultdict(list))
    for point in points:
        layers[point.indices[2]][point.indices[1]].append(point)
    ordered = []
    for ih in sorted(layers):
        candidates = [
            _layer_order(layers[ih], reverse_v, reverse_u)
            for reverse_v in (False, True) for reverse_u in (False, True)
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
    rotation = config.scan.rotation_magnet_from_scan
    _validate_physical_up(config, rotation)
    trajectory = Trajectory(
        geometry.radius_mm, geometry.step_mm,
        serpentine_order(generate_grid(geometry.radius_mm, geometry.step_mm, rotation)),
        rotation,
    )
    validate_trajectory(trajectory, config)
    return trajectory


def _validate_physical_up(config: PlanConfig, rotation) -> None:
    if config.simulation is not None:
        rotation_base_magnet = np.asarray(config.simulation.T_base_magnet, dtype=float)[:3, :3]
        height_base = rotation_base_magnet @ np.asarray(rotation, dtype=float)[:, 2]
        if not np.allclose(height_base, config.scan.up_base, atol=1e-10, rtol=0):
            raise ValueError("El eje +h del barrido no coincide con la vertical física up_base tras T_base_magnet.")


def validate_trajectory(trajectory: Trajectory, config: PlanConfig) -> None:
    """Comprueba compatibilidad y cobertura antes de exportar o utilizar un plan."""
    geometry = config.geometry
    if (trajectory.radius_mm, trajectory.step_mm) != (geometry.radius_mm, geometry.step_mm):
        raise ValueError("La trayectoria y la configuración tienen radio/paso incompatibles.")
    rotation = config.scan.rotation_magnet_from_scan
    _validate_physical_up(config, rotation)
    if not np.allclose(trajectory.rotation_magnet_from_scan, rotation, atol=1e-10, rtol=0):
        raise ValueError("La trayectoria y la configuración tienen marcos de barrido incompatibles.")
    expected = {point.indices for point in generate_grid(geometry.radius_mm, geometry.step_mm, rotation)}
    indices = [point.indices for point in trajectory.points]
    if (
        len(indices) != len(expected)
        or set(indices) != expected
        or any(
            len(point.indices) != 3
            or any(type(index) is not int for index in point.indices)
            or point.step_mm != geometry.step_mm
            or not np.allclose(point.rotation_magnet_from_scan, rotation, atol=1e-10, rtol=0)
            for point in trajectory.points
        )
    ):
        raise ValueError("La trayectoria no contiene exactamente la malla configurada, sin duplicados.")
    heights = [index[2] for index in indices]
    extent = grid_extent(geometry.radius_mm, geometry.step_mm)
    if (
        any(current < previous for previous, current in zip(heights, heights[1:]))
        or indices[0] != (0, 0, -extent)
        or indices[-1] != (0, 0, extent)
    ):
        raise ValueError("El barrido debe completar capas horizontales contiguas de abajo hacia arriba, sin descender.")
    positions = trajectory.positions_mm
    if not geometry.sphere.contains(positions).all() or not geometry.bore.contains(positions).all():
        raise ValueError("Hay puntos fuera de la ROI o del BORE de referencia.")
