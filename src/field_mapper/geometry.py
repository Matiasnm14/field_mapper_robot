"""Geometría en milímetros y transformaciones del punto sensible."""

from dataclasses import dataclass
import math
from typing import Literal

import numpy as np
from numpy.typing import ArrayLike, NDArray


# Límite explícito de recursos de la primera versión; no modifica el paso pedido.
MAX_GRID_CANDIDATES = 2_000_000


def positive_finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} debe ser un número positivo y finito.")
    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} debe ser un número positivo y finito.") from exc
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{name} debe ser un número positivo y finito.")
    return number


def grid_extent(radius_mm: float, step_mm: float) -> int:
    """Número de pasos por radio, con tolerancia solo al redondeo numérico."""
    radius = positive_finite(radius_mm, "radius_mm")
    step = positive_finite(step_mm, "step_mm")
    ratio = radius / step
    if not math.isfinite(ratio) or ratio < 1:
        raise ValueError("radio/paso debe ser un entero positivo.")
    extent = round(ratio)
    if not math.isclose(ratio, extent, rel_tol=0, abs_tol=8 * math.ulp(ratio)):
        raise ValueError("radio/paso debe ser entero; no se ajustará el paso.")
    if (2 * extent + 1) ** 3 > MAX_GRID_CANDIDATES:
        raise ValueError(
            f"La retícula supera el límite de {MAX_GRID_CANDIDATES:,} "
            "candidatos de esta versión; elija un paso mayor."
        )
    return extent


def _positions(value: ArrayLike) -> NDArray[np.float64]:
    array = np.asarray(value, dtype=float)
    if array.ndim < 1 or array.shape[-1] != 3 or not np.isfinite(array).all():
        raise ValueError("Las posiciones deben tener forma (..., 3) y ser finitas.")
    return array


def _rotation(value: ArrayLike) -> NDArray[np.float64]:
    rotation = np.array(value, dtype=float, copy=True)
    if (
        rotation.shape != (3, 3)
        or not np.isfinite(rotation).all()
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-10, rtol=0)
        or not math.isclose(float(np.linalg.det(rotation)), 1, abs_tol=1e-10)
    ):
        raise ValueError("La rotación debe ser una matriz 3x3 ortonormal con det=+1.")
    rotation.setflags(write=False)
    return rotation


def _vector(value: ArrayLike) -> NDArray[np.float64]:
    vector = np.array(_positions(value), copy=True)
    if vector.shape != (3,):
        raise ValueError("Se esperaba un vector de tres componentes.")
    vector.setflags(write=False)
    return vector


def _boundary_limit(value: float) -> float:
    # Absorbe únicamente redondeos de operaciones como 3 * 0.1.
    return value + 16 * np.finfo(float).eps * value


@dataclass(frozen=True)
class Sphere:
    radius_mm: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "radius_mm", positive_finite(self.radius_mm, "radius_mm"))

    def contains(self, points: ArrayLike) -> NDArray[np.bool_]:
        positions = _positions(points)
        distance = np.hypot(np.hypot(positions[..., 0], positions[..., 1]), positions[..., 2])
        return distance <= _boundary_limit(self.radius_mm)


@dataclass(frozen=True)
class Bore:
    """Cilindro centrado en el origen, con eje longitudinal z."""

    radius_mm: float
    length_mm: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "radius_mm", positive_finite(self.radius_mm, "bore_radius_mm"))
        object.__setattr__(self, "length_mm", positive_finite(self.length_mm, "bore_length_mm"))

    def contains(self, points: ArrayLike) -> NDArray[np.bool_]:
        positions = _positions(points)
        return (
            np.hypot(positions[..., 0], positions[..., 1]) <= _boundary_limit(self.radius_mm)
        ) & (np.abs(positions[..., 2]) <= _boundary_limit(self.length_mm / 2))

    def contains_sphere(self, sphere: Sphere) -> bool:
        return sphere.radius_mm <= min(self.radius_mm, self.length_mm / 2)


@dataclass(frozen=True, eq=False)
class RigidTransform:
    """Transformación rígida destino <- origen: p_dest = R @ p_orig + t."""

    rotation: ArrayLike
    translation_mm: ArrayLike

    def __post_init__(self) -> None:
        object.__setattr__(self, "rotation", _rotation(self.rotation))
        object.__setattr__(self, "translation_mm", _vector(self.translation_mm))

    @classmethod
    def from_matrix(cls, matrix: ArrayLike) -> "RigidTransform":
        matrix = np.asarray(matrix, dtype=float)
        if (
            matrix.shape != (4, 4)
            or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
        ):
            raise ValueError("La transformación homogénea debe ser 4x4 con última fila [0,0,0,1].")
        return cls(matrix[:3, :3], matrix[:3, 3])

    def as_matrix(self) -> NDArray[np.float64]:
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.translation_mm
        return matrix

    def apply(self, points: ArrayLike) -> NDArray[np.float64]:
        return _positions(points) @ self.rotation.T + self.translation_mm

    def inverse_apply(self, points: ArrayLike) -> NDArray[np.float64]:
        return (_positions(points) - self.translation_mm) @ self.rotation


@dataclass(frozen=True, eq=False)
class ToolTransform:
    """Convierte posiciones de sensor/imán y poses de brida o TCP.

    La orientación es fija. No resuelve cinemática ni certifica colisiones.
    Si mando y realimentación usan marcos distintos, usar una instancia por marco.
    """

    base_from_magnet: RigidTransform
    rotation_base_flange: ArrayLike
    offset_flange_sensor_mm: ArrayLike
    pose_frame: Literal["flange", "tcp"]

    def __post_init__(self) -> None:
        if self.pose_frame not in ("flange", "tcp"):
            raise ValueError("pose_frame debe ser 'flange' o 'tcp'.")
        object.__setattr__(self, "rotation_base_flange", _rotation(self.rotation_base_flange))
        object.__setattr__(self, "offset_flange_sensor_mm", _vector(self.offset_flange_sensor_mm))

    @property
    def offset_base_mm(self) -> NDArray[np.float64]:
        if self.pose_frame == "tcp":
            return np.zeros(3)
        return self.rotation_base_flange @ self.offset_flange_sensor_mm

    def magnet_to_robot(self, sensor_magnet_mm: ArrayLike) -> NDArray[np.float64]:
        return self.base_from_magnet.apply(sensor_magnet_mm) - self.offset_base_mm

    def robot_to_magnet(self, reported_base_mm: ArrayLike) -> NDArray[np.float64]:
        return self.base_from_magnet.inverse_apply(_positions(reported_base_mm) + self.offset_base_mm)
