"""Perfiles estrictos de planificación y adquisición simulada."""

from dataclasses import MISSING, asdict, dataclass, fields
import json
import math
from pathlib import Path
from typing import Any

from .geometry import Bore, RigidTransform, Sphere, ToolTransform, grid_extent, positive_finite


@dataclass(frozen=True)
class Units:
    distance: str
    angle: str
    field: str

    def __post_init__(self) -> None:
        if (self.distance, self.angle, self.field) != ("mm", "deg", "T"):
            raise ValueError("Las unidades deben ser distance=mm, angle=deg y field=T.")


@dataclass(frozen=True)
class GeometryConfig:
    radius_mm: float
    step_mm: float
    bore_radius_mm: float
    bore_length_mm: float

    def __post_init__(self) -> None:
        for name in ("radius_mm", "step_mm", "bore_radius_mm", "bore_length_mm"):
            object.__setattr__(self, name, positive_finite(getattr(self, name), name))
        grid_extent(self.radius_mm, self.step_mm)
        if not self.bore.contains_sphere(self.sphere):
            raise ValueError("La ROI esférica no cabe en el BORE de referencia centrado.")

    @property
    def sphere(self) -> Sphere:
        return Sphere(self.radius_mm)

    @property
    def bore(self) -> Bore:
        return Bore(self.bore_radius_mm, self.bore_length_mm)


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} debe ser un número finito.")
    try:
        value = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} debe ser un número finito.") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} debe ser un número finito.")
    return value


def _vector3(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} debe contener tres componentes.")
    return tuple(_finite(component, name) for component in value)


def _matrix(value: Any, size: int, name: str) -> tuple[tuple[float, ...], ...]:
    if (
        not isinstance(value, (list, tuple)) or len(value) != size
        or any(not isinstance(row, (list, tuple)) or len(row) != size for row in value)
    ):
        raise ValueError(f"{name} debe ser una matriz {size}x{size}.")
    return tuple(tuple(_finite(component, name) for component in row) for row in value)


@dataclass(frozen=True)
class AcquisitionConfig:
    settle_time_s: float
    samples_per_point: int
    motion_timeout_s: float
    read_timeout_s: float
    position_tolerance_mm: float
    angle_tolerance_deg: float
    stable_time_s: float
    max_sensor_retries: int

    def __post_init__(self) -> None:
        for name in ("settle_time_s", "stable_time_s"):
            value = _finite(getattr(self, name), name)
            if value < 0:
                raise ValueError(f"{name} no puede ser negativo.")
            object.__setattr__(self, name, value)
        for name in ("motion_timeout_s", "read_timeout_s", "position_tolerance_mm", "angle_tolerance_deg"):
            object.__setattr__(self, name, positive_finite(getattr(self, name), name))
        if type(self.samples_per_point) is not int or self.samples_per_point < 1:
            raise ValueError("samples_per_point debe ser un entero positivo.")
        if type(self.max_sensor_retries) is not int or not 0 <= self.max_sensor_retries <= 10:
            raise ValueError("max_sensor_retries debe ser un entero entre 0 y 10.")


@dataclass(frozen=True)
class SimulationConfig:
    seed: int
    initial_sensor_position_magnet_mm: tuple[float, float, float]
    T_base_magnet: tuple[tuple[float, ...], ...]
    rotation_base_flange: tuple[tuple[float, ...], ...]
    offset_flange_sensor_mm: tuple[float, float, float]
    pose_frame: str
    angle_deg: float
    sensitive_axis_magnet: tuple[float, float, float]
    speed_mm_s: float
    move_latency_s: float
    base_field_T: float
    gradient_T_per_mm: tuple[float, float, float]
    quadratic_T_per_mm2: tuple[float, float, float]
    drift_T_per_s: float
    noise_std_T: float
    sample_interval_s: float
    original_unit: str
    motion_fault_at_visit: int | None = None
    motion_fault_mode: str = "timeout"
    sensor_fault_at_read: int | None = None
    sensor_fault_mode: str = "timeout"
    sensor_fault_persistent: bool = True

    def __post_init__(self) -> None:
        if type(self.seed) is not int or self.seed < 0:
            raise ValueError("seed debe ser un entero no negativo.")
        for name in (
            "initial_sensor_position_magnet_mm", "offset_flange_sensor_mm",
            "sensitive_axis_magnet", "gradient_T_per_mm", "quadratic_T_per_mm2",
        ):
            object.__setattr__(self, name, _vector3(getattr(self, name), name))
        object.__setattr__(self, "T_base_magnet", _matrix(self.T_base_magnet, 4, "T_base_magnet"))
        object.__setattr__(self, "rotation_base_flange", _matrix(self.rotation_base_flange, 3, "rotation_base_flange"))
        self.tool_transform()  # Valida rigidez, orientación y marco explícito.
        if not math.isclose(math.hypot(*self.sensitive_axis_magnet), 1, abs_tol=1e-10, rel_tol=0):
            raise ValueError("sensitive_axis_magnet debe ser un vector unitario.")
        for name in ("angle_deg", "base_field_T", "drift_T_per_s"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        for name in ("move_latency_s", "noise_std_T"):
            value = _finite(getattr(self, name), name)
            if value < 0:
                raise ValueError(f"{name} no puede ser negativo.")
            object.__setattr__(self, name, value)
        for name in ("speed_mm_s", "sample_interval_s"):
            object.__setattr__(self, name, positive_finite(getattr(self, name), name))
        if self.original_unit not in ("T", "G"):
            raise ValueError("original_unit debe ser 'T' o 'G'.")
        for name, minimum in (("motion_fault_at_visit", 0), ("sensor_fault_at_read", 1)):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or value < minimum):
                raise ValueError(f"{name} debe ser null o un entero >= {minimum}.")
        if self.motion_fault_mode not in ("timeout", "out_of_tolerance", "connection_loss", "accepted_only"):
            raise ValueError("motion_fault_mode no reconocido.")
        if self.sensor_fault_mode not in ("timeout", "stale", "invalid", "unit", "connection_loss"):
            raise ValueError("sensor_fault_mode no reconocido.")
        if type(self.sensor_fault_persistent) is not bool:
            raise ValueError("sensor_fault_persistent debe ser booleano.")

    def tool_transform(self) -> ToolTransform:
        return ToolTransform(
            RigidTransform.from_matrix(self.T_base_magnet), self.rotation_base_flange,
            self.offset_flange_sensor_mm, self.pose_frame,
        )


@dataclass(frozen=True)
class PlanConfig:
    schema_version: int
    name: str
    mode: str
    units: Units
    geometry: GeometryConfig
    acquisition: AcquisitionConfig | None = None
    simulation: SimulationConfig | None = None

    def __post_init__(self) -> None:
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("schema_version debe ser el entero 1.")
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("name debe ser un texto no vacío.")
        if self.mode != "simulation":
            raise ValueError("Esta versión solo admite perfiles con mode='simulation'.")
        if not isinstance(self.units, Units) or not isinstance(self.geometry, GeometryConfig):
            raise ValueError("units y geometry deben ser configuraciones validadas.")
        if self.acquisition is not None and not isinstance(self.acquisition, AcquisitionConfig):
            raise ValueError("acquisition debe ser una configuración validada o null.")
        if self.simulation is not None and not isinstance(self.simulation, SimulationConfig):
            raise ValueError("simulation debe ser una configuración validada o null.")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _object(value: Any, fields: set[str], name: str, optional: set[str] | None = None) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{name} debe ser un objeto JSON.")
    missing, extra = fields - value.keys(), value.keys() - fields - (optional or set())
    if missing or extra:
        raise ValueError(
            f"Campos incorrectos en {name}: faltantes={sorted(missing)}, "
            f"desconocidos={sorted(extra)}."
        )
    return value


def config_from_dict(data: Any) -> PlanConfig:
    root = _object(data, {"schema_version", "name", "mode", "units", "geometry"}, "config", {"acquisition", "simulation"})
    units = _object(root["units"], {"distance", "angle", "field"}, "units")
    geometry = _object(
        root["geometry"], {"radius_mm", "step_mm", "bore_radius_mm", "bore_length_mm"}, "geometry"
    )
    return PlanConfig(
        schema_version=root["schema_version"], name=root["name"], mode=root["mode"],
        units=Units(**units), geometry=GeometryConfig(**geometry),
        acquisition=_optional_config(root.get("acquisition"), AcquisitionConfig, "acquisition"),
        simulation=_optional_config(root.get("simulation"), SimulationConfig, "simulation"),
    )


def _optional_config(value: Any, config_type: type, name: str):
    if value is None:
        return None
    required = {field.name for field in fields(config_type) if field.default is MISSING and field.default_factory is MISSING}
    optional = {field.name for field in fields(config_type)} - required
    return config_type(**_object(value, required, name, optional))


def _unique_keys(pairs: list[tuple[str, Any]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Clave JSON duplicada: {key}.")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ValueError(f"Constante JSON no finita: {value}.")


def load_config(path: str | Path) -> PlanConfig:
    with Path(path).open(encoding="utf-8") as source:
        data = json.load(source, object_pairs_hook=_unique_keys, parse_constant=_reject_constant)
    return config_from_dict(data)
