"""Perfil independiente y estricto para la futura puesta a punto del M1 Pro."""

from dataclasses import asdict, dataclass, fields
import json
import math
from pathlib import Path
import re

from .geometry import RigidTransform, ToolTransform


def _number(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} debe ser numérico y finito.")
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError(f"{name} excede el rango finito.") from error
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f"{name} debe ser finito" + (" y positivo." if positive else "."))
    return value


def _vector(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} necesita tres componentes.")
    return tuple(_number(component, name) for component in value)


def _matrix(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} necesita una matriz {size}x{size}.")
    rows = []
    for row in value:
        if not isinstance(row, (list, tuple)) or len(row) != size:
            raise ValueError(f"{name} necesita una matriz {size}x{size}.")
        rows.append(tuple(_number(component, name) for component in row))
    return tuple(rows)


def _text(value, name, *, nullable=False):
    if nullable and value is None:
        return
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} debe ser texto no vacío.")


def _strict_object(value, config_type, name):
    expected = {field.name for field in fields(config_type)}
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError(f"{name} requiere exactamente estos campos: {', '.join(sorted(expected))}.")
    return value


@dataclass(frozen=True)
class M1ProCalibration:
    confirmed: bool
    source: str
    T_base_magnet: tuple
    rotation_base_flange: tuple
    offset_flange_sensor_mm: tuple
    pose_frame: str
    fixed_angle_deg: float
    sensitive_axis_magnet: tuple
    user_index: int
    tool_index: int
    coordinate_frames_verified: bool
    paths_verified: bool

    def __post_init__(self):
        for name in ("confirmed", "coordinate_frames_verified", "paths_verified"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} debe ser booleano.")
        _text(self.source, "calibration.source")
        object.__setattr__(self, "T_base_magnet", _matrix(self.T_base_magnet, 4, "T_base_magnet"))
        object.__setattr__(self, "rotation_base_flange", _matrix(self.rotation_base_flange, 3, "rotation_base_flange"))
        for name in ("offset_flange_sensor_mm", "sensitive_axis_magnet"):
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        if not math.isclose(math.hypot(*self.sensitive_axis_magnet), 1, rel_tol=0, abs_tol=1e-10):
            raise ValueError("sensitive_axis_magnet debe ser unitario.")
        object.__setattr__(self, "fixed_angle_deg", _number(self.fixed_angle_deg, "fixed_angle_deg"))
        for name in ("user_index", "tool_index"):
            value = getattr(self, name)
            if type(value) is not int or not 0 <= value <= 9:
                raise ValueError(f"{name} debe ser un entero entre 0 y 9.")
        if self.user_index != 0:
            raise ValueError("T_base_magnet requiere User=0 (base); otros marcos no están implementados.")
        ToolTransform(RigidTransform.from_matrix(self.T_base_magnet), self.rotation_base_flange,
                      self.offset_flange_sensor_mm, self.pose_frame)


@dataclass(frozen=True)
class M1ProConfig:
    schema_version: int
    target_model: str
    mode: str
    host: str | None
    dashboard_port: int
    move_port: int
    feedback_port: int
    connect_timeout_s: float
    command_timeout_s: float
    poll_interval_s: float
    firmware_version: str | None
    compatible_confirmed: bool
    compatibility_evidence: str | None
    motion_enabled: bool
    speed_l_percent: int | None
    acc_l_percent: int | None
    emergency_stop_confirmed: bool
    calibration: M1ProCalibration | None

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("schema_version debe ser el entero 1.")
        if self.target_model != "Dobot M1 Pro" or self.mode != "hardware":
            raise ValueError("El perfil debe declarar target_model='Dobot M1 Pro' y mode='hardware'.")
        _text(self.host, "host", nullable=True)
        if self.host is not None and (
            len(self.host) > 253 or re.fullmatch(r"[A-Za-z0-9_.:\-]+", self.host) is None
        ):
            raise ValueError("host debe ser una dirección o nombre sin URL, espacios ni credenciales.")
        for name in ("dashboard_port", "move_port", "feedback_port"):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 65535:
                raise ValueError(f"{name} debe ser un puerto TCP entero válido.")
        if len({self.dashboard_port, self.move_port, self.feedback_port}) != 3:
            raise ValueError("Los puertos de los tres canales deben ser distintos.")
        for name in ("connect_timeout_s", "command_timeout_s", "poll_interval_s"):
            object.__setattr__(self, name, _number(getattr(self, name), name, positive=True))
        for name in ("compatible_confirmed", "motion_enabled", "emergency_stop_confirmed"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} debe ser booleano.")
        _text(self.compatibility_evidence, "compatibility_evidence", nullable=True)
        if self.firmware_version is not None and (
            not isinstance(self.firmware_version, str)
            or not re.fullmatch(r"V?\d+\.\d+\.\d+\.\d+", self.firmware_version)
        ):
            raise ValueError("firmware_version debe ser null o una versión documentada como V1.6.0.0.")
        for name in ("speed_l_percent", "acc_l_percent"):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or not 1 <= value <= 100):
                raise ValueError(f"{name} debe ser null o un entero entre 1 y 100.")
        if self.calibration is not None and not isinstance(self.calibration, M1ProCalibration):
            raise ValueError("calibration debe ser null o una M1ProCalibration validada.")

    def preflight_movement(self):
        """Comprueba declaraciones requeridas; no certifica su veracidad física."""
        pending = []
        if self.host is None:
            pending.append("host")
        if self.firmware_version is None:
            pending.append("firmware_version")
        elif tuple(map(int, self.firmware_version.removeprefix("V").split("."))) < (1, 5, 6, 0):
            pending.append("firmware dentro del mínimo declarado por el SDK (V1.5.6.0+)")
        if not self.compatible_confirmed or not self.compatibility_evidence:
            pending.append("compatibilidad documentada")
        if not self.motion_enabled:
            pending.append("motion_enabled")
        if not self.emergency_stop_confirmed:
            pending.append("procedimiento EmergencyStop confirmado (corta alimentación y genera alarma)")
        if self.speed_l_percent is None or self.acc_l_percent is None:
            pending.append("velocidad/aceleración puestas a punto")
        calibration = self.calibration
        if calibration is None or not calibration.confirmed:
            pending.append("calibración física confirmada")
        elif not calibration.coordinate_frames_verified or not calibration.paths_verified:
            pending.append("marcos y rutas verificados")
        if pending:
            raise ValueError("Movimiento bloqueado; pendiente: " + ", ".join(pending) + ".")

    def to_dict(self):
        return asdict(self)


def m1pro_config_from_dict(data):
    values = dict(_strict_object(data, M1ProConfig, "M1ProConfig"))
    if values["calibration"] is not None:
        values["calibration"] = M1ProCalibration(**_strict_object(values["calibration"], M1ProCalibration, "calibration"))
    return M1ProConfig(**values)


def _unique_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Clave JSON duplicada: {key}.")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"Constante JSON no finita: {value}.")


def load_m1pro_config(path: str | Path) -> M1ProConfig:
    with Path(path).open(encoding="utf-8") as handle:
        data = json.load(handle, object_pairs_hook=_unique_keys, parse_constant=_reject_constant)
    return m1pro_config_from_dict(data)
