"""Contrato de lectura escalar B_n y conversión de unidades originales."""

from dataclasses import dataclass
import math
from typing import Protocol

from ..errors import SensorError


@dataclass(frozen=True)
class SensorSample:
    value: float | None
    unit: str
    acquired_time_s: float | None
    received_time_s: float
    sequence_id: int | None
    status: str = "ok"
    fresh: bool = True
    freshness_evidence: str = "simulated_request_sequence"


class SensorAdapter(Protocol):
    simulated: bool

    def connect(self) -> None: ...
    def identity(self) -> dict: ...
    def read_sample(self, *, not_before_s: float, timeout_s: float) -> SensorSample: ...
    def disconnect(self) -> None: ...


def normalize_value_to_tesla(value, unit: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise SensorError("La muestra debe contener un valor numérico finito.")
    if unit not in {"T", "G"}:
        raise SensorError(f"Unidad sin conversión validada: {unit!r}.")
    result = float(value) * (1e-4 if unit == "G" else 1.0)
    if not math.isfinite(result):
        raise SensorError("La muestra normalizada no es finita.")
    return result
