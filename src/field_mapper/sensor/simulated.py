"""Modelo escalar sintético de campo con ruido reproducible."""

import math
from collections.abc import Callable

import numpy as np

from ..clock import VirtualClock
from ..errors import ConnectionLost, SensorError, SensorTimeout
from .base import SensorSample


def _vector(values) -> tuple[float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError("El modelo necesita un vector de tres componentes finitas.")
    return result


class SimulatedSensor:
    simulated = True

    def __init__(
        self, clock: VirtualClock, position_provider: Callable,
        seed: int = 0, base_field_T: float = 0.1,
        gradient_T_per_mm=(1e-6, -2e-6, 3e-6),
        quadratic_T_per_mm2=(1e-8, 2e-8, -1e-8),
        drift_T_per_s: float = 1e-9, noise_std_T: float = 1e-7,
        sample_interval_s: float = 0.002, original_unit: str = "G",
        fault_at_read: int | None = None, fault_mode: str = "timeout",
        fault_persistent: bool = True,
    ):
        self.clock = clock
        self.position_provider = position_provider
        self.seed = seed
        self.base_field_T = float(base_field_T)
        self.gradient_T_per_mm = _vector(gradient_T_per_mm)
        self.quadratic_T_per_mm2 = _vector(quadratic_T_per_mm2)
        self.drift_T_per_s = float(drift_T_per_s)
        self.noise_std_T = float(noise_std_T)
        self.sample_interval_s = float(sample_interval_s)
        if not all(math.isfinite(v) for v in (self.base_field_T, self.drift_T_per_s, self.noise_std_T, self.sample_interval_s)) or self.noise_std_T < 0 or self.sample_interval_s <= 0:
            raise ValueError("Parámetros sintéticos no finitos o intervalos/ruido inválidos.")
        if original_unit not in {"T", "G"}:
            raise ValueError("original_unit debe ser T o G.")
        if fault_at_read is not None and (type(fault_at_read) is not int or fault_at_read < 1):
            raise ValueError("fault_at_read debe ser un entero desde 1.")
        if fault_mode not in {"timeout", "stale", "invalid", "unit", "connection_loss"}:
            raise ValueError("Modo de fallo de sensor desconocido.")
        self.original_unit = original_unit
        self.fault_at_read = fault_at_read
        self.fault_mode = fault_mode
        self.fault_persistent = fault_persistent
        self.read_count = 0
        self._sequence = 0
        self._rng = np.random.default_rng(seed)
        self._connected = False
        self._last_acquired = None

    def connect(self) -> None:
        self._connected = True

    def _require_connection(self) -> None:
        if not self._connected:
            raise ConnectionLost("Sensor simulado desconectado.")

    def identity(self) -> dict:
        self._require_connection()
        return {"simulated": True, "adapter": "SimulatedSensor", "model": "synthetic-scalar-B_n", "seed": self.seed, "original_unit": self.original_unit}

    def read_sample(self, *, not_before_s: float, timeout_s: float) -> SensorSample:
        self._require_connection()
        if not math.isfinite(not_before_s) or not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError("Límite de frescura/plazo debe ser finito y timeout_s no negativo.")
        self.read_count += 1
        fault_active = self.fault_at_read is not None and (self.read_count >= self.fault_at_read if self.fault_persistent else self.read_count == self.fault_at_read)
        fault = self.fault_mode if fault_active else None
        if fault == "connection_loss":
            self._connected = False
            raise ConnectionLost("Pérdida inyectada de conexión del sensor simulado.")
        next_time = max(self.clock.now(), not_before_s) + self.sample_interval_s
        wait_s = next_time - self.clock.now()
        if fault == "timeout" or wait_s > timeout_s:
            self.clock.sleep(timeout_s)
            raise SensorTimeout("Sensor simulado sin muestra nueva dentro del plazo.")
        self.clock.sleep(wait_s)
        position = _vector(self.position_provider())
        value_T = self.base_field_T + sum(g * p + q * p * p for g, q, p in zip(self.gradient_T_per_mm, self.quadratic_T_per_mm2, position)) + self.drift_T_per_s * self.clock.now() + float(self._rng.normal(0, self.noise_std_T))
        if not math.isfinite(value_T):
            raise SensorError("El modelo sintético produjo un valor no finito.")
        if fault == "stale":
            return SensorSample(value_T / (1e-4 if self.original_unit == "G" else 1), self.original_unit, self._last_acquired if self._last_acquired is not None else not_before_s, self.clock.now(), self._sequence, status="stale", fresh=False, freshness_evidence="simulated_repeated_sequence")
        self._sequence += 1
        self._last_acquired = self.clock.now()
        value = value_T / (1e-4 if self.original_unit == "G" else 1)
        return SensorSample(float("nan") if fault == "invalid" else value, "mV" if fault == "unit" else self.original_unit, self.clock.now(), self.clock.now(), self._sequence, status="invalid" if fault == "invalid" else "ok")

    def disconnect(self) -> None:
        self._connected = False
