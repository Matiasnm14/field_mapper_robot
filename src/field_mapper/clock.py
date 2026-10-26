"""Reloj monotónico virtual compartido por los adaptadores de simulación."""

import math


class VirtualClock:
    def __init__(self, start_time_s: float = 0.0):
        if not math.isfinite(start_time_s) or start_time_s < 0:
            raise ValueError("start_time_s debe ser finito y no negativo.")
        self._time_s = float(start_time_s)

    def now(self) -> float:
        return self._time_s

    def sleep(self, seconds: float) -> None:
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("seconds debe ser finito y no negativo.")
        result = self._time_s + seconds
        if not math.isfinite(result):
            raise ValueError("El tiempo virtual excede el rango finito.")
        self._time_s = result
