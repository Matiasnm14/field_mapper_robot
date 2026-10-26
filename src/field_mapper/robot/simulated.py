"""Movimiento lineal sintético con realimentación y fallos inyectables."""

import math

from ..clock import VirtualClock
from ..errors import ConnectionLost, MotionTimeout
from .base import RobotPose


def _finite(value: float, name: str, *, positive: bool = False) -> float:
    value = float(value)
    if not math.isfinite(value) or (value <= 0 if positive else value < 0):
        raise ValueError(f"{name} debe ser finito y {'positivo' if positive else 'no negativo'}.")
    return value


def _position(position_mm) -> tuple[float, float, float]:
    values = tuple(float(value) for value in position_mm)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("position_mm debe tener tres coordenadas finitas.")
    return values


def _angle(angle_deg: float) -> float:
    result = float(angle_deg)
    if not math.isfinite(result):
        raise ValueError("angle_deg debe ser finito.")
    return result


class SimulatedRobot:
    simulated = True

    def __init__(
        self, clock: VirtualClock, initial_position_mm,
        angle_deg: float = 0.0, speed_mm_s: float = 100.0,
        move_latency_s: float = 0.01, fault_at_move: int | None = None,
        fault_mode: str = "timeout",
    ):
        self.clock = clock
        self.speed_mm_s = _finite(speed_mm_s, "speed_mm_s", positive=True)
        self.move_latency_s = _finite(move_latency_s, "move_latency_s")
        if fault_at_move is not None and (type(fault_at_move) is not int or fault_at_move < 1):
            raise ValueError("fault_at_move debe ser un entero desde 1.")
        if fault_mode not in {"timeout", "out_of_tolerance", "connection_loss", "accepted_only"}:
            raise ValueError("Modo de fallo de movimiento desconocido.")
        self.fault_at_move = fault_at_move
        self.fault_mode = fault_mode
        self.command_count = 0
        self.stop_count = 0
        self.move_history: list[RobotPose] = []
        self._pose = RobotPose(_position(initial_position_mm), _angle(angle_deg))
        self._connected = False
        self._moving = False
        self._fault: str | None = None

    def connect(self) -> None:
        self._connected = True

    def _require_connection(self) -> None:
        if not self._connected:
            raise ConnectionLost("Robot simulado desconectado.")

    def identity(self) -> dict:
        self._require_connection()
        return {"simulated": True, "adapter": "SimulatedRobot", "model": "synthetic-linear-robot"}

    def _refresh(self) -> None:
        if not self._moving or self._fault == "timeout":
            return
        elapsed = self.clock.now() - self._start_time
        duration = self._end_time - self._start_time
        fraction = min(1.0, max(0.0, elapsed / duration)) if duration else 1.0
        if fraction >= 1:
            self._pose = self._target
            if self._fault == "out_of_tolerance":
                p = self._target.position_mm
                self._pose = RobotPose((p[0] + 1.0, p[1], p[2]), self._target.angle_deg)
            self._moving = self._fault == "accepted_only"
        else:
            self._pose = RobotPose(
                tuple(a + fraction * (b - a) for a, b in zip(self._start_pose.position_mm, self._target.position_mm)),
                self._start_pose.angle_deg + fraction * (self._target.angle_deg - self._start_pose.angle_deg),
            )

    def pose(self) -> RobotPose:
        self._require_connection()
        self._refresh()
        return self._pose

    def state(self) -> str:
        self._require_connection()
        self._refresh()
        return "moving" if self._moving else "idle"

    def move_to(self, position_mm, angle_deg: float) -> None:
        self._require_connection()
        target = RobotPose(_position(position_mm), _angle(angle_deg))
        self._refresh()
        self.command_count += 1
        self.move_history.append(target)
        self._fault = self.fault_mode if self.fault_at_move is not None and self.command_count >= self.fault_at_move else None
        if self._fault == "connection_loss":
            self._connected = False
            self._moving = False
            raise ConnectionLost("Pérdida inyectada de conexión del robot simulado.")
        self._start_pose = self._pose
        self._target = target
        self._start_time = self.clock.now()
        distance = math.dist(self._pose.position_mm, target.position_mm)
        self._end_time = self._start_time + self.move_latency_s + distance / self.speed_mm_s
        self._moving = True

    def wait_until_arrived(
        self, target_position_mm, angle_deg: float, timeout_s: float,
        position_tolerance_mm: float, angle_tolerance_deg: float,
        stable_time_s: float,
    ) -> RobotPose:
        self._require_connection()
        target = RobotPose(_position(target_position_mm), _angle(angle_deg))
        timeout_s = _finite(timeout_s, "timeout_s")
        position_tolerance_mm = _finite(position_tolerance_mm, "position_tolerance_mm")
        angle_tolerance_deg = _finite(angle_tolerance_deg, "angle_tolerance_deg")
        stable_time_s = _finite(stable_time_s, "stable_time_s")
        deadline = self.clock.now() + timeout_s
        stable_since = None
        while True:
            observed = self.pose()
            angle_error = abs((observed.angle_deg - target.angle_deg + 180) % 360 - 180)
            valid = self.state() == "idle" and math.dist(observed.position_mm, target.position_mm) <= position_tolerance_mm and angle_error <= angle_tolerance_deg
            if valid:
                if stable_since is None:
                    stable_since = self.clock.now()
                if self.clock.now() - stable_since >= stable_time_s:
                    return observed
            else:
                stable_since = None
            if self.clock.now() >= deadline:
                raise MotionTimeout("Robot simulado sin llegada estable dentro de tolerancias y plazo.")
            remaining_stable = stable_time_s - (self.clock.now() - stable_since) if stable_since is not None else 0.01
            self.clock.sleep(min(deadline - self.clock.now(), max(remaining_stable, 0.001), 0.01))

    def stop(self) -> None:
        self.stop_count += 1
        self._require_connection()
        self._refresh()
        self._moving = False

    def disconnect(self) -> None:
        if self._connected:
            self._refresh()
        self._moving = False
        self._connected = False
