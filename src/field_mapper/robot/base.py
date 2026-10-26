"""Contrato interno del robot; no representa una API de fabricante."""

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class RobotPose:
    position_mm: tuple[float, float, float]
    angle_deg: float


class RobotAdapter(Protocol):
    simulated: bool

    def connect(self) -> None: ...
    def identity(self) -> dict: ...
    def pose(self) -> RobotPose: ...
    def state(self) -> str: ...
    def move_to(self, position_mm: tuple[float, float, float], angle_deg: float) -> None: ...
    def wait_until_arrived(
        self, target_position_mm: tuple[float, float, float], angle_deg: float,
        timeout_s: float, position_tolerance_mm: float,
        angle_tolerance_deg: float, stable_time_s: float,
    ) -> RobotPose: ...
    def stop(self) -> None: ...
    def disconnect(self) -> None: ...
