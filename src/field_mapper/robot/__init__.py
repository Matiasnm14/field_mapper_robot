"""Adaptadores de movimiento."""

from .base import RobotAdapter, RobotPose
from .simulated import SimulatedRobot

__all__ = ["RobotAdapter", "RobotPose", "SimulatedRobot"]
