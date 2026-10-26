"""Adaptadores de lectura de campo."""

from .base import SensorAdapter, SensorSample, normalize_value_to_tesla
from .simulated import SimulatedSensor

__all__ = ["SensorAdapter", "SensorSample", "normalize_value_to_tesla", "SimulatedSensor"]
