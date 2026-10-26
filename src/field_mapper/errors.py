"""Errores de adquisición independientes del fabricante."""


class AcquisitionError(RuntimeError):
    pass


class MotionError(AcquisitionError):
    pass


class MotionTimeout(MotionError):
    pass


class ConnectionLost(AcquisitionError):
    pass


class SensorError(AcquisitionError):
    pass


class SensorTimeout(SensorError):
    pass
