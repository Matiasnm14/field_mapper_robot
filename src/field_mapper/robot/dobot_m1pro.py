"""Adaptador TCP/IP cuatro ejes opcional; no habilita ni modifica el controlador.

Referencia: Dobot-Arm/TCP-IP-4Axis-Python, revisión fijada en docs/M1PRO_SDK.md.
La confirmación usa Sync y consultas ASCII nuevas, sin asumir formato binario.
"""

import json
import math
import socket
import time

from ..errors import ConnectionLost, MotionError, MotionTimeout
from ..hardware_config import M1ProConfig, _number, _vector
from .base import RobotPose
from .m1pro_transport import ASCIIChannel


SDK_COMMIT = "b5f95647cfc44a60f92babf5a464d38dd1e4f72c"
PROTOCOL_COMMIT = "be74b056be664d1a1a0c95c89e4ecdf950ff3ecf"
MODES = {
    1: "initializing", 2: "brake_open", 3: "power_off", 4: "disabled", 5: "idle",
    6: "drag", 7: "moving", 8: "recording", 9: "error", 10: "paused", 11: "jogging",
}


class DobotM1Pro:
    simulated = False

    def __init__(self, config: M1ProConfig, *, socket_factory=None, monotonic=None, sleep=None):
        if not isinstance(config, M1ProConfig):
            raise ValueError("DobotM1Pro necesita un M1ProConfig validado.")
        self.config = config
        self._socket_factory = socket_factory
        self._now = monotonic or time.monotonic
        self._sleep = sleep or time.sleep
        self._dashboard = None
        self._move = None
        self._pending = None
        self._motion_failed = False
        self._observed_queries = set()

    def _open(self, port):
        factory = self._socket_factory or socket.create_connection
        handle = None
        try:
            handle = factory((self.config.host, port), timeout=self.config.connect_timeout_s)
            handle.settimeout(self.config.command_timeout_s)
            return ASCIIChannel(handle, monotonic=self._now)
        except OSError as error:
            if handle is not None:
                try:
                    handle.close()
                except OSError:
                    pass  # Conservar el error de apertura original.
            raise ConnectionLost(f"No se pudo abrir el canal TCP/IP {port}: {error}.") from error

    def connect(self):
        if self.config.host is None:
            raise ValueError("host del M1 Pro pendiente; no se abre ninguna conexión.")
        if self._dashboard is None or self._dashboard.closed:
            self._dashboard = self._open(self.config.dashboard_port)
        # Conectar no abre movimiento/feedback ni envía EnableRobot/ClearError.

    def _require_connection(self):
        if self._dashboard is None or self._dashboard.closed:
            raise ConnectionLost("Dashboard M1 Pro desconectado.")

    def _query(self, command, *, deadline=None):
        self._require_connection()
        command_deadline = self._now() + self.config.command_timeout_s
        if deadline is not None:
            command_deadline = min(command_deadline, deadline)
        reply = self._dashboard.request(command, deadline=command_deadline)
        self._observed_queries.add(command.split("(", 1)[0])
        return reply

    def identity(self):
        self._require_connection()
        return {
            "simulated": False, "adapter": "DobotM1Pro", "transport": "TCP/IP-4Axis",
            "target_model": self.config.target_model,
            "model": None, "model_verified": False,
            "firmware_version": None, "firmware_verified": False,
            "firmware_version_declared": self.config.firmware_version,
            "identity_limitation": "No se verificó una consulta de modelo/firmware en esta API; TCP o eco no los identifican.",
            "compatible_confirmed_by_configuration": self.config.compatible_confirmed,
            "compatibility_evidence": self.config.compatibility_evidence,
            "sdk_source": "https://github.com/Dobot-Arm/TCP-IP-4Axis-Python",
            "sdk_commit": SDK_COMMIT, "protocol_commit": PROTOCOL_COMMIT,
            "tcp_connected": True, "successful_queries": sorted(self._observed_queries),
            "capabilities": {
                "inspection": True, "binary_feedback": False,
                "binary_feedback_reason": "Formato/versión del controlador local no verificados; puerto no abierto.",
                "arrival_confirmation": "bounded_Sync_then_fresh_GetPose_RobotMode_GetErrorID",
                "automatic_enable": False, "generic_stop": False,
                "emergency_stop": "explicitly_authorized_only; powers_off_and_alarms",
                "real_sensor_integration": False,
            },
        }

    def _pose(self, *, deadline=None):
        calibration = self.config.calibration
        command = "GetPose()" if calibration is None else f"GetPose(User={calibration.user_index},Tool={calibration.tool_index})"
        reply = self._query(command, deadline=deadline)
        parts = reply.payload.split(",")
        if len(parts) != 4:
            raise MotionError("GetPose no devolvió exactamente X,Y,Z,R del protocolo cuatro ejes.")
        try:
            values = tuple(float(part) for part in parts)
        except ValueError as error:
            raise MotionError("GetPose devolvió coordenadas no numéricas.") from error
        if not all(math.isfinite(value) for value in values):
            raise MotionError("GetPose devolvió coordenadas no finitas.")
        return RobotPose(values[:3], values[3])

    def pose(self):
        return self._pose()

    def _mode(self, *, deadline=None):
        payload = self._query("RobotMode()", deadline=deadline).payload
        try:
            mode = int(payload)
        except ValueError as error:
            raise MotionError("RobotMode no devolvió un entero.") from error
        if str(mode) != payload.strip():
            raise MotionError("RobotMode no tiene el esquema entero documentado.")
        return mode

    def state(self):
        mode = self._mode()
        return MODES.get(mode, f"unknown:{mode}")

    def get_error_ids(self, *, deadline=None):
        payload = self._query("GetErrorID()", deadline=deadline).payload
        try:
            groups = json.loads(payload)
        except (ValueError, RecursionError) as error:
            raise MotionError("GetErrorID no devolvió una lista de grupos válida.") from error
        # El manual muestra siete grupos en el retorno y describe cuatro servos
        # más controlador (cinco). Admitir solo estas dos formas explícitas.
        if not isinstance(groups, list) or len(groups) not in (5, 7) or any(
            not isinstance(group, list) or any(type(code) is not int for code in group)
            for group in groups
        ):
            raise MotionError("GetErrorID no cumple el esquema de grupos de enteros.")
        return groups

    def inspect(self):
        """Solo lectura: devuelve valores y evidencia sin cambiar el estado."""
        self._require_connection()
        mode = self._mode()
        pose = self._pose()
        alarms = self.get_error_ids()
        return {
            "identity": self.identity(), "robot_mode": mode,
            "state": MODES.get(mode, f"unknown:{mode}"),
            "pose": {"position_mm": list(pose.position_mm), "angle_deg": pose.angle_deg},
            "pose_frame_evidence": "GetPose con User/Tool declarados" if self.config.calibration is not None else "marcos globales activos del controlador; no calibrados/verificados",
            "error_ids": alarms, "read_only": True, "hardware_connected": True,
            "compatibility_proven_by_connection": False,
        }

    def preflight_movement(self):
        self.config.preflight_movement()

    def _authorize(self, authorize):
        if authorize is not True:
            raise MotionError("Operación física bloqueada: requiere authorize=True explícito en esta llamada.")
        try:
            self.config.preflight_movement()
        except ValueError as error:
            raise MotionError(str(error)) from error

    def _check_operational_state(self, *, deadline=None):
        alarms = self.get_error_ids(deadline=deadline)
        if any(alarms):
            raise MotionError(f"Alarmas/códigos no vacíos del controlador: {alarms}; no se interpretan como ausencia ni se limpian automáticamente.")
        mode = self._mode(deadline=deadline)
        if mode not in (5, 7):
            raise MotionError(f"RobotMode={mode} ({MODES.get(mode, 'desconocido')}) no permite confirmar movimiento.")
        return mode

    def move_to(self, position_mm, angle_deg, *, authorize=False):
        self._authorize(authorize)
        self._require_connection()
        if self._motion_failed or self._pending is not None:
            raise MotionError("Existe un movimiento pendiente/incierto; no se envía otro destino.")
        position = _vector(position_mm, "position_mm")
        angle = _number(angle_deg, "angle_deg")
        if not math.isclose(angle, self.config.calibration.fixed_angle_deg, rel_tol=0, abs_tol=1e-9):
            raise MotionError("R debe coincidir con el ángulo fijo calibrado.")
        try:
            if self._check_operational_state() != 5:
                raise MotionError("El robot no está habilitado y detenido antes de ordenar movimiento.")
            if self._move is None:
                self._move = self._open(self.config.move_port)
            calibration = self.config.calibration
            coordinates = ",".join(format(value, ".12g") for value in (*position, angle))
            command = (
                f"MovL({coordinates},User={calibration.user_index},Tool={calibration.tool_index},"
                f"SpeedL={self.config.speed_l_percent},AccL={self.config.acc_l_percent},CP=0)"
            )
            reply = self._move.request(command, deadline=self._now() + self.config.command_timeout_s)
            if reply.payload:
                raise MotionError("MovL devolvió un payload no documentado para este perfil.")
            self._pending = RobotPose(position, angle)
        except BaseException:
            self._motion_failed = True
            raise

    def wait_until_arrived(
        self, target_position_mm, angle_deg, timeout_s,
        position_tolerance_mm, angle_tolerance_deg, stable_time_s,
    ):
        self._require_connection()
        target = RobotPose(_vector(target_position_mm, "target_position_mm"), _number(angle_deg, "angle_deg"))
        timeout = _number(timeout_s, "timeout_s", positive=True)
        position_tolerance = _number(position_tolerance_mm, "position_tolerance_mm", positive=True)
        angle_tolerance = _number(angle_tolerance_deg, "angle_tolerance_deg", positive=True)
        stable_time = _number(stable_time_s, "stable_time_s")
        if stable_time < 0:
            raise ValueError("stable_time_s no puede ser negativo.")
        if self._pending != target or self._move is None or self._motion_failed:
            raise MotionError("No existe un MovL aceptado para ese destino; una pose previa no confirma llegada.")
        deadline = self._now() + timeout
        stable_since = None
        try:
            if self._check_operational_state(deadline=deadline) not in (5, 7):
                raise MotionError("Estado incompatible antes de Sync.")
            reply = self._move.request("Sync()", deadline=deadline)
            if reply.payload:
                raise MotionError("Sync devolvió un payload no documentado.")
            while self._now() < deadline:
                observed = self._pose(deadline=deadline)
                mode = self._check_operational_state(deadline=deadline)
                angle_error = abs((observed.angle_deg - target.angle_deg + 180) % 360 - 180)
                valid = mode == 5 and math.dist(observed.position_mm, target.position_mm) <= position_tolerance and angle_error <= angle_tolerance
                if valid:
                    if stable_since is None:
                        stable_since = self._now()
                    if self._now() - stable_since >= stable_time:
                        self._pending = None
                        return observed
                else:
                    stable_since = None
                self._sleep(min(self.config.poll_interval_s, max(0.0, deadline - self._now())))
            raise MotionTimeout("No se confirmó llegada estable dentro de tolerancias y plazo.")
        except BaseException:
            self._motion_failed = True
            raise

    def stop(self, *, authorize=False):
        self._authorize(authorize)
        if not self.config.emergency_stop_confirmed:
            raise MotionError("Parada genérica no disponible; EmergencyStop corta alimentación y requiere confirmación específica.")
        self._require_connection()
        # Incluso sin respuesta, el corte pudo ejecutarse: reconectar no autoriza
        # continuar con otro destino desde ese estado incierto.
        self._motion_failed = True
        reply = self._query("EmergencyStop()")
        if reply.payload:
            raise MotionError("EmergencyStop devolvió un payload no documentado.")
        self._pending = None
        # La aceptación no verifica detención física ni sustituye un paro cableado.

    def disconnect(self):
        first_error = None
        for channel in (self._move, self._dashboard):
            if channel is not None:
                try:
                    channel.close()
                except OSError as error:
                    first_error = first_error or error
        self._move = self._dashboard = None
        # No borrar el bloqueo de movimiento incierto ni enviar retorno/home.
        if first_error is not None:
            raise ConnectionLost(f"Error al cerrar canales TCP/IP: {first_error}.") from first_error


def inspect_robot(config: M1ProConfig, **test_dependencies):
    robot = DobotM1Pro(config, **test_dependencies)
    try:
        robot.connect()
        return robot.inspect()
    finally:
        robot.disconnect()
