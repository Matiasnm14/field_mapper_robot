"""Revisión independiente: inspección, recursos y plazos del protocolo ASCII."""

from dataclasses import replace
from pathlib import Path
import socket
import threading
import time

import pytest

from field_mapper.errors import ConnectionLost, MotionError, MotionTimeout
from field_mapper.hardware_config import M1ProCalibration, load_m1pro_config
from field_mapper.robot.dobot_m1pro import DobotM1Pro, inspect_robot
from field_mapper.robot.m1pro_transport import ASCIIChannel, parse_reply


@pytest.fixture
def hardware_profile():
    source = Path(__file__).resolve().parents[1] / "configs/m1pro_lab.json"
    config = load_m1pro_config(source)
    calibration = M1ProCalibration(
        confirmed=True, source="Calibración ficticia, solo prueba de protocolo",
        T_base_magnet=((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
        rotation_base_flange=((1, 0, 0), (0, 1, 0), (0, 0, 1)),
        offset_flange_sensor_mm=(0, 0, 0), pose_frame="flange", fixed_angle_deg=0,
        sensitive_axis_magnet=(0, 0, 1), user_index=0, tool_index=0,
        coordinate_frames_verified=True, paths_verified=True,
    )
    return replace(
        config, host="127.0.0.1", firmware_version="V1.6.0.0",
        compatible_confirmed=True, compatibility_evidence="Controlador ficticio de pruebas",
        motion_enabled=True, emergency_stop_confirmed=True,
        speed_l_percent=10, acc_l_percent=10, calibration=calibration,
    )


class InspectionSocket:
    def __init__(self):
        self.sent = []
        self.pending = b""
        self.closed = False

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendall(self, command):
        self.sent.append(command.decode("ascii"))
        if command.startswith(b"RobotMode("):
            payload = "5"
        elif command.startswith(b"GetPose("):
            payload = "1,2,3,4"
        elif command.startswith(b"GetErrorID("):
            payload = "[[],[],[],[],[],[],[]]"
        else:
            raise AssertionError(f"La inspección envió un comando de control: {command!r}")
        self.pending = f"0,{{{payload}}},{command.decode('ascii')};".encode("ascii")

    def recv(self, size):
        result, self.pending = self.pending[:size], self.pending[size:]
        return result

    def close(self):
        self.closed = True


def test_inspection_only_opens_dashboard_and_sends_read_only_commands(hardware_profile):
    handle = InspectionSocket()
    opened = []

    def factory(address, *, timeout):
        opened.append(address)
        return handle

    inspection = inspect_robot(hardware_profile, socket_factory=factory)
    assert opened == [(hardware_profile.host, hardware_profile.dashboard_port)]
    assert handle.sent == ["RobotMode()", "GetPose(User=0,Tool=0)", "GetErrorID()"]
    assert handle.closed
    assert inspection["read_only"] is True
    assert inspection["compatibility_proven_by_connection"] is False
    assert inspection["identity"]["model_verified"] is False
    assert inspection["identity"]["firmware_verified"] is False
    assert inspection["identity"]["firmware_version"] is None


def test_failed_socket_setup_closes_the_already_open_handle(hardware_profile):
    class ConfigurationFailureSocket(InspectionSocket):
        def settimeout(self, timeout):
            raise OSError("fallo al configurar socket recién abierto")

    handle = ConfigurationFailureSocket()
    robot = DobotM1Pro(hardware_profile, socket_factory=lambda *args, **kwargs: handle)
    with pytest.raises(ConnectionLost, match="configurar socket"):
        robot.connect()
    assert handle.closed


def test_a_queued_request_must_expire_before_the_inflight_request_finishes():
    entered_recv = threading.Event()
    release_recv = threading.Event()
    second_finished = threading.Event()
    errors = []

    class BlockingSocket(InspectionSocket):
        def recv(self, size):
            entered_recv.set()
            if not release_recv.wait(self.timeout):
                raise socket.timeout("espera ficticia")
            return super().recv(size)

    channel = ASCIIChannel(BlockingSocket())

    def first_request():
        try:
            channel.request("RobotMode()", deadline=time.monotonic() + 2)
        except BaseException as error:
            errors.append(("first", error))

    def short_request():
        try:
            channel.request("RobotMode()", deadline=time.monotonic() + 0.02)
        except BaseException as error:
            errors.append(("second", error))
        finally:
            second_finished.set()

    first = threading.Thread(target=first_request, daemon=True)
    second = threading.Thread(target=short_request, daemon=True)
    first.start()
    try:
        assert entered_recv.wait(1)
        second.start()
        assert second_finished.wait(0.5), "El plazo de una petición incluye la espera por el canal ocupado."
        assert any(name == "second" and isinstance(error, MotionTimeout) for name, error in errors)
    finally:
        release_recv.set()
        first.join(timeout=2)
        if second.ident is not None:
            second.join(timeout=2)
        channel.close()


@pytest.mark.parametrize("echo,expected", [
    ("MovL(1,,2,3,4)", "MovL(1,2,3,4)"),
    ("MovL(1,2,3,4,)", "MovL(1,2,3,4)"),
    ("GetPose(,)", "GetPose()"),
    ("RobotMode(,)", "RobotMode()"),
])
def test_empty_argument_fields_do_not_turn_malformed_echo_into_valid_reply(echo, expected):
    with pytest.raises(MotionError):
        parse_reply(f"0,{{}},{echo};", expected)
