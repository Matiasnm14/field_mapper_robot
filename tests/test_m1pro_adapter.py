"""Protocolo real probado solo contra sockets en memoria, nunca en el robot."""

from collections import deque
from dataclasses import replace
import importlib
import socket

import pytest

from field_mapper.clock import VirtualClock
from field_mapper.errors import ConnectionLost, MotionError, MotionTimeout
from field_mapper.robot.dobot_m1pro import DobotM1Pro, inspect_robot
from field_mapper.robot.m1pro_transport import ASCIIChannel, parse_reply
from test_m1pro_config import pending_config, ready_config, ready_data  # Fixtures compartidas.


class MemorySocket:
    def __init__(self, replies=None, chunks=()):
        self.replies = replies
        self.chunks = deque(chunks)
        self.sent = []
        self.closed = False
        self.timeout = None

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendall(self, data):
        assert not self.closed
        self.sent.append(data.decode("ascii"))
        if self.replies is not None:
            reply = self.replies(self.sent[-1])
            self.chunks.extend(reply if isinstance(reply, list) else [reply])

    def recv(self, count):
        if not self.chunks:
            raise socket.timeout("timeout fake")
        result = self.chunks.popleft()
        if isinstance(result, BaseException):
            raise result
        return result

    def close(self):
        self.closed = True


class Controller:
    def __init__(self, config, **kwargs):
        self.clock = VirtualClock()
        self.mode = 5
        self.alarms = "[[],[],[],[],[]]"
        self.position = "1,2,3,10"
        self.sync_delay = 0
        self.sync_chunks = None
        self.calls = []
        self.handles = []
        self.config = config
        self.robot = DobotM1Pro(config, socket_factory=self.open,
                               monotonic=self.clock.now, sleep=self.clock.sleep, **kwargs)

    def reply(self, command):
        payload = ""
        if command == "RobotMode()":
            payload = str(self.mode)
        elif command.startswith("GetPose("):
            return f"0,{{{self.position}}},GetPose();".encode()
        elif command == "GetErrorID()":
            payload = self.alarms
        elif command == "Sync()":
            self.clock.sleep(self.sync_delay)
            if self.sync_chunks is not None:
                return self.sync_chunks
        elif command.startswith("MovL("):
            assert self.calls[-1][1] == self.config.move_port
        elif command != "EmergencyStop()":
            pytest.fail(f"Comando no autorizado en fixture: {command}")
        return f"0,{{{payload}}},{command};".encode()

    def open(self, address, *, timeout):
        assert address[0] == "test.invalid"
        self.calls.append(address)
        handle = MemorySocket(self.reply)
        self.handles.append(handle)
        return handle

    def connect(self):
        self.robot.connect()
        return self.robot


@pytest.fixture(autouse=True)
def prohibit_real_network(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Una prueba intentó contactar hardware real.")
    monkeypatch.setattr(socket, "create_connection", forbidden)


def test_import_and_instantiation_never_open_sockets(pending_config):
    importlib.reload(importlib.import_module("field_mapper.robot.dobot_m1pro"))
    # La recarga usa las mismas funciones; no importa ni instancia el SDK oficial.
    DobotM1Pro(pending_config)


def test_pending_host_blocks_even_inspection_before_open(pending_config):
    with pytest.raises(ValueError, match="host"):
        inspect_robot(pending_config)


def test_inspection_is_read_only_opens_dashboard_only_and_closes(pending_config):
    config = replace(pending_config, host="test.invalid")
    fixture = Controller(config)
    result = inspect_robot(config, socket_factory=fixture.open,
                           monotonic=fixture.clock.now, sleep=fixture.clock.sleep)
    assert fixture.calls == [("test.invalid", config.dashboard_port)]
    assert fixture.handles[0].sent == ["RobotMode()", "GetPose()", "GetErrorID()"]
    assert fixture.handles[0].closed
    assert result["read_only"] and result["state"] == "idle"
    identity = result["identity"]
    assert identity["model"] is None and identity["firmware_version"] is None
    assert not identity["model_verified"] and not identity["firmware_verified"]
    assert not identity["capabilities"]["binary_feedback"]
    assert not result["compatibility_proven_by_connection"]


def test_inspection_closes_on_malformed_payload(pending_config):
    fixture = Controller(replace(pending_config, host="test.invalid"))
    fixture.position = "nan,0,0,0"
    with pytest.raises(MotionError, match="no finitas"):
        inspect_robot(fixture.config, socket_factory=fixture.open)
    assert fixture.handles[0].closed


def test_connect_never_enables_or_opens_motion_and_disconnect_does_not_home(ready_config):
    fixture = Controller(ready_config)
    robot = fixture.connect()
    assert len(fixture.calls) == 1 and fixture.handles[0].sent == []
    robot.disconnect()
    assert fixture.handles[0].sent == [] and fixture.handles[0].closed


@pytest.mark.parametrize("operation", ["move", "stop"])
def test_physical_operations_require_explicit_per_call_authorization(ready_config, operation):
    fixture = Controller(ready_config)
    robot = fixture.connect()
    with pytest.raises(MotionError, match="authorize=True"):
        robot.move_to((1, 2, 3), 10) if operation == "move" else robot.stop()
    assert fixture.handles[0].sent == []


def test_profile_flags_alone_do_not_prove_observed_model_or_firmware(ready_config):
    robot = Controller(ready_config).connect()
    identity = robot.identity()
    assert identity["firmware_version_declared"] == "V1.6.0.0"
    assert identity["firmware_version"] is None and identity["model"] is None


def test_movement_requires_complete_config_even_when_authorized(pending_config):
    fixture = Controller(replace(pending_config, host="test.invalid"))
    robot = fixture.connect()
    with pytest.raises(MotionError, match="Movimiento bloqueado"):
        robot.move_to((1, 2, 3), 10, authorize=True)
    assert fixture.handles[0].sent == []
    assert len(fixture.calls) == 1


def wait(robot, **kwargs):
    arguments = dict(target_position_mm=(1, 2, 3), angle_deg=10, timeout_s=1,
                     position_tolerance_mm=.01, angle_tolerance_deg=.01, stable_time_s=.1)
    arguments.update(kwargs)
    return robot.wait_until_arrived(**arguments)


def test_movl_keeps_fixed_r_and_requires_sync_then_fresh_stable_queries(ready_config):
    fixture = Controller(ready_config)
    robot = fixture.connect()
    robot.move_to((1, 2, 3), 10, authorize=True)
    assert fixture.calls == [("test.invalid", 29999), ("test.invalid", 30003)]
    motion = fixture.handles[1]
    assert motion.sent == ["MovL(1,2,3,10,User=0,Tool=0,SpeedL=1,AccL=1,CP=0)"]
    assert fixture.clock.now() == 0  # Aceptado, sin llegada confirmada todavía.
    pose = wait(robot)
    assert pose.position_mm == (1, 2, 3)
    assert motion.sent[-1] == "Sync()"
    assert fixture.clock.now() >= .1
    assert fixture.handles[0].sent.count("GetPose(User=0,Tool=0)") >= 2


def test_fixed_r_change_blocked_before_commands(ready_config):
    fixture = Controller(ready_config)
    with pytest.raises(MotionError, match="ángulo fijo"):
        fixture.connect().move_to((1, 2, 3), 11, authorize=True)
    assert fixture.handles[0].sent == []


def test_old_pose_without_accepted_movl_cannot_confirm_arrival(ready_config):
    fixture = Controller(ready_config)
    with pytest.raises(MotionError, match="No existe"):
        wait(fixture.connect())
    assert fixture.handles[0].sent == []


@pytest.mark.parametrize("mode", [1, 2, 3, 4, 6, 8, 9, 10, 11, 99, 7])
def test_bad_mode_blocks_new_destination_without_clearing_or_enabling(ready_config, mode):
    fixture = Controller(ready_config)
    fixture.mode = mode
    robot = fixture.connect()
    with pytest.raises(MotionError):
        robot.move_to((1, 2, 3), 10, authorize=True)
    assert len(fixture.calls) == 1
    assert set(fixture.handles[0].sent) <= {"GetErrorID()", "RobotMode()"}


def test_controller_alarm_blocks_before_movl_without_clearerror(ready_config):
    fixture = Controller(ready_config)
    fixture.alarms = "[[123],[],[],[],[]]"
    with pytest.raises(MotionError, match="Alarmas"):
        fixture.connect().move_to((1, 2, 3), 10, authorize=True)
    assert fixture.handles[0].sent == ["GetErrorID()"]


@pytest.mark.parametrize("kind", ["position", "angle", "moving", "alarm", "sync_timeout"])
def test_arrival_faults_block_following_destinations(ready_config, kind):
    fixture = Controller(ready_config)
    robot = fixture.connect()
    robot.move_to((1, 2, 3), 10, authorize=True)
    if kind == "position":
        fixture.position = "2,2,3,10"
    elif kind == "angle":
        fixture.position = "1,2,3,11"
    elif kind == "moving":
        fixture.mode = 7
    elif kind == "alarm":
        fixture.alarms = "[[-2],[],[],[],[]]"
    else:
        fixture.sync_chunks = [socket.timeout("sync fake")]
    with pytest.raises(MotionError):
        wait(robot, timeout_s=.2)
    assert fixture.clock.now() <= .2
    with pytest.raises(MotionError, match="pendiente/incierto"):
        robot.move_to((4, 5, 6), 10, authorize=True)
    assert sum(command.startswith("MovL") for command in fixture.handles[1].sent) == 1


def test_sync_cannot_exceed_total_arrival_deadline(ready_config):
    fixture = Controller(ready_config)
    robot = fixture.connect()
    robot.move_to((1, 2, 3), 10, authorize=True)
    fixture.sync_delay = .8
    with pytest.raises(MotionTimeout):
        wait(robot, timeout_s=.5)
    assert fixture.handles[1].closed


def test_unconfirmed_emergency_stop_blocks_movement_as_well(ready_config):
    fixture = Controller(replace(ready_config, emergency_stop_confirmed=False))
    robot = fixture.connect()
    for operation in (lambda: robot.move_to((1, 2, 3), 10, authorize=True),
                      lambda: robot.stop(authorize=True)):
        with pytest.raises(MotionError, match="EmergencyStop"):
            operation()
    assert fixture.handles[0].sent == []


def test_emergency_stop_is_explicit_and_never_clears_its_alarm(ready_config):
    fixture = Controller(ready_config)
    robot = fixture.connect()
    robot.stop(authorize=True)
    assert fixture.handles[0].sent == ["EmergencyStop()"]
    with pytest.raises(MotionError, match="incierto"):
        robot.move_to((1, 2, 3), 10, authorize=True)


def test_uncertain_emergency_stop_stays_blocked_after_reconnect(ready_config):
    fixture = Controller(ready_config)
    robot = fixture.connect()
    fixture.handles[0].replies = lambda _: socket.timeout("respuesta de parada perdida")
    with pytest.raises(MotionTimeout):
        robot.stop(authorize=True)
    robot.connect()
    with pytest.raises(MotionError, match="incierto"):
        robot.move_to((1, 2, 3), 10, authorize=True)
    assert len(fixture.calls) == 2  # Solo se reabre Dashboard, nunca movimiento.


@pytest.mark.parametrize("payload", ["[[],[],[],[],[]]", "[[],[],[],[],[],[],[]]"])
def test_documented_alarm_group_variants_preserve_their_shape(ready_config, payload):
    fixture = Controller(ready_config)
    fixture.alarms = payload
    groups = fixture.connect().get_error_ids()
    assert len(groups) in (5, 7) and not any(groups)


def test_zero_alarm_code_does_not_invent_absence_of_alarms(ready_config):
    fixture = Controller(ready_config)
    fixture.alarms = "[[0],[],[],[],[]]"
    with pytest.raises(MotionError, match="códigos no vacíos"):
        fixture.connect().move_to((1, 2, 3), 10, authorize=True)
    assert len(fixture.calls) == 1


def test_ascii_fragments_and_concatenated_messages_are_framed_separately():
    socket = MemorySocket(chunks=[b"0,{5},Robo", b"tMode();0,{1,2,3,4},GetPose();"])
    channel = ASCIIChannel(socket, monotonic=lambda: 0)
    first = channel.read_message(deadline=1)
    second = channel.read_message(deadline=1)
    assert parse_reply(first, "RobotMode()").payload == "5"
    assert parse_reply(second, "GetPose()").payload == "1,2,3,4"


def test_buffered_unsolicited_reply_is_not_a_new_pose_response():
    handle = MemorySocket(replies=lambda _: b"0,{5},RobotMode();0,{5},RobotMode();")
    channel = ASCIIChannel(handle, monotonic=lambda: 0)
    with pytest.raises(MotionError, match="frescura"):
        channel.request("RobotMode()", deadline=1)
    assert handle.closed and len(handle.sent) == 1


@pytest.mark.parametrize("chunks,error", [
    ([b""], ConnectionLost), ([socket.timeout("fake")], MotionTimeout),
    ([b"malformed;"], MotionError), ([b"0,{5},GetPose();"], MotionError),
    ([b"-10000,{},RobotMode();"], MotionError), ([b"\xff;"], MotionError),
])
def test_bad_tcp_results_close_channel_without_automatic_retry(chunks, error):
    handle = MemorySocket(chunks=chunks)
    channel = ASCIIChannel(handle, monotonic=lambda: 0)
    with pytest.raises(error):
        channel.request("RobotMode()", deadline=1)
    assert handle.closed and handle.sent == ["RobotMode()"]
    with pytest.raises(ConnectionLost):
        channel.request("RobotMode()", deadline=1)
    assert len(handle.sent) == 1


def test_oversized_and_unterminated_reply_has_limit():
    handle = MemorySocket(chunks=[b"x" * 65])
    channel = ASCIIChannel(handle, monotonic=lambda: 0, max_message_bytes=64)
    with pytest.raises(MotionError, match="tamaño"):
        channel.request("RobotMode()", deadline=1)
    assert handle.closed


@pytest.mark.parametrize("payload", ["{1,2,3}", "{1,2,3,4,5,6}", "{1,2,nan,4}", "{x,2,3,4}"])
def test_pose_requires_exact_four_axis_finite_schema(ready_config, payload):
    fixture = Controller(ready_config)
    robot = fixture.connect()
    fixture.handles[0].replies = lambda _: f"0,{payload},GetPose();".encode()
    with pytest.raises(MotionError):
        robot.pose()


@pytest.mark.parametrize("payload", ["[]", "[1,2]", "[[true]]", "null", "[[]", "{}"])
def test_alarm_schema_must_be_groups_of_integer_ids(ready_config, payload):
    fixture = Controller(ready_config)
    fixture.alarms = payload
    with pytest.raises(MotionError):
        fixture.connect().get_error_ids()
