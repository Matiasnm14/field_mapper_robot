"""La CLI inspecciona solo por petición y rechaza el barrido real incompleto."""

import json
from pathlib import Path
from types import SimpleNamespace

from field_mapper.cli import main
from field_mapper.errors import ConnectionLost


def test_placeholder_inspection_does_not_open_sockets(monkeypatch, capsys):
    import socket

    def forbidden(*args, **kwargs):
        raise AssertionError("Un perfil sin IP no debe abrir sockets.")

    monkeypatch.setattr(socket, "create_connection", forbidden)
    monkeypatch.setattr(socket.socket, "connect", forbidden)
    path = Path(__file__).resolve().parents[1] / "configs/m1pro_lab.json"
    assert main(["inspect-robot", "--config", str(path)]) == 2
    assert "Error:" in capsys.readouterr().err


def test_inspection_calls_read_only_entrypoint_and_outputs_unknown_firmware(monkeypatch, capsys):
    from field_mapper import hardware_config
    from field_mapper.robot import dobot_m1pro

    profile = object()
    monkeypatch.setattr(hardware_config, "load_m1pro_config", lambda path: profile)
    calls = []

    def inspect(config):
        calls.append(config)
        return {"firmware_version": None, "simulated": False, "pose": [1, 2, 3, 4]}

    monkeypatch.setattr(dobot_m1pro, "inspect_robot", inspect)
    assert main(["inspect-robot", "--config", "local.json"]) == 0
    assert calls == [profile]
    assert json.loads(capsys.readouterr().out)["firmware_version"] is None


def test_inspection_reports_connection_failure(monkeypatch, capsys):
    from field_mapper import hardware_config
    from field_mapper.robot import dobot_m1pro

    monkeypatch.setattr(hardware_config, "load_m1pro_config", lambda path: object())

    def disconnected(config):
        raise ConnectionLost("El controlador cerró la conexión.")

    monkeypatch.setattr(dobot_m1pro, "inspect_robot", disconnected)
    assert main(["inspect-robot", "--config", "local.json"]) == 1
    assert "controlador" in capsys.readouterr().err


def test_scan_checks_pending_configuration_without_connecting(monkeypatch, capsys):
    from field_mapper import hardware_config
    import socket

    def unavailable():
        raise ValueError("Calibración pendiente.")

    def forbidden(*args, **kwargs):
        raise AssertionError("scan incompleto no debe abrir sockets.")

    monkeypatch.setattr(hardware_config, "load_m1pro_config", lambda path: SimpleNamespace(preflight_movement=unavailable))
    monkeypatch.setattr(socket, "create_connection", forbidden)
    monkeypatch.setattr(socket.socket, "connect", forbidden)
    assert main(["scan", "--config", "local.json", "--mode", "hardware"]) == 2
    assert "Calibración pendiente" in capsys.readouterr().err


def test_scan_is_explicitly_unavailable_until_real_sensor_is_integrated(monkeypatch, capsys):
    from field_mapper import hardware_config
    import socket

    def forbidden(*args, **kwargs):
        raise AssertionError("No se debe conectar un barrido sin sensor real.")

    monkeypatch.setattr(hardware_config, "load_m1pro_config", lambda path: SimpleNamespace(preflight_movement=lambda: None))
    monkeypatch.setattr(socket, "create_connection", forbidden)
    monkeypatch.setattr(socket.socket, "connect", forbidden)
    assert main(["scan", "--config", "local.json", "--mode", "hardware"]) == 2
    assert "sensor real" in capsys.readouterr().err
