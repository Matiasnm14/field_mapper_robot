"""Perfiles de hardware pendientes y permisos sin contactar ningún equipo."""

from dataclasses import replace
import json
from pathlib import Path

import pytest

from field_mapper.hardware_config import load_m1pro_config, m1pro_config_from_dict


@pytest.fixture
def pending_config():
    return load_m1pro_config(Path(__file__).resolve().parents[1] / "configs/m1pro_lab.json")


def ready_data(pending_config):
    """Declaraciones ficticias usadas únicamente con sockets dobles de pruebas."""
    data = pending_config.to_dict()
    data.update(host="test.invalid", firmware_version="V1.6.0.0", compatible_confirmed=True,
                compatibility_evidence="fixture sin hardware", motion_enabled=True,
                speed_l_percent=1, acc_l_percent=1, emergency_stop_confirmed=True)
    data["calibration"] = {
        "confirmed": True, "source": "fixture sin validez física",
        "T_base_magnet": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        "rotation_base_flange": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "offset_flange_sensor_mm": [0, 0, 0], "pose_frame": "flange",
        "fixed_angle_deg": 10, "sensitive_axis_magnet": [0, 0, 1],
        "user_index": 0, "tool_index": 0,
        "coordinate_frames_verified": True, "paths_verified": True,
    }
    return data


@pytest.fixture
def ready_config(pending_config):
    return m1pro_config_from_dict(ready_data(pending_config))


def test_shipped_profile_stays_pending_and_contains_only_documented_ports(pending_config):
    assert pending_config.host is None
    assert pending_config.firmware_version is None
    assert not pending_config.compatible_confirmed and not pending_config.motion_enabled
    assert pending_config.calibration is None
    assert (pending_config.dashboard_port, pending_config.move_port, pending_config.feedback_port) == (29999, 30003, 30004)
    with pytest.raises(ValueError, match="Movimiento bloqueado"):
        pending_config.preflight_movement()


def test_profile_roundtrip_is_strict_and_preserves_pending_values(pending_config, ready_config, tmp_path):
    for config in (pending_config, ready_config):
        path = tmp_path / "test.json"
        path.write_text(json.dumps(config.to_dict()), encoding="utf-8")
        assert load_m1pro_config(path) == config
    ready_config.preflight_movement()


@pytest.mark.parametrize("field,value", [
    ("host", ""), ("host", " robot"), ("host", "tcp://127.0.0.1"), ("host", 3),
    ("dashboard_port", True), ("dashboard_port", 0), ("move_port", 65536),
    ("feedback_port", 30003), ("connect_timeout_s", 0), ("poll_interval_s", -1),
    ("command_timeout_s", float("inf")), ("poll_interval_s", float("nan")),
    ("command_timeout_s", "2"), ("connect_timeout_s", True),
    ("schema_version", True), ("target_model", "M1"), ("mode", "simulation"),
    ("firmware_version", "unknown"), ("firmware_version", 1560),
    ("compatible_confirmed", 1), ("motion_enabled", "true"),
    ("emergency_stop_confirmed", 1), ("compatibility_evidence", ""),
    ("speed_l_percent", 0), ("speed_l_percent", 101), ("acc_l_percent", 1.0),
])
def test_invalid_fields_rejected_before_connect(pending_config, field, value):
    data = pending_config.to_dict()
    data[field] = value
    with pytest.raises(ValueError):
        m1pro_config_from_dict(data)


@pytest.mark.parametrize("field,value", [
    ("host", None), ("firmware_version", None), ("firmware_version", "V1.5.5.0"),
    ("compatible_confirmed", False), ("compatibility_evidence", None),
    ("motion_enabled", False), ("speed_l_percent", None), ("acc_l_percent", None),
    ("emergency_stop_confirmed", False), ("calibration", None),
])
def test_each_pending_requirement_blocks_movement(ready_config, field, value):
    with pytest.raises(ValueError, match="Movimiento bloqueado"):
        replace(ready_config, **{field: value}).preflight_movement()


@pytest.mark.parametrize("field", ["confirmed", "coordinate_frames_verified", "paths_verified"])
def test_calibration_declarations_must_be_confirmed(ready_config, field):
    calibration = replace(ready_config.calibration, **{field: False})
    with pytest.raises(ValueError, match="Movimiento bloqueado"):
        replace(ready_config, calibration=calibration).preflight_movement()


@pytest.mark.parametrize("field,value", [
    ("confirmed", 1), ("source", ""), ("offset_flange_sensor_mm", [0, 0]),
    ("offset_flange_sensor_mm", [0, float("nan"), 0]),
    ("sensitive_axis_magnet", [1e308, 0, 0]),
    ("sensitive_axis_magnet", [0, 0, 0]), ("fixed_angle_deg", True),
    ("user_index", 1), ("tool_index", -1), ("tool_index", 1.0),
    ("rotation_base_flange", [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]),
    ("T_base_magnet", [[1, 0, 0, 0]]), ("pose_frame", "unknown"),
])
def test_invalid_calibration_rejected(pending_config, field, value):
    data = ready_data(pending_config)
    data["calibration"][field] = value
    with pytest.raises(ValueError):
        m1pro_config_from_dict(data)


@pytest.mark.parametrize("kind", ["missing", "unknown", "wrong_container"])
def test_profile_schema_is_strict(pending_config, kind):
    data = pending_config.to_dict()
    if kind == "missing":
        data.pop("host")
    elif kind == "unknown":
        data["unused"] = True
    else:
        data = []
    with pytest.raises(ValueError):
        m1pro_config_from_dict(data)


@pytest.mark.parametrize("raw", ['{"host":null,"host":"test.invalid"}', '{"timeout":NaN}', '{"timeout":Infinity}'])
def test_ambiguous_json_rejected(raw, tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(raw, encoding="utf-8")
    with pytest.raises(ValueError):
        load_m1pro_config(path)
