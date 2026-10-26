"""Validación de perfiles sintéticos sin restringir perfiles solo de plan."""

from dataclasses import MISSING, fields
import json
import math

import pytest

from field_mapper.config import SimulationConfig, config_from_dict, load_config
from field_mapper.planner import plan_trajectory


@pytest.mark.parametrize("section,field", [
    ("acquisition", "settle_time_s"), ("acquisition", "stable_time_s"),
    ("acquisition", "motion_timeout_s"), ("acquisition", "read_timeout_s"),
    ("acquisition", "position_tolerance_mm"), ("acquisition", "angle_tolerance_deg"),
    ("simulation", "angle_deg"), ("simulation", "speed_mm_s"),
    ("simulation", "move_latency_s"), ("simulation", "base_field_T"),
    ("simulation", "drift_T_per_s"), ("simulation", "noise_std_T"),
    ("simulation", "sample_interval_s"),
])
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, True, "0.01", None])
def test_numeric_fields_require_actual_finite_numbers(config, section, field, value):
    data = config.to_dict()
    data[section][field] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("section,field", [
    ("acquisition", "motion_timeout_s"), ("acquisition", "read_timeout_s"),
    ("acquisition", "position_tolerance_mm"), ("acquisition", "angle_tolerance_deg"),
    ("simulation", "speed_mm_s"), ("simulation", "sample_interval_s"),
])
@pytest.mark.parametrize("value", [0, -0.01])
def test_strictly_positive_fields_reject_zero_and_negative(config, section, field, value):
    data = config.to_dict()
    data[section][field] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("section,field", [
    ("acquisition", "settle_time_s"), ("acquisition", "stable_time_s"),
    ("simulation", "move_latency_s"), ("simulation", "noise_std_T"),
])
def test_zero_allowed_but_negative_rejected(config, section, field):
    data = config.to_dict()
    data[section][field] = 0
    assert getattr(getattr(config_from_dict(data), section), field) == 0
    data[section][field] = -1e-6
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("field", ["base_field_T", "drift_T_per_s", "angle_deg"])
def test_signed_synthetic_coefficients_and_angle_are_allowed(config, field):
    data = config.to_dict()
    data["simulation"][field] = -0.1
    assert getattr(config_from_dict(data).simulation, field) == -0.1


@pytest.mark.parametrize("section,field,value", [
    ("simulation", "seed", -1), ("simulation", "seed", True),
    ("simulation", "seed", 1.0), ("simulation", "seed", "42"),
    ("simulation", "seed", None), ("simulation", "seed", math.inf),
    ("acquisition", "samples_per_point", 0), ("acquisition", "samples_per_point", -1),
    ("acquisition", "samples_per_point", True), ("acquisition", "samples_per_point", 1.0),
    ("acquisition", "max_sensor_retries", -1), ("acquisition", "max_sensor_retries", 11),
    ("acquisition", "max_sensor_retries", True), ("acquisition", "max_sensor_retries", 1.0),
    ("simulation", "motion_fault_at_visit", -1), ("simulation", "motion_fault_at_visit", True),
    ("simulation", "motion_fault_at_visit", 0.0),
    ("simulation", "sensor_fault_at_read", 0), ("simulation", "sensor_fault_at_read", -1),
    ("simulation", "sensor_fault_at_read", True), ("simulation", "sensor_fault_at_read", 1.0),
])
def test_counts_seeds_and_fault_indices_are_strict_integers(config, section, field, value):
    data = config.to_dict()
    data[section][field] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


def test_seed_zero_no_retries_and_first_visit_failure_are_valid(config):
    data = config.to_dict()
    data["simulation"]["seed"] = 0
    data["simulation"]["motion_fault_at_visit"] = 0  # visit_id es base cero.
    data["simulation"]["sensor_fault_at_read"] = 1  # Contador de peticiones base uno.
    data["acquisition"]["max_sensor_retries"] = 0
    parsed = config_from_dict(data)
    assert parsed.simulation.seed == 0
    assert parsed.simulation.motion_fault_at_visit == 0
    assert parsed.simulation.sensor_fault_at_read == 1
    assert parsed.acquisition.max_sensor_retries == 0


@pytest.mark.parametrize("field", [
    "initial_sensor_position_magnet_mm", "offset_flange_sensor_mm",
    "sensitive_axis_magnet", "gradient_T_per_mm", "quadratic_T_per_mm2",
])
@pytest.mark.parametrize("value", [[], [1, 2], [1, 2, 3, 4], "123", None,
                                  [0, math.inf, 0], [0, math.nan, 0], [0, True, 0]])
def test_vectors_require_three_actual_finite_numbers(config, field, value):
    data = config.to_dict()
    data["simulation"][field] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("value", [[0, 0, 0], [0, 0, 2], [1e308, 0, 0]])
def test_sensitive_axis_must_be_unit_length_without_overflow(config, value):
    data = config.to_dict()
    data["simulation"]["sensitive_axis_magnet"] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("field,size", [("T_base_magnet", 4), ("rotation_base_flange", 3)])
@pytest.mark.parametrize("malformation", ["shape", "ragged", "nonfinite", "boolean", "scale", "reflection"])
def test_transform_matrices_are_finite_rigid_and_proper(config, field, size, malformation):
    data = config.to_dict()
    matrix = [[float(i == j) for j in range(size)] for i in range(size)]
    if malformation == "shape":
        matrix.pop()
    elif malformation == "ragged":
        matrix[0].pop()
    elif malformation == "nonfinite":
        matrix[0][0] = math.nan
    elif malformation == "boolean":
        matrix[0][0] = True
    elif malformation == "scale":
        matrix[0][0] = 2
    elif malformation == "reflection":
        matrix[0][0] = -1
    data["simulation"][field] = matrix
    with pytest.raises(ValueError):
        config_from_dict(data)


def test_homogeneous_last_row_is_validated(config):
    data = config.to_dict()
    matrix = [list(row) for row in data["simulation"]["T_base_magnet"]]
    matrix[3][0] = 1
    data["simulation"]["T_base_magnet"] = matrix
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("field,value", [
    ("pose_frame", "unknown"), ("original_unit", "mV"),
    ("motion_fault_mode", "unknown"), ("sensor_fault_mode", "unknown"),
    ("sensor_fault_persistent", 0), ("sensor_fault_persistent", "true"),
])
def test_enumerations_and_explicit_boolean_are_checked(config, field, value):
    data = config.to_dict()
    data["simulation"][field] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("section", ["acquisition", "simulation"])
@pytest.mark.parametrize("kind", ["missing", "unknown", "list"])
def test_new_sections_reject_missing_unknown_and_wrong_container(config, section, kind):
    data = config.to_dict()
    if kind == "missing":
        data[section].pop("samples_per_point" if section == "acquisition" else "seed")
    elif kind == "unknown":
        data[section]["typo"] = 1
    else:
        data[section] = []
    with pytest.raises(ValueError):
        config_from_dict(data)


def test_default_optional_fault_fields_roundtrip(config, tmp_path):
    data = config.to_dict()
    for field in fields(SimulationConfig):
        if field.default is not MISSING or field.default_factory is not MISSING:
            data["simulation"].pop(field.name)
    parsed = config_from_dict(data)
    assert parsed.simulation.motion_fault_at_visit is None
    assert parsed.simulation.sensor_fault_at_read is None
    assert parsed.simulation.sensor_fault_persistent is True
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(parsed.to_dict(), allow_nan=False), encoding="utf-8")
    assert load_config(path) == parsed


@pytest.mark.parametrize("mode", ["omit", "null"])
def test_plan_only_profiles_remain_compatible_and_roundtrip(config, mode):
    data = config.to_dict()
    for section in ("acquisition", "simulation"):
        if mode == "omit":
            data.pop(section)
        else:
            data[section] = None
    parsed = config_from_dict(data)
    assert parsed.acquisition is None and parsed.simulation is None
    assert len(plan_trajectory(parsed).points) == 925
    assert config_from_dict(parsed.to_dict()) == parsed


def test_complete_example_roundtrips_and_transforms_consistently(config):
    assert config_from_dict(config.to_dict()) == config
    tool = config.simulation.tool_transform()
    for point in [(0, 0, 0), (30, 0, 0), (5, -10, 15)]:
        assert tool.robot_to_magnet(tool.magnet_to_robot(point)) == pytest.approx(point)


def test_json_exponent_overflow_is_rejected(config, tmp_path):
    raw = json.dumps(config.to_dict()).replace('"base_field_T": 0.1', '"base_field_T": 1e309')
    assert "1e309" in raw
    path = tmp_path / "overflow.json"
    path.write_text(raw, encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(path)
