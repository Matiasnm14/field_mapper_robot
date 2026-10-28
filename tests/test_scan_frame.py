"""Altura física, marco del imán y retícula siguen siendo conceptos distintos."""

from dataclasses import replace
from itertools import groupby
import csv
import json

import numpy as np
import pytest

from field_mapper.config import ScanConfig, config_from_dict, load_config
from field_mapper.planner import (
    DEFAULT_SCAN_ROTATION, GridPoint, generate_grid, plan_trajectory, validate_trajectory,
)


def _rotation_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rotation_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rotation_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotated_profile(config, *, yaw=0.3, tilt=0.7, base_tilt=0):
    base_from_magnet = _rotation_z(0.6) @ _rotation_x(tilt) @ _rotation_y(-0.2)
    base_from_scan = _rotation_x(base_tilt) @ _rotation_z(yaw)
    magnet_from_scan = base_from_magnet.T @ base_from_scan
    transform = np.array(config.simulation.T_base_magnet)
    transform[:3, :3] = base_from_magnet
    return replace(
        config,
        scan=replace(config.scan, rotation_magnet_from_scan=tuple(map(tuple, magnet_from_scan)),
                     up_base=tuple(base_from_scan[:, 2])),
        simulation=replace(config.simulation, T_base_magnet=tuple(map(tuple, transform))),
    )


def test_reference_scan_basis_preserves_magnet_axes_and_physical_height(config):
    trajectory = plan_trajectory(config)
    np.testing.assert_array_equal(trajectory.rotation_magnet_from_scan, DEFAULT_SCAN_ROTATION)
    scan = trajectory.positions_scan_mm
    magnet = trajectory.positions_mm
    np.testing.assert_array_equal(magnet, scan[:, [2, 0, 1]])
    np.testing.assert_array_equal(scan[0], [0, 0, -30])
    np.testing.assert_array_equal(scan[-1], [0, 0, 30])
    np.testing.assert_array_equal(magnet[0], [-30, 0, 0])
    np.testing.assert_array_equal(magnet[-1], [30, 0, 0])
    assert np.all(np.diff(scan[:, 2]) >= 0)
    # El eje longitudinal z_imán avanza y retrocede dentro de capas; no es altura.
    assert np.any(np.diff(magnet[:, 2]) < 0)
    assert np.any(np.diff(magnet[:, 2]) > 0)
    layer_counts = [len(list(points)) for _, points in groupby(trajectory.points, lambda point: point.indices[2])]
    assert layer_counts == [1, 37, 69, 89, 101, 109, 113, 109, 101, 89, 69, 37, 1]


@pytest.mark.parametrize("yaw,tilt,base_tilt", [(0, 0, 0), (0.3, 0.7, 0), (-1.2, -0.8, 0), (0.8, 1.1, 0.4)])
def test_rotated_calibration_keeps_layers_horizontal_and_height_ascending(config, yaw, tilt, base_tilt):
    profile = rotated_profile(config, yaw=yaw, tilt=tilt, base_tilt=base_tilt)
    trajectory = plan_trajectory(profile)
    assert len(trajectory.points) == 925
    scan = trajectory.positions_scan_mm
    magnet = trajectory.positions_mm
    base = profile.simulation.tool_transform().base_from_magnet.apply(magnet)
    translation = np.asarray(profile.simulation.T_base_magnet)[:3, 3]
    physical_height = (base - translation) @ np.asarray(profile.scan.up_base)
    np.testing.assert_allclose(physical_height, scan[:, 2], atol=1e-12, rtol=0)
    assert np.min(np.diff(physical_height)) >= -1e-12
    for height, points in groupby(zip(trajectory.points, physical_height), lambda pair: pair[0].indices[2]):
        observed_height = [item[1] for item in points]
        np.testing.assert_allclose(observed_height, height * profile.geometry.step_mm, atol=1e-12, rtol=0)
    np.testing.assert_allclose(np.linalg.norm(magnet, axis=1), np.linalg.norm(scan, axis=1), atol=1e-12)
    assert profile.geometry.sphere.contains(magnet).all()
    assert profile.geometry.bore.contains(magnet).all()


def test_rotating_the_scan_frame_preserves_point_ids_and_integer_grid(config):
    reference = plan_trajectory(config)
    rotated = plan_trajectory(rotated_profile(config))
    assert [point.point_id for point in rotated.points] == [point.point_id for point in reference.points]
    np.testing.assert_array_equal(rotated.positions_scan_mm, reference.positions_scan_mm)
    assert not np.allclose(rotated.positions_mm, reference.positions_mm)
    np.testing.assert_allclose(rotated.positions_mm @ np.asarray(rotated.rotation_magnet_from_scan),
                               rotated.positions_scan_mm, atol=1e-12)


@pytest.mark.parametrize("up", [(0, 0, -1), (1, 0, 0)])
def test_incoherent_physical_vertical_is_rejected(config, up):
    profile = replace(config, scan=replace(config.scan, up_base=up))
    with pytest.raises(ValueError, match="vertical física"):
        plan_trajectory(profile)


def test_calibration_free_plan_still_uses_explicit_scan_frame(config):
    profile = replace(config, simulation=None)
    trajectory = plan_trajectory(profile)
    assert len(trajectory.points) == 925
    np.testing.assert_array_equal(trajectory.points[0].position_mm, [-30, 0, 0])


def test_validation_rejects_descending_or_interleaved_complete_layers(config):
    trajectory = plan_trajectory(config)
    points = trajectory.points
    interleaved = list(points)
    # Cambiar dos puntos de alturas distintas conserva cobertura y ambos polos.
    interleaved[1], interleaved[38] = interleaved[38], interleaved[1]
    for corrupted in (tuple(reversed(points)), tuple(interleaved)):
        with pytest.raises(ValueError, match="abajo hacia arriba"):
            validate_trajectory(replace(trajectory, points=corrupted), config)


def test_validation_rejects_stale_trajectory_or_individual_point_frame(config):
    trajectory = plan_trajectory(config)
    identity = tuple(map(tuple, np.eye(3)))
    wrong_trajectory = replace(trajectory, rotation_magnet_from_scan=identity)
    wrong_point = replace(trajectory.points[12], rotation_magnet_from_scan=identity)
    wrong_points = trajectory.points[:12] + (wrong_point,) + trajectory.points[13:]
    for corrupted in (wrong_trajectory, replace(trajectory, points=wrong_points)):
        with pytest.raises(ValueError):
            validate_trajectory(corrupted, config)


def test_scan_rotation_copies_mutable_input():
    rotation = np.array(DEFAULT_SCAN_ROTATION)
    point = GridPoint((1, 2, 3), 5, rotation)
    rotation[:] = 0
    assert point.position_scan_mm == (5, 10, 15)
    np.testing.assert_array_equal(point.position_mm, [15, 5, 10])


@pytest.mark.parametrize("rotation", [np.zeros((3, 3)), np.diag([1, 1, -1]), np.eye(2), np.full((3, 3), np.nan)])
def test_invalid_scan_rotations_are_rejected_by_generator(rotation):
    with pytest.raises(ValueError):
        generate_grid(30, 5, rotation)


def test_rotated_trajectory_figure_is_exported_without_display(config, tmp_path, monkeypatch):
    from field_mapper.visualization import save_trajectory_figure

    monkeypatch.setenv("DISPLAY", "")
    monkeypatch.setenv("MPLBACKEND", "TkAgg")
    profile = rotated_profile(config)
    destination = tmp_path / "rotated.png"
    save_trajectory_figure(plan_trajectory(profile), profile, destination)
    assert destination.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_missing_or_empty_scan_uses_reference_basis_and_external_references(config):
    data = config.to_dict()
    data.pop("scan")
    assert config_from_dict(data).scan == ScanConfig()
    data["scan"] = {}
    assert config_from_dict(data).scan == ScanConfig()


@pytest.mark.parametrize("value", [None, True, False, 0, 1, "scan", []])
def test_scan_section_must_be_an_object(config, value):
    data = config.to_dict()
    data["scan"] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("unknown", ["typo", "units", "height_axis"])
def test_scan_unknown_fields_are_rejected(config, unknown):
    data = config.to_dict()
    data["scan"][unknown] = "mm"
    with pytest.raises(ValueError, match="desconocidos"):
        config_from_dict(data)


@pytest.mark.parametrize("name", ["pre_reference", "post_reference"])
@pytest.mark.parametrize("value", [0, 1, "true", "false", None, [], {}])
def test_reference_flags_are_strict_json_booleans(config, name, value):
    data = config.to_dict()
    data["scan"][name] = value
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("pre,post", [(False, False), (False, True), (True, False), (True, True)])
def test_scan_reference_policy_roundtrips_json(config, tmp_path, pre, post):
    profile = replace(config, scan=replace(config.scan, pre_reference=pre, post_reference=post))
    destination = tmp_path / "profile.json"
    destination.write_text(json.dumps(profile.to_dict()), encoding="utf-8")
    assert load_config(destination) == profile


@pytest.mark.parametrize("rotation", [
    None, True, "identity", [], [[1, 0], [0, 1]],
    [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[2, 0, 0], [0, 2, 0], [0, 0, 2]],
    [[1, 0, 0], [0, 1, 0], [0, 0, -1]],
    [[1, 0.1, 0], [0, 1, 0], [0, 0, 1]],
    [[True, 0, 0], [0, 1, 0], [0, 0, 1]],
    [["1", 0, 0], [0, 1, 0], [0, 0, 1]],
    [[float("nan"), 0, 0], [0, 1, 0], [0, 0, 1]],
    [[float("inf"), 0, 0], [0, 1, 0], [0, 0, 1]],
])
def test_scan_rotation_requires_numeric_finite_rigid_3x3_matrix(config, rotation):
    data = config.to_dict()
    data["scan"]["rotation_magnet_from_scan"] = rotation
    with pytest.raises(ValueError):
        config_from_dict(data)


@pytest.mark.parametrize("up", [
    None, True, "vertical", [], [0, 1], [0, 0, 0, 1],
    [0, 0, 0], [0, 0, 2], [True, 0, 0], [0, 0, "1"],
    [0, 0, float("nan")], [0, 0, float("inf")], [1e308, 0, 0],
])
def test_physical_up_requires_finite_dimensionless_unit_vector(config, up):
    data = config.to_dict()
    data["scan"]["up_base"] = up
    with pytest.raises(ValueError):
        config_from_dict(data)


def test_scan_does_not_retain_mutable_json_arrays():
    rotation = [list(row) for row in DEFAULT_SCAN_ROTATION]
    up = [0, 0, 1]
    scan = ScanConfig(rotation_magnet_from_scan=rotation, up_base=up)
    rotation[0][0] = 9
    up[:] = [0, 0, 0]
    assert scan.rotation_magnet_from_scan == DEFAULT_SCAN_ROTATION
    assert scan.up_base == (0, 0, 1)


def test_duplicate_scan_policy_keys_are_rejected_in_json(config, tmp_path):
    data = config.to_dict()
    data.pop("scan")
    source = json.dumps(data)[:-1] + ', "scan":{"pre_reference":true,"pre_reference":false}}'
    destination = tmp_path / "ambiguous.json"
    destination.write_text(source, encoding="utf-8")
    with pytest.raises(ValueError, match="duplicada"):
        load_config(destination)


@pytest.mark.parametrize("base_tilt", [0, 0.4])
def test_rotated_scan_acquires_feedback_and_field_in_the_correct_frames(config, tmp_path, monkeypatch, base_tilt):
    from field_mapper import field_visualization, visualization
    from field_mapper.acquisition import run_simulation

    monkeypatch.setattr(visualization, "save_trajectory_figure", lambda *args: None)
    monkeypatch.setattr(field_visualization, "save_field_figures", lambda *args: [])
    profile = rotated_profile(config, base_tilt=base_tilt)
    profile = replace(
        profile, geometry=replace(profile.geometry, radius_mm=10, step_mm=10),
        acquisition=replace(profile.acquisition, samples_per_point=2),
        simulation=replace(profile.simulation, noise_std_T=0, drift_T_per_s=0),
    )
    result = run_simulation(profile, tmp_path / "rotated-scan")
    assert result.summary["status"] == "complete"
    assert result.summary["visits_completed"] == 9
    assert result.summary["used_samples"] == 18
    with (result.directory / "measurements.csv").open(encoding="utf-8", newline="") as handle:
        measurements = list(csv.DictReader(handle))
    assert [row["phase"] for row in measurements] == ["pre_reference"] + ["grid_scan"] * 7 + ["post_reference"]
    rotation = np.asarray(profile.scan.rotation_magnet_from_scan)
    transform = profile.simulation.tool_transform()
    height_in_base = []
    for row in measurements:
        target_scan = [float(row[f"{axis}_target_scan_mm"]) for axis in ("u", "v", "h")]
        reported = [float(row[f"{axis}_reported_base_mm"]) for axis in "xyz"]
        sensor_magnet = transform.robot_to_magnet(reported)
        np.testing.assert_allclose(sensor_magnet, rotation @ target_scan, atol=1e-12)
        np.testing.assert_allclose([float(row[f"{axis}_sensor_magnet_mm"]) for axis in "xyz"], sensor_magnet)
        expected = profile.simulation.base_field_T + sum(
            linear * position + quadratic * position ** 2
            for linear, quadratic, position in zip(profile.simulation.gradient_T_per_mm,
                                                  profile.simulation.quadratic_T_per_mm2, sensor_magnet)
        )
        assert float(row["B_n_mean_T"]) == pytest.approx(expected)
        if row["kind"] == "grid":
            sensor_base = transform.base_from_magnet.apply(sensor_magnet)
            height_in_base.append(float(np.dot(sensor_base, profile.scan.up_base)))
    assert np.min(np.diff(height_in_base)) >= -1e-12
