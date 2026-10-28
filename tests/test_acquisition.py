"""Contrato de adquisición: muestras reales del simulador y cierre seguro."""

import csv
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from field_mapper.clock import VirtualClock
from field_mapper.planner import plan_trajectory


@pytest.fixture(autouse=True)
def skip_figures_in_logic_tests(monkeypatch):
    from field_mapper import acquisition, field_visualization, visualization

    def trajectory_placeholder(trajectory, config, path):
        Path(path).write_bytes(b"placeholder")

    def field_placeholder(directory):
        return []

    monkeypatch.setattr(visualization, "save_trajectory_figure", trajectory_placeholder)
    monkeypatch.setattr(field_visualization, "save_field_figures", field_placeholder)
    if hasattr(acquisition, "save_trajectory_figure"):
        monkeypatch.setattr(acquisition, "save_trajectory_figure", trajectory_placeholder)
    if hasattr(acquisition, "save_field_figures"):
        monkeypatch.setattr(acquisition, "save_field_figures", field_placeholder)


@pytest.fixture
def scan_config(config):
    return replace(
        config,
        geometry=replace(config.geometry, radius_mm=10, step_mm=10),
        acquisition=replace(config.acquisition, samples_per_point=3, max_sensor_retries=1),
    )


def rows(directory, filename):
    with (directory / filename).open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def adapters(config):
    from field_mapper.acquisition import make_simulated_adapters

    clock = VirtualClock()
    robot, sensor, transform = make_simulated_adapters(config, clock)
    return clock, robot, sensor, transform


def run(config, destination, **kwargs):
    from field_mapper.acquisition import run_simulation

    return run_simulation(config, destination, **kwargs)


def test_complete_scan_preserves_grid_reference_and_sample_relationships(scan_config, tmp_path):
    result = run(scan_config, tmp_path / "complete")
    assert result.summary["status"] == "complete"
    assert result.summary["grid_points"] == 7
    assert result.summary["grid_points_completed"] == 7
    assert result.summary["reference_visits_expected"] == 2
    assert result.summary["reference_visits_completed"] == 2
    assert result.summary["visits_expected"] == result.summary["visits_completed"] == 9
    assert result.summary["valid_samples"] == result.summary["used_samples"] == 27
    measurements = rows(result.directory, "measurements.csv")
    samples = rows(result.directory, "samples.csv")
    path = rows(result.directory, "path.csv")
    assert len(measurements) == 9
    assert len(samples) == 27
    assert [int(row["visit_index"]) for row in measurements] == list(range(9))
    assert measurements[0]["kind"] == "reference"
    assert measurements[-1]["kind"] == "reference"
    grid = [row for row in measurements if row["kind"] == "grid"]
    references = [row for row in measurements if row["kind"] == "reference"]
    assert [row["point_id"] for row in grid] == [row["point_id"] for row in path]
    assert len({row["point_id"] for row in grid}) == 7
    assert len(references) == 2
    assert len({row["visit_id"] for row in measurements}) == 9
    expected_kinds = ["reference"] + ["grid"] * 7 + ["reference"]
    assert [row["kind"] for row in measurements] == expected_kinds
    assert all(float(row[f"{axis}_target_magnet_mm"]) == 0 for row in references for axis in "xyz")
    for measurement in measurements:
        batch = [row for row in samples if row["visit_id"] == measurement["visit_id"]]
        assert len(batch) == 3
        assert {row["attempt"] for row in batch} == {measurement["attempt"]}
        values = [float(row["B_n_T"]) for row in batch]
        assert float(measurement["B_n_mean_T"]) == pytest.approx(np.mean(values))
        assert float(measurement["B_n_std_T"]) == pytest.approx(np.std(values, ddof=1))
        assert all(row["simulated"] == "true" for row in batch)
        assert all(float(row["acquired_time_s"]) >= float(measurement["read_started_time_s"]) for row in batch)
        assert float(measurement["read_started_time_s"]) >= (
            float(measurement["arrived_time_s"]) + scan_config.acquisition.settle_time_s
        )
    for name in ("summary.json", "metadata.json", "progress.json"):
        document = json.loads((result.directory / name).read_text())
        assert document["status"] == "complete"
        assert document["simulated"] is True


def test_same_seed_reproduces_numeric_samples_and_different_seed_changes_noise(scan_config, tmp_path):
    first = run(scan_config, tmp_path / "first")
    second = run(scan_config, tmp_path / "second")
    other_config = replace(scan_config, simulation=replace(scan_config.simulation, seed=scan_config.simulation.seed + 1))
    third = run(other_config, tmp_path / "third")
    columns = ("B_n_T", "original_value", "original_unit", "acquired_time_s", "received_time_s", "sequence_id")
    signatures = [
        [tuple(row[key] for key in columns) for row in rows(result.directory, "samples.csv")]
        for result in (first, second, third)
    ]
    assert signatures[0] == signatures[1]
    assert signatures[0] != signatures[2]


def test_gauss_is_normalized_and_retains_original_units(scan_config, tmp_path):
    gauss = replace(scan_config, simulation=replace(scan_config.simulation, original_unit="G"))
    tesla = replace(scan_config, simulation=replace(scan_config.simulation, original_unit="T"))
    gs = rows(run(gauss, tmp_path / "gauss").directory, "samples.csv")
    ts = rows(run(tesla, tmp_path / "tesla").directory, "samples.csv")
    for g, t in zip(gs, ts, strict=True):
        assert g["original_unit"] == "G"
        assert t["original_unit"] == "T"
        assert float(g["B_n_T"]) == pytest.approx(float(g["original_value"]) * 1e-4)
        assert float(g["B_n_T"]) == pytest.approx(float(t["B_n_T"]))


def test_single_sample_has_no_fabricated_standard_deviation(scan_config, tmp_path):
    profile = replace(scan_config, acquisition=replace(scan_config.acquisition, samples_per_point=1))
    result = run(profile, tmp_path / "single")
    measurements = rows(result.directory, "measurements.csv")
    assert result.summary["status"] == "complete"
    assert all(row["B_n_std_T"] == "" and row["valid_samples"] == "1" for row in measurements)


@pytest.mark.parametrize("mode", ["timeout", "out_of_tolerance", "connection_loss", "accepted_only"])
def test_motion_failure_does_not_read_or_send_next_target(scan_config, tmp_path, mode):
    profile = replace(scan_config, simulation=replace(scan_config.simulation, motion_fault_at_visit=1, motion_fault_mode=mode))
    clock, robot, sensor, _ = adapters(profile)
    result = run(profile, tmp_path / mode, robot=robot, sensor=sensor, clock=clock)
    assert result.summary["status"] == "failed"
    assert result.summary["visits_completed"] == 1
    assert robot.command_count == 2
    assert sensor.read_count == profile.acquisition.samples_per_point
    assert robot.stop_count >= 1
    assert robot._connected is False
    assert sensor._connected is False
    measurements = rows(result.directory, "measurements.csv")
    assert len(measurements) == 2
    assert measurements[-1]["B_n_mean_T"] == ""
    assert measurements[-1]["valid_samples"] == "0"
    assert len(rows(result.directory, "samples.csv")) == profile.acquisition.samples_per_point


@pytest.mark.parametrize("mode", ["timeout", "stale", "invalid", "unit"])
def test_sensor_failure_has_bounded_retries_at_same_position(scan_config, tmp_path, mode):
    profile = replace(scan_config, simulation=replace(scan_config.simulation, sensor_fault_at_read=1, sensor_fault_mode=mode))
    clock, robot, sensor, _ = adapters(profile)
    result = run(profile, tmp_path / mode, robot=robot, sensor=sensor, clock=clock)
    assert result.summary["status"] == "failed"
    assert result.summary["visits_completed"] == 0
    assert robot.command_count == 1
    assert sensor.read_count == 2
    assert robot.stop_count >= 1
    measurements = rows(result.directory, "measurements.csv")
    assert len(measurements) == 2
    assert len({row["visit_id"] for row in measurements}) == 1
    assert len({row["attempt"] for row in measurements}) == 2
    assert all(row["B_n_mean_T"] == "" and row["valid_samples"] == "0" for row in measurements)


def test_transient_sensor_failure_retries_a_whole_batch_without_moving(scan_config, tmp_path):
    profile = replace(scan_config, simulation=replace(scan_config.simulation, sensor_fault_at_read=2, sensor_fault_mode="timeout", sensor_fault_persistent=False))
    clock, robot, sensor, _ = adapters(profile)
    result = run(profile, tmp_path / "transient", robot=robot, sensor=sensor, clock=clock)
    assert result.summary["status"] == "complete"
    assert robot.command_count == 9
    measurements = rows(result.directory, "measurements.csv")
    assert len(measurements) == 10
    assert measurements[0]["visit_id"] == measurements[1]["visit_id"]
    assert measurements[0]["attempt"] != measurements[1]["attempt"]
    assert measurements[0]["valid_samples"] == "1"
    assert measurements[1]["valid_samples"] == "3"
    assert result.summary["valid_samples"] == 28
    assert result.summary["used_samples"] == 27


def test_invalid_trajectory_is_rejected_before_output_or_connection(scan_config, config, tmp_path, monkeypatch):
    trajectory = plan_trajectory(config)
    clock, robot, sensor, _ = adapters(scan_config)

    def unexpected_connection():
        raise AssertionError("Se conectó antes de validar la trayectoria.")

    monkeypatch.setattr(robot, "connect", unexpected_connection)
    monkeypatch.setattr(sensor, "connect", unexpected_connection)
    destination = tmp_path / "invalid"
    with pytest.raises(ValueError):
        run(scan_config, destination, trajectory=trajectory, robot=robot, sensor=sensor, clock=clock)
    assert not destination.exists()


def test_cancel_during_sampling_preserves_partial_data_without_next_move(scan_config, tmp_path, monkeypatch):
    clock, robot, sensor, _ = adapters(scan_config)
    original_read = sensor.read_sample
    cancelled = False

    def cancel_after_first_sample(**kwargs):
        nonlocal cancelled
        sample = original_read(**kwargs)
        cancelled = True
        return sample

    monkeypatch.setattr(sensor, "read_sample", cancel_after_first_sample)
    result = run(scan_config, tmp_path / "cancelled", robot=robot, sensor=sensor, clock=clock, cancel_requested=lambda: cancelled)
    assert result.summary["status"] == "cancelled"
    assert robot.command_count == 1
    assert sensor.read_count == 1
    assert robot.stop_count >= 1
    assert len(rows(result.directory, "samples.csv")) == 1
    assert robot._connected is False
    assert sensor._connected is False


def test_write_failure_preserves_prior_rows_and_stops_motion(scan_config, tmp_path):
    from field_mapper.scan_storage import ScanWriter

    class FailingWriter(ScanWriter):
        def write_sample(self, row):
            if int(row["visit_index"]) == 1:
                raise OSError("disco simulado lleno")
            super().write_sample(row)

    clock, robot, sensor, _ = adapters(scan_config)
    result = run(scan_config, tmp_path / "disk-full", robot=robot, sensor=sensor, clock=clock, writer_factory=FailingWriter)
    assert result.summary["status"] == "failed"
    assert "disco simulado lleno" in result.summary["error"]
    assert robot.command_count == 2
    assert robot.stop_count >= 1
    assert len(rows(result.directory, "samples.csv")) == 3
    assert rows(result.directory, "measurements.csv")[0]["valid_samples"] == "3"
    assert json.loads((result.directory / "progress.json").read_text())["status"] == "failed"
    assert robot._connected is False
    assert sensor._connected is False


def test_field_uses_sensor_position_reconstructed_from_feedback(scan_config, tmp_path, monkeypatch):
    from field_mapper.robot.base import RobotPose

    profile = replace(
        scan_config,
        acquisition=replace(scan_config.acquisition, position_tolerance_mm=0.1),
        simulation=replace(
            scan_config.simulation,
            gradient_T_per_mm=(0.001, 0.002, 0.003),
            quadratic_T_per_mm2=(0, 0, 0), noise_std_T=0, drift_T_per_s=0,
        ),
    )
    clock, robot, sensor, transform = adapters(profile)
    wait_original = robot.wait_until_arrived

    def shifted_arrival(*args, **kwargs):
        observed = wait_original(*args, **kwargs)
        position = tuple(value + shift for value, shift in zip(observed.position_mm, (0.02, 0, 0)))
        robot._pose = RobotPose(position, observed.angle_deg)
        return robot._pose

    monkeypatch.setattr(robot, "wait_until_arrived", shifted_arrival)
    result = run(profile, tmp_path / "feedback", robot=robot, sensor=sensor, clock=clock)
    assert result.summary["status"] == "complete"
    for row in rows(result.directory, "measurements.csv"):
        reported = [float(row[f"{axis}_reported_base_mm"]) for axis in "xyz"]
        actual = transform.robot_to_magnet(reported)
        recorded = [float(row[f"{axis}_sensor_magnet_mm"]) for axis in "xyz"]
        assert recorded == pytest.approx(actual)
        expected_field = profile.simulation.base_field_T + np.dot(profile.simulation.gradient_T_per_mm, actual)
        assert float(row["B_n_mean_T"]) == pytest.approx(expected_field)
        assert float(row["x_reported_base_mm"]) - float(row["x_command_base_mm"]) == pytest.approx(0.02)


@pytest.mark.parametrize("missing", ["acquisition", "simulation"])
def test_missing_simulation_section_is_rejected_before_creating_output(scan_config, tmp_path, missing):
    profile = replace(scan_config, **{missing: None})
    destination = tmp_path / missing
    with pytest.raises(ValueError):
        run(profile, destination)
    assert not destination.exists()


def test_progress_and_metadata_exist_before_connecting(scan_config, tmp_path, monkeypatch):
    clock, robot, sensor, _ = adapters(scan_config)
    destination = tmp_path / "startup"
    connect_original = robot.connect
    observed = []

    def inspect_before_connect():
        observed.append(json.loads((destination / "metadata.json").read_text())["status"])
        observed.append(json.loads((destination / "progress.json").read_text())["status"])
        connect_original()

    monkeypatch.setattr(robot, "connect", inspect_before_connect)
    result = run(scan_config, destination, robot=robot, sensor=sensor, clock=clock)
    assert result.summary["status"] == "complete"
    assert observed == ["running", "running"]


@pytest.mark.parametrize("bad_evidence", ["repeated_sequence", "old_acquisition"])
def test_ok_flag_does_not_override_stale_sample_evidence(scan_config, tmp_path, monkeypatch, bad_evidence):
    profile = replace(scan_config, acquisition=replace(scan_config.acquisition, max_sensor_retries=0))
    clock, robot, sensor, _ = adapters(profile)
    read_original = sensor.read_sample

    def misleading_sample(**kwargs):
        sample = read_original(**kwargs)
        if bad_evidence == "repeated_sequence":
            return replace(sample, sequence_id=1)
        return replace(sample, acquired_time_s=kwargs["not_before_s"] - 0.001)

    monkeypatch.setattr(sensor, "read_sample", misleading_sample)
    result = run(profile, tmp_path / bad_evidence, robot=robot, sensor=sensor, clock=clock)
    assert result.summary["status"] == "failed"
    assert robot.command_count == 1
    assert result.summary["used_samples"] == 0
    assert result.summary["valid_samples"] == (1 if bad_evidence == "repeated_sequence" else 0)


def test_simulation_does_not_open_network_connections(scan_config, tmp_path, monkeypatch):
    import socket

    def unexpected_connection(*args, **kwargs):
        raise AssertionError("La simulación intentó abrir una conexión de red.")

    monkeypatch.setattr(socket.socket, "connect", unexpected_connection)
    monkeypatch.setattr(socket, "create_connection", unexpected_connection)
    assert run(scan_config, tmp_path / "offline").summary["status"] == "complete"


def test_keyboard_interrupt_preserves_completed_visit_and_disconnects(scan_config, tmp_path, monkeypatch):
    clock, robot, sensor, _ = adapters(scan_config)
    original_read = sensor.read_sample

    def operator_interrupt(**kwargs):
        if sensor.read_count >= scan_config.acquisition.samples_per_point:
            raise KeyboardInterrupt
        return original_read(**kwargs)

    monkeypatch.setattr(sensor, "read_sample", operator_interrupt)
    result = run(scan_config, tmp_path / "interrupt", robot=robot, sensor=sensor, clock=clock)
    assert result.summary["status"] == "cancelled"
    assert result.summary["visits_completed"] == 1
    assert robot.command_count == 2
    assert robot.stop_count >= 1
    assert len(rows(result.directory, "samples.csv")) == 3
    assert robot._connected is False
    assert sensor._connected is False


def test_main_profile_has_927_visits_and_9270_samples(config, tmp_path):
    result = run(config, tmp_path / "main-profile")
    assert result.summary["status"] == "complete"
    assert result.summary["grid_points_completed"] == 925
    assert result.summary["reference_visits_completed"] == 2
    assert result.summary["visits_completed"] == 927
    assert result.summary["used_samples"] == result.summary["valid_samples"] == 9270
    assert len(rows(result.directory, "measurements.csv")) == 927
    assert len(rows(result.directory, "samples.csv")) == 9270


def test_interrupt_during_final_figures_closes_files_and_keeps_data(scan_config, tmp_path, monkeypatch):
    from field_mapper import field_visualization
    from field_mapper.scan_storage import ScanWriter

    writers = []

    def capture_writer(*args, **kwargs):
        writer = ScanWriter(*args, **kwargs)
        writers.append(writer)
        return writer

    def interrupt_plot(directory):
        raise KeyboardInterrupt

    monkeypatch.setattr(field_visualization, "save_field_figures", interrupt_plot)
    result = run(scan_config, tmp_path / "plot-interrupt", writer_factory=capture_writer)
    assert result.summary["status"] == "cancelled"
    assert result.summary["visits_completed"] == 9
    assert len(rows(result.directory, "samples.csv")) == 27
    assert all(handle.closed for handle in writers[0]._handles)
    assert json.loads((result.directory / "metadata.json").read_text())["status"] == "cancelled"


def test_transient_finalization_error_updates_saved_status_to_failed(scan_config, tmp_path):
    from field_mapper.scan_storage import ScanWriter

    class FailingFinalizationWriter(ScanWriter):
        def __init__(self, *args, **kwargs):
            self.failed_once = False
            super().__init__(*args, **kwargs)

        def finish(self, summary):
            super().finish(summary)
            if not self.failed_once:
                self.failed_once = True
                raise OSError("fallo transitorio al finalizar")

    result = run(scan_config, tmp_path / "finalization-error", writer_factory=FailingFinalizationWriter)
    assert result.summary["status"] == "failed"
    assert result.summary["visits_completed"] == 9
    assert "fallo transitorio al finalizar" in result.summary["error"]
    for name in ("summary.json", "metadata.json", "progress.json"):
        assert json.loads((result.directory / name).read_text())["status"] == "failed"
    assert len(rows(result.directory, "samples.csv")) == 27


@pytest.mark.parametrize("adapter_name", ["robot", "sensor"])
def test_simulate_rejects_real_adapter_before_output_or_connections(scan_config, tmp_path, monkeypatch, adapter_name):
    clock, robot, sensor, _ = adapters(scan_config)
    adapter = robot if adapter_name == "robot" else sensor
    monkeypatch.setattr(adapter, "simulated", False)

    def unexpected_connection():
        raise AssertionError("simulate intentó conectar un adaptador real.")

    monkeypatch.setattr(robot, "connect", unexpected_connection)
    monkeypatch.setattr(sensor, "connect", unexpected_connection)
    destination = tmp_path / adapter_name
    with pytest.raises(ValueError):
        run(scan_config, destination, robot=robot, sensor=sensor, clock=clock)
    assert not destination.exists()
