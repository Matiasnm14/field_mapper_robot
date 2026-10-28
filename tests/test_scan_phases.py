"""Las referencias rodean una malla ascendente sin introducir descensos internos."""

import csv
from dataclasses import replace
import json

import numpy as np
import pytest

from field_mapper.acquisition import build_visits, make_simulated_adapters, run_simulation
from field_mapper.clock import VirtualClock
from field_mapper.planner import plan_trajectory
from field_mapper.scan_storage import ScanWriter
from field_mapper.storage import PATH_COLUMNS, export_plan


@pytest.fixture(autouse=True)
def skip_figures(monkeypatch):
    from field_mapper import field_visualization, visualization
    monkeypatch.setattr(visualization, "save_trajectory_figure", lambda *args: None)
    monkeypatch.setattr(field_visualization, "save_field_figures", lambda *args: [])


def read_rows(directory, filename):
    with (directory / filename).open(encoding="utf-8", newline="") as source:
        return list(csv.DictReader(source))


@pytest.mark.parametrize("pre,post", [(False, False), (True, False), (False, True), (True, True)])
@pytest.mark.parametrize("step,grid_count", [(10, 123), (5, 925), (2.5, 7153)])
def test_visits_preserve_contiguous_ascending_grid_with_optional_references(config, pre, post, step, grid_count):
    profile = replace(config, geometry=replace(config.geometry, step_mm=step))
    trajectory = plan_trajectory(profile)
    visits = build_visits(trajectory, pre, post)
    assert len(visits) == grid_count + pre + post
    grid = [visit for visit in visits if visit.phase == "grid_scan"]
    assert [visit.point_id for visit in grid] == [point.point_id for point in trajectory.points]
    assert [visit.grid_visit_index for visit in grid] == list(range(grid_count))
    assert all(visit.kind == "grid" for visit in grid)
    heights = [visit.position_scan_mm[2] for visit in grid]
    assert heights == sorted(heights)
    assert grid[0].position_scan_mm == (0, 0, -30)
    assert grid[-1].position_scan_mm == (0, 0, 30)
    assert [visit.visit_index for visit in visits] == list(range(len(visits)))
    assert sum(visit.phase == "pre_reference" for visit in visits) == pre
    assert sum(visit.phase == "post_reference" for visit in visits) == post
    for visit in visits:
        if visit.kind == "reference":
            assert visit.position_scan_mm == visit.position_magnet_mm == (0, 0, 0)
            assert visit.layer_index is None and visit.grid_visit_index is None


@pytest.mark.parametrize("pre,post", [(False, False), (True, False), (False, True), (True, True)])
def test_runtime_phases_coordinates_and_policy_match_csv_events_metadata(config, tmp_path, pre, post):
    profile = replace(config, geometry=replace(config.geometry, radius_mm=10, step_mm=10),
                      scan=replace(config.scan, pre_reference=pre, post_reference=post),
                      acquisition=replace(config.acquisition, samples_per_point=2))
    result = run_simulation(profile, tmp_path / "scan")
    assert result.summary["status"] == "complete"
    expected = (["pre_reference"] if pre else []) + ["grid_scan"] + (["post_reference"] if post else [])
    assert [phase["phase"] for phase in result.summary["phases"]] == expected
    assert all(phase["status"] == "complete" for phase in result.summary["phases"])
    assert result.summary["reference_visits_expected"] == result.summary["reference_visits_completed"] == pre + post
    assert result.summary["visits_completed"] == 7 + pre + post
    assert result.summary["used_samples"] == 2 * (7 + pre + post)
    measurements = read_rows(result.directory, "measurements.csv")
    grid = [row for row in measurements if row["phase"] == "grid_scan"]
    assert len(grid) == 7
    heights = [float(row["h_target_scan_mm"]) for row in grid]
    assert heights == sorted(heights)
    base_heights = [float(row["z_reported_base_mm"]) for row in grid]
    assert np.diff(base_heights).min() >= -1e-10
    matrix = np.asarray(profile.scan.rotation_magnet_from_scan)
    for row in measurements:
        scan = [float(row[f"{axis}_target_scan_mm"]) for axis in ("u", "v", "h")]
        magnet = [float(row[f"{axis}_target_magnet_mm"]) for axis in "xyz"]
        assert matrix @ scan == pytest.approx(magnet)
        if row["kind"] == "grid":
            assert float(row["layer_height_scan_mm"]) == scan[2]
        else:
            assert row["layer_height_scan_mm"] == row["layer_index"] == ""
    events = [json.loads(line) for line in (result.directory / "events.jsonl").read_text().splitlines()]
    assert [event["phase"] for event in events if event["event"] == "phase_started"] == expected
    assert [event["phase"] for event in events if event["event"] == "phase_completed"] == expected
    transitions = [event for event in events if event["event"] == "phase_transition"]
    assert [event["to_phase"] for event in transitions] == expected
    assert all(event["hardware_path_validated"] is False for event in transitions)
    metadata = json.loads((result.directory / "metadata.json").read_text())
    assert metadata["runtime_phases"] == result.summary["phases"]
    assert metadata["reference_policy"]["pre_reference"] is pre
    assert metadata["reference_policy"]["post_reference"] is post
    assert metadata["reference_policy"]["between_layers"] is False
    assert metadata["scan_basis"]["index_order"] == ["iu", "iv", "ih"]
    assert metadata["hardware_transition_paths_validated"] is False
    assert metadata["source_code"]["revision"] is not None
    assert isinstance(metadata["source_code"]["worktree_modified"], bool)


def test_plan_and_scan_export_same_explicit_scan_and_magnet_coordinates(config, tmp_path):
    profile = replace(config, geometry=replace(config.geometry, step_mm=10))
    trajectory = plan_trajectory(profile)
    destination = export_plan(trajectory, profile, tmp_path / "plan")
    writer = ScanWriter(profile, trajectory, tmp_path / "scan")
    writer.close()
    assert (destination / "path.csv").read_bytes() == (writer.directory / "path.csv").read_bytes()
    rows = read_rows(destination, "path.csv")
    assert tuple(rows[0]) == PATH_COLUMNS
    assert {"ix", "iy", "iz"}.isdisjoint(rows[0])
    assert tuple(float(rows[0][f"{axis}_scan_mm"]) for axis in ("u", "v", "h")) == (0, 0, -30)
    for row, point in zip(rows, trajectory.points):
        assert tuple(int(row[key]) for key in ("iu", "iv", "ih")) == point.indices
        assert tuple(float(row[f"{axis}_magnet_mm"]) for axis in "xyz") == pytest.approx(point.position_mm)


def test_failure_in_grid_never_starts_post_reference(config, tmp_path):
    profile = replace(config, geometry=replace(config.geometry, radius_mm=10, step_mm=10),
                      simulation=replace(config.simulation, motion_fault_at_visit=2, motion_fault_mode="timeout"))
    clock = VirtualClock()
    robot, sensor, _ = make_simulated_adapters(profile, clock)
    result = run_simulation(profile, tmp_path / "failed-grid", robot=robot, sensor=sensor, clock=clock)
    assert result.summary["status"] == "failed"
    assert [phase["phase"] for phase in result.summary["phases"]] == ["pre_reference", "grid_scan"]
    assert result.summary["phases"][-1]["status"] == "failed"
    assert result.summary["reference_visits_completed"] == 1
    assert robot.command_count == 3 and robot.stop_count >= 1
    metadata = json.loads((result.directory / "metadata.json").read_text())
    assert metadata["runtime_phases"][-1]["status"] == "failed"
    events = [json.loads(line) for line in (result.directory / "events.jsonl").read_text().splitlines()]
    assert not any(event.get("phase") == "post_reference" for event in events)


def test_cancel_in_grid_records_partial_phase_and_skips_post_reference(config, tmp_path):
    profile = replace(config, geometry=replace(config.geometry, radius_mm=10, step_mm=10),
                      acquisition=replace(config.acquisition, samples_per_point=2))
    clock = VirtualClock()
    robot, sensor, _ = make_simulated_adapters(profile, clock)
    result = run_simulation(profile, tmp_path / "cancel-grid", robot=robot, sensor=sensor, clock=clock,
                            cancel_requested=lambda: sensor.read_count >= 3)
    assert result.summary["status"] == "cancelled"
    assert [phase["phase"] for phase in result.summary["phases"]] == ["pre_reference", "grid_scan"]
    assert result.summary["phases"][-1]["status"] == "cancelled"
    assert result.summary["reference_visits_completed"] == 1
    assert robot.command_count == 2 and robot.stop_count >= 1
    measurements = read_rows(result.directory, "measurements.csv")
    assert measurements[-1]["phase"] == "grid_scan"
    assert measurements[-1]["status"] == "cancelled"
    metadata = json.loads((result.directory / "metadata.json").read_text())
    assert metadata["runtime_phases"][-1]["status"] == "cancelled"


@pytest.mark.parametrize("pre,post", [(False, False), (True, False), (False, True), (True, True)])
def test_phase_starts_after_arrival_and_grid_window_excludes_descending_transitions(config, tmp_path, pre, post):
    profile = replace(config, geometry=replace(config.geometry, radius_mm=10, step_mm=10),
                      scan=replace(config.scan, pre_reference=pre, post_reference=post),
                      acquisition=replace(config.acquisition, samples_per_point=2))
    result = run_simulation(profile, tmp_path / "timing")
    assert result.summary["status"] == "complete"
    measurements = read_rows(result.directory, "measurements.csv")
    events = [json.loads(line) for line in (result.directory / "events.jsonl").read_text().splitlines()]
    for phase in result.summary["phases"]:
        first = next(row for row in measurements if row["phase"] == phase["phase"])
        assert phase["started_time_s"] >= float(first["arrived_time_s"])
        assert phase["started_time_s"] <= float(first["read_started_time_s"])
        start = next(i for i, event in enumerate(events) if event["event"] == "phase_started" and event["phase"] == phase["phase"])
        arrived = next(i for i, event in enumerate(events) if event["event"] == "arrived" and event["visit_id"] == first["visit_id"])
        transition_done = next(i for i, event in enumerate(events) if event["event"] == "phase_transition_completed"
                               and event["visit_id"] == first["visit_id"])
        assert arrived < transition_done < start
        assert events[arrived]["phase"] == events[transition_done]["phase"] == "transition"
    grid_start = next(i for i, event in enumerate(events) if event["event"] == "phase_started" and event["phase"] == "grid_scan")
    grid_finish = next(i for i, event in enumerate(events) if event["event"] == "phase_completed" and event["phase"] == "grid_scan")
    inside = events[grid_start:grid_finish + 1]
    assert not any(event["event"].startswith("phase_transition") for event in inside)
    assert all(event["phase"] == "grid_scan" for event in inside)
    heights = [-10] + [event["h_sensor_scan_mm"] for event in inside if event["event"] == "arrived"]
    assert np.diff(heights).min() >= -1e-10
    transitions = result.summary["transitions"]
    assert len(transitions) == len(result.summary["phases"])
    assert all(transition["status"] == "complete" for transition in transitions)
    metadata = json.loads((result.directory / "metadata.json").read_text())
    assert metadata["runtime_transitions"] == transitions


@pytest.mark.parametrize("fault_visit,target,expected_phases", [
    (0, "pre_reference", []),
    (1, "grid_scan", ["pre_reference"]),
    (8, "post_reference", ["pre_reference", "grid_scan"]),
])
def test_transition_failure_preserves_planned_measurement_without_starting_destination_phase(
    config, tmp_path, fault_visit, target, expected_phases,
):
    profile = replace(config, geometry=replace(config.geometry, radius_mm=10, step_mm=10),
                      simulation=replace(config.simulation, motion_fault_at_visit=fault_visit,
                                         motion_fault_mode="timeout"))
    clock = VirtualClock()
    robot, sensor, _ = make_simulated_adapters(profile, clock)
    result = run_simulation(profile, tmp_path / "entry-failure", robot=robot, sensor=sensor, clock=clock)
    assert result.summary["status"] == "failed"
    assert result.summary["current_phase"] == "transition"
    assert [phase["phase"] for phase in result.summary["phases"]] == expected_phases
    assert all(phase["status"] == "complete" for phase in result.summary["phases"])
    assert result.summary["transitions"][-1]["to_phase"] == target
    assert result.summary["transitions"][-1]["status"] == "failed"
    assert robot.command_count == fault_visit + 1 and robot.stop_count >= 1
    assert sensor.read_count == fault_visit * profile.acquisition.samples_per_point
    measurements = read_rows(result.directory, "measurements.csv")
    assert len(measurements) == fault_visit + 1
    assert measurements[-1]["phase"] == target
    assert measurements[-1]["status"] == "failed"
    assert measurements[-1]["B_n_mean_T"] == measurements[-1]["arrived_time_s"] == ""
    events = [json.loads(line) for line in (result.directory / "events.jsonl").read_text().splitlines()]
    assert not any(event["event"] == "phase_started" and event["phase"] == target for event in events)
    assert any(event["event"] == "phase_transition_failed" and event["to_phase"] == target for event in events)
    metadata = json.loads((result.directory / "metadata.json").read_text())
    assert metadata["runtime_transitions"][-1]["status"] == "failed"
    assert [phase["phase"] for phase in metadata["runtime_phases"]] == expected_phases


def test_cancel_on_lower_endpoint_arrival_keeps_transition_cancelled_and_grid_unstarted(config, tmp_path, monkeypatch):
    profile = replace(config, geometry=replace(config.geometry, radius_mm=10, step_mm=10),
                      acquisition=replace(config.acquisition, samples_per_point=2))
    clock = VirtualClock()
    robot, sensor, _ = make_simulated_adapters(profile, clock)
    arrived = robot.wait_until_arrived
    cancelled = False

    def request_cancel_on_grid_arrival(*args, **kwargs):
        nonlocal cancelled
        pose = arrived(*args, **kwargs)
        cancelled = robot.command_count == 2
        return pose

    monkeypatch.setattr(robot, "wait_until_arrived", request_cancel_on_grid_arrival)
    result = run_simulation(profile, tmp_path / "cancel-entry", robot=robot, sensor=sensor, clock=clock,
                            cancel_requested=lambda: cancelled)
    assert result.summary["status"] == "cancelled"
    assert result.summary["current_phase"] == "transition"
    assert result.summary["visits_cancelled"] == 1
    assert result.summary["grid_points_completed"] == 0
    assert result.summary["reference_visits_completed"] == 1
    assert [phase["phase"] for phase in result.summary["phases"]] == ["pre_reference"]
    assert result.summary["phases"][0]["status"] == "complete"
    assert result.summary["transitions"][-1]["to_phase"] == "grid_scan"
    assert result.summary["transitions"][-1]["status"] == "cancelled"
    assert robot.command_count == 2 and robot.stop_count >= 1
    assert sensor.read_count == 2
    measurement = read_rows(result.directory, "measurements.csv")[-1]
    assert measurement["phase"] == "grid_scan" and measurement["status"] == "cancelled"
    assert measurement["arrived_time_s"] != "" and measurement["B_n_mean_T"] == ""
    metadata = json.loads((result.directory / "metadata.json").read_text())
    assert metadata["runtime_transitions"][-1]["status"] == "cancelled"
