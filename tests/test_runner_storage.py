"""La finalización mantiene coherencia entre datos, estado y errores de E/S."""

import csv
from dataclasses import replace
import json

import pytest

from field_mapper.acquisition import make_simulated_adapters, run_simulation
from field_mapper.clock import VirtualClock
from field_mapper.scan_storage import ScanWriter


@pytest.fixture
def profile(config, monkeypatch):
    from field_mapper import field_visualization, visualization
    monkeypatch.setattr(visualization, "save_trajectory_figure", lambda *args: None)
    monkeypatch.setattr(field_visualization, "save_field_figures", lambda *args: [])
    return replace(config, geometry=replace(config.geometry, radius_mm=10, step_mm=10),
                   acquisition=replace(config.acquisition, samples_per_point=2))


def test_failed_close_updates_all_persisted_states(profile, tmp_path):
    class FailingCloseWriter(ScanWriter):
        failed_once = False

        def close(self):
            super().close()
            if not self.failed_once:
                self.failed_once = True
                raise OSError("simulated close error")

    result = run_simulation(profile, tmp_path / "close-error", writer_factory=FailingCloseWriter)
    assert result.summary["status"] == "failed"
    assert result.summary["visits_completed"] == 11
    assert "simulated close error" in result.summary["error"]
    for filename in ("metadata.json", "summary.json", "progress.json"):
        saved = json.loads((result.directory / filename).read_text())
        assert saved["status"] == "failed"
    with (result.directory / "samples.csv").open(newline="") as source:
        assert len(list(csv.DictReader(source))) == 22


@pytest.mark.parametrize("failure", ["event", "checkpoint"])
def test_failure_after_committed_visit_does_not_count_visit_twice(profile, tmp_path, failure):
    class FailingAfterVisitWriter(ScanWriter):
        failed_once = False

        def event(self, name, *, time_s, **details):
            super().event(name, time_s=time_s, **details)
            if failure == "event" and name == "visit_completed" and details.get("kind") == "grid" and not self.failed_once:
                self.failed_once = True
                raise OSError("event failure after successful visit")

        def checkpoint(self, summary):
            super().checkpoint(summary)
            if failure == "checkpoint" and summary.get("grid_points_completed") == 1 and not self.failed_once:
                self.failed_once = True
                raise OSError("checkpoint failure after successful visit")

    clock = VirtualClock()
    robot, sensor, _ = make_simulated_adapters(profile, clock)
    result = run_simulation(profile, tmp_path / failure, robot=robot, sensor=sensor, clock=clock,
                            writer_factory=FailingAfterVisitWriter)
    assert result.summary["status"] == "failed"
    assert result.summary["visits_completed"] == 2
    assert result.summary["visits_failed"] == 0
    assert result.summary["grid_points_completed"] == 1
    assert result.summary["grid_points_failed"] == 0
    assert robot.command_count == 2
    assert robot.stop_count >= 1
    assert not robot._connected and not sensor._connected
    with (result.directory / "measurements.csv").open(newline="") as source:
        measurements = list(csv.DictReader(source))
    assert len(measurements) == 2
    assert all(row["status"] == "ok" for row in measurements)


def test_uncertain_measurement_write_is_not_retried_as_duplicate_row(profile, tmp_path):
    class UncertainWriteWriter(ScanWriter):
        failed_once = False

        def write_measurement(self, row):
            super().write_measurement(row)
            if not self.failed_once:
                self.failed_once = True
                raise OSError("uncertain CSV flush after append")

    result = run_simulation(profile, tmp_path / "uncertain-append", writer_factory=UncertainWriteWriter)
    assert result.summary["status"] == "failed"
    with (result.directory / "measurements.csv").open(newline="") as source:
        measurements = list(csv.DictReader(source))
    keys = [(row["visit_id"], row["attempt"]) for row in measurements]
    assert len(keys) == len(set(keys))
    assert len(measurements) == 1
