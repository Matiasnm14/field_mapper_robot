"""Los datos parciales conservan vínculos, ausencia de evidencia y fallos de E/S."""

import csv
import hashlib
import json
from pathlib import Path

import pytest

from field_mapper.field_visualization import save_field_figures
from field_mapper.planner import Trajectory, plan_trajectory
from field_mapper.scan_storage import ScanWriter
import field_mapper.scan_storage as storage


def read_rows(path):
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def test_incremental_records_keep_links_and_missing_pose(config, tmp_path):
    trajectory = plan_trajectory(config)
    writer = ScanWriter(config, trajectory, tmp_path / "scan")
    try:
        writer.write_sample({"visit_id": "v0001", "point_id": "p:0:0:0", "attempt": 2,
                             "status": "ok", "original_value": 5000, "original_unit": "G", "B_n_T": 0.5})
        writer.write_measurement({"visit_id": "v0001", "point_id": "p:0:0:0", "attempt": 2,
                                  "kind": "reference", "status": "ok", "x_target_magnet_mm": 0,
                                  "B_n_mean_T": 0.5, "valid_samples": 1})
        sample, = read_rows(writer.directory / "samples.csv")
        measurement, = read_rows(writer.directory / "measurements.csv")
        assert sample["run_id"] == measurement["run_id"] == writer.run_id
        assert sample["visit_id"] == measurement["visit_id"] == "v0001"
        assert sample["attempt"] == measurement["attempt"] == "2"
        assert measurement["simulated"] == "true"
        assert measurement["x_target_magnet_mm"] == "0"
        assert measurement["x_reported_base_mm"] == measurement["x_sensor_magnet_mm"] == ""
        assert measurement["B_n_std_T"] == ""
        writer.event("read_complete", time_s=12.25, visit_id="v0001")
        event = json.loads((writer.directory / "events.jsonl").read_text())
        assert event["time_s"] == 12.25 and event["visit_id"] == "v0001"
        assert event["simulated"] is True
        metadata = json.loads((writer.directory / "metadata.json").read_text())
        assert metadata["status"] == "running"
        for name in ("path", "config"):
            filename = "path.csv" if name == "path" else "config_used.json"
            assert metadata[f"{name}_sha256"] == hashlib.sha256((writer.directory / filename).read_bytes()).hexdigest()
        assert len(read_rows(writer.directory / "path.csv")) == len(trajectory.points)
        writer.checkpoint({"status": "running", "completed_visits": 1})
        assert json.loads((writer.directory / "progress.json").read_text())["completed_visits"] == 1
        writer.finish({"status": "failed", "completed_visits": 1, "error": "sensor timeout"})
        assert json.loads((writer.directory / "metadata.json").read_text())["status"] == "failed"
        assert json.loads((writer.directory / "summary.json").read_text())["error"] == "sensor timeout"
    finally:
        writer.close()


def test_existing_results_and_invalid_plan_do_not_get_written(config, tmp_path):
    trajectory = plan_trajectory(config)
    destination = tmp_path / "scan"
    destination.mkdir()
    marker = destination / "keep"
    marker.write_text("existing")
    with pytest.raises(FileExistsError):
        ScanWriter(config, trajectory, destination)
    assert list(destination.iterdir()) == [marker]
    invalid = Trajectory(trajectory.radius_mm, trajectory.step_mm, trajectory.points[:-1])
    with pytest.raises(ValueError):
        ScanWriter(config, invalid, tmp_path / "invalid")
    assert not (tmp_path / "invalid").exists()


def test_interruption_preserves_running_status_and_rows(config, tmp_path):
    writer = ScanWriter(config, plan_trajectory(config), tmp_path / "partial")
    writer.write_measurement({"visit_id": "v0", "status": "failed", "error": "movement timeout"})
    writer.close()
    assert json.loads((writer.directory / "metadata.json").read_text())["status"] == "running"
    assert read_rows(writer.directory / "measurements.csv")[0]["error"] == "movement timeout"
    assert not (writer.directory / "summary.json").exists()


def test_atomic_checkpoint_and_finish_can_retry(config, tmp_path, monkeypatch):
    writer = ScanWriter(config, plan_trajectory(config), tmp_path / "scan")
    original = storage.os.replace
    try:
        def fail_replace(source, destination):
            raise OSError("disk unavailable")
        with monkeypatch.context() as context:
            context.setattr(storage.os, "replace", fail_replace)
            with pytest.raises(OSError, match="disk unavailable"):
                writer.checkpoint({"status": "failed"})
            with pytest.raises(OSError, match="disk unavailable"):
                writer.finish({"status": "complete"})
        assert storage.os.replace is original
        assert json.loads((writer.directory / "progress.json").read_text())["status"] == "running"
        assert json.loads((writer.directory / "metadata.json").read_text())["status"] == "running"
        writer.finish({"status": "complete"})
        assert json.loads((writer.directory / "metadata.json").read_text())["status"] == "complete"
        assert not list(writer.directory.glob(".*.tmp"))
    finally:
        writer.close()


def test_write_failure_propagates_and_close_attempts_all_handles(config, tmp_path):
    writer = ScanWriter(config, plan_trajectory(config), tmp_path / "scan")

    class BrokenHandle:
        closed = False
        def flush(self):
            raise OSError("flush failed")
        def close(self):
            self.closed = True
            raise OSError("close failed")

    broken = BrokenHandle()
    writer._sample_handle = broken
    with pytest.raises(OSError, match="flush failed"):
        writer.write_sample({"visit_id": "v0"})
    writer._handles.insert(0, broken)
    with pytest.raises(OSError, match="close failed"):
        writer.close()
    assert broken.closed
    assert all(handle.closed for handle in writer._handles)


def test_init_failure_closes_handles(config, tmp_path, monkeypatch):
    opened = []
    original = ScanWriter._open_csv

    def failing_open(self, filename, columns):
        result = original(self, filename, columns)
        opened.append(result[0])
        if filename == "samples.csv":
            raise OSError("creation failed")
        return result

    monkeypatch.setattr(ScanWriter, "_open_csv", failing_open)
    with pytest.raises(OSError, match="creation failed"):
        ScanWriter(config, plan_trajectory(config), tmp_path / "scan")
    assert len(opened) == 2 and all(handle.closed for handle in opened)


def test_figures_empty_partial_and_invalid_data(config, tmp_path, monkeypatch):
    from matplotlib.axes import Axes
    writer = ScanWriter(config, plan_trajectory(config), tmp_path / "scan")
    expected = ["field_maps.png", "field_profiles.png", "reference_drift.png"]
    assert save_field_figures(writer.directory) == expected
    captured_colors = []
    original_scatter = Axes.scatter

    def capture_scatter(self, *args, **kwargs):
        captured_colors.extend(kwargs.get("c", []))
        return original_scatter(self, *args, **kwargs)

    monkeypatch.setattr(Axes, "scatter", capture_scatter)
    try:
        common = {"x_target_magnet_mm": 0, "y_target_magnet_mm": 0, "z_target_magnet_mm": 0,
                  "valid_samples": 2, "B_n_std_T": 0.01, "finished_time_s": 1}
        writer.write_measurement({**common, "kind": "grid", "status": "ok", "B_n_mean_T": 0.5})
        writer.write_measurement({**common, "kind": "grid", "status": "failed", "B_n_mean_T": 999})
        writer.write_measurement({**common, "kind": "grid", "status": "ok", "valid_samples": 0, "B_n_mean_T": 998})
        writer.write_measurement({**common, "kind": "reference", "status": "ok", "B_n_mean_T": 0.6})
        assert save_field_figures(writer.directory) == expected
        assert captured_colors == [0.5, 0.5, 0.5]
        for name in expected:
            assert (writer.directory / name).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    finally:
        writer.close()


def test_nonfinite_raw_sample_is_traceable_but_json_is_strict(config, tmp_path):
    writer = ScanWriter(config, plan_trajectory(config), tmp_path / "scan")
    try:
        writer.write_sample({"visit_id": "v0", "status": "failed", "error": "nonfinite sensor value",
                             "original_value": float("nan"), "original_unit": "G"})
        sample, = read_rows(writer.directory / "samples.csv")
        assert sample["original_value"] == "nan"  # Evidencia original, sin inventar conversión.
        assert sample["B_n_T"] == ""
        original_progress = (writer.directory / "progress.json").read_bytes()
        with pytest.raises(ValueError):
            writer.checkpoint({"status": "running", "invalid": float("nan")})
        assert (writer.directory / "progress.json").read_bytes() == original_progress
        with pytest.raises(ValueError):
            writer.event("invalid_json", time_s=0, invalid=float("inf"))
        assert (writer.directory / "events.jsonl").read_text() == ""
        writer.event("valid_json", time_s=0)
        assert json.loads((writer.directory / "events.jsonl").read_text())["event"] == "valid_json"
    finally:
        writer.close()


def test_default_runs_are_exclusive(config, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    trajectory = plan_trajectory(config)
    first = ScanWriter(config, trajectory)
    second = ScanWriter(config, trajectory)
    try:
        assert first.directory != second.directory
        assert first.directory.parent == second.directory.parent == Path("runs")
        assert first.directory.name == first.run_id
        assert second.directory.name == second.run_id
    finally:
        first.close()
        second.close()


def test_final_metadata_stays_running_if_final_write_fails(config, tmp_path, monkeypatch):
    writer = ScanWriter(config, plan_trajectory(config), tmp_path / "scan")
    original = storage._atomic_json

    def fail_metadata(path, data):
        if path.name == "metadata.json":
            raise OSError("metadata unavailable")
        original(path, data)

    try:
        with monkeypatch.context() as context:
            context.setattr(storage, "_atomic_json", fail_metadata)
            with pytest.raises(OSError, match="metadata unavailable"):
                writer.finish({"status": "complete"})
        assert json.loads((writer.directory / "metadata.json").read_text())["status"] == "running"
        writer.finish({"status": "complete"})
        assert json.loads((writer.directory / "metadata.json").read_text())["status"] == "complete"
    finally:
        writer.close()


def test_metadata_identity_and_clock_epoch_update_atomically(config, tmp_path, monkeypatch):
    writer = ScanWriter(config, plan_trajectory(config), tmp_path / "scan")
    try:
        original_bytes = (writer.directory / "metadata.json").read_bytes()
        def failed_write(path, data):
            raise OSError("metadata unavailable")
        with monkeypatch.context() as context:
            context.setattr(storage, "_atomic_json", failed_write)
            with pytest.raises(OSError, match="metadata unavailable"):
                writer.update_metadata({"time": {"virtual_epoch_s": 20}, "robot_identity": {"model": "test"}})
        assert (writer.directory / "metadata.json").read_bytes() == original_bytes
        assert "robot_identity" not in writer._metadata
        writer.update_metadata({"time": {"virtual_epoch_s": 20}, "robot_identity": {"model": "simulator"}})
        metadata = json.loads((writer.directory / "metadata.json").read_text())
        assert metadata["time"]["virtual_epoch_s"] == 20
        assert metadata["time"]["wall_clock"] == "UTC"
        assert metadata["time"]["acquisition"] == "shared_virtual_clock_seconds"
        assert metadata["robot_identity"] == {"model": "simulator"}
        writer.finish({"status": "complete"})
        assert json.loads((writer.directory / "metadata.json").read_text())["robot_identity"] == {"model": "simulator"}
    finally:
        writer.close()
