import csv
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from field_mapper.cli import main
from field_mapper.planner import plan_trajectory
from field_mapper.storage import export_plan


def test_plan_command_headless_exports_and_preserves_existing_results(config, tmp_path):
    profile = replace(config, geometry=replace(config.geometry, step_mm=10))
    config_path = tmp_path / "profile.json"
    config_path.write_text(json.dumps(profile.to_dict()), encoding="utf-8")
    destination = tmp_path / "run"
    environment = dict(os.environ, MPLBACKEND="TkAgg", DISPLAY="", MPLCONFIGDIR=str(tmp_path / "mpl"))
    command = [sys.executable, "-m", "field_mapper", "plan", "--config", str(config_path), "--output-dir", str(destination)]
    result = subprocess.run(command, capture_output=True, text=True, env=environment, timeout=60)
    assert result.returncode == 0, result.stderr
    assert "123 puntos, 7 capas" in result.stdout
    assert {path.name for path in destination.iterdir()} == {
        "path.csv", "trajectory.png", "config_used.json", "metadata.json", "summary.json"
    }
    with (destination / "path.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 123
    assert len({row["point_id"] for row in rows}) == 123
    assert [int(row["visit_index"]) for row in rows] == list(range(123))
    assert all(row["simulated"] == "true" for row in rows)
    assert rows[0]["z_magnet_mm"] == "-30.0"
    assert rows[-1]["z_magnet_mm"] == "30.0"
    assert (destination / "trajectory.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    metadata = json.loads((destination / "metadata.json").read_text())
    assert metadata["simulated"] is True
    assert metadata["hardware_connected"] is False
    assert metadata["calibration"] is None
    assert metadata["config_sha256"] == hashlib.sha256((destination / "config_used.json").read_bytes()).hexdigest()
    assert metadata["path_sha256"] == hashlib.sha256((destination / "path.csv").read_bytes()).hexdigest()
    summary = json.loads((destination / "summary.json").read_text())
    assert summary["grid_points"] == 123
    assert summary["reference_visits_included"] is False
    assert summary["collision_check_performed"] is False
    original = {path.name: path.read_bytes() for path in destination.iterdir()}
    rerun = subprocess.run(command, capture_output=True, text=True, env=environment, timeout=60)
    assert rerun.returncode == 2
    assert "Error:" in rerun.stderr
    assert {path.name: path.read_bytes() for path in destination.iterdir()} == original


def test_plan_rejects_invalid_config_before_creating_outputs(config, tmp_path, capsys):
    data = config.to_dict()
    data["geometry"]["step_mm"] = 7
    source = tmp_path / "invalid.json"
    source.write_text(json.dumps(data), encoding="utf-8")
    destination = tmp_path / "must-not-exist"
    assert main(["plan", "--config", str(source), "--output-dir", str(destination)]) == 2
    assert not destination.exists()
    assert "radio/paso" in capsys.readouterr().err


def test_export_rejects_incompatible_path_before_creating_outputs(config, tmp_path):
    trajectory = plan_trajectory(config)
    other = replace(config, geometry=replace(config.geometry, step_mm=10))
    destination = tmp_path / "must-not-exist"
    with pytest.raises(ValueError):
        export_plan(trajectory, other, destination)
    assert not destination.exists()


def test_plan_does_not_open_network_connections(config, tmp_path, monkeypatch):
    import socket
    from field_mapper import visualization

    def forbidden_connection(*args, **kwargs):
        raise AssertionError("La planificación no debe abrir conexiones.")

    # La figura real ya se verifica en la prueba de CLI sin pantalla.
    def placeholder_figure(trajectory, config, path):
        Path(path).write_bytes(b"placeholder")

    monkeypatch.setattr(socket.socket, "connect", forbidden_connection)
    monkeypatch.setattr(socket, "create_connection", forbidden_connection)
    monkeypatch.setattr(visualization, "save_trajectory_figure", placeholder_figure)
    monkeypatch.chdir(tmp_path)
    trajectory = plan_trajectory(config)
    first = export_plan(trajectory, config)
    second = export_plan(trajectory, config)
    assert first != second
    assert first.parent == Path("runs")
    assert (first / "path.csv").read_bytes() == (second / "path.csv").read_bytes()
