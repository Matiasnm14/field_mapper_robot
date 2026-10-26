"""La demo pública funciona sin pantalla y comunica errores por exit code."""

import csv
from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


def command(config, tmp_path, *, destination="run", options=()):
    source = tmp_path / f"{destination}.json"
    source.write_text(json.dumps(config.to_dict()), encoding="utf-8")
    target = tmp_path / destination
    environment = dict(os.environ, MPLBACKEND="TkAgg", DISPLAY="", MPLCONFIGDIR=str(tmp_path / "mpl"))
    result = subprocess.run(
        [sys.executable, "-m", "field_mapper", "simulate", "--config", str(source),
         "--output-dir", str(target), *options],
        capture_output=True, text=True, env=environment, timeout=60,
    )
    return result, target


def test_simulate_cli_real_headless_exports_pilot_and_figures(config, tmp_path):
    profile = replace(config, geometry=replace(config.geometry, step_mm=10))
    result, destination = command(profile, tmp_path)
    assert result.returncode == 0, result.stderr
    summary = json.loads((destination / "summary.json").read_text())
    assert summary["status"] == "complete"
    assert summary["grid_points_completed"] == 123
    assert summary["reference_visits_completed"] == 8
    assert summary["visits_completed"] == 131
    assert summary["used_samples"] == 131 * profile.acquisition.samples_per_point
    figures = {"trajectory.png", "field_maps.png", "field_profiles.png", "reference_drift.png"}
    for filename in figures:
        assert (destination / filename).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert set(summary["figures"]) == figures
    with (destination / "measurements.csv").open(encoding="utf-8", newline="") as handle:
        measurements = list(csv.DictReader(handle))
    assert len(measurements) == 131
    assert all(row["simulated"] == "true" for row in measurements)
    metadata = json.loads((destination / "metadata.json").read_text())
    assert metadata["hardware_connected"] is False
    assert metadata["calibration"]["synthetic"] is True


@pytest.mark.parametrize("options, expected_status, expected_exit", [
    (("--fail-motion-at-visit", "1", "--motion-fault", "accepted_only"), "failed", 1),
    (("--fail-sensor-at-read", "1", "--sensor-fault", "timeout"), "failed", 1),
    (("--fail-sensor-at-read", "1", "--sensor-fault", "timeout", "--sensor-fault-once"), "complete", 0),
])
def test_cli_fault_options_are_effective_and_recorded(config, tmp_path, options, expected_status, expected_exit):
    profile = replace(config, geometry=replace(config.geometry, radius_mm=10, step_mm=10))
    result, destination = command(profile, tmp_path, options=options)
    assert result.returncode == expected_exit, result.stderr
    summary = json.loads((destination / "summary.json").read_text())
    assert summary["status"] == expected_status
    effective = json.loads((destination / "config_used.json").read_text())["simulation"]
    if "--fail-motion-at-visit" in options:
        assert effective["motion_fault_at_visit"] == 1
        assert effective["motion_fault_mode"] == "accepted_only"
        assert summary["visits_completed"] == 1
    else:
        assert effective["sensor_fault_at_read"] == 1
        assert effective["sensor_fault_mode"] == "timeout"
        assert effective["sensor_fault_persistent"] is ("--sensor-fault-once" not in options)


def test_cli_invalid_fault_index_does_not_create_output(config, tmp_path):
    result, destination = command(config, tmp_path, options=("--fail-motion-at-visit", "-1"))
    assert result.returncode == 2
    assert not destination.exists()


def test_cli_existing_output_is_preserved(config, tmp_path):
    target = tmp_path / "existing"
    target.mkdir()
    (target / "marker.txt").write_text("resultado anterior", encoding="utf-8")
    result, destination = command(config, tmp_path, destination="existing")
    assert result.returncode == 2
    assert {path.name for path in destination.iterdir()} == {"marker.txt"}
    assert (target / "marker.txt").read_text() == "resultado anterior"
