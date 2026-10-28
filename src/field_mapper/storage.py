"""Exportación de planes a un directorio nuevo, sin sobrescribir resultados."""

import csv
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import platform
import subprocess
from uuid import uuid4

from .config import PlanConfig
from .planner import Trajectory, validate_trajectory


PATH_COLUMNS = (
    "point_id", "visit_index", "iu", "iv", "ih", "u_scan_mm", "v_scan_mm", "h_scan_mm",
    "x_magnet_mm", "y_magnet_mm", "z_magnet_mm", "simulated",
)


def path_row(point, visit_index: int) -> tuple:
    return (point.point_id, visit_index, *point.indices, *point.position_scan_mm, *point.position_mm, "true")


def source_revision() -> dict:
    """Registra HEAD y cambios locales; no confunde HEAD con código sin commit."""
    root = Path(__file__).resolve().parents[2]
    try:
        result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, timeout=5)
        if result.returncode:
            return {"revision": None, "worktree_modified": None}
        state = subprocess.run(["git", "status", "--porcelain"], cwd=root, capture_output=True, text=True, timeout=5)
        return {"revision": result.stdout.strip(), "worktree_modified": bool(state.stdout) if state.returncode == 0 else None,
                "revision_scope": "HEAD; worktree_modified identifies uncommitted source changes"}
    except (OSError, subprocess.TimeoutExpired):
        return {"revision": None, "worktree_modified": None}


def scan_metadata(config: PlanConfig) -> dict:
    data = config.to_dict()
    scan = data["scan"]
    simulation = data.get("simulation")
    return {
        "scan_basis": {"origin": "bore_center", "axes": ["u", "v", "h"], "h": "physical_up",
                       "rotation_magnet_from_scan": scan["rotation_magnet_from_scan"],
                       "index_order": ["iu", "iv", "ih"]},
        "frame_mappings": {"scan_to_magnet": "p_magnet = rotation_magnet_from_scan @ p_scan",
                           "magnet_to_base": "p_sensor_base = R_base_magnet @ p_magnet + t_base_magnet",
                           "T_base_magnet": simulation.get("T_base_magnet") if simulation else None,
                           "physical_up_base": scan["up_base"],
                           "tool_compensation": "see calibration and config_used.json; reported pose is flange or TCP as configured"},
        "reference_policy": {"pre_reference": scan["pre_reference"], "post_reference": scan["post_reference"],
                             "between_layers": False, "scope": "outside grid_scan"},
        "scan": scan, "runtime_phases": [], "runtime_transitions": [], "current_phase": None,
        "layer_index_scope": "signed scan lattice ih; layer_height_scan_mm = ih * step_mm; references have no layer",
        "hardware_transition_paths_validated": False,
        "source_code": source_revision(),
    }


def _json_bytes(data: dict) -> bytes:
    return json.dumps(data, ensure_ascii=False, sort_keys=True, allow_nan=False, indent=2).encode("utf-8") + b"\n"


def export_plan(trajectory: Trajectory, config: PlanConfig, output_dir: str | Path | None = None) -> Path:
    """Exporta solo la malla; no incluye referencias ni movimientos de entrada/salida."""
    validate_trajectory(trajectory, config)
    if output_dir is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        destination = Path("runs") / f"plan-{stamp}-{uuid4().hex[:8]}"
    else:
        destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=False)

    config_bytes = _json_bytes(config.to_dict())
    (destination / "config_used.json").write_bytes(config_bytes)
    csv_path = destination / "path.csv"
    with csv_path.open("x", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, delimiter=",")
        writer.writerow(PATH_COLUMNS)
        for visit_index, point in enumerate(trajectory.points):
            writer.writerow(path_row(point, visit_index))

    from .visualization import save_trajectory_figure
    save_trajectory_figure(trajectory, config, destination / "trajectory.png")

    summary = {
        "status": "complete",
        "kind": "geometry_plan",
        "simulated": True,
        "grid_points": len(trajectory.points),
        "layers": trajectory.layer_count,
        "grid_path_length_mm": trajectory.length_mm,
        "reference_visits_included": False,
        "entry_exit_included": False,
        "collision_check_performed": False,
        "height_order": "physical_up_ascending",
    }
    metadata = {
        "schema_version": 1,
        "kind": "geometry_plan",
        "mode": config.mode,
        "simulated": True,
        "hardware_connected": False,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "units": config.to_dict()["units"],
        "coordinate_frame": {"origin": "bore_center", "x": "vertical_in_reference_mounting", "z": "bore_axis", "handedness": "right", "physical_up": "scan.h"},
        "point_id_scope": "scan lattice indices (iu,iv,ih); interpret with config_used.json",
        "visit_index_base": 0,
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "path_sha256": hashlib.sha256(csv_path.read_bytes()).hexdigest(),
        "software": {
            "field_mapper": version("field-mapper"), "python": platform.python_version(),
            "numpy": version("numpy"), "matplotlib": version("matplotlib"),
        },
        "calibration": None,
        "summary": summary,
        **scan_metadata(config),
    }
    (destination / "summary.json").write_bytes(_json_bytes(summary))
    (destination / "metadata.json").write_bytes(_json_bytes(metadata))
    return destination
