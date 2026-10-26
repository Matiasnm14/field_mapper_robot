"""Exportación de planes a un directorio nuevo, sin sobrescribir resultados."""

import csv
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import platform
from uuid import uuid4

from .config import PlanConfig
from .planner import Trajectory, validate_trajectory


PATH_COLUMNS = (
    "point_id", "visit_index", "ix", "iy", "iz",
    "x_magnet_mm", "y_magnet_mm", "z_magnet_mm", "simulated",
)


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
            writer.writerow((point.point_id, visit_index, *point.indices, *point.position_mm, "true"))

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
    }
    metadata = {
        "schema_version": 1,
        "kind": "geometry_plan",
        "mode": config.mode,
        "simulated": True,
        "hardware_connected": False,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "units": config.to_dict()["units"],
        "coordinate_frame": {"origin": "bore_center", "x": "vertical", "z": "bore_axis", "handedness": "right"},
        "point_id_scope": "integer_lattice_indices; interpret with config_used.json",
        "visit_index_base": 0,
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "path_sha256": hashlib.sha256(csv_path.read_bytes()).hexdigest(),
        "software": {
            "field_mapper": version("field-mapper"), "python": platform.python_version(),
            "numpy": version("numpy"), "matplotlib": version("matplotlib"),
        },
        "calibration": None,
        "summary": summary,
    }
    (destination / "summary.json").write_bytes(_json_bytes(summary))
    (destination / "metadata.json").write_bytes(_json_bytes(metadata))
    return destination
