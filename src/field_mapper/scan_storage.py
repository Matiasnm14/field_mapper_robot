"""Persistencia incremental de adquisiciones simuladas y sus resultados parciales."""

import csv
from datetime import datetime, timezone
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import os
from pathlib import Path
import platform
from uuid import uuid4

from .config import PlanConfig
from .planner import Trajectory, validate_trajectory
from .storage import PATH_COLUMNS


MEASUREMENT_COLUMNS = (
    "run_id", "visit_id", "visit_index", "point_id", "grid_visit_index", "attempt", "kind",
    "layer_index", "status", "error", "simulated", "x_target_magnet_mm", "y_target_magnet_mm",
    "z_target_magnet_mm", "x_command_base_mm", "y_command_base_mm", "z_command_base_mm",
    "x_reported_base_mm", "y_reported_base_mm", "z_reported_base_mm", "x_sensor_magnet_mm",
    "y_sensor_magnet_mm", "z_sensor_magnet_mm", "angle_target_deg", "angle_reported_deg", "pose_frame",
    "B_n_mean_T", "B_n_std_T", "valid_samples", "requested_samples", "started_time_s",
    "arrived_time_s", "read_started_time_s", "finished_time_s",
)
SAMPLE_COLUMNS = (
    "run_id", "visit_id", "visit_index", "point_id", "attempt", "sample_index", "status", "error",
    "simulated", "original_value", "original_unit", "B_n_T", "acquired_time_s", "received_time_s",
    "sequence_id", "fresh", "freshness_evidence",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_bytes(data: dict) -> bytes:
    return (json.dumps(data, ensure_ascii=False, sort_keys=True, allow_nan=False, indent=2) + "\n").encode("utf-8")


def _installed_version(package: str) -> str | None:
    try:
        return version(package)
    except PackageNotFoundError:
        return None


def _atomic_json(path: Path, data: dict) -> None:
    """Un fallo conserva el documento anterior; los lectores nunca ven JSON truncado."""
    payload = _json_bytes(data)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    try:
        with temporary.open("xb") as handle:
            handle.write(payload)
            handle.flush()
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


class ScanWriter:
    """Cada CSV/evento se vacía al disco antes de aceptar la siguiente fila.

    Un proceso interrumpido deja metadata.status='running'. Los tiempos *_time_s
    corresponden exclusivamente al reloj virtual, y los *_utc al reloj de pared.
    ``finish`` no cierra los archivos: puede reintentarse tras un fallo de E/S.
    """

    def __init__(self, config: PlanConfig, trajectory: Trajectory, output_dir: Path | str | None = None):
        validate_trajectory(trajectory, config)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        self.run_id = f"simulate-{stamp}-{uuid4().hex[:12]}"
        self.directory = Path(output_dir) if output_dir is not None else Path("runs") / self.run_id
        self._handles = []
        self.directory.mkdir(parents=True, exist_ok=False)
        try:
            config_dict = config.to_dict()
            config_bytes = _json_bytes(config_dict)
            (self.directory / "config_used.json").write_bytes(config_bytes)
            path = self.directory / "path.csv"
            with path.open("x", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(PATH_COLUMNS)
                for index, point in enumerate(trajectory.points):
                    writer.writerow((point.point_id, index, *point.indices, *point.position_mm, "true"))
            simulation = config_dict.get("simulation")
            calibration = None
            if isinstance(simulation, dict):
                calibration = {"synthetic": True, "source": "config_used.json:simulation", "parameters": simulation}
            self._metadata = {
                "schema_version": 1, "run_id": self.run_id, "kind": "field_scan", "status": "running",
                "mode": config.mode, "simulated": True, "hardware_connected": False,
                "created_at_utc": _utc_now(), "units": config_dict["units"],
                "time": {"acquisition": "shared_virtual_clock_seconds", "wall_clock": "UTC", "virtual_epoch_s": 0.0},
                "coordinate_frame": {"origin": "bore_center", "x": "vertical", "z": "bore_axis", "handedness": "right"},
                "pose_evidence": "simulated controller pose; not independent metrological verification",
                "point_id_scope": "integer_lattice_indices; interpret with config_used.json",
                "visit_index_base": 0, "grid_points": len(trajectory.points), "layers": trajectory.layer_count,
                "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
                "path_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "software": {"field_mapper": _installed_version("field-mapper"), "python": platform.python_version(),
                             "numpy": _installed_version("numpy"), "matplotlib": _installed_version("matplotlib")},
                "calibration": calibration,
            }
            _atomic_json(self.directory / "metadata.json", self._metadata)
            self._measurement_handle, self._measurements = self._open_csv("measurements.csv", MEASUREMENT_COLUMNS)
            self._sample_handle, self._samples = self._open_csv("samples.csv", SAMPLE_COLUMNS)
            self._event_handle = (self.directory / "events.jsonl").open("x", encoding="utf-8")
            self._handles.append(self._event_handle)
            self.checkpoint({"status": "running", "completed_visits": 0})
        except BaseException:
            try:
                self.close()
            except BaseException:
                pass  # El error original de inicialización debe conservar su causa.
            raise

    def _open_csv(self, filename, columns):
        handle = (self.directory / filename).open("x", encoding="utf-8", newline="")
        self._handles.append(handle)
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        handle.flush()
        return handle, writer

    def _row(self, row: dict) -> dict:
        return {**row, "run_id": self.run_id, "simulated": "true"}

    def write_sample(self, row: dict) -> None:
        self._samples.writerow(self._row(row))
        self._sample_handle.flush()

    def write_measurement(self, row: dict) -> None:
        self._measurements.writerow(self._row(row))
        self._measurement_handle.flush()

    def event(self, event: str, *, time_s: float, **details) -> None:
        record = {**details, "run_id": self.run_id, "simulated": True, "event": event,
                  "time_s": time_s, "recorded_at_utc": _utc_now()}
        self._event_handle.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self._event_handle.flush()

    def checkpoint(self, summary: dict) -> None:
        progress = {**summary, "run_id": self.run_id, "simulated": True, "updated_at_utc": _utc_now()}
        _atomic_json(self.directory / "progress.json", progress)

    def update_metadata(self, details: dict) -> None:
        """Persiste identidades/época virtual conservando los campos de tiempo.

        Combina un nivel de objetos (por ejemplo ``time.virtual_epoch_s``),
        y solo modifica el estado en memoria después de una escritura exitosa.
        """
        metadata = dict(self._metadata)
        for key, value in details.items():
            previous = metadata.get(key)
            metadata[key] = {**previous, **value} if isinstance(previous, dict) and isinstance(value, dict) else value
        metadata.update(run_id=self.run_id, simulated=True)
        _atomic_json(self.directory / "metadata.json", metadata)
        self._metadata = metadata

    def finish(self, summary: dict) -> None:
        final = {**summary, "run_id": self.run_id, "simulated": True}
        _atomic_json(self.directory / "summary.json", final)
        self.checkpoint(final)
        metadata = {**self._metadata, "status": final["status"], "finished_at_utc": _utc_now(), "summary": final}
        _atomic_json(self.directory / "metadata.json", metadata)
        self._metadata = metadata

    def close(self) -> None:
        first_error = None
        for handle in self._handles:
            try:
                handle.close()
            except BaseException as error:
                if first_error is None:
                    first_error = error
        if first_error is not None:
            raise first_error
