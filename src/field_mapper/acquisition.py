"""Adquisición exclusivamente simulada, con referencias y persistencia incremental."""

from dataclasses import dataclass
from itertools import groupby
import math
from pathlib import Path
import statistics
import time

import numpy as np

from .clock import VirtualClock
from .config import PlanConfig
from .errors import ConnectionLost, MotionError, MotionTimeout, SensorError, SensorTimeout
from .geometry import ToolTransform
from .planner import Trajectory, plan_trajectory, validate_trajectory
from .robot.base import RobotAdapter, RobotPose
from .robot.simulated import SimulatedRobot
from .scan_storage import ScanWriter
from .sensor.base import SensorAdapter, SensorSample, normalize_value_to_tesla
from .sensor.simulated import SimulatedSensor


@dataclass(frozen=True)
class Visit:
    visit_index: int
    point_id: str
    kind: str
    position_magnet_mm: tuple[float, float, float]
    grid_visit_index: int | None
    layer_index: int | None

    @property
    def visit_id(self) -> str:
        return f"v:{self.visit_index:06d}"


@dataclass(frozen=True)
class ScanResult:
    directory: Path
    summary: dict


class _Cancelled(Exception):
    pass


def build_visits(trajectory: Trajectory) -> tuple[Visit, ...]:
    """Una referencia inicial y otra después de cada capa completa."""
    visits = [Visit(0, "p:0:0:0", "reference", (0.0, 0.0, 0.0), None, None)]
    grid_index = 0
    seen_layers = set()
    for layer, points in groupby(trajectory.points, key=lambda point: point.indices[2]):
        if layer in seen_layers:
            raise ValueError("Las visitas de una capa deben ser contiguas en la trayectoria.")
        seen_layers.add(layer)
        for point in points:
            visits.append(Visit(len(visits), point.point_id, "grid", point.position_mm, grid_index, layer))
            grid_index += 1
        visits.append(Visit(len(visits), "p:0:0:0", "reference", (0.0, 0.0, 0.0), None, layer))
    return tuple(visits)


def _make_sensor(config: PlanConfig, clock: VirtualClock, robot: RobotAdapter, transform: ToolTransform) -> SimulatedSensor:
    simulation = config.simulation
    return SimulatedSensor(
        clock, lambda: transform.robot_to_magnet(robot.pose().position_mm),
        seed=simulation.seed, base_field_T=simulation.base_field_T,
        gradient_T_per_mm=simulation.gradient_T_per_mm,
        quadratic_T_per_mm2=simulation.quadratic_T_per_mm2,
        drift_T_per_s=simulation.drift_T_per_s, noise_std_T=simulation.noise_std_T,
        sample_interval_s=simulation.sample_interval_s, original_unit=simulation.original_unit,
        fault_at_read=simulation.sensor_fault_at_read, fault_mode=simulation.sensor_fault_mode,
        fault_persistent=simulation.sensor_fault_persistent,
    )


def make_simulated_adapters(config: PlanConfig, clock: VirtualClock):
    if config.simulation is None or config.acquisition is None:
        raise ValueError("simulate requiere las secciones acquisition y simulation completas.")
    simulation = config.simulation
    transform = simulation.tool_transform()
    initial = tuple(transform.magnet_to_robot(simulation.initial_sensor_position_magnet_mm))
    robot = SimulatedRobot(
        clock, initial, angle_deg=simulation.angle_deg, speed_mm_s=simulation.speed_mm_s,
        move_latency_s=simulation.move_latency_s,
        fault_at_move=None if simulation.motion_fault_at_visit is None else simulation.motion_fault_at_visit + 1,
        fault_mode=simulation.motion_fault_mode,
    )
    return robot, _make_sensor(config, clock, robot, transform), transform


def _position_fields(prefix: str, position) -> dict:
    return {f"{axis}_{prefix}_mm": float(value) for axis, value in zip("xyz", position)}


def _pose_fields(pose: RobotPose, transform: ToolTransform) -> dict:
    position = np.asarray(pose.position_mm, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all() or not math.isfinite(pose.angle_deg):
        raise MotionError("La realimentación de pose no es finita o tiene forma incorrecta.")
    return {
        **_position_fields("reported_base", position),
        **_position_fields("sensor_magnet", transform.robot_to_magnet(position)),
        "angle_reported_deg": float(pose.angle_deg),
    }


def _confirmed_pose(robot: RobotAdapter, target, angle: float, config: PlanConfig) -> RobotPose:
    pose = robot.pose()
    _pose_fields(pose, config.simulation.tool_transform())
    angle_error = abs((pose.angle_deg - angle + 180) % 360 - 180)
    if (
        robot.state() != "idle"
        or math.dist(pose.position_mm, target) > config.acquisition.position_tolerance_mm
        or angle_error > config.acquisition.angle_tolerance_deg
    ):
        raise MotionError("El robot no está detenido en la pose objetivo dentro de tolerancias.")
    return pose


def _validate_sample(sample: SensorSample, not_before_s: float, last_sequence: int | None,
                     last_acquired: float | None, now_s: float) -> float:
    if sample.status != "ok" or sample.fresh is not True or not sample.freshness_evidence:
        raise SensorError("La muestra no acredita estado válido y frescura.")
    if type(sample.sequence_id) is not int or sample.sequence_id < 1:
        raise SensorError("La muestra simulada no tiene secuencia válida.")
    if last_sequence is not None and sample.sequence_id <= last_sequence:
        raise SensorError("Secuencia de muestra repetida o anterior.")
    acquired, received = sample.acquired_time_s, sample.received_time_s
    if (
        acquired is None or not math.isfinite(acquired) or not math.isfinite(received)
        or acquired < not_before_s or acquired > received or received > now_s + 1e-10
        or (last_acquired is not None and acquired <= last_acquired)
    ):
        raise SensorError("Los tiempos de la muestra no acreditan una adquisición nueva en este punto.")
    return normalize_value_to_tesla(sample.value, sample.unit)


def run_simulation(
    config: PlanConfig, output_dir: str | Path | None = None, *,
    trajectory: Trajectory | None = None, robot: RobotAdapter | None = None,
    sensor: SensorAdapter | None = None, clock: VirtualClock | None = None,
    writer_factory=ScanWriter, cancel_requested=None,
) -> ScanResult:
    """Ejecuta la demo; nunca acepta adaptadores reales ni reanuda una corrida.

    Errores de configuración/salida inicial propagan antes de conectar. Los fallos
    durante la corrida retornan un resultado parcial y detienen la secuencia.
    """
    if config.mode != "simulation" or config.acquisition is None or config.simulation is None:
        raise ValueError("simulate requiere un perfil completo y exclusivamente simulado.")
    trajectory = plan_trajectory(config) if trajectory is None else trajectory
    validate_trajectory(trajectory, config)
    visits = build_visits(trajectory)
    acquisition, simulation = config.acquisition, config.simulation
    if simulation.motion_fault_at_visit is not None and simulation.motion_fault_at_visit >= len(visits):
        raise ValueError("motion_fault_at_visit queda fuera de las visitas previstas.")
    if simulation.sensor_fault_at_read is not None and simulation.sensor_fault_at_read > len(visits) * acquisition.samples_per_point:
        raise ValueError("sensor_fault_at_read queda fuera de las lecturas previstas antes del fallo.")
    if clock is None:
        clock = getattr(robot, "clock", None) or getattr(sensor, "clock", None) or VirtualClock()
    default_robot, default_sensor, transform = make_simulated_adapters(config, clock)
    if robot is None:
        robot = default_robot
    if sensor is None:
        sensor = default_sensor if robot is default_robot else _make_sensor(config, clock, robot, transform)
    for adapter in (robot, sensor):
        if getattr(adapter, "simulated", False) is not True:
            raise ValueError("simulate solo acepta adaptadores identificados como simulados.")
        if getattr(adapter, "clock", clock) is not clock:
            raise ValueError("Los adaptadores simulados deben compartir el mismo reloj virtual.")

    writer = writer_factory(config, trajectory, output_dir)
    wall_start = time.perf_counter()
    virtual_start = clock.now()
    summary = {
        "status": "running", "kind": "simulated_acquisition", "simulated": True,
        "run_id": writer.run_id, "grid_points": len(trajectory.points), "grid_points_completed": 0,
        "grid_points_failed": 0, "reference_visits_expected": trajectory.layer_count + 1,
        "reference_visits_completed": 0, "visits_expected": len(visits), "visits_completed": 0,
        "visits_failed": 0, "visits_cancelled": 0, "measurement_attempts": 0,
        "uncertain_measurement_writes": 0,
        "valid_samples": 0, "used_samples": 0, "samples_attempted": 0,
        "virtual_duration_s": 0.0, "wall_duration_s": 0.0, "error": None,
        "figures": [], "cleanup_errors": [], "robot_identity": None, "sensor_identity": None,
        "B_n_min_T": None, "B_n_max_T": None, "B_n_mean_T": None,
        "reference_change_T": None,
    }
    active = None
    active_written = False
    measurement_write_attempted = False
    last_sequence = None
    last_acquired = None
    grid_values, reference_values = [], []

    def check_cancel():
        if cancel_requested is not None and cancel_requested():
            raise _Cancelled("Parada solicitada por el operador.")

    def event(name, **details):
        writer.event(name, time_s=clock.now(), **details)

    def safe_event(name, **details):
        try:
            event(name, **details)
        except (Exception, KeyboardInterrupt) as error:
            summary["cleanup_errors"].append(f"registro de {name}: {error}")

    def write_measurement(row):
        nonlocal active_written, measurement_write_attempted
        # Un append que falla al vaciar el búfer pudo guardar ya su fila. No
        # repetirlo desde el cierre ni darlo por confirmado.
        measurement_write_attempted = True
        try:
            writer.write_measurement(row)
        except (Exception, KeyboardInterrupt):
            summary["uncertain_measurement_writes"] += 1
            raise
        summary["measurement_attempts"] += 1
        active_written = True

    def checkpoint():
        summary["virtual_duration_s"] = clock.now() - virtual_start
        summary["wall_duration_s"] = time.perf_counter() - wall_start
        writer.checkpoint(summary)

    try:
        checkpoint()
        if hasattr(writer, "update_metadata"):
            writer.update_metadata({"time": {
                "acquisition": "shared_virtual_clock_seconds", "wall_clock": "UTC", "virtual_epoch_s": virtual_start,
            }})
        from .visualization import save_trajectory_figure
        save_trajectory_figure(trajectory, config, writer.directory / "trajectory.png")
        summary["figures"].append("trajectory.png")
        check_cancel()
        robot.connect()
        summary["robot_identity"] = robot.identity()
        sensor.connect()
        summary["sensor_identity"] = sensor.identity()
        if hasattr(writer, "update_metadata"):
            writer.update_metadata({"robot_identity": summary["robot_identity"], "sensor_identity": summary["sensor_identity"]})
        event("connected", robot_identity=summary["robot_identity"], sensor_identity=summary["sensor_identity"])
        initial = tuple(transform.magnet_to_robot(simulation.initial_sensor_position_magnet_mm))
        _confirmed_pose(robot, initial, simulation.angle_deg, config)

        for visit in visits:
            check_cancel()
            command = tuple(transform.magnet_to_robot(visit.position_magnet_mm))
            active = {
                "visit_id": visit.visit_id, "visit_index": visit.visit_index, "point_id": visit.point_id,
                "grid_visit_index": visit.grid_visit_index, "attempt": 1, "kind": visit.kind,
                "layer_index": visit.layer_index, "status": "failed", "error": None,
                **_position_fields("target_magnet", visit.position_magnet_mm),
                **_position_fields("command_base", command),
                "angle_target_deg": simulation.angle_deg, "pose_frame": simulation.pose_frame,
                "B_n_mean_T": None, "B_n_std_T": None, "valid_samples": 0,
                "requested_samples": acquisition.samples_per_point, "started_time_s": clock.now(),
            }
            active_written = False
            measurement_write_attempted = False
            event("visit_started", visit_id=visit.visit_id, kind=visit.kind, point_id=visit.point_id)
            movement_started = clock.now()
            robot.move_to(command, simulation.angle_deg)
            event("motion_accepted", visit_id=visit.visit_id)
            remaining = acquisition.motion_timeout_s - (clock.now() - movement_started)
            if remaining <= 0:
                raise MotionTimeout("El envío del movimiento agotó el plazo.")
            robot.wait_until_arrived(
                target_position_mm=command, angle_deg=simulation.angle_deg,
                timeout_s=remaining, position_tolerance_mm=acquisition.position_tolerance_mm,
                angle_tolerance_deg=acquisition.angle_tolerance_deg, stable_time_s=acquisition.stable_time_s,
            )
            if clock.now() - movement_started > acquisition.motion_timeout_s + 1e-10:
                raise MotionTimeout("La llegada excedió el plazo del movimiento.")
            pose = _confirmed_pose(robot, command, simulation.angle_deg, config)
            active.update(_pose_fields(pose, transform))
            active["arrived_time_s"] = clock.now()
            event("arrived", visit_id=visit.visit_id)
            clock.sleep(acquisition.settle_time_s)
            base_row = dict(active)

            for attempt in range(1, acquisition.max_sensor_retries + 2):
                active = {**base_row, "attempt": attempt, "valid_samples": 0, "read_started_time_s": clock.now()}
                active_written = False
                measurement_write_attempted = False
                pose = _confirmed_pose(robot, command, simulation.angle_deg, config)
                active.update(_pose_fields(pose, transform))
                values = []
                try:
                    for sample_index in range(acquisition.samples_per_point):
                        check_cancel()
                        sample = None
                        sample_row = {
                            "visit_id": visit.visit_id, "visit_index": visit.visit_index,
                            "point_id": visit.point_id, "attempt": attempt, "sample_index": sample_index,
                            "status": "failed", "error": None, "B_n_T": None,
                        }
                        summary["samples_attempted"] += 1
                        request_time = clock.now()
                        try:
                            sample = sensor.read_sample(
                                not_before_s=active["read_started_time_s"], timeout_s=acquisition.read_timeout_s,
                            )
                            if clock.now() - request_time > acquisition.read_timeout_s + 1e-10:
                                raise SensorTimeout("La muestra excedió el plazo de lectura.")
                            value = _validate_sample(sample, active["read_started_time_s"], last_sequence, last_acquired, clock.now())
                        except (SensorError, ConnectionLost) as error:
                            if sample is not None:
                                sample_row.update(_sample_fields(sample))
                            sample_row["error"] = str(error)
                            writer.write_sample(sample_row)
                            event("sample_rejected", visit_id=visit.visit_id, attempt=attempt, error=str(error))
                            raise
                        sample_row.update(_sample_fields(sample))
                        sample_row.update(status="ok", B_n_T=value)
                        writer.write_sample(sample_row)
                        last_sequence, last_acquired = sample.sequence_id, sample.acquired_time_s
                        values.append(value)
                        active["valid_samples"] = len(values)
                        summary["valid_samples"] += 1
                except SensorError as error:
                    active.update(status="failed", error=str(error), finished_time_s=clock.now())
                    write_measurement(active)
                    event("sensor_attempt_failed", visit_id=visit.visit_id, attempt=attempt, error=str(error))
                    if attempt > acquisition.max_sensor_retries:
                        raise
                    event("sensor_retry", visit_id=visit.visit_id, next_attempt=attempt + 1)
                    continue

                mean = math.fsum(value / len(values) for value in values)
                deviation = statistics.stdev(values) if len(values) > 1 else None
                if not math.isfinite(mean) or (deviation is not None and not math.isfinite(deviation)):
                    raise SensorError("Las estadísticas de las muestras no son finitas.")
                active.update(
                    status="ok", B_n_mean_T=mean,
                    B_n_std_T=deviation,
                    finished_time_s=clock.now(),
                )
                write_measurement(active)
                summary["visits_completed"] += 1
                summary["used_samples"] += len(values)
                if visit.kind == "grid":
                    summary["grid_points_completed"] += 1
                    grid_values.append(active["B_n_mean_T"])
                else:
                    summary["reference_visits_completed"] += 1
                    reference_values.append(active["B_n_mean_T"])
                active = None  # La fila ya es persistente; un error posterior no falla esta visita.
                event("visit_completed", visit_id=visit.visit_id, kind=visit.kind)
                checkpoint()
                break
        summary["status"] = "complete"
    except (KeyboardInterrupt, _Cancelled) as error:
        summary["status"] = "cancelled"
        summary["error"] = str(error) or "Interrupción del operador."
    except Exception as error:
        summary["status"] = "failed"
        summary["error"] = f"{type(error).__name__}: {error}"
    finally:
        if summary["status"] != "complete":
            try:
                robot.stop()
                safe_event("stop_requested")
            except (Exception, KeyboardInterrupt) as error:
                summary["cleanup_errors"].append(f"detención: {error}")
                safe_event("stop_unconfirmed", error=str(error))
            if active is not None:
                cancelled = summary["status"] == "cancelled"
                summary["visits_cancelled" if cancelled else "visits_failed"] += 1
                if not cancelled and active["kind"] == "grid":
                    summary["grid_points_failed"] += 1
                if not active_written and not measurement_write_attempted:
                    try:
                        try:
                            active.update(_pose_fields(robot.pose(), transform))
                        except (Exception, KeyboardInterrupt):
                            pass  # Pose desconocida queda vacía; no usar el objetivo.
                        active.update(status="cancelled" if cancelled else "failed", error=summary["error"], finished_time_s=clock.now())
                        write_measurement(active)
                    except (Exception, KeyboardInterrupt) as error:
                        summary["cleanup_errors"].append(f"registro del intento incompleto: {error}")
            safe_event(summary["status"], error=summary["error"])
        for name, adapter in (("sensor", sensor), ("robot", robot)):
            try:
                adapter.disconnect()
                safe_event("disconnected", adapter=name)
            except (Exception, KeyboardInterrupt) as error:
                summary["cleanup_errors"].append(f"desconexión {name}: {error}")
                if summary["status"] == "complete":
                    summary["status"] = "failed"
                    summary["error"] = f"No se pudo cerrar {name}: {error}"

    if grid_values:
        summary.update(
            B_n_min_T=min(grid_values), B_n_max_T=max(grid_values),
            B_n_mean_T=math.fsum(value / len(grid_values) for value in grid_values),
        )
    if len(reference_values) > 1:
        summary["reference_change_T"] = reference_values[-1] - reference_values[0]
    try:
        from .field_visualization import save_field_figures
        summary["figures"].extend(save_field_figures(writer.directory))
        event("figures_saved", figures=summary["figures"])
    except KeyboardInterrupt:
        summary["status"] = "cancelled"
        summary["error"] = "Interrupción del operador durante la generación de figuras."
    except Exception as error:
        summary["cleanup_errors"].append(f"figuras: {error}")
        if summary["status"] == "complete":
            summary["status"] = "failed"
            summary["error"] = f"No se pudieron exportar las figuras: {error}"
    summary["virtual_duration_s"] = clock.now() - virtual_start
    summary["wall_duration_s"] = time.perf_counter() - wall_start
    try:
        writer.finish(summary)
    except (Exception, KeyboardInterrupt) as error:
        summary["status"] = "cancelled" if isinstance(error, KeyboardInterrupt) else "failed"
        summary["error"] = f"No se pudo finalizar la persistencia: {error or 'interrupción del operador'}"
        summary["cleanup_errors"].append(summary["error"])
        safe_event("storage_finalization_failed", error=str(error))
        # Un único intento adicional publica el fallo si el error de escritura fue
        # transitorio. Si persiste, metadata.running permite reconocer el parcial.
        try:
            writer.finish(summary)
        except (Exception, KeyboardInterrupt) as retry_error:
            summary["cleanup_errors"].append(f"resumen parcial no persistido: {retry_error}")
    finally:
        try:
            writer.close()
        except (Exception, KeyboardInterrupt) as error:
            summary["status"] = "cancelled" if isinstance(error, KeyboardInterrupt) else "failed"
            summary["error"] = f"No se pudo cerrar la persistencia: {error or 'interrupción del operador'}"
            summary["cleanup_errors"].append(summary["error"])
            try:
                writer.finish(summary)  # Los JSON de estado no necesitan los CSV abiertos.
            except (Exception, KeyboardInterrupt) as final_error:
                summary["cleanup_errors"].append(f"error de cierre no persistido: {final_error}")
    return ScanResult(writer.directory, summary)


def _sample_fields(sample: SensorSample) -> dict:
    return {
        "original_value": sample.value, "original_unit": sample.unit,
        "acquired_time_s": sample.acquired_time_s, "received_time_s": sample.received_time_s,
        "sequence_id": sample.sequence_id, "fresh": sample.fresh,
        "freshness_evidence": sample.freshness_evidence,
    }
