"""Planificación, simulación e inspección explícita del M1 Pro."""

import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys

from .config import load_config
from .errors import AcquisitionError
from .planner import plan_trajectory
from .storage import export_plan


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Mapeo magnético con M1 Pro: barrido ascendente e inspección.")
    commands = parser.add_subparsers(dest="command", required=True)
    plan = commands.add_parser("plan", help="Generar malla, recorrido serpenteante, CSV y figura.")
    plan.add_argument("--config", required=True, type=Path, help="Perfil JSON de simulación.")
    plan.add_argument("--output-dir", type=Path, help="Directorio nuevo; por defecto runs/plan-<identificador>.")
    simulate = commands.add_parser("simulate", help="Adquirir campo sintético con robot y sensor simulados.")
    simulate.add_argument("--config", required=True, type=Path, help="Perfil JSON completo de simulación.")
    simulate.add_argument("--output-dir", type=Path, help="Directorio nuevo; por defecto runs/simulate-<identificador>.")
    simulate.add_argument("--fail-motion-at-visit", type=int, help="Inyectar fallo de movimiento en visita desde 0, incluidas referencias.")
    simulate.add_argument("--motion-fault", choices=("timeout", "out_of_tolerance", "connection_loss", "accepted_only"))
    simulate.add_argument("--fail-sensor-at-read", type=int, help="Inyectar fallo del sensor en lectura desde 1.")
    simulate.add_argument("--sensor-fault", choices=("timeout", "stale", "invalid", "unit", "connection_loss"))
    simulate.add_argument("--sensor-fault-once", action="store_true", help="Hacer transitorio el fallo de sensor para probar reintentos.")
    inspect = commands.add_parser("inspect-robot", help="Consultar pose, estado y alarmas del M1 Pro sin habilitar ni mover.")
    inspect.add_argument("--config", required=True, type=Path, help="Perfil del M1 Pro con IP real configurada.")
    scan = commands.add_parser("scan", help="Validar un perfil real; la adquisición con sensor real aún no está disponible.")
    scan.add_argument("--config", required=True, type=Path, help="Perfil de laboratorio M1 Pro.")
    scan.add_argument("--mode", required=True, choices=("hardware",), help="Selección explícita de operación real.")
    arguments = parser.parse_args(argv)
    try:
        if arguments.command in ("inspect-robot", "scan"):
            from .hardware_config import load_m1pro_config
            hardware = load_m1pro_config(arguments.config)
            if arguments.command == "scan":
                hardware.preflight_movement()
                raise ValueError(
                    "Adquisición hardware aún no disponible: falta integrar un sensor real con protocolo "
                    "validado y rutas de entrada/retirada verificadas. No se abrió ninguna conexión."
                )
            from .robot.dobot_m1pro import inspect_robot
            inspection = inspect_robot(hardware)
            print(json.dumps(inspection, ensure_ascii=False, indent=2, allow_nan=False))
            return 0
        config = load_config(arguments.config)
        if arguments.command == "plan":
            trajectory = plan_trajectory(config)
            destination = export_plan(trajectory, config, arguments.output_dir)
        else:
            from .acquisition import run_simulation
            if config.simulation is None or config.acquisition is None:
                raise ValueError("simulate requiere las secciones acquisition y simulation completas.")
            overrides = {}
            for argument, field in (
                ("fail_motion_at_visit", "motion_fault_at_visit"), ("motion_fault", "motion_fault_mode"),
                ("fail_sensor_at_read", "sensor_fault_at_read"), ("sensor_fault", "sensor_fault_mode"),
            ):
                value = getattr(arguments, argument)
                if value is not None:
                    overrides[field] = value
            if arguments.sensor_fault_once:
                overrides["sensor_fault_persistent"] = False
            config = replace(config, simulation=replace(config.simulation, **overrides))
            if arguments.motion_fault is not None and config.simulation.motion_fault_at_visit is None:
                raise ValueError("--motion-fault requiere un índice de fallo de movimiento.")
            if (arguments.sensor_fault is not None or arguments.sensor_fault_once) and config.simulation.sensor_fault_at_read is None:
                raise ValueError("Las opciones de fallo del sensor requieren un índice de lectura.")
            result = run_simulation(config, arguments.output_dir)
            summary = result.summary
            print(
                f"Adquisición simulada {summary['status']}: "
                f"{summary['grid_points_completed']}/{summary['grid_points']} puntos, "
                f"{summary['reference_visits_completed']}/{summary['reference_visits_expected']} referencias, "
                f"{summary['used_samples']} muestras utilizadas."
            )
            print(f"Duración virtual: {summary['virtual_duration_s']:.3f} s; ejecución: {summary['wall_duration_s']:.3f} s.")
            print(f"Resultados: {result.directory.resolve()}")
            if summary["error"]:
                print(f"Error: {summary['error']}", file=sys.stderr)
            return {"complete": 0, "failed": 1, "cancelled": 130}[summary["status"]]
    except AcquisitionError as exc:
        print(f"Error de comunicación con M1 Pro: {exc}", file=sys.stderr)
        return 1
    except (OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    print(f"Plan simulado: {len(trajectory.points)} puntos, {trajectory.layer_count} capas, paso {trajectory.step_mm:g} mm.")
    print(f"Longitud del recorrido de malla: {trajectory.length_mm:.2f} mm (sin referencias ni entrada/salida).")
    print(f"Resultados: {destination.resolve()}")
    return 0
