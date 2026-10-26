"""Mapas discretos, perfiles centrales y referencias de adquisiciones guardadas."""

import csv
import math
from pathlib import Path


def _number(row: dict, key: str) -> float | None:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _valid_rows(directory: Path) -> list[dict]:
    path = directory / "measurements.csv"
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as handle:
        return [row for row in csv.DictReader(handle)
                if row.get("status") == "ok"
                and (_number(row, "valid_samples") or 0) > 0
                and _number(row, "B_n_mean_T") is not None]


def _empty(axis, message: str) -> None:
    axis.text(0.5, 0.5, message, ha="center", va="center", transform=axis.transAxes, wrap=True)


def _central(row: dict, axis: str) -> bool:
    value = _number(row, f"{axis}_target_magnet_mm")
    return value is not None and abs(value) < 1e-9


def save_field_figures(directory: Path) -> list[str]:
    """Dibuja solo mediciones válidas, sin interpolar ni completar datos ausentes.

    Las coordenadas de los cortes son objetivos del marco del imán; las poses
    reportadas permanecen como evidencia separada en los CSV. Una fila válida
    sin desviación estándar se dibuja como punto, sin barra de error inventada.
    """
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.colors import Normalize

    directory = Path(directory)
    rows = _valid_rows(directory)
    grid = [row for row in rows if row.get("kind") == "grid"
            and all(_number(row, f"{axis}_target_magnet_mm") is not None for axis in "xyz")]
    references = [row for row in rows if row.get("kind") == "reference"
                  and _number(row, "finished_time_s") is not None]
    names = []

    def save(figure, filename):
        FigureCanvasAgg(figure)
        try:
            figure.savefig(directory / filename, dpi=150, bbox_inches="tight")
            names.append(filename)
        finally:
            figure.clear()

    maps = Figure(figsize=(15, 5), layout="constrained")
    color_values = [_number(row, "B_n_mean_T") for row in grid]
    normalization = Normalize(vmin=min(color_values), vmax=max(color_values)) if color_values else None
    for panel, (fixed, first, second) in enumerate((("x", "y", "z"), ("y", "x", "z"), ("z", "x", "y")), 1):
        axis = maps.add_subplot(1, 3, panel)
        selected = [row for row in grid if _central(row, fixed)]
        axis.set_title(f"Corte {fixed} = 0 mm")
        axis.set_xlabel(f"{first} objetivo en imán [mm]")
        axis.set_ylabel(f"{second} objetivo en imán [mm]")
        axis.set_aspect("equal", adjustable="datalim")
        if selected:
            points = axis.scatter([_number(row, f"{first}_target_magnet_mm") for row in selected],
                                  [_number(row, f"{second}_target_magnet_mm") for row in selected],
                                  c=[_number(row, "B_n_mean_T") for row in selected],
                                  norm=normalization, cmap="viridis", marker="s", s=45)
            maps.colorbar(points, ax=axis, label="B_n medio [T]", shrink=0.75)
        else:
            _empty(axis, "Sin mediciones válidas en este corte")
    maps.suptitle("CAMPO SIMULADO · muestras discretas, sin interpolación")
    save(maps, "field_maps.png")

    profiles = Figure(figsize=(15, 5), layout="constrained")
    for panel, coordinate in enumerate("xyz", 1):
        axis = profiles.add_subplot(1, 3, panel)
        selected = sorted([row for row in grid if all(_central(row, other) for other in "xyz" if other != coordinate)],
                          key=lambda row: _number(row, f"{coordinate}_target_magnet_mm"))
        axis.set_title(f"Perfil por el centro: {coordinate}")
        axis.set_xlabel(f"{coordinate} objetivo en imán [mm]")
        axis.set_ylabel("B_n medio [T]")
        axis.grid(alpha=0.25)
        if selected:
            axis.plot([_number(row, f"{coordinate}_target_magnet_mm") for row in selected],
                      [_number(row, "B_n_mean_T") for row in selected], "o", markersize=4,
                      label="Media válida")
            with_std = [row for row in selected if (_number(row, "valid_samples") or 0) > 1
                        and _number(row, "B_n_std_T") is not None and _number(row, "B_n_std_T") >= 0]
            if with_std:
                axis.errorbar([_number(row, f"{coordinate}_target_magnet_mm") for row in with_std],
                              [_number(row, "B_n_mean_T") for row in with_std],
                              yerr=[_number(row, "B_n_std_T") for row in with_std],
                              fmt="none", capsize=3, label="Desviación muestral (ddof=1)")
            axis.legend(fontsize=8)
        else:
            _empty(axis, "Sin mediciones válidas sobre esta línea")
    profiles.suptitle("PERFILES SIMULADOS · sin interpolación ni incertidumbre imputada")
    save(profiles, "field_profiles.png")

    drift = Figure(figsize=(9, 5), layout="constrained")
    axis = drift.add_subplot(111)
    axis.set_title("REFERENCIAS SIMULADAS · centro de la ROI")
    axis.set_xlabel("Tiempo virtual al terminar la lectura [s]")
    axis.set_ylabel("B_n medio [T]")
    axis.grid(alpha=0.25)
    if references:
        references.sort(key=lambda row: _number(row, "finished_time_s"))
        axis.plot([_number(row, "finished_time_s") for row in references],
                  [_number(row, "B_n_mean_T") for row in references], "o-", label="Referencia central")
        with_std = [row for row in references if (_number(row, "valid_samples") or 0) > 1
                    and _number(row, "B_n_std_T") is not None and _number(row, "B_n_std_T") >= 0]
        if with_std:
            axis.errorbar([_number(row, "finished_time_s") for row in with_std],
                          [_number(row, "B_n_mean_T") for row in with_std],
                          yerr=[_number(row, "B_n_std_T") for row in with_std], fmt="none", capsize=3,
                          label="Desviación muestral (ddof=1)")
        axis.legend()
    else:
        _empty(axis, "Sin referencias centrales válidas con tiempo disponible")
    save(drift, "reference_drift.png")
    return names
