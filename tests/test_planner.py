from dataclasses import replace
from itertools import groupby

import numpy as np
import pytest

from field_mapper.planner import GridPoint, generate_grid, plan_trajectory, serpentine_order, validate_trajectory
from field_mapper.geometry import Sphere


@pytest.mark.parametrize("step, count", [(10, 123), (5, 925), (2.5, 7153)])
def test_complete_global_spherical_grid(config, step, count):
    profile = replace(config, geometry=replace(config.geometry, step_mm=step))
    trajectory = plan_trajectory(profile)
    points = trajectory.positions_mm
    coordinates = {tuple(point) for point in points}
    extent = round(30 / step)
    assert len(trajectory.points) == len(coordinates) == count
    assert len({point.point_id for point in trajectory.points}) == count
    assert trajectory.layer_count == 2 * extent + 1
    assert (0, 0, 0) in coordinates
    assert profile.geometry.sphere.contains(points).all()
    assert profile.geometry.bore.contains(points).all()
    np.testing.assert_allclose(points / step, np.round(points / step), atol=1e-12)
    for axis in range(3):
        for sign in (-1, 1):
            boundary = [0, 0, 0]
            boundary[axis] = sign * 30
            assert tuple(boundary) in coordinates
        reflected = points.copy()
        reflected[:, axis] *= -1
        assert {tuple(point) for point in reflected} == coordinates
    # Un punto dentro del cubo pero fuera de la esfera no pertenece a la malla.
    assert (30, 30, 30) not in coordinates


def test_serpentine_layers_rows_spacing_and_entry_choice(config):
    trajectory = plan_trajectory(config)
    layers = [(iz, list(points)) for iz, points in groupby(trajectory.points, key=lambda point: point.indices[2])]
    assert [iz for iz, _ in layers] == list(range(-6, 7))
    previous = None
    for _, layer in layers:
        rows = [(ix, list(points)) for ix, points in groupby(layer, key=lambda point: point.indices[0])]
        x_values = [ix for ix, _ in rows]
        assert x_values in (sorted(x_values), sorted(x_values, reverse=True))
        assert len(x_values) == len(set(x_values))
        previous_direction = None
        previous_row_index = None
        for row_index, (_, row) in enumerate(rows):
            y_values = [point.indices[1] for point in row]
            if len(row) > 1:
                differences = np.diff(y_values)
                assert np.all(differences == differences[0])
                assert abs(differences[0]) == 1
                if previous_direction is not None:
                    expected = previous_direction * (-1) ** (row_index - previous_row_index)
                    assert differences[0] == expected
                previous_direction = differences[0]
                previous_row_index = row_index
        if previous is not None:
            # Cualquier extremo de las dos filas externas puede iniciar esta capa.
            endpoint_candidates = [point for row in (rows[0][1], rows[-1][1]) for point in (row[0], row[-1])]
            chosen_distance = sum((a - b) ** 2 for a, b in zip(previous.indices, layer[0].indices))
            assert chosen_distance == min(
                sum((a - b) ** 2 for a, b in zip(previous.indices, point.indices))
                for point in endpoint_candidates
            )
        previous = layer[-1]


def test_ids_and_order_are_independent_of_input_order():
    grid = generate_grid(30, 5)
    original_ids = {point.indices: point.point_id for point in grid}
    ordered = serpentine_order(grid)
    assert ordered == serpentine_order(tuple(reversed(grid)))
    assert ordered == serpentine_order(grid)
    assert {point.indices: point.point_id for point in ordered} == original_ids


@pytest.mark.parametrize("radius", [0.3, 1.2, 30, 31.7])
def test_grid_scales_without_losing_boundary_points(radius):
    grid = generate_grid(radius, radius / 6)
    assert len(grid) == 925
    assert Sphere(radius).contains([point.position_mm for point in grid]).all()


def test_length_matches_euclidean_segments(config):
    trajectory = plan_trajectory(config)
    expected = sum(
        np.linalg.norm(np.subtract(a.position_mm, b.position_mm))
        for a, b in zip(trajectory.points, trajectory.points[1:])
    )
    assert trajectory.length_mm == pytest.approx(expected)


def test_incompatible_config_and_corrupted_trajectory_fail(config):
    trajectory = plan_trajectory(config)
    other = replace(config, geometry=replace(config.geometry, step_mm=10))
    with pytest.raises(ValueError, match="incompatibles"):
        validate_trajectory(trajectory, other)
    corruptions = [
        trajectory.points[:-1],
        (trajectory.points[0],) + trajectory.points[:-1],
        (GridPoint((100, 100, 100), 5),) + trajectory.points[1:],
        (replace(trajectory.points[0], step_mm=10),) + trajectory.points[1:],
    ]
    for points in corruptions:
        with pytest.raises(ValueError):
            validate_trajectory(replace(trajectory, points=points), config)
