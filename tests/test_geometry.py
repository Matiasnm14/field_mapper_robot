import numpy as np
import pytest

from field_mapper.geometry import Bore, RigidTransform, Sphere, ToolTransform, grid_extent


def test_roi_and_bore_have_distinct_boundaries():
    sphere = Sphere(30)
    bore = Bore(50, 254)
    points = [[0, 0, 0], [30, 0, 0], [0, 0, -30], [30, 30, 0], [50, 0, 127], [0, 0, 127.001]]
    np.testing.assert_array_equal(sphere.contains(points), [True, True, True, False, False, False])
    np.testing.assert_array_equal(bore.contains(points), [True, True, True, True, True, False])
    assert bore.contains_sphere(sphere)
    assert not Bore(29, 254).contains_sphere(sphere)
    assert not Bore(50, 59).contains_sphere(sphere)


def test_decimal_grid_boundary_is_not_lost_to_roundoff():
    assert grid_extent(0.3, 0.1) == 3
    assert Sphere(0.3).contains([3 * 0.1, 0, 0])
    assert not Sphere(30).contains([30.000001, 0, 0])


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), -float("inf"), True, "30", None])
def test_invalid_dimensions(value):
    with pytest.raises(ValueError):
        Sphere(value)
    with pytest.raises(ValueError):
        Bore(50, value)
    with pytest.raises(ValueError):
        grid_extent(30, value)


@pytest.mark.parametrize("step", [7, 60, 5.000000000001, 0.001, 1e-320])
def test_non_integral_or_excessive_grids_fail_explicitly(step):
    with pytest.raises(ValueError):
        grid_extent(30, step)


@pytest.mark.parametrize("positions", [[1, 2], [1, 2, 3, 4], [1, 2, float("nan")], 0])
def test_invalid_positions_are_rejected(positions):
    with pytest.raises(ValueError):
        Sphere(30).contains(positions)


def test_rigid_transform_against_known_coordinates_and_matrix():
    rotation = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    transform = RigidTransform(rotation, [100, 200, 300])
    np.testing.assert_allclose(transform.apply([1, 2, 3]), [98, 201, 303])
    points = [[1, 2, 3], [-20, 10, 5]]
    np.testing.assert_allclose(transform.inverse_apply(transform.apply(points)), points)
    restored = RigidTransform.from_matrix(transform.as_matrix())
    np.testing.assert_allclose(restored.apply(points), transform.apply(points))


@pytest.mark.parametrize("pose_frame, expected", [("flange", [68, 181, 313]), ("tcp", [98, 201, 303])])
def test_tool_offset_with_nonzero_rotation_translation_and_extension(pose_frame, expected):
    transform = ToolTransform(
        RigidTransform([[0, -1, 0], [1, 0, 0], [0, 0, 1]], [100, 200, 300]),
        [[0, 0, 1], [0, 1, 0], [-1, 0, 0]],
        [10, 20, 30],
        pose_frame,
    )
    np.testing.assert_allclose(transform.magnet_to_robot([1, 2, 3]), expected)
    np.testing.assert_allclose(transform.robot_to_magnet(expected), [1, 2, 3])
    batch = [[1, 2, 3], [-5, -10, 0], [0, 0, 0]]
    np.testing.assert_allclose(transform.robot_to_magnet(transform.magnet_to_robot(batch)), batch)


@pytest.mark.parametrize("rotation", [np.diag([-1, 1, 1]), np.eye(3) * 2, np.zeros((3, 3)), np.eye(2), np.full((3, 3), np.nan)])
def test_non_rigid_transforms_are_rejected(rotation):
    with pytest.raises(ValueError):
        RigidTransform(rotation, [0, 0, 0])


def test_invalid_homogeneous_matrix_offset_and_frame():
    matrix = np.eye(4)
    matrix[3, 0] = 1
    with pytest.raises(ValueError):
        RigidTransform.from_matrix(matrix)
    with pytest.raises(ValueError):
        RigidTransform(np.eye(3), [0, 0, np.inf])
    with pytest.raises(ValueError):
        ToolTransform(RigidTransform(np.eye(3), [0, 0, 0]), np.eye(3), [0, 0, 0], "unknown")


def test_transform_copies_input_arrays():
    rotation = np.eye(3)
    translation = np.array([1.0, 2.0, 3.0])
    transform = RigidTransform(rotation, translation)
    rotation[:] = 0
    translation[:] = 0
    np.testing.assert_allclose(transform.apply([0, 0, 0]), [1, 2, 3])
