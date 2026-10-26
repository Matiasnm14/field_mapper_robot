"""Las simulaciones confirman llegada y frescura, no solo aceptación."""

import math

import pytest

from field_mapper.clock import VirtualClock
from field_mapper.errors import ConnectionLost, MotionTimeout, SensorError, SensorTimeout
from field_mapper.robot import RobotPose, SimulatedRobot
from field_mapper.sensor import SimulatedSensor, normalize_value_to_tesla


def arrival(robot, target=(10.0, 0.0, 0.0), **overrides):
    kwargs = dict(target_position_mm=target, angle_deg=0.0, timeout_s=2.0,
                  position_tolerance_mm=0.01, angle_tolerance_deg=0.01, stable_time_s=0.02)
    kwargs.update(overrides)
    return robot.wait_until_arrived(**kwargs)


def robot_fixture(**kwargs):
    clock = VirtualClock()
    robot = SimulatedRobot(clock, initial_position_mm=(0, 0, 0), **kwargs)
    robot.connect()
    return clock, robot


def sensor_fixture(**kwargs):
    clock = VirtualClock()
    sensor = SimulatedSensor(clock, position_provider=lambda: (1, 2, 3), **kwargs)
    sensor.connect()
    return clock, sensor


def test_virtual_clock_never_waits_in_wall_time_and_is_monotonic():
    clock = VirtualClock(start_time_s=3)
    clock.sleep(3600)
    assert clock.now() == 3603
    for value in [-1, math.nan, math.inf]:
        with pytest.raises(ValueError):
            clock.sleep(value)
    assert clock.now() == 3603


def test_accepted_command_does_not_complete_motion():
    clock, robot = robot_fixture(speed_mm_s=10, move_latency_s=0.1)
    robot.move_to(position_mm=(10, 0, 0), angle_deg=0)
    assert robot.state() == "moving"
    assert robot.pose().position_mm == (0, 0, 0)
    clock.sleep(0.55)
    assert robot.pose().position_mm == pytest.approx((5, 0, 0))
    result = arrival(robot, stable_time_s=0.2)
    assert result.position_mm == (10, 0, 0)
    assert robot.state() == "idle"
    assert clock.now() >= 1.3
    assert robot.command_count == 1
    assert robot.move_history == [RobotPose((10, 0, 0), 0)]


@pytest.mark.parametrize("fault_mode", ["timeout", "out_of_tolerance", "accepted_only"])
def test_fault_prevents_stable_arrival_and_expires_at_deadline(fault_mode):
    clock, robot = robot_fixture(fault_at_move=1, fault_mode=fault_mode)
    robot.move_to((10, 0, 0), 0)
    with pytest.raises(MotionTimeout):
        arrival(robot, timeout_s=0.5)
    assert clock.now() == pytest.approx(0.5)
    robot.stop()
    assert robot.stop_count == 1


def test_accepted_only_pose_at_target_still_requires_completion():
    clock, robot = robot_fixture(fault_at_move=1, fault_mode="accepted_only")
    robot.move_to((10, 0, 0), 0)
    clock.sleep(1)
    assert robot.pose().position_mm == (10, 0, 0)
    assert robot.state() == "moving"
    with pytest.raises(MotionTimeout):
        arrival(robot, timeout_s=0.1)


def test_motion_stability_must_fit_timeout():
    clock, robot = robot_fixture()
    with pytest.raises(MotionTimeout):
        arrival(robot, target=(0, 0, 0), timeout_s=0.1, stable_time_s=0.2)
    assert clock.now() == pytest.approx(0.1)


def test_angle_and_position_both_checked():
    _, robot = robot_fixture()
    robot.move_to((10, 0, 0), 45)
    with pytest.raises(MotionTimeout):
        arrival(robot, angle_deg=0)
    assert arrival(robot, angle_deg=45).angle_deg == 45


def test_angles_are_compared_modulo_full_turn():
    _, robot = robot_fixture(angle_deg=359.999)
    assert arrival(robot, target=(0, 0, 0), angle_deg=0, angle_tolerance_deg=0.01).angle_deg == 359.999


def test_disconnect_does_not_home_and_stop_preserves_current_position():
    clock, robot = robot_fixture(speed_mm_s=10, move_latency_s=0)
    robot.move_to((10, 0, 0), 0)
    clock.sleep(0.5)
    robot.stop()
    assert robot.pose().position_mm == (5, 0, 0)
    robot.disconnect()
    with pytest.raises(ConnectionLost):
        robot.pose()
    robot.connect()
    assert robot.pose().position_mm == (5, 0, 0)
    assert robot.command_count == 1


def test_robot_connection_loss_does_not_report_a_valid_pose():
    _, robot = robot_fixture(fault_at_move=1, fault_mode="connection_loss")
    with pytest.raises(ConnectionLost):
        robot.move_to((1, 0, 0), 0)
    with pytest.raises(ConnectionLost):
        robot.pose()


def test_sensor_model_reproduces_gradient_quadratic_drift_and_unit():
    clock, sensor = sensor_fixture(noise_std_T=0, base_field_T=0.1,
                                  gradient_T_per_mm=(1e-4, 2e-4, 3e-4),
                                  quadratic_T_per_mm2=(1e-6, 2e-6, 3e-6),
                                  drift_T_per_s=0.01, original_unit="G")
    sample = sensor.read_sample(not_before_s=2, timeout_s=3)
    expected = 0.1 + 1e-4 + 4e-4 + 9e-4 + 1e-6 + 8e-6 + 27e-6 + 0.01 * 2.002
    assert normalize_value_to_tesla(sample.value, sample.unit) == pytest.approx(expected)
    assert sample.acquired_time_s == sample.received_time_s == clock.now()
    assert sample.acquired_time_s > 2
    assert sample.fresh and sample.status == "ok"
    assert sample.freshness_evidence == "simulated_request_sequence"


def test_sensor_sequences_and_times_strictly_increase():
    clock, sensor = sensor_fixture()
    samples = [sensor.read_sample(not_before_s=clock.now(), timeout_s=1) for _ in range(5)]
    assert [s.sequence_id for s in samples] == [1, 2, 3, 4, 5]
    assert all(a.acquired_time_s < b.acquired_time_s for a, b in zip(samples, samples[1:]))
    assert sensor.read_count == 5


def test_noise_is_reproducible_for_same_seed_and_positions():
    _, one = sensor_fixture(seed=42)
    _, two = sensor_fixture(seed=42)
    _, other = sensor_fixture(seed=43)
    def values(sensor):
        return [sensor.read_sample(not_before_s=0, timeout_s=1).value for _ in range(8)]
    first = values(one)
    assert first == values(two)
    assert first != values(other)


@pytest.mark.parametrize("not_before_s, timeout_s", [(0, 0.001), (5, 1)])
def test_sensor_timeout_includes_wait_for_fresh_sample(not_before_s, timeout_s):
    clock, sensor = sensor_fixture()
    with pytest.raises(SensorTimeout):
        sensor.read_sample(not_before_s=not_before_s, timeout_s=timeout_s)
    assert clock.now() == timeout_s


@pytest.mark.parametrize("fault_mode", ["timeout", "stale", "invalid", "unit", "connection_loss"])
def test_sensor_injected_failures_are_visible(fault_mode):
    clock, sensor = sensor_fixture(fault_at_read=2, fault_mode=fault_mode)
    first = sensor.read_sample(not_before_s=0, timeout_s=0.1)
    if fault_mode == "timeout":
        started = clock.now()
        with pytest.raises(SensorTimeout):
            sensor.read_sample(not_before_s=started, timeout_s=0.1)
        assert clock.now() == pytest.approx(started + 0.1)
    elif fault_mode == "connection_loss":
        with pytest.raises(ConnectionLost):
            sensor.read_sample(not_before_s=clock.now(), timeout_s=0.1)
        with pytest.raises(ConnectionLost):
            sensor.identity()
    else:
        sample = sensor.read_sample(not_before_s=clock.now(), timeout_s=0.1)
        if fault_mode == "stale":
            assert not sample.fresh
            assert sample.acquired_time_s == first.acquired_time_s
            assert sample.sequence_id == first.sequence_id
        else:
            with pytest.raises(SensorError):
                normalize_value_to_tesla(sample.value, sample.unit)


def test_persistent_sensor_failure_and_bounded_one_shot_fault():
    _, persistent = sensor_fixture(fault_at_read=1, fault_mode="timeout")
    for _ in range(3):
        with pytest.raises(SensorTimeout):
            persistent.read_sample(not_before_s=0, timeout_s=0.1)
    _, transient = sensor_fixture(fault_at_read=1, fault_mode="timeout", fault_persistent=False)
    with pytest.raises(SensorTimeout):
        transient.read_sample(not_before_s=0, timeout_s=0.1)
    assert transient.read_sample(not_before_s=0, timeout_s=0.1).fresh


@pytest.mark.parametrize("value,unit", [(None, "T"), (math.nan, "T"), (math.inf, "G"), (True, "T"), (1, "V"), (1, "gauss")])
def test_invalid_measurements_never_normalize_to_zero(value, unit):
    with pytest.raises(SensorError):
        normalize_value_to_tesla(value, unit)


def test_only_validated_units_are_normalized():
    assert normalize_value_to_tesla(2.5, "T") == 2.5
    assert normalize_value_to_tesla(-2500, "G") == -0.25


def test_simulated_identities_are_explicit_and_require_connection():
    _, robot = robot_fixture()
    _, sensor = sensor_fixture()
    for adapter in (robot, sensor):
        assert adapter.simulated is True
        assert adapter.identity()["simulated"] is True
        adapter.disconnect()
        with pytest.raises(ConnectionLost):
            adapter.identity()
