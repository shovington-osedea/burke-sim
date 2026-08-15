import math

import pytest

from aircraft_navigation.pure_pursuit import ControllerConfig, PurePursuitController


def controller(**overrides):
    values = dict(lookahead_distance_m=0.5, nominal_linear_speed_mps=0.3,
                  maximum_linear_speed_mps=0.5, minimum_linear_speed_mps=0.05,
                  maximum_angular_speed_radps=0.5,
                  maximum_linear_acceleration_mps2=0.4,
                  maximum_angular_acceleration_radps2=0.8,
                  completion_tolerance_m=0.0, forward_search_window_points=8)
    values.update(overrides)
    return PurePursuitController(ControllerConfig(**values))


def test_straight_segment_drives_forward_and_only_planar_fields_are_needed():
    node = controller()
    node.set_path([(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)])
    result = node.step((1.0, 0.0, 0.0), 0.1)
    assert result.linear_x > 0.0
    assert result.angular_z == 0.0


@pytest.mark.parametrize('y, expected_sign', [(0.5, -1), (-0.5, 1)])
def test_curve_turn_sign(y, expected_sign):
    node = controller()
    node.set_path([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)])
    result = node.step((1.0, y, 0.0), 0.1)
    assert math.copysign(1.0, result.angular_z) == expected_sign


def test_acceleration_and_velocity_limits_are_enforced():
    node = controller(maximum_linear_acceleration_mps2=0.2,
                      maximum_angular_acceleration_radps2=0.3)
    node.set_path([(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)])
    first = node.step((1.0, 0.0, 0.0), 0.1)
    second = node.step((1.1, 0.0, 0.0), 0.1)
    assert first.linear_x <= 0.02 + 1e-9
    assert abs(second.linear_x - first.linear_x) <= 0.02 + 1e-9
    assert abs(second.angular_z - first.angular_z) <= 0.03 + 1e-9
    assert abs(second.angular_z) <= 0.5


def test_nearby_segment_does_not_cause_backward_progress():
    node = controller()
    node.set_path([(0.0, 0.0), (5.0, 0.0), (5.0, 0.2), (0.0, 0.2)])
    node.step((4.5, 0.0, 0.0), 0.1)
    before = node.progress
    node.step((4.5, 0.19, 0.0), 0.1)
    assert node.progress >= before


def test_backward_noise_at_loop_seam_does_not_complete_the_path():
    node = controller()
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, -2.0), (0.0, -2.0)]
    node.set_path(path)

    # These poses are both just before P0 on the closing segment.  The second
    # projection moves slightly backward, as stationary wheel odometry can.
    first = node.step((0.0, -1e-6, 0.0), 0.1)
    second = node.step((0.0, -2e-6, 0.0), 0.1)

    assert first.progress_fraction == 0.0
    assert second.progress_fraction == 0.0
    assert not second.completed
    assert second.linear_x > 0.0


def test_one_loop_completes_and_stays_stopped():
    node = controller(completion_tolerance_m=0.05)
    node.set_path([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    node.step((0.0, 0.0, 0.0), 0.1)
    node.progress = node.start_progress + node.path.total_length
    result = node.step((0.0, 0.0, 0.0), 0.1)
    assert result.completed
    assert result.linear_x == 0.0
    assert node.step((0.5, 0.5, 0.0), 0.1).linear_x == 0.0


@pytest.mark.parametrize('field', ['lookahead_distance_m', 'maximum_linear_speed_mps'])
def test_invalid_configuration_is_rejected(field):
    values = {field: float('nan')}
    with pytest.raises(ValueError):
        ControllerConfig(**values)
