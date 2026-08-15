from math import isclose, pi

import pytest

from aircraft_navigation.aircraft_frame_publisher import make_aircraft_transform


def test_aircraft_pose_has_expected_axes_and_translation():
    transform = make_aircraft_transform('odom', 'aircraft', 0.0, 12.0, 0.0, pi)
    assert transform.header.frame_id == 'odom'
    assert transform.child_frame_id == 'aircraft'
    assert transform.transform.translation.y == 12.0
    assert isclose(transform.transform.rotation.z, 1.0)
    assert isclose(transform.transform.rotation.w, 0.0, abs_tol=1e-12)


def test_pose_can_be_changed_without_code_changes():
    transform = make_aircraft_transform('map', 'aircraft', 3.5, -2.0, 0.4, 0.25)
    assert transform.header.frame_id == 'map'
    assert transform.transform.translation.x == 3.5
    assert transform.transform.translation.y == -2.0
    assert transform.transform.translation.z == 0.4


@pytest.mark.parametrize('parent, child', [('', 'aircraft'), ('odom', ''), ('odom', 'odom')])
def test_frame_names_must_be_valid(parent, child):
    with pytest.raises(ValueError):
        make_aircraft_transform(parent, child, 0.0, 0.0, 0.0, 0.0)
