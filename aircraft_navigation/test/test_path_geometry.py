from pathlib import Path
from math import isclose

import yaml

from aircraft_navigation.path_geometry import (
    PolygonBoundary, validate_path)
from aircraft_navigation.perimeter_path_publisher import planar_yaw


def _configured_path():
    config_file = Path(__file__).parents[1] / 'config' / 'perimeter_path.yaml'
    config = yaml.safe_load(config_file.read_text())['perimeter_path']
    footprint = tuple(tuple(point) for point in config['footprint'])
    boundary = PolygonBoundary(footprint)
    return footprint, boundary, config


def test_configured_path_is_clockwise_and_clear():
    footprint, boundary, config = _configured_path()
    points = [tuple(point) for point in config['waypoints']]
    metrics = validate_path(
        points,
        boundary,
        config['clearance_m'],
        maximum_clearance_m=config['maximum_clearance_m'],
        maximum_segment_length_m=config['maximum_pose_spacing_m'])
    assert len(points) > 10
    assert points[0][0] > footprint[0][0]
    assert abs(points[0][1]) < 1e-9
    assert metrics['signed_area_m2'] < 0.0
    assert metrics['minimum_clearance_m'] >= 1.0 - 1e-9
    assert metrics['total_length_m'] > 0.0


def test_path_closure_is_non_degenerate():
    footprint, boundary, config = _configured_path()
    points = [tuple(point) for point in config['waypoints']]
    assert points[-1] != points[0]
    validate_path(
        points,
        boundary,
        config['clearance_m'],
        maximum_clearance_m=config['maximum_clearance_m'],
        maximum_segment_length_m=config['maximum_pose_spacing_m'])


def test_waypoint_orientation_points_to_next_waypoint():
    assert isclose(planar_yaw((0.0, 0.0), (1.0, 0.0)), 0.0)
    assert isclose(planar_yaw((0.0, 0.0), (0.0, 1.0)), 1.5707963267948966)
