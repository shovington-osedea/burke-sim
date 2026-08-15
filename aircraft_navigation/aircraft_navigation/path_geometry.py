"""Pure geometry and validation helpers for the fixed perimeter path."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


Point = tuple[float, float]


@dataclass(frozen=True)
class AxisAlignedBoundary:
    """A conservative projected aircraft collision envelope."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass(frozen=True)
class PolygonBoundary:
    """A top-down polygonal aircraft footprint."""

    points: tuple[Point, ...]


def signed_area(points: Sequence[Point]) -> float:
    """Return the signed area of a closed polygon."""
    return 0.5 * sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]))


def segment_length(start: Point, end: Point) -> float:
    """Return the Euclidean length of one segment."""
    return math.hypot(end[0] - start[0], end[1] - start[1])


def path_length(points: Sequence[Point]) -> float:
    """Return the closed-path arc length."""
    return sum(segment_length(start, end) for start, end in _closed_segments(points))


def point_to_boundary_distance(point: Point, boundary: AxisAlignedBoundary) -> float:
    """Return the distance from a point outside an axis-aligned rectangle."""
    dx = max(boundary.x_min - point[0], 0.0, point[0] - boundary.x_max)
    dy = max(boundary.y_min - point[1], 0.0, point[1] - boundary.y_max)
    return math.hypot(dx, dy)


def segment_to_boundary_distance(
        start: Point, end: Point, boundary: AxisAlignedBoundary) -> float:
    """Return the exact minimum distance between a segment and the rectangle."""
    rectangle = (
        (boundary.x_min, boundary.y_min),
        (boundary.x_max, boundary.y_min),
        (boundary.x_max, boundary.y_max),
        (boundary.x_min, boundary.y_max),
    )
    if _segment_intersects_rectangle(start, end, boundary):
        return 0.0
    distances = [
        point_to_boundary_distance(start, boundary),
        point_to_boundary_distance(end, boundary),
    ]
    distances.extend(
        _point_to_segment_distance(corner, start, end) for corner in rectangle)
    return min(distances)


def segment_to_polygon_distance(
        start: Point, end: Point, boundary: PolygonBoundary) -> float:
    """Return the minimum distance between a segment and polygon edges."""
    edges = list(_closed_segments(boundary.points))
    if any(_segments_intersect(start, end, edge_start, edge_end)
           for edge_start, edge_end in edges):
        return 0.0
    return min(
        _segment_to_segment_distance(start, end, edge_start, edge_end)
        for edge_start, edge_end in edges)


def point_to_polygon_distance(point: Point, boundary: PolygonBoundary) -> float:
    """Return the distance from a point to the polygon boundary."""
    return min(
        _point_to_segment_distance(point, edge_start, edge_end)
        for edge_start, edge_end in _closed_segments(boundary.points))


def _point_to_boundary_distance_for_boundary(
        point: Point, boundary: AxisAlignedBoundary | PolygonBoundary) -> float:
    """Return point distance for either supported boundary representation."""
    if isinstance(boundary, AxisAlignedBoundary):
        return point_to_boundary_distance(point, boundary)
    return point_to_polygon_distance(point, boundary)


def offset_polygon(points: Sequence[Point], offset_m: float) -> list[Point]:
    """Return the outward constant-distance offset of a clockwise polygon."""
    if len(points) < 3 or signed_area(points) >= 0.0:
        raise ValueError('footprint must contain a clockwise polygon')
    if offset_m <= 0.0 or not math.isfinite(offset_m):
        raise ValueError('offset must be a positive finite value')
    shifted_edges = []
    for start, end in _closed_segments(points):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length == 0.0:
            raise ValueError('footprint contains a zero-length edge')
        outward = (-dy / length * offset_m, dx / length * offset_m)
        shifted_edges.append((
            (start[0] + outward[0], start[1] + outward[1]),
            (end[0] + outward[0], end[1] + outward[1])))
    return [
        _line_intersection(shifted_edges[index - 1], shifted_edges[index])
        for index in range(len(points))]


def densify_polygon(points: Sequence[Point], maximum_segment_length: float) -> list[Point]:
    """Add points along polygon edges without changing its geometric outline."""
    if maximum_segment_length <= 0.0 or not math.isfinite(maximum_segment_length):
        raise ValueError('maximum segment length must be positive and finite')
    result = []
    for start, end in _closed_segments(points):
        count = max(1, math.ceil(segment_length(start, end) / maximum_segment_length))
        for index in range(count):
            fraction = index / count
            result.append((
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1])))
    return result


def rounded_offset_polygon(
        points: Sequence[Point], radius_m: float, maximum_segment_length: float) -> list[Point]:
    """Build a rounded outward offset with bounded pose spacing."""
    if len(points) < 3 or signed_area(points) >= 0.0:
        raise ValueError('footprint must contain a clockwise polygon')
    if radius_m <= 0.0 or not math.isfinite(radius_m):
        raise ValueError('radius must be positive and finite')
    if maximum_segment_length <= 0.0 or not math.isfinite(maximum_segment_length):
        raise ValueError('maximum segment length must be positive and finite')

    edge_normals = []
    for start, end in _closed_segments(points):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length == 0.0:
            raise ValueError('footprint contains a zero-length edge')
        edge_normals.append((-dy / length, dx / length))

    result = []
    for index, vertex in enumerate(points):
        previous_normal = edge_normals[index - 1]
        next_normal = edge_normals[index]
        start_angle = math.atan2(previous_normal[1], previous_normal[0])
        end_angle = math.atan2(next_normal[1], next_normal[0])
        clockwise_delta = (end_angle - start_angle) % (2.0 * math.pi)
        clockwise_delta -= 2.0 * math.pi
        arc_segments = max(1, math.ceil(
            abs(clockwise_delta) * radius_m / maximum_segment_length))
        angles = [
            start_angle + clockwise_delta * step / arc_segments
            for step in range(arc_segments + 1)]
        if index == 0 and start_angle > 0.0 > start_angle + clockwise_delta:
            angles.append(0.0)
            angles.sort(reverse=True)
        arc = [
            (vertex[0] + radius_m * math.cos(angle),
             vertex[1] + radius_m * math.sin(angle))
            for angle in angles]
        if index == 0:
            result.extend(arc)
        else:
            result.extend(arc[1:])

        next_vertex = points[(index + 1) % len(points)]
        edge_start = arc[-1]
        edge_end = (
            next_vertex[0] + radius_m * next_normal[0],
            next_vertex[1] + radius_m * next_normal[1])
        edge_segments = max(1, math.ceil(
            segment_length(edge_start, edge_end) / maximum_segment_length))
        edge_range = range(1, edge_segments + 1)
        if index == len(points) - 1:
            edge_range = range(1, edge_segments)
        result.extend(
            (edge_start[0] + (edge_end[0] - edge_start[0]) * step / edge_segments,
             edge_start[1] + (edge_end[1] - edge_start[1]) * step / edge_segments)
            for step in edge_range)

    nose_index = min(range(len(result)), key=lambda index: (
        -result[index][0], abs(result[index][1])))
    return result[nose_index:] + result[:nose_index]


def validate_path(
        points: Sequence[Point],
        boundary: AxisAlignedBoundary | PolygonBoundary,
        clearance_m: float,
        tolerance_m: float = 1e-6,
        maximum_clearance_m: float | None = None,
        maximum_segment_length_m: float | None = None) -> dict[str, float]:
    """Validate a finite, clockwise, closed path outside the boundary."""
    if not 3 <= len(points) <= 1000:
        raise ValueError('path must contain at least three points')
    if clearance_m < 0.0 or tolerance_m < 0.0:
        raise ValueError('clearance and tolerance must be non-negative')
    if any(not all(math.isfinite(value) for value in point) for point in points):
        raise ValueError('path points must be finite')
    segments = list(_closed_segments(points))
    if any(segment_length(start, end) <= tolerance_m for start, end in segments):
        raise ValueError('path contains a zero-length segment')
    area = signed_area(points)
    if area >= 0.0:
        raise ValueError('path must be clockwise in the aircraft frame')
    if isinstance(boundary, AxisAlignedBoundary):
        distance = segment_to_boundary_distance
    else:
        distance = segment_to_polygon_distance
    minimum_clearance = min(distance(start, end, boundary) for start, end in segments)
    point_distance = _point_to_boundary_distance_for_boundary
    maximum_clearance = max(
        point_distance(point, boundary) for point in points)
    if minimum_clearance + tolerance_m < clearance_m:
        raise ValueError(
            f'path clearance {minimum_clearance:.6f} m is below '
            f'{clearance_m:.6f} m')
    if (maximum_clearance_m is not None
            and maximum_clearance > maximum_clearance_m + tolerance_m):
        raise ValueError(
            f'path clearance {maximum_clearance:.6f} m exceeds '
            f'{maximum_clearance_m:.6f} m')
    maximum_spacing = max(segment_length(start, end) for start, end in segments)
    if (maximum_segment_length_m is not None
            and maximum_spacing > maximum_segment_length_m + tolerance_m):
        raise ValueError(
            f'path pose spacing {maximum_spacing:.6f} m exceeds '
            f'{maximum_segment_length_m:.6f} m')
    return {
        'waypoint_count': float(len(points)),
        'signed_area_m2': area,
        'total_length_m': path_length(points),
        'minimum_clearance_m': minimum_clearance,
        'maximum_clearance_m': maximum_clearance,
        'maximum_pose_spacing_m': maximum_spacing,
    }


def _closed_segments(points: Sequence[Point]) -> Iterable[tuple[Point, Point]]:
    """Yield every path segment, including the final-to-first segment."""
    return zip(points, points[1:] + points[:1])


def _point_to_segment_distance(point: Point, start: Point, end: Point) -> float:
    """Return the distance between a point and a finite segment."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.dist(point, start)
    projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    projection = min(1.0, max(0.0, projection))
    closest = (start[0] + projection * dx, start[1] + projection * dy)
    return math.dist(point, closest)


def _segment_to_segment_distance(
        start_a: Point, end_a: Point, start_b: Point, end_b: Point) -> float:
    """Return the distance between two non-intersecting segments."""
    return min(
        _point_to_segment_distance(start_a, start_b, end_b),
        _point_to_segment_distance(end_a, start_b, end_b),
        _point_to_segment_distance(start_b, start_a, end_a),
        _point_to_segment_distance(end_b, start_a, end_a))


def _segments_intersect(
        start_a: Point, end_a: Point, start_b: Point, end_b: Point) -> bool:
    """Return whether two closed segments intersect."""
    def orientation(a: Point, b: Point, c: Point) -> float:
        return ((b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0]))

    def on_segment(a: Point, b: Point, c: Point) -> bool:
        return (min(a[0], c[0]) <= b[0] <= max(a[0], c[0])
                and min(a[1], c[1]) <= b[1] <= max(a[1], c[1]))

    tolerance = 1e-12
    values = (
        orientation(start_a, end_a, start_b),
        orientation(start_a, end_a, end_b),
        orientation(start_b, end_b, start_a),
        orientation(start_b, end_b, end_a))
    if (values[0] * values[1] < -tolerance
            and values[2] * values[3] < -tolerance):
        return True
    return (
        (abs(values[0]) <= tolerance and on_segment(start_a, start_b, end_a))
        or (abs(values[1]) <= tolerance and on_segment(start_a, end_b, end_a))
        or (abs(values[2]) <= tolerance and on_segment(start_b, start_a, end_b))
        or (abs(values[3]) <= tolerance and on_segment(start_b, end_a, end_b)))


def _line_intersection(
        first: tuple[Point, Point], second: tuple[Point, Point]) -> Point:
    """Intersect two non-parallel infinite lines."""
    first_start, first_end = first
    second_start, second_end = second
    dx_a = first_end[0] - first_start[0]
    dy_a = first_end[1] - first_start[1]
    dx_b = second_end[0] - second_start[0]
    dy_b = second_end[1] - second_start[1]
    denominator = dx_a * dy_b - dy_a * dx_b
    if abs(denominator) < 1e-12:
        raise ValueError('footprint contains parallel adjacent edges')
    factor = ((second_start[0] - first_start[0]) * dy_b
              - (second_start[1] - first_start[1]) * dx_b) / denominator
    return (first_start[0] + factor * dx_a, first_start[1] + factor * dy_a)


def _segment_intersects_rectangle(
        start: Point, end: Point, boundary: AxisAlignedBoundary) -> bool:
    """Test segment intersection using the Liang-Barsky clipping method."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    lower, upper = 0.0, 1.0
    for coordinate, delta, minimum, maximum in (
            (start[0], dx, boundary.x_min, boundary.x_max),
            (start[1], dy, boundary.y_min, boundary.y_max)):
        if delta == 0.0:
            if coordinate < minimum or coordinate > maximum:
                return False
            continue
        entry = (minimum - coordinate) / delta
        leaving = (maximum - coordinate) / delta
        if entry > leaving:
            entry, leaving = leaving, entry
        lower = max(lower, entry)
        upper = min(upper, leaving)
        if lower > upper:
            return False
    return True
