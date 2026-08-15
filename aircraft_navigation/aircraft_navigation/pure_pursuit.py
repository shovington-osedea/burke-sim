"""Pure-Python closed-path pure-pursuit controller."""

from dataclasses import dataclass
import math
from typing import Sequence

from aircraft_navigation.path_geometry import Point, segment_length


@dataclass(frozen=True)
class ControllerConfig:
    """Validated pure-pursuit and safety limits."""

    lookahead_distance_m: float = 1.0
    nominal_linear_speed_mps: float = 0.3
    maximum_linear_speed_mps: float = 0.5
    minimum_linear_speed_mps: float = 0.05
    maximum_angular_speed_radps: float = 0.5
    maximum_linear_acceleration_mps2: float = 0.4
    maximum_angular_acceleration_radps2: float = 0.8
    completion_tolerance_m: float = 0.2
    forward_search_window_points: int = 8

    def __post_init__(self) -> None:
        positive = ('lookahead_distance_m', 'nominal_linear_speed_mps',
                    'maximum_linear_speed_mps',
                    'maximum_angular_speed_radps',
                    'maximum_linear_acceleration_mps2',
                    'maximum_angular_acceleration_radps2')
        nonnegative = ('minimum_linear_speed_mps', 'completion_tolerance_m')
        for name in positive + nonnegative:
            value = float(getattr(self, name))
            if not math.isfinite(value) or (value <= 0.0 if name in positive else value < 0.0):
                raise ValueError(f'{name} must be finite and valid')
        if (self.minimum_linear_speed_mps > self.nominal_linear_speed_mps
                or self.nominal_linear_speed_mps > self.maximum_linear_speed_mps):
            raise ValueError('linear speed limits are inconsistent')
        if (not isinstance(self.forward_search_window_points, int)
                or self.forward_search_window_points < 1):
            raise ValueError('forward_search_window_points must be positive')


@dataclass(frozen=True)
class ControlResult:
    """One deterministic controller result and its debug values."""

    linear_x: float
    angular_z: float
    closest_point: Point
    lookahead_point: Point
    cross_track_error_m: float
    progress_fraction: float
    completed: bool


class ClosedPath:
    """Segment and cumulative-length representation of a closed path."""

    def __init__(self, points: Sequence[Point]) -> None:
        if len(points) < 3 or any(not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('path must contain at least three finite points')
        self.points = tuple((float(p[0]), float(p[1])) for p in points)
        self.lengths = tuple(
            segment_length(self.points[i], self.points[(i + 1) % len(self.points)])
            for i in range(len(self.points)))
        if any(length <= 1e-9 for length in self.lengths):
            raise ValueError('path contains a degenerate segment')
        self.cumulative = (0.0,)
        for length in self.lengths:
            self.cumulative += (self.cumulative[-1] + length,)
        self.total_length = self.cumulative[-1]

    def point_at(self, progress: float) -> Point:
        """Return the point at wrapped arc-length progress."""
        distance = progress % self.total_length
        for index, length in enumerate(self.lengths):
            if distance <= self.cumulative[index + 1] or index == len(self.lengths) - 1:
                fraction = (distance - self.cumulative[index]) / length
                start, end = self.points[index], self.points[(index + 1) % len(self.points)]
                return (start[0] + fraction * (end[0] - start[0]),
                        start[1] + fraction * (end[1] - start[1]))
        return self.points[0]

    def project(self, point: Point, start_progress: float | None,
                window_points: int) -> tuple[Point, float, float]:
        """Project onto a bounded forward segment window."""
        if start_progress is None:
            indices = range(len(self.points))
            base = 0.0
        else:
            base = start_progress
            start_index = self._segment_index(start_progress)
            indices = ((start_index + offset) % len(self.points)
                       for offset in range(window_points + 1))
        candidates = []
        for index in indices:
            start, end = self.points[index], self.points[(index + 1) % len(self.points)]
            dx, dy = end[0] - start[0], end[1] - start[1]
            denominator = dx * dx + dy * dy
            fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx
                                          + (point[1] - start[1]) * dy) / denominator))
            closest = (start[0] + fraction * dx, start[1] + fraction * dy)
            progress = self.cumulative[index] + fraction * self.lengths[index]
            if start_progress is not None:
                # Express wrapped progress on the lap nearest the previous
                # projection.  Merely forcing every smaller value onto the
                # next lap turns harmless odometry noise into a complete lap,
                # especially when the robot starts at the path seam.
                progress += round(
                    (base - progress) / self.total_length) * self.total_length
            candidates.append((math.dist(point, closest), closest, progress))
        return min(candidates, key=lambda value: value[0])

    def _segment_index(self, progress: float) -> int:
        wrapped = progress % self.total_length
        for index in range(len(self.lengths)):
            if wrapped < self.cumulative[index + 1]:
                return index
        return len(self.lengths) - 1


class PurePursuitController:
    """Stateful, side-effect-free controller for one closed path."""

    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self.path: ClosedPath | None = None
        self.progress: float | None = None
        self.start_progress: float | None = None
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.completed = False

    def set_path(self, points: Sequence[Point], preserve_state: bool = False) -> None:
        new_path = ClosedPath(points)
        if not preserve_state:
            self.progress = None
            self.start_progress = None
            self.previous_linear = 0.0
            self.previous_angular = 0.0
            self.completed = False
        self.path = new_path

    def reset(self) -> None:
        self.progress = None
        self.start_progress = None
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.completed = False

    def step(self, robot_pose: tuple[float, float, float], dt: float) -> ControlResult:
        """Compute a bounded command from an odom-frame robot pose."""
        zero = (0.0, 0.0)
        if (self.path is None or not math.isfinite(dt) or dt <= 0.0
                or not all(math.isfinite(value) for value in robot_pose)
                or self.completed):
            return ControlResult(0.0, 0.0, zero, zero, 0.0,
                                 1.0 if self.completed else 0.0, self.completed)
        point = robot_pose[:2]
        distance, closest, projected = self.path.project(
            point, self.progress, self.config.forward_search_window_points)
        initializing = self.progress is None
        if initializing:
            self.progress = projected
            self.start_progress = projected
        else:
            self.progress = max(self.progress, projected)
        travelled = self.progress - self.start_progress
        fraction = min(1.0, max(0.0, travelled / self.path.total_length))
        if (not initializing
                and travelled + self.config.completion_tolerance_m >= self.path.total_length):
            self.completed = True
            return ControlResult(0.0, 0.0, closest, closest, distance, 1.0, True)

        lookahead = self.path.point_at(self.progress + self.config.lookahead_distance_m)
        dx = lookahead[0] - robot_pose[0]
        dy = lookahead[1] - robot_pose[1]
        cosine, sine = math.cos(robot_pose[2]), math.sin(robot_pose[2])
        local_x, local_y = cosine * dx + sine * dy, -sine * dx + cosine * dy
        squared_distance = local_x * local_x + local_y * local_y
        if squared_distance <= 1e-12:
            return ControlResult(0.0, 0.0, closest, lookahead, distance, fraction, False)
        curvature = 2.0 * local_y / squared_distance
        speed = self.config.nominal_linear_speed_mps / (1.0 + abs(curvature))
        speed = max(self.config.minimum_linear_speed_mps,
                    min(self.config.maximum_linear_speed_mps, speed))
        angular = max(-self.config.maximum_angular_speed_radps,
                      min(self.config.maximum_angular_speed_radps, speed * curvature))
        speed = _rate_limit(speed, self.previous_linear,
                            self.config.maximum_linear_acceleration_mps2 * dt)
        angular = _rate_limit(angular, self.previous_angular,
                              self.config.maximum_angular_acceleration_radps2 * dt)
        self.previous_linear, self.previous_angular = speed, angular
        return ControlResult(speed, angular, closest, lookahead, distance, fraction, False)


def _rate_limit(value: float, previous: float, maximum_delta: float) -> float:
    """Limit a scalar change to one control step."""
    return previous + max(-maximum_delta, min(maximum_delta, value - previous))
