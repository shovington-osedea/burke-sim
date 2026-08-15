"""ROS adapter for the fail-closed pure-pursuit controller."""

import math
import time

from aircraft_navigation.pure_pursuit import ControllerConfig, PurePursuitController
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


class PurePursuitFollower(Node):
    """Follow the fixed path using only odometry and the declared TF frames."""

    def __init__(self) -> None:
        super().__init__('pure_pursuit_follower')
        self._declare_parameters()
        self._config = ControllerConfig(
            lookahead_distance_m=float(self.get_parameter('lookahead_distance_m').value),
            nominal_linear_speed_mps=float(self.get_parameter('nominal_linear_speed_mps').value),
            maximum_linear_speed_mps=float(self.get_parameter('maximum_linear_speed_mps').value),
            minimum_linear_speed_mps=float(self.get_parameter('minimum_linear_speed_mps').value),
            maximum_angular_speed_radps=float(
                self.get_parameter('maximum_angular_speed_radps').value),
            maximum_linear_acceleration_mps2=float(
                self.get_parameter('maximum_linear_acceleration_mps2').value),
            maximum_angular_acceleration_radps2=float(
                self.get_parameter('maximum_angular_acceleration_radps2').value),
            completion_tolerance_m=float(self.get_parameter('completion_tolerance_m').value),
            forward_search_window_points=int(
                self.get_parameter('forward_search_window_points').value))
        self._control_rate_hz = self._finite_positive('control_rate_hz')
        self._odom_timeout_s = self._finite_positive('odom_timeout_s')
        self._tf_timeout_s = self._finite_positive('tf_timeout_s')
        self._enabled = bool(self.get_parameter('enabled').value)
        self._controller = PurePursuitController(self._config)
        self._path_points = None
        self._path_frame = ''
        self._path_identity = None
        self._active_path_identity = None
        self._last_odom_time = None
        self._last_tick = self.get_clock().now()
        self._last_nonzero = False
        self._last_command = (0.0, 0.0)
        self._last_progress = 0.0
        self._metrics = {
            'max_cross_track_error_m': 0.0, 'sum_cross_track_squared_m2': 0.0,
            'cross_track_samples': 0, 'max_heading_error_rad': 0.0,
            'progress_regressions': 0, 'max_linear_command_mps': 0.0,
            'max_angular_command_radps': 0.0, 'max_linear_acceleration_mps2': 0.0,
            'max_angular_acceleration_radps2': 0.0, 'cycles': 0}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._lookahead_pub = self.create_publisher(PointStamped, '/path_follower/lookahead', 10)
        self._closest_pub = self.create_publisher(PointStamped, '/path_follower/closest_point', 10)
        self._progress_pub = self.create_publisher(Float64, '/path_follower/progress', 10)
        self._error_pub = self.create_publisher(Float64, '/path_follower/cross_track_error', 10)
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(Path, '/aircraft_path', self._path_callback, path_qos)
        self.create_service(SetBool, '~/set_enabled', self._set_enabled)
        self._timer = self.create_timer(1.0 / self._control_rate_hz, self._control_tick)
        self._started_at = time.monotonic()
        if not self._enabled:
            self.get_logger().info(
                'Follower is waiting for /pure_pursuit_follower/set_enabled')

    def _declare_parameters(self) -> None:
        defaults = {
            'control_rate_hz': 20.0, 'lookahead_distance_m': 1.0,
            'nominal_linear_speed_mps': 0.3, 'maximum_linear_speed_mps': 0.5,
            'minimum_linear_speed_mps': 0.05, 'maximum_angular_speed_radps': 0.5,
            'maximum_linear_acceleration_mps2': 0.4,
            'maximum_angular_acceleration_radps2': 0.8,
            'completion_tolerance_m': 0.2, 'forward_search_window_points': 8,
            'odom_timeout_s': 0.5, 'tf_timeout_s': 0.2, 'enabled': True}
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _finite_positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _odom_callback(self, message: Odometry) -> None:
        self._last_odom_time = self._message_time(message.header.stamp)

    def _path_callback(self, message: Path) -> None:
        try:
            points = tuple((float(p.pose.position.x), float(p.pose.position.y))
                           for p in message.poses)
            identity = (message.header.frame_id, points)
            if not message.header.frame_id or len(points) < 3:
                raise ValueError('path frame and at least three poses are required')
            if any(not all(math.isfinite(value) for value in point) for point in points):
                raise ValueError('path points must be finite')
            if identity != self._path_identity:
                self._controller.reset()
                self._active_path_identity = None
            self._path_points = points
            self._path_frame = message.header.frame_id
            self._path_identity = identity
        except (TypeError, ValueError) as error:
            self._controller.reset()
            self._path_points = None
            self.get_logger().error(f'Rejecting aircraft path: {error}')

    def _set_enabled(self, request: SetBool.Request,
                     response: SetBool.Response) -> SetBool.Response:
        requested = bool(request.data)
        if requested != self._enabled:
            self._controller.reset()
            self._active_path_identity = None
            self._last_tick = self.get_clock().now()
            self._last_command = (0.0, 0.0)
        self._enabled = requested
        if not self._enabled:
            self._publish_zero()
        response.success = True
        response.message = 'path follower enabled' if requested else 'path follower disabled'
        self.get_logger().info(response.message)
        return response

    def _control_tick(self) -> None:
        now = self.get_clock().now()
        dt = max(1e-6, (now - self._last_tick).nanoseconds * 1e-9)
        self._last_tick = now
        if (not self._enabled or self._path_points is None or self._last_odom_time is None
                or now.nanoseconds * 1e-9 - self._last_odom_time > self._odom_timeout_s
                or self._unexpected_cmd_vel_publisher()):
            self._publish_zero()
            return
        try:
            timeout = Duration(seconds=self._tf_timeout_s)
            aircraft_tf = self._tf_buffer.lookup_transform(
                'odom', self._path_frame, Time(), timeout)
            robot_tf = self._tf_buffer.lookup_transform(
                'odom', 'base_footprint', Time(), timeout)
            aircraft_pose = _transform_pose(aircraft_tf)
            robot_pose = _transform_pose(robot_tf)
            odom_points = tuple(_transform_point(point, aircraft_pose)
                                for point in self._path_points)
            if self._active_path_identity != self._path_identity:
                self._controller.set_path(odom_points, preserve_state=False)
                self._active_path_identity = self._path_identity
            elif self._controller.path is None or odom_points != self._controller.path.points:
                self._controller.set_path(odom_points, preserve_state=True)
            result = self._controller.step(robot_pose, dt)
            self._publish_result(result, now)
            self._record_metrics(result, robot_pose, dt)
        except (TransformException, ValueError, RuntimeError) as error:
            self.get_logger().warning(f'Controller stopped: {error}')
            self._publish_zero()
        except Exception as error:  # fail closed on unexpected controller errors
            self.get_logger().error(f'Controller exception; publishing zero: {error}')
            self._publish_zero()

    def _unexpected_cmd_vel_publisher(self) -> bool:
        try:
            publishers = self.get_publishers_info_by_topic('/cmd_vel')
            return any(info.node_name != self.get_name() for info in publishers)
        except AttributeError:
            return False

    def _publish_result(self, result, stamp: Time) -> None:
        command = Twist()
        command.linear.x, command.angular.z = result.linear_x, result.angular_z
        self._cmd_pub.publish(command)
        self._publish_point(self._lookahead_pub, result.lookahead_point, stamp)
        self._publish_point(self._closest_pub, result.closest_point, stamp)
        self._publish_scalar(self._progress_pub, result.progress_fraction)
        self._publish_scalar(self._error_pub, result.cross_track_error_m)
        self._last_nonzero = abs(result.linear_x) > 1e-9 or abs(result.angular_z) > 1e-9

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())
        self._last_nonzero = False

    def _publish_point(self, publisher, point, stamp: Time) -> None:
        message = PointStamped()
        message.header.frame_id = 'odom'
        message.header.stamp = stamp.to_msg()
        message.point.x, message.point.y = point
        publisher.publish(message)

    @staticmethod
    def _publish_scalar(publisher, value: float) -> None:
        message = Float64()
        message.data = float(value)
        publisher.publish(message)

    def _record_metrics(self, result, robot_pose, dt: float) -> None:
        metrics = self._metrics
        metrics['cycles'] += 1
        metrics['max_cross_track_error_m'] = max(
            metrics['max_cross_track_error_m'], result.cross_track_error_m)
        metrics['sum_cross_track_squared_m2'] += result.cross_track_error_m ** 2
        metrics['cross_track_samples'] += 1
        target_heading = math.atan2(
            result.lookahead_point[1] - robot_pose[1],
            result.lookahead_point[0] - robot_pose[0])
        heading = abs(math.atan2(math.sin(target_heading - robot_pose[2]),
                                 math.cos(target_heading - robot_pose[2])))
        metrics['max_heading_error_rad'] = max(metrics['max_heading_error_rad'], heading)
        metrics['max_linear_command_mps'] = max(
            metrics['max_linear_command_mps'], abs(result.linear_x))
        metrics['max_angular_command_radps'] = max(
            metrics['max_angular_command_radps'], abs(result.angular_z))
        if result.progress_fraction + 1e-9 < self._last_progress:
            metrics['progress_regressions'] += 1
        if self._last_nonzero:
            metrics['max_linear_acceleration_mps2'] = max(
                metrics['max_linear_acceleration_mps2'],
                abs(result.linear_x - self._last_command[0]) / dt)
            metrics['max_angular_acceleration_radps2'] = max(
                metrics['max_angular_acceleration_radps2'],
                abs(result.angular_z - self._last_command[1]) / dt)
        self._last_command = (result.linear_x, result.angular_z)
        self._last_progress = result.progress_fraction

    @staticmethod
    def _message_time(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def destroy_node(self):
        samples = self._metrics['cross_track_samples']
        rms = math.sqrt(self._metrics['sum_cross_track_squared_m2'] / samples) if samples else 0.0
        summary = dict(self._metrics, rms_cross_track_error_m=rms,
                       execution_time_s=time.monotonic() - self._started_at,
                       completed=self._controller.completed)
        self.get_logger().info(f'controller_summary={summary}')
        self._publish_zero()
        return super().destroy_node()


def _transform_pose(transform) -> tuple[float, float, float]:
    rotation = transform.transform.rotation
    yaw = math.atan2(2.0 * (rotation.w * rotation.z),
                     1.0 - 2.0 * rotation.z * rotation.z)
    translation = transform.transform.translation
    return translation.x, translation.y, yaw


def _transform_point(point, pose) -> tuple[float, float]:
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + cosine * point[0] - sine * point[1],
            pose[1] + sine * point[0] + cosine * point[1])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuitFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
