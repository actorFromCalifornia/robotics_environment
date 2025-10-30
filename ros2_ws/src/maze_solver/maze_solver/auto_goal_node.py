import math
from dataclasses import dataclass
from typing import Optional, Set

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class GoalCandidate:
    cell_index: int
    x: float
    y: float
    distance: float


class AutoGoalSetter(Node):
    """Automatically send Nav2 goals to the most distant explored cell the robot has not yet visited."""

    def __init__(self) -> None:
        super().__init__('auto_goal_setter')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('free_threshold', 25)
        self.declare_parameter('visited_radius', 0.4)
        self.declare_parameter('min_goal_distance', 1.0)
        self.declare_parameter('goal_timeout_sec', 30.0)

        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.goal_frame = self.get_parameter('goal_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.free_threshold = self.get_parameter('free_threshold').get_parameter_value().integer_value
        self.visited_radius = float(self.get_parameter('visited_radius').get_parameter_value().double_value)
        self.min_goal_distance = float(self.get_parameter('min_goal_distance').get_parameter_value().double_value)
        self.goal_timeout = Duration(seconds=float(
            self.get_parameter('goal_timeout_sec').get_parameter_value().double_value
        ))

        self.map: Optional[OccupancyGrid] = None
        self.visited_cells: Set[int] = set()
        self.failed_cells: Set[int] = set()
        self.current_goal: Optional[GoalCandidate] = None
        self.goal_sent_time: Optional[rclpy.time.Time] = None

        self._last_warn: dict[str, rclpy.time.Time] = {}

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, 5)

        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle_future = None
        self.result_future = None

        self._nav_server_ready = False

        self.timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"AutoGoalSetter listening on map='{self.map_topic}', frames: {self.goal_frame}->{self.robot_frame}"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    def _map_callback(self, msg: OccupancyGrid) -> None:
        if self.map is None:
            self.get_logger().info(
                f"Received map: size=({msg.info.width} x {msg.info.height}), resolution={msg.info.resolution:.3f}"
            )
        else:
            if msg.info.width != self.map.info.width or msg.info.height != self.map.info.height:
                self.get_logger().warn("Map size changed, clearing visited cells cache")
                self.visited_cells.clear()

        self.map = msg

    def _tick(self) -> None:
        if self.map is None:
            return

        try:
            pose = self._lookup_robot_pose()
        except TransformException as exc:
            self._warn_throttled('tf_lookup_failed', f'TF lookup failed: {exc}', 5.0)
            return

        self._mark_visited(pose[0], pose[1])

        if not self._check_nav_server():
            return

        # Process possible async completions
        self._process_action_futures()

        if self.current_goal is not None:
            # Cancel goal if timeout exceeded
            if self.goal_sent_time is not None:
                elapsed = self.get_clock().now() - self.goal_sent_time
                if elapsed > self.goal_timeout:
                    if self.current_goal:
                        self.get_logger().warn(
                            f"Goal at ({self.current_goal.x:.2f}, {self.current_goal.y:.2f}) timed out, canceling"
                        )
                    self._cancel_active_goal(add_to_failed=True)
            return

        candidate = self._select_candidate(pose)
        if candidate is None:
            return

        if candidate.distance < self.min_goal_distance:
            self.visited_cells.add(candidate.cell_index)
            return

        self._send_goal(candidate)

    # ------------------------------------------------------------------
    # Pose utilities
    def _lookup_robot_pose(self) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            self.goal_frame,
            self.robot_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        trans = transform.transform.translation
        rot = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)
        return trans.x, trans.y, yaw

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    # ------------------------------------------------------------------
    # Visited tracking
    def _mark_visited(self, x: float, y: float) -> None:
        if self.map is None:
            return
        info = self.map.info
        resolution = info.resolution
        radius_cells = max(1, int(self.visited_radius / resolution))

        cell_x = int((x - info.origin.position.x) / resolution)
        cell_y = int((y - info.origin.position.y) / resolution)

        if cell_x < 0 or cell_y < 0 or cell_x >= info.width or cell_y >= info.height:
            return

        width = info.width
        for dx in range(-radius_cells, radius_cells + 1):
            cx = cell_x + dx
            if cx < 0 or cx >= width:
                continue
            for dy in range(-radius_cells, radius_cells + 1):
                cy = cell_y + dy
                if cy < 0 or cy >= info.height:
                    continue
                self.visited_cells.add(cy * width + cx)

    # ------------------------------------------------------------------
    # Goal search and send
    def _select_candidate(self, pose: tuple[float, float, float]) -> Optional[GoalCandidate]:
        if self.map is None:
            return None

        info = self.map.info
        data = self.map.data
        width = info.width
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution

        robot_x, robot_y, _ = pose
        best: Optional[GoalCandidate] = None

        for idx, value in enumerate(data):
            if idx in self.visited_cells or idx in self.failed_cells:
                continue
            if value < 0:
                continue
            if value > self.free_threshold:
                continue

            col = idx % width
            row = idx // width
            wx = origin_x + (col + 0.5) * resolution
            wy = origin_y + (row + 0.5) * resolution

            distance = math.hypot(wx - robot_x, wy - robot_y)
            if best is None or distance > best.distance:
                best = GoalCandidate(idx, wx, wy, distance)

        if best is None:
            self.get_logger().info_throttle(10.0, "Нет доступных неизведанных точек для постановки цели")
        return best

    def _send_goal(self, candidate: GoalCandidate) -> None:
        if not self._nav_server_ready and not self.goal_client.wait_for_server(timeout_sec=0.5):
            self._warn_throttled('action_server_unavailable', 'Навигационный action сервер недоступен', 5.0)
            return

        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = candidate.x
        pose.pose.position.y = candidate.y
        pose.pose.position.z = 0.0

        try:
            _, _, yaw = self._lookup_robot_pose()
        except TransformException:
            yaw = 0.0
        quat = self._quaternion_from_yaw(yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.goal_handle_future = self.goal_client.send_goal_async(goal_msg)
        self.goal_handle_future.add_done_callback(self._goal_response_callback)

        self.current_goal = candidate
        self.goal_sent_time = self.get_clock().now()
        self.visited_cells.add(candidate.cell_index)

        self.get_logger().info(
            f"Отправлена цель: ({candidate.x:.2f}, {candidate.y:.2f}), дистанция {candidate.distance:.2f} м"
        )

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Не удалось отправить цель: {exc}")
            if self.current_goal:
                self.failed_cells.add(self.current_goal.cell_index)
            self._reset_goal_state()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Навигационная цель отклонена")
            if self.current_goal:
                self.failed_cells.add(self.current_goal.cell_index)
            self._reset_goal_state()
            return

        self.get_logger().info("Цель принята Nav2")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        if future.cancelled():
            self.get_logger().info("Результат цели отменён")
            self._reset_goal_state()
            return

        exception = future.exception()
        if exception is not None:
            self.get_logger().error(f"Ошибка при получении результата цели: {exception}")
            if self.current_goal:
                self.failed_cells.add(self.current_goal.cell_index)
            self._reset_goal_state()
            return

        response = future.result()
        status = response.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Цель достигнута")
        elif status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_UNKNOWN):
            self.get_logger().warn("Цель отклонена (aborted)")
            if self.current_goal:
                self.failed_cells.add(self.current_goal.cell_index)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Цель отменена")
        else:
            self.get_logger().info(f"Цель завершилась со статусом {status}")

        self._reset_goal_state()

    def _cancel_active_goal(self, add_to_failed: bool = False) -> None:
        if self.current_goal is None:
            return
        if self.goal_handle_future and self.goal_handle_future.done():
            goal_handle = self.goal_handle_future.result()
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
        if add_to_failed and self.current_goal:
            self.failed_cells.add(self.current_goal.cell_index)
        self._reset_goal_state()

    def _process_action_futures(self) -> None:
        if self.goal_handle_future and self.goal_handle_future.done():
            pass
        if self.result_future and self.result_future.done():
            pass

    def _reset_goal_state(self) -> None:
        self.current_goal = None
        self.goal_sent_time = None
        self.goal_handle_future = None
        self.result_future = None


    def _check_nav_server(self) -> bool:
        if self._nav_server_ready:
            return True
        if self.goal_client.server_is_ready():
            self._nav_server_ready = True
            self.get_logger().info('Nav2 action сервер доступен')
            return True
        if self.goal_client.wait_for_server(timeout_sec=0.1):
            self._nav_server_ready = True
            self.get_logger().info('Nav2 action сервер доступен')
            return True
        self._warn_throttled('action_server_wait', 'Nav2 action сервер недоступен', 5.0)
        return False

    def _warn_throttled(self, key: str, message: str, period_sec: float) -> None:
        now = self.get_clock().now()
        last = self._last_warn.get(key)
        if last is None or (now - last) > Duration(seconds=period_sec):
            self.get_logger().warn(message)
            self._last_warn[key] = now

def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoGoalSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
