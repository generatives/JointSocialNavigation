import math
import random
import time
from typing import List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from hunav_msgs.msg import Agents, Agent
from social_navigation.mcts.decoupled_mcts import MCTS, MCTSConfig
from social_navigation.simulator.pathfinding import a_star, simplify_path
from social_navigation.simulator.scenario_map import ScenarioMap, inflate_grid
from social_navigation.simulator.mcts_game_state import MCTSGameState, MCTSGameStateConfig, navigation_rollout

# Time is running at 1/10 speed in the simulation and we need to account for that in some logic
TIME_FACTOR = 10.0

def yaw_to_quat_wz(yaw: float):
    """
    Returns (w, z) for a planar yaw-only quaternion (x=y=0).
    """
    half = yaw * 0.5
    return math.cos(half), math.sin(half)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available at /navigate_to_pose.")
            self.get_logger().error("Is Nav2 running? (bt_navigator, controller_server, planner_server, etc.)")
            raise RuntimeError("navigate_to_pose action server not available")
        self._navigate_to_pose_client = client

        plan_dt = self._get_prediction_dt()
        plan_period = plan_dt * TIME_FACTOR
        self.plan_timer = self.create_timer(plan_period, self.plan_timer_callback)

        execute_period = plan_period / 10.0
        self.execute_plan_timer = self.create_timer(execute_period, self.execute_mcts_plan_callback)

        self.clicked_goal_subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)

        self.evaluation_goal_subscription = self.create_subscription(
            PoseStamped,
            '/evaluation_goal_set',
            self.evaluation_goal_set_callback,
            10)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )

        self.robot_linear_velocity = 0.0
        self.robot_angular_velocity = 0.0
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        
        self.human_states: np.ndarray | None = None
        self.human_ids: list[int] = []
        self.human_velocity_ema: dict[int, np.ndarray] = {}
        self.ema_alpha = 0.02
        self.human_states_subscription = self.create_subscription(
            Agents,
            '/human_states',
            self.human_states_callback,
            10)
        self.human_goal_markers_publisher = self.create_publisher(
            MarkerArray,
            '/predicted_human_goals',
            10,
        )

        self.global_path_publisher = self.create_publisher(
            Path,
            '/mcts_global_path',
            10,
        )
        self.local_waypoint_publisher = self.create_publisher(
            Marker,
            '/mcts_local_waypoint',
            10,
        )
        self.mcts_child_state_marker_publisher = self.create_publisher(
            Marker,
            '/mcts_child_state',
            10,
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1,
        )
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._goal_point: PointStamped | None = None
        self._scenario_map = ScenarioMap.build_empty()
        self._global_plan_cells: list[tuple[int, int]] = []
        self._global_plan_period = 1.0
        self._local_waypoint_lookahead = 2.0
        self._last_global_plan_time = None
        self._tree_depth = 6
        self._robot_speed = 1.0
        self._robot_angular_velocity_limit = 1.82
        self._robot_radius = 0.5
        self._path_obstacle_inflation_radius = 0.6
        self._tracking_feedback_blend = 0.65
        self._tracking_min_lookahead = 0.35
        self._tracking_plan_lookahead_distance = 0.75
        self._tracking_plan_lookahead_time = 0.5
        self._tracking_min_heading_speed_scale = 0.15
        self._tracking_goal_tolerance = 0.15

        self._mcts_plan_start_time = None
        self._mcts_action_plan = None
        self._mcts_predicted_robot_positions: np.ndarray | None = None
        self._mcts_predicted_robot_orientations: np.ndarray | None = None
        self._mcts_tracking_state_idx = 0

        self.get_logger().info('Initialized successfully')


    def odom_callback(self, msg: Odometry):
        self.robot_linear_velocity = msg.twist.twist.linear.x
        self.robot_angular_velocity = msg.twist.twist.angular.z

    def _clear_mcts_plan(self) -> None:
        self._mcts_plan_start_time = None
        self._mcts_action_plan = None
        self._mcts_predicted_robot_positions = None
        self._mcts_predicted_robot_orientations = None
        self._mcts_tracking_state_idx = 0

    def _compute_tracking_command(
        self,
        robot_position: np.ndarray,
        robot_yaw: float,
    ) -> tuple[float, float] | None:
        if not self._mcts_action_plan:
            return None
        if self._mcts_predicted_robot_positions is None or self._mcts_predicted_robot_orientations is None:
            return None
        if len(self._mcts_predicted_robot_positions) == 0 or len(self._mcts_predicted_robot_orientations) == 0:
            return None

        # Pure Pursuit tracking controller
        plan_points = self._mcts_predicted_robot_positions
        plan_orientations = self._mcts_predicted_robot_orientations
        search_start = min(
            max(0, self._mcts_tracking_state_idx - 1),
            len(plan_points) - 1,
        )
        remaining_points = plan_points[search_start:]
        distances = np.linalg.norm(remaining_points - robot_position, axis=1)
        nearest_idx = search_start + int(np.argmin(distances))
        self._mcts_tracking_state_idx = nearest_idx

        dt = self._get_prediction_dt()
        time_diff_limit = self._tracking_plan_lookahead_time + time.time() - self._mcts_plan_start_time
        furthest_idx = int(time_diff_limit / dt)
        furthest_idx = min(furthest_idx, len(self._mcts_action_plan) - 1)
        action_idx = min(nearest_idx, len(self._mcts_action_plan) - 1)
        target_idx = nearest_idx
        travelled = 0.0
        for idx in range(nearest_idx, furthest_idx):
            segment = float(np.linalg.norm(plan_points[idx + 1] - plan_points[idx]))
            travelled += segment
            target_idx = idx + 1
            if travelled >= self._tracking_plan_lookahead_distance:
                break

        target_position = plan_points[target_idx]
        target_vector = target_position - robot_position
        target_distance = float(np.linalg.norm(target_vector))

        if action_idx == len(self._mcts_action_plan) - 1 and target_distance < self._tracking_goal_tolerance:
            return 0.0, 0.0

        nominal_linear, nominal_angular = self._mcts_action_plan[action_idx][0]
        commanded_linear = max(0.0, float(nominal_linear))

        if target_distance > 1e-6:
            target_heading = math.atan2(float(target_vector[1]), float(target_vector[0]))
        else:
            target_heading = plan_orientations[target_idx]
        heading_error = normalize_angle(target_heading - robot_yaw)

        heading_speed_scale = float(
            np.clip(
                1.0 - abs(heading_error) / math.pi,
                self._tracking_min_heading_speed_scale,
                1.0,
            )
        )
        commanded_linear *= heading_speed_scale
        if target_distance < self._tracking_min_lookahead:
            commanded_linear *= target_distance / self._tracking_min_lookahead

        lookahead = max(target_distance, self._tracking_min_lookahead)
        if commanded_linear > 1e-3:
            pure_pursuit_angular = (
                2.0 * commanded_linear * math.sin(heading_error) / lookahead
            )
        else:
            pure_pursuit_angular = 1.2 * heading_error

        commanded_angular = (
            (1.0 - self._tracking_feedback_blend) * float(nominal_angular)
            + self._tracking_feedback_blend * pure_pursuit_angular
        )
        commanded_angular = float(
            np.clip(
                commanded_angular,
                -self._robot_angular_velocity_limit,
                self._robot_angular_velocity_limit,
            )
        )
        return commanded_linear, commanded_angular

    def execute_mcts_plan_callback(self):
        if self._mcts_action_plan is None:
            return

        transform = self._lookup_robot_transform()
        if transform is None:
            return

        robot_position = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
            ],
        )
        robot_yaw = quat_to_yaw(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        command = self._compute_tracking_command(robot_position, robot_yaw)
        if command is None:
            self.send_cmd_vel(0.0, 0.0)
            return

        lin_vel, ang_vel = command
        self.send_cmd_vel(lin_vel, ang_vel)

    def clicked_point_callback(self, msg: PointStamped):
        self.get_logger().info('I heard clicked goal: "%s"' % msg)
        self._goal_point = msg
        self._global_plan_cells = []
        self._last_global_plan_time = None
        self._clear_mcts_plan()
        self._plan(True)

    def evaluation_goal_set_callback(self, msg: PoseStamped):
        self.get_logger().info('I heard eval goal: "%s"' % msg)
        if self._goal_point is not None:
            self.get_logger().error(
                "An evaluation goal has been set after another goal. Something is probably be wrong.")
            
        self._goal_point = PointStamped(
            point=Point(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                z=msg.pose.position.z,
            )
        )
        self._global_plan_cells = []
        self._last_global_plan_time = None
        self._clear_mcts_plan()
        self._plan(True)


    def human_states_callback(self, msg: Agents):
        num_agents = len(msg.agents)
        num_states = 4  # x, y, vx, vy
        if num_agents:
            self.human_states = np.zeros((num_agents, num_states), dtype=np.float32)
            active_ids = set()
            self.human_ids = []

            for i, agent in enumerate(msg.agents):
                agent: Agent = agent
                active_ids.add(agent.id)
                self.human_ids.append(agent.id)

                velocity = np.array(
                    [agent.velocity.linear.x, agent.velocity.linear.y],
                )
                if np.linalg.norm(velocity) < 1e-6:
                    velocity = np.array(
                        [
                            agent.linear_vel * math.cos(agent.yaw),
                            agent.linear_vel * math.sin(agent.yaw),
                        ]
                    )

                previous_ema = self.human_velocity_ema.get(agent.id)
                if previous_ema is None:
                    ema_velocity = velocity.copy()
                else:
                    ema_velocity = (
                        self.ema_alpha * velocity
                        + (1.0 - self.ema_alpha) * previous_ema
                    )
                self.human_velocity_ema[agent.id] = ema_velocity

                self.human_states[i][0] = agent.position.position.x
                self.human_states[i][1] = agent.position.position.y
                self.human_states[i][2:] = velocity

            stale_ids = set(self.human_velocity_ema) - active_ids
            for stale_id in stale_ids:
                self.human_velocity_ema.pop(stale_id, None)
        else:
            self.human_states = None
            self.human_ids = []
            self.human_velocity_ema.clear()

        self._update_human_goal_markers()

    def _predict_human_goals(
        self,
        human_positions: np.ndarray,
        human_velocities: np.ndarray,
        dt: float,
        tree_depth: int,
    ) -> np.ndarray:
        if human_positions.shape[0] == 0:
            return np.empty((0, 2))

        ema_velocities = np.array(
            [
                self.human_velocity_ema.get(agent_id, human_velocities[i])
                for i, agent_id in enumerate(self.human_ids)
            ],
        )
        horizon = np.linalg.norm(human_velocities, axis=1) * dt * tree_depth
        ema_speeds = np.linalg.norm(ema_velocities, axis=1, keepdims=True)
        moving = ema_speeds > 0.1
        safe_ema_speeds = np.where(moving, ema_speeds, 1.0)
        ema_directions = np.where(moving, ema_velocities / safe_ema_speeds, 0.0)
        return human_positions + ema_directions * horizon[:, np.newaxis]

    def _get_prediction_dt(self, tree_depth: int = None, robot_speed: float = None) -> float:
        return 0.5

    def _update_human_goal_markers(self) -> None:
        if self.human_states is None:
            self._publish_human_goal_markers(
                np.empty((0, 2)),
                np.empty((0, 2)),
            )
            return

        tree_depth = 6
        robot_speed = 1.7
        dt = self._get_prediction_dt(tree_depth, robot_speed)
        human_positions = self.human_states[:, :2]
        human_velocities = self.human_states[:, 2:]
        human_goals = self._predict_human_goals(
            human_positions,
            human_velocities,
            dt,
            tree_depth,
        )
        self._publish_human_goal_markers(human_positions, human_goals)

    def _publish_human_goal_markers(
        self,
        human_positions: np.ndarray,
        human_goals: np.ndarray,
    ) -> None:
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, (position, goal) in enumerate(zip(human_positions, human_goals)):
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = clear_marker.header.stamp
            goal_marker.ns = "predicted_human_goals"
            goal_marker.id = i
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = float(goal[0])
            goal_marker.pose.position.y = float(goal[1])
            goal_marker.pose.position.z = 0.25
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.3
            goal_marker.scale.z = 0.3
            goal_marker.color.a = 0.9
            goal_marker.color.r = 0.15
            goal_marker.color.g = 0.8
            goal_marker.color.b = 0.2
            goal_marker.lifetime.nanosec = 800_000_000
            marker_array.markers.append(goal_marker)

            path_marker = Marker()
            path_marker.header.frame_id = "map"
            path_marker.header.stamp = clear_marker.header.stamp
            path_marker.ns = "predicted_human_goal_paths"
            path_marker.id = i
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.06
            path_marker.color.a = 0.85
            path_marker.color.r = 0.1
            path_marker.color.g = 0.55
            path_marker.color.b = 0.95
            path_marker.lifetime.nanosec = 800_000_000

            start_point = Point()
            start_point.x = float(position[0])
            start_point.y = float(position[1])
            start_point.z = 0.1
            end_point = Point()
            end_point.x = float(goal[0])
            end_point.y = float(goal[1])
            end_point.z = 0.1
            path_marker.points = [start_point, end_point]
            marker_array.markers.append(path_marker)

        self.human_goal_markers_publisher.publish(marker_array)

    def map_callback(self, msg: OccupancyGrid):
        self._scenario_map = ScenarioMap.build_from_occupancy_data(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            data=msg.data,
        )
        self._global_plan_cells = []
        self._last_global_plan_time = None
        self._clear_mcts_plan()

    def _lookup_robot_transform(self):
        try:
            if not self.tf_buffer.can_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            ):
                self.get_logger().warn("TF map<-base_link not available yet; skipping this planning cycle.")
                return None

            return self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return None

    def _publish_global_path(self, path_cells: list[tuple[int, int]]) -> None:
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for cell in path_cells:
            pose = PoseStamped()
            pose.header = path_msg.header
            waypoint = self._scenario_map.cell_to_world(cell)
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.global_path_publisher.publish(path_msg)

    def _publish_local_waypoint(self, waypoint: np.ndarray | None) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mcts_local_waypoint"
        marker.id = 0

        if waypoint is None:
            marker.action = Marker.DELETE
            self.local_waypoint_publisher.publish(marker)
            return

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(waypoint[0])
        marker.pose.position.y = float(waypoint[1])
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.a = 0.95
        marker.color.r = 0.95
        marker.color.g = 0.45
        marker.color.b = 0.1
        self.local_waypoint_publisher.publish(marker)

    def _publish_child_state_marker(self, goals: List[np.ndarray] | None) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mcts_child_state"
        marker.id = 0

        if goals is None:
            marker.action = Marker.DELETE
            self.mcts_child_state_marker_publisher.publish(marker)
            return

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = [
            Point(x=goal[0], y=goal[1], z=0.2) for goal in goals
        ]
        marker.scale.x = 0.05

        marker.color.a = 0.95
        marker.color.r = 0.1
        marker.color.g = 0.1
        marker.color.b = 0.95
        self.mcts_child_state_marker_publisher.publish(marker)

    def _update_global_plan(self, robot_position: np.ndarray) -> None:
        if self._goal_point is None:
            self._global_plan_cells = []
            self._publish_global_path([])
            self._publish_local_waypoint(None)
            self._publish_child_state_marker(None)
            return

        now = self.get_clock().now()
        if self._last_global_plan_time is not None:
            elapsed = (now - self._last_global_plan_time).nanoseconds * 1e-9
            if elapsed < self._global_plan_period and self._global_plan_cells:
                return

        start_cell = self._scenario_map.nearest_free(self._scenario_map.world_to_cell(robot_position))
        goal_world = np.array([self._goal_point.point.x, self._goal_point.point.y])
        goal_cell = self._scenario_map.nearest_free(self._scenario_map.world_to_cell(goal_world))

        if start_cell is None or goal_cell is None:
            self._global_plan_cells = []
            self._publish_global_path([])
            self._publish_local_waypoint(None)
            self._publish_child_state_marker(None)
            self._last_global_plan_time = now
            return

        inflated_cells = max(
            1,
            int(
                math.ceil(
                    self._path_obstacle_inflation_radius
                    / max(self._scenario_map.resolution, 1e-6)
                )
            ),
        )
        inflated_grid = inflate_grid(self._scenario_map.grid, inflated_cells)
        path_cells = a_star(inflated_grid, start_cell, goal_cell)
        #path_cells = simplify_path(path_cells)
        self._global_plan_cells = path_cells
        self._publish_global_path(path_cells)
        if not path_cells:
            self._publish_local_waypoint(None)
            self._publish_child_state_marker(None)
        self._last_global_plan_time = now

    def _select_local_waypoint(self, robot_position: np.ndarray) -> np.ndarray | None:
        if not self._global_plan_cells:
            self._publish_local_waypoint(None)
            self._publish_child_state_marker(None)
            return None

        path_points = np.array(
            [self._scenario_map.cell_to_world(cell) for cell in self._global_plan_cells],
        )

        distances = np.linalg.norm(path_points - robot_position, axis=1)
        nearest_idx = np.argmin(distances)

        #waypoint = path_points[0]
        #origin = robot_position
        #for idx in range(0, len(path_points)):
        #    candidate_waypoint = path_points[idx]
        #    diff = abs(candidate_waypoint - origin)
        #    threshold = 0.3
        #    if np.all(diff > threshold):
        #        break
        #    waypoint = candidate_waypoint

        waypoint = path_points[-1]
        travelled = 0.0
        for idx in range(nearest_idx, len(path_points) - 1):
            current_point = path_points[idx]
            next_point = path_points[idx + 1]
            segment = np.linalg.norm(next_point - current_point)
            travelled += segment
            waypoint = next_point
            if travelled >= self._local_waypoint_lookahead:
                break


        self._publish_local_waypoint(waypoint)
        return waypoint

    def _plan(self, do_global_plan: bool = False):
        #self.send_cmd_vel(0.0, 0.0)

        if self._goal_point is None:
            return

        plan_start_time = time.perf_counter()

        transform = self._lookup_robot_transform()
        if transform is None:
            return

        robot_position = np.array([transform.transform.translation.x, transform.transform.translation.y])
        goal_position = np.array([self._goal_point.point.x, self._goal_point.point.y])
        distance_to_goal = np.linalg.norm(robot_position - goal_position)
        if distance_to_goal < 0.3:
            self.get_logger().info('Reached final goal')
            self._goal_point = None
            self._global_plan_cells = []
            self._last_global_plan_time = None
            self._clear_mcts_plan()
            self._publish_global_path([])
            self._publish_local_waypoint(None)
            self._publish_child_state_marker(None)
            self.send_cmd_vel(0.0, 0.0)
            return

        astar_start_time = time.perf_counter()
        if do_global_plan or not self._global_plan_cells:
            self._update_global_plan(robot_position)
        astar_elapsed_ms = (time.perf_counter() - astar_start_time) * 1000.0
        local_waypoint = self._select_local_waypoint(robot_position)
        if local_waypoint is None:
            self.get_logger().warn('No global A* path available; skipping local MCTS plan.')
            self._clear_mcts_plan()
            self.send_cmd_vel(0.0, 0.0)
            return
        
        tree_depth = self._tree_depth

        robot_speed = self._robot_speed
        robot_angular_velocity = self._robot_angular_velocity_limit
        robot_radius = self._robot_radius
        dt = self._get_prediction_dt(tree_depth, robot_speed)
        robot_yaw = quat_to_yaw(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )

        positions = np.array([
            [transform.transform.translation.x, transform.transform.translation.y]
        ])
        orientations = np.array([
            robot_yaw
        ])
        linear_velocities = np.array([
            self.robot_linear_velocity
        ])
        angular_velocities = np.array([
            self.robot_angular_velocity
        ])
        goal_positions = np.array([
            [local_waypoint[0], local_waypoint[1]]
        ])

        if self.human_states is not None:
            human_positions = self.human_states[:, :2]
            human_velocities = self.human_states[:, 2:]
            human_linear_velocities = np.linalg.norm(human_velocities, axis=1)
            human_orientations = np.arctan2(human_velocities[:, 1], human_velocities[:, 0])
            human_angular_velocities = np.zeros_like(human_orientations)
            human_goals = self._predict_human_goals(
                human_positions,
                human_velocities,
                dt,
                tree_depth,
            )
            positions = np.concat((positions, human_positions), axis=0)
            orientations = np.concat((orientations, human_orientations), axis=0)
            linear_velocities = np.concat((linear_velocities, human_linear_velocities), axis=0)
            angular_velocities = np.concat((angular_velocities, human_angular_velocities), axis=0)

            if human_goals.shape[0] > 0:
                goal_positions = np.concat((goal_positions, human_goals), axis=0)


        num_agents = positions.shape[0]
        num_actions = [8] + [1] * (num_agents - 1)
        mcts_config = MCTSConfig(
            num_actors=num_agents,
            max_actions=num_actions,
            max_depth=tree_depth,
            rng=np.random.default_rng(),
            c_puct=1.4 * 2.0,
            pw_c=2.0,
            pw_alpha=0.3
        )
        starting_distances = np.linalg.norm(goal_positions - positions, axis=1)

        state_config = MCTSGameStateConfig(
            mcts_config=mcts_config,
            dt=dt,
            robot_speed=robot_speed,
            robot_max_linear_accel=2.5,
            robot_angular_velocity=robot_angular_velocity,
            robot_max_angular_accel=1.2,
            robot_radius=robot_radius,
            human_radius=0.5,
            uncomfortable_distance=1.75,
            map=self._scenario_map,
            starting_distances=starting_distances,
        )
        mcts = MCTS(mcts_config, navigation_rollout)

        root_state = MCTSGameState(
            positions=positions,
            linear_velocities=linear_velocities,
            orientations=orientations,
            angular_velocities=angular_velocities,
            agent_goal_positions=goal_positions,
            accumulated_value=None,
            config=state_config,
            depth=0
        )

        mcts_start_time = time.perf_counter()
        mcts_plan_start_time = time.time()
        actions, states, _, _ = mcts.search(root_state, num_simulations=500)
        mcts_elapsed_ms = (time.perf_counter() - mcts_start_time) * 1000.0
        
        if actions is not None and states is not None:
            marker_goals = [robot_position]
            marker_goals.extend([state.positions[0].copy() for state in states])
            self._publish_child_state_marker(marker_goals)

            goal_orientations = [robot_yaw]
            goal_orientations.extend([state.orientations[0].copy() for state in states])

            #print([action[0] for action in actions])
            self._mcts_action_plan = actions
            self._mcts_predicted_robot_positions = np.asarray(marker_goals, dtype=np.float32)
            self._mcts_predicted_robot_orientations = np.asarray(goal_orientations, dtype=np.float32)
            self._mcts_tracking_state_idx = 0
            self._mcts_plan_start_time = mcts_plan_start_time

            total_plan_elapsed_ms = (time.perf_counter() - plan_start_time) * 1000.0
            self.get_logger().info(
                "Planning timing: total=%.1f ms, astar=%.1f ms, mcts.search=%.1f ms"
                % (total_plan_elapsed_ms, astar_elapsed_ms, mcts_elapsed_ms)
            )
        else:
            self.get_logger().warn('MCTS returned no action.')
            self._clear_mcts_plan()
            self.send_cmd_vel(0.0, 0.0)


    def send_cmd_vel(self, v: float, w: float):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(v)
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = float(w)
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def send_goal(self, x: float, y: float, yaw: float, frame_id: str = "map"):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        w, z = yaw_to_quat_wz(yaw)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w

        self.get_logger().info(f"Sending goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad (frame={frame_id})")

        self._goal_handle_future = self._navigate_to_pose_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        self._goal_handle_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg: NavigateToPose.Feedback):
        fb = feedback_msg.feedback
        # Feedback fields can differ slightly across Nav2 versions; distance_remaining is commonly present.
        if hasattr(fb, "distance_remaining"):
            pass
            #self.get_logger().info(f"Feedback: distance_remaining={fb.distance_remaining:.3f}")

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        status = future.result().status
        # Status codes come from action_msgs/msg/GoalStatus
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED (common ones)
        if status == 4:
            self.get_logger().info("Navigation SUCCEEDED.")
        elif status == 5:
            self.get_logger().warn("Navigation CANCELED.")
        elif status == 6:
            self.get_logger().error("Navigation ABORTED.")
        else:
            self.get_logger().warn(f"Navigation finished with status={status}")


    def plan_timer_callback(self):
        self._plan()


def main(args=None):
    rclpy.init(args=args)

    navigator = Navigator()

    rclpy.spin(navigator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
