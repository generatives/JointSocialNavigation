import math
import time
from typing import List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from hunav_msgs.msg import Agents, Agent
from social_navigation.mcts.decoupled_mcts import MCTS, MCTSConfig
from social_navigation.simulator.pathfinding import a_star
from social_navigation.simulator.scenario_map import ScenarioMap, inflate_grid
from social_navigation.simulator.mcts_game_state import MCTSGameState, MCTSGameStateConfig, navigation_rollout

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

class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        ### Parameters

        # Time is running at 1/10 speed in the simulation and 
        # we need to account for that in some logic
        self._time_factor = 10.0
        self._prediction_dt = 0.6
        plan_dt = self._prediction_dt
        plan_period = plan_dt * self._time_factor
        execute_period = plan_period / 10.0
        self.ema_alpha = 0.02
        self._global_plan_period = 1.0
        self._local_waypoint_lookahead = 2.0
        self._tree_depth = 6
        self._robot_speed = 1.0
        self._robot_angular_velocity_limit = 1.82
        self._robot_radius = 0.22
        self._path_obstacle_inflation_radius = 0.6
        self._tracking_feedback_blend = 0.65
        self._tracking_min_lookahead = 0.35
        self._tracking_plan_lookahead_distance = 0.75
        self._tracking_plan_lookahead_time = 0.5
        self._tracking_min_heading_speed_scale = 0.15
        self._tracking_goal_tolerance = 0.15

        ### Robot states
        self.robot_linear_velocity = 0.0
        self.robot_angular_velocity = 0.0
        
        ### Human States
        self.human_states: np.ndarray | None = None
        self.human_ids: list[int] = []
        self.human_velocity_ema: dict[int, np.ndarray] = {}

        ### Planning states
        self._goal_point: PointStamped | None = None
        self._scenario_map = ScenarioMap.build_empty()
        self._global_plan_cells: list[tuple[int, int]] = []
        self._last_global_plan_time = None
        self._mcts_plan_start_time = None
        self._mcts_action_plan = None
        self._mcts_predicted_robot_positions: np.ndarray | None = None
        self._mcts_predicted_robot_orientations: np.ndarray | None = None
        self._mcts_tracking_state_idx = 0

        ### Transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        ### Planning Timers
        self.plan_timer = self.create_timer(plan_period, self.plan_timer_callback)
        self.execute_plan_timer = self.create_timer(execute_period, self.execute_mcts_plan_callback)

        ### Pubs / Subs
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
            MarkerArray,
            '/mcts_child_state',
            10,
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1,
        )

        self.get_logger().info('Initialized successfully')

    ################################
    ####### MCTS Controller ########
    ################################

    def _clear_mcts_plan(self) -> None:
        # Reset both the open-loop MCTS plan and the cached state used by the
        # feedback tracker that follows it.
        self._mcts_plan_start_time = None
        self._mcts_action_plan = None
        self._mcts_predicted_robot_positions = None
        self._mcts_predicted_robot_orientations = None
        self._mcts_tracking_state_idx = 0

    def _plan(self, do_global_plan: bool = False):
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

        # First use A* for long-range obstacle avoidance, then hand a local
        # waypoint to MCTS for short-horizon social navigation.
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
        dt = self._prediction_dt
        robot_yaw = quat_to_yaw(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )

        # Build the joint planning state from the current robot estimate plus
        # the latest human positions and predicted goals.
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
            
            # Smooth human velocity estimates before projecting likely goals so
            # the planner does not react to frame-to-frame noise.
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

        # Number of HuNavSim agents
        num_agents = positions.shape[0]
        # Each HuNavSim agent only has 1 action
        # Robot has 8 actions
        num_actions = [8] + [3] * (num_agents - 1)
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
            human_radius=0.4,
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

        # Search for a short-horizon joint plan over robot controls and human
        # motion disturbances, then keep the robot branch for execution.
        mcts_start_time = time.perf_counter()
        mcts_plan_start_time = time.time()
        actions, states, _, _ = mcts.search(root_state, num_simulations=500)
        mcts_elapsed_ms = (time.perf_counter() - mcts_start_time) * 1000.0
        
        if actions is not None and states is not None:
            marker_states = [(float(robot_position[0]), float(robot_position[1]), float(robot_yaw))]
            marker_states.extend(
                (float(state.positions[0][0]),float(state.positions[0][1]), float(state.orientations[0]))
                for state in states
            )
            self._publish_child_state_marker(marker_states)

            #print([action[0] for action in actions])
            self._mcts_action_plan = actions

            robot_states = np.array(marker_states)
            self._mcts_predicted_robot_positions = robot_states[:, :2]
            self._mcts_predicted_robot_orientations = robot_states[:, 2]

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

        # Inflate obstacles so the global path leaves enough clearance for the
        # robot body and the local tracker.
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

        # Walk forward along the global path until the configured lookahead
        # distance is reached; that point becomes the MCTS subgoal.
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

        # Visualize waypoint
        self._publish_local_waypoint(waypoint)
        return waypoint

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

        # MCTS plans with a simplified dynamics model, so execution uses a
        # pure-pursuit-style feedback controller to stay close to that plan.
        plan_points = self._mcts_predicted_robot_positions
        plan_orientations = self._mcts_predicted_robot_orientations

        # Resume tracking near the last matched plan state instead of searching
        # from the beginning every cycle.
        search_start = min(
            max(0, self._mcts_tracking_state_idx - 1),
            len(plan_points) - 1,
        )
        remaining_points = plan_points[search_start:]
        distances = np.linalg.norm(remaining_points - robot_position, axis=1)
        nearest_idx = search_start + int(np.argmin(distances))
        self._mcts_tracking_state_idx = nearest_idx

        # Pick a target point ahead on the plan, but never farther forward than
        # the elapsed plan time would reasonably allow.
        dt = self._prediction_dt
        time_diff_limit = self._tracking_plan_lookahead_time + time.time() - self._mcts_plan_start_time
        furthest_idx = int(time_diff_limit / dt)
        furthest_idx = min(furthest_idx, len(plan_points) - 1)
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

        # Start from the open-loop command suggested by MCTS.
        nominal_linear, nominal_angular = self._mcts_action_plan[action_idx][0]
        commanded_linear = max(0.0, float(nominal_linear))

        # Either face the target point directly or, if already there, align with
        # the stored plan heading.
        if target_distance > 1e-6:
            target_heading = math.atan2(float(target_vector[1]), float(target_vector[0]))
        else:
            # Rotate towards MCTS plan
            target_heading = plan_orientations[target_idx]

        angle = target_heading - robot_yaw
        heading_error = math.atan2(math.sin(angle), math.cos(angle))

        # Reduce forward speed if the robot is not roughly facing the target.
        heading_speed_scale = float(
            np.clip(
                1.0 - abs(heading_error) / math.pi,
                self._tracking_min_heading_speed_scale,
                1.0,
            )
        )
        # Reduce speed when not facing the target position
        commanded_linear *= heading_speed_scale
        if target_distance < self._tracking_min_lookahead:
            # If state on MCTS plan is very close, reduce speed
            commanded_linear *= target_distance / self._tracking_min_lookahead

        lookahead = max(target_distance, self._tracking_min_lookahead)
        if commanded_linear > 1e-3:
            # Standard pure-pursuit curvature command.
            pure_pursuit_angular = (
                commanded_linear * (2.0*math.sin(heading_error) / lookahead)
            )
        else:
            # If essentially stopped, fall back to in-place heading correction.
            pure_pursuit_angular = 1.2 * heading_error

        # Blend the nominal MCTS turn rate with the feedback correction to keep
        # the executed motion close to the predicted trajectory.
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
        # Tracker runs at once every 5 seconds
        command = self._compute_tracking_command(robot_position, robot_yaw)
        if command is None:
            self.send_cmd_vel(0.0, 0.0)
            return

        lin_vel, ang_vel = command
        self.send_cmd_vel(lin_vel, ang_vel)

    ############################
    ####### Human States #######
    ############################

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

                # Maintain a per-agent EMA so downstream goal prediction is less
                # sensitive to noisy velocity measurements.
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

        # Project each human forward along its smoothed heading for roughly the
        # same horizon the tree will simulate.
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

    ##########################
    ####### Start Plan #######
    ##########################

    def plan_timer_callback(self):
        self._plan()

    def send_cmd_vel(self, v: float, w: float):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(v)
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = float(w)
        self.cmd_vel_publisher.publish(cmd_vel_msg)

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

    ##############################
    ####### Visualizations #######
    ##############################

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

    def _publish_child_state_marker(
        self,
        marker_states: List[tuple[float, float, float]] | None,
    ) -> None:
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        if marker_states is None:
            self.mcts_child_state_marker_publisher.publish(marker_array)
            return

        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = clear_marker.header.stamp
        path_marker.ns = "mcts_child_state_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.points = []
        path_marker.scale.x = 0.05
        path_marker.color.a = 0.95
        path_marker.color.r = 0.1
        path_marker.color.g = 0.1
        path_marker.color.b = 0.95
        
        for idx, (x, y, yaw) in enumerate(marker_states):
            path_marker.points.append(Point(x=x, y=y, z=0.2))

            heading_marker = Marker()
            heading_marker.header.frame_id = "map"
            heading_marker.header.stamp = clear_marker.header.stamp
            heading_marker.ns = "mcts_child_state_heading"
            heading_marker.id = idx
            heading_marker.type = Marker.ARROW
            heading_marker.action = Marker.ADD
            heading_marker.pose.position.x = x
            heading_marker.pose.position.y = y
            heading_marker.pose.position.z = 0.28
            heading_marker.pose.orientation.x = 0.0
            heading_marker.pose.orientation.y = 0.0
            heading_marker.pose.orientation.z = math.sin(yaw * 0.5)
            heading_marker.pose.orientation.w = math.cos(yaw * 0.5)
            heading_marker.scale.x = 0.30
            heading_marker.scale.y = 0.05
            heading_marker.scale.z = 0.05
            heading_marker.color.a = 0.95
            heading_marker.color.r = 1.0
            heading_marker.color.g = 0.0
            heading_marker.color.b = 0.0
            marker_array.markers.append(heading_marker)

        marker_array.markers.append(path_marker)

        self.mcts_child_state_marker_publisher.publish(marker_array)

    def _update_human_goal_markers(self) -> None:
        if self.human_states is None:
            self._publish_human_goal_markers(
                np.empty((0, 2)),
                np.empty((0, 2)),
            )
            return

        tree_depth = self._tree_depth
        dt = self._prediction_dt
        human_positions = self.human_states[:, :2]
        human_velocities = self.human_states[:, 2:]
        human_goals = self._predict_human_goals(
            human_positions,
            human_velocities,
            dt,
            tree_depth,
        )
        self._publish_human_goal_markers(human_positions, human_goals)

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

    ###########################
    ####### Robot State #######
    ###########################

    def odom_callback(self, msg: Odometry):
        self.robot_linear_velocity = msg.twist.twist.linear.x
        self.robot_angular_velocity = msg.twist.twist.angular.z

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
