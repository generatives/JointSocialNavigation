import math
import random

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from geometry_msgs.msg import Point, PointStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray

from hunav_msgs.msg import Agents, Agent
from social_navigation.mcts.decoupled_mcts import MCTS, MCTSConfig
from social_navigation.simulator.scenario_map import ScenarioMap
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

        client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available at /navigate_to_pose.")
            self.get_logger().error("Is Nav2 running? (bt_navigator, controller_server, planner_server, etc.)")
            raise RuntimeError("navigate_to_pose action server not available")
        self._navigate_to_pose_client = client

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.goal_subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        
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
        
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._goal_point: PointStamped | None = None

        self.get_logger().info('Initialized successfully')

    def clicked_point_callback(self, msg: PointStamped):
        self.get_logger().info('I heard: "%s"' % msg)
        self._goal_point = msg
        self._plan_intermediate_goal()

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

    def _get_prediction_dt(self, tree_depth: int, robot_speed: float) -> float:
        default_dt = 0.5
        if self._goal_point is None:
            return default_dt

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )
        except Exception:
            return default_dt

        robot_position = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
            ],
            dtype=np.float32,
        )
        goal_position = np.array(
            [self._goal_point.point.x, self._goal_point.point.y],
            dtype=np.float32,
        )
        distance_to_goal = np.linalg.norm(robot_position - goal_position)
        return min(0.5, max(0.2, distance_to_goal / robot_speed / tree_depth))

    def _update_human_goal_markers(self) -> None:
        if self.human_states is None:
            self._publish_human_goal_markers(
                np.empty((0, 2), dtype=np.float32),
                np.empty((0, 2), dtype=np.float32),
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

    def _plan_intermediate_goal(self):
        if self._goal_point is None:
            return
        
        transform = self.tf_buffer.lookup_transform(
            "map",        # target frame
            "base_link",  # robot frame
            rclpy.time.Time()
        )

        robot_position = np.array([transform.transform.translation.x, transform.transform.translation.y])
        goal_position = np.array([self._goal_point.point.x, self._goal_point.point.y])
        distance_to_goal = np.linalg.norm(robot_position - goal_position)
        if distance_to_goal < 1.0:
            self.get_logger().info('Reached final goal')
            self._goal_point = None
            return
        
        tree_depth = 6

        robot_speed = 1.7
        robot_radius = 0.5
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
        velocities = np.array([
            [math.cos(robot_yaw), math.sin(robot_yaw)]
        ])

        human_goals = np.empty((0, 2))
        human_positions = np.empty((0, 2))
        if self.human_states is not None:
            human_positions = self.human_states[:, :2]
            human_velocities = self.human_states[:, 2:]
            human_goals = self._predict_human_goals(
                human_positions,
                human_velocities,
                dt,
                tree_depth,
            )
            positions = np.vstack((positions, human_positions))
            velocities = np.vstack((velocities, human_velocities))

        goal_positions = np.array([
            [self._goal_point.point.x, self._goal_point.point.y]
        ])
        if human_goals.shape[0] > 0:
            goal_positions = np.vstack((goal_positions, human_goals))

        num_agents = positions.shape[0]
        num_actions = [4] + [1] * (num_agents - 1)
        mcts_config = MCTSConfig(
            num_actors=num_agents,
            max_actions=num_actions,
            max_depth=tree_depth,
            rng=random.Random()
        )
        starting_distances = np.linalg.norm(goal_positions - positions, axis=1)

        state_config = MCTSGameStateConfig(
            mcts_config=mcts_config,
            robot_speed=robot_speed,
            dt=dt,
            robot_radius=robot_radius,
            human_radius=0.5,
            robot_angular_velocity=np.pi / 4.0,
            uncomfortable_distance=1.75,
            map=ScenarioMap.build_empty(),
            starting_distances=starting_distances,
        )
        mcts = MCTS(mcts_config, navigation_rollout)

        root_state = MCTSGameState(
            positions=positions,
            velocities=velocities,
            agent_goal_positions=goal_positions,
            accumulated_value=None,
            config=state_config,
            depth=0
        )

        _, child_state, _, _ = mcts.search(root_state, num_simulations=500)
        intermediate_goal = child_state.positions[0].copy()

        self.send_goal(intermediate_goal[0], intermediate_goal[1], 0)

    def send_goal(self, x: float, y: float, yaw: float, frame_id: str = "map"):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        w, z = yaw_to_quat_wz(float(yaw))
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


    def timer_callback(self):
        self._plan_intermediate_goal()


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
