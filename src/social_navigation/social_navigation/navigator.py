import math
import random

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose

from social_navigation.mcts.decoupled_mcts import MCTS, MCTSConfig
from social_navigation.simulator.map import ScenarioMap
from social_navigation.simulator.mcts_game_state import MCTSGameState, MCTSGameStateConfig, navigation_rollout


def yaw_to_quat_wz(yaw: float):
    """
    Returns (w, z) for a planar yaw-only quaternion (x=y=0).
    """
    half = yaw * 0.5
    return math.cos(half), math.sin(half)

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

        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._goal_point: PointStamped | None = None

        self.get_logger().info('Initialized successfully')

    def clicked_point_callback(self, msg: PointStamped):
        self.get_logger().info('I heard: "%s"' % msg)
        self._goal_point = msg
        self._plan_intermediate_goal()

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
        
        num_agents = 1
        num_actions = [6]
        tree_depth = 6

        mcts_config = MCTSConfig(
            num_actors=num_agents,
            num_actions=num_actions,
            max_depth=tree_depth
        )

        human_speed = 1.7
        robot_radius = 0.5

        state_config = MCTSGameStateConfig(
            mcts_config=mcts_config,
            robot_speed=human_speed,
            dt=0.5,
            robot_radius=robot_radius,
            human_radius=0.5,
            angle=np.pi / 4.0,
            uncomfortable_distance=1.5,
            map=ScenarioMap.build_empty(),
        )
        mcts = MCTS(mcts_config, navigation_rollout, rng=random.Random(random.randint(0, 2**31 - 1)))

        transform = self.tf_buffer.lookup_transform(
            "map",        # target frame
            "base_link",  # robot frame
            rclpy.time.Time()
        )

        positions = np.array([
            [transform.transform.translation.x, transform.transform.translation.y]
        ])
        velocities = np.array([
            [math.cos(transform.transform.rotation.z), math.sin(transform.transform.rotation.z)]
        ])
        goal_positions = np.array([
            [self._goal_point.point.x, self._goal_point.point.y]
        ])

        root_state = MCTSGameState(
            positions=positions,
            velocities=velocities,
            agent_goal_positions=goal_positions,
            accumulated_value=None,
            config=state_config,
            depth=0
        )

        _, child_state, _ = mcts.search(root_state, num_simulations=500)
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