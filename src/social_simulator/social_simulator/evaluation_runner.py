import math
import random
import time
from typing import List

import numpy as np

from hunav_msgs.msg._agent import Agent
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist, Pose, Quaternion
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from hunav_msgs.srv import StartEvaluation

# Time is running at 1/10 speed in the simulation and we need to account for that in some logic
TIME_FACTOR = 10.0

class EvaluationRunner(Node):

    def __init__(self):
        super().__init__('evaluation_runner')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('experiment_tag', 'untagged')
        self.declare_parameter('run_id', -1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.start_evaluation_client = self.create_client(StartEvaluation, "hunav_start_recording")
        if not self.start_evaluation_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("StartEvaluation action server not available at /hunav_start_recording.")
            self.get_logger().error("Is hunav_evaluator_node running?")
            raise RuntimeError("hunav_start_recording action server not available")

        self.stop_evaluation_client = self.create_client(Empty, "hunav_stop_recording")
        if not self.stop_evaluation_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("EndEvaluation action server not available at /hunav_stop_recording.")
            self.get_logger().error("Is hunav_evaluator_node running?")
            raise RuntimeError("hunav_stop_recording action server not available")

        self.set_evaluation_goal_publisher = self.create_publisher(
            PoseStamped,
            '/evaluation_goal_set',
            10,
        )

        self.robot_sub = self.create_subscription(
            Agent, "robot_states", self.robot_callback, 1
        )

        self.start_timer = self.create_timer(0.5, self.start_evaluation)
        self.stop_timer = self.create_timer(60.0 * TIME_FACTOR, self.stop_evaluation)

        self.get_logger().info('Initialized successfully')

    def start_evaluation(self):
        if not self.tf_buffer.can_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            ):
            return

        self.get_logger().info('Starting Evaluation')
        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value

        goal = PoseStamped(
            header=tf2_ros.Header(frame_id='map'),
            pose=Pose(
                position=Point(x=goal_x, y=goal_y, z=0.0),
                orientation=Quaternion(w=0.0)
            )
        )

        experiment_tag = self.get_parameter('experiment_tag').get_parameter_value().string_value
        run_id = self.get_parameter('run_id').get_parameter_value().integer_value

        start_evaluation_msg = StartEvaluation.Request(
            robot_goal=goal,
            experiment_tag=experiment_tag,
            run_id=run_id
        )
        evaluation_result = self.start_evaluation_client.call_async(start_evaluation_msg)
        evaluation_result.add_done_callback(self.evaluation_done_callback)

        self.set_evaluation_goal_publisher.publish(goal)

        self.start_timer.cancel()

    def evaluation_done_callback(self, result):
        self.get_logger().info('Evaluation Completed')

    def robot_callback(self, msg: Agent):
        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value

        robot_x = msg.position.position.x
        robot_y = msg.position.position.y

        distance = math.sqrt((robot_x - goal_x) ** 2 + (robot_y - goal_y) ** 2)

        if distance < 0.5:
            self.stop_evaluation()


    def stop_evaluation(self):
        self.get_logger().info('Stopping Evaluation')
        self.stop_evaluation_client.call_async(Empty.Request())
        self.stop_timer.cancel()
        self.destroy_node()
        
def main(args=None):
    rclpy.init(args=args)

    runner = EvaluationRunner()

    rclpy.spin(runner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()