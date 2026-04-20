import math

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from hunav_msgs.msg import Agent
from hunav_msgs.srv import StartEvaluation
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from std_srvs.srv import Empty


class EvaluationRunner(Node):

    def __init__(self):
        super().__init__('evaluation_runner')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('experiment_tag', 'untagged')
        self.declare_parameter('run_id', -1)
        self.declare_parameter('evaluation_goal_topic', '/evaluation_goal_set')

        # Time is running at 1/10 speed in the simulation and we need to account for that in some logic
        self._time_factor = 10.0

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

        # Use '/evaluation_goal_set' for MCTS
        # Use '/goal_pose' for Nav2
        self.evaluation_goal_topic = self.get_parameter('evaluation_goal_topic').value
        self.set_evaluation_goal_publisher = self.create_publisher(
            PoseStamped,
            self.evaluation_goal_topic,
            10,
        )

        # For debugging
        self.debug_print_counter = 0
        # TODO: don't subscribe to this topic it isn't accurate
        # Using it as a "timer" for now
        self.robot_sub = self.create_subscription(
            Agent, "robot_states", self.robot_callback, 1
        )

        self.start_timer = self.create_timer(0.5, self.start_evaluation)
        self.stop_timer = self.create_timer(60.0 * self._time_factor, self.stop_evaluation)

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
        self.get_logger().info(f'Published scenario goal to {self.evaluation_goal_topic}')

        self.start_timer.cancel()

    def evaluation_done_callback(self, result):
        self.get_logger().info('Evaluation Completed')

    def _lookup_robot_transform(self):
        try:
            if not self.tf_buffer.can_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            ):
                self.get_logger().warn("TF map<-base_link not available yet; skipping this evaluation cycle.")
                return None

            return self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return None

    def robot_callback(self, msg: Agent):
        # TODO: Make this a timer callback instead
        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value

        transform = self._lookup_robot_transform()
        if transform is None:
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        distance = math.sqrt((robot_x - goal_x) ** 2 + (robot_y - goal_y) ** 2)

        if distance < 0.5:
            self.stop_evaluation()

        self.debug_print_counter += 1

        if self.debug_print_counter % 100 == 0:
            self.get_logger().info(f'Distance to goal is: {distance}')
            self.debug_print_counter = 0

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
