import time
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node

from social_simulator.scenario_utils import load_robot_config


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    half = yaw * 0.5
    return (math.cos(half), math.sin(half))


class ScenarioRobotPublisher(Node):
    def __init__(self) -> None:
        super().__init__("scenario_robot_publisher")

        self.declare_parameter("scenario_file", "")
        scenario_file = self.get_parameter("scenario_file").get_parameter_value().string_value
        if not scenario_file:
            raise RuntimeError("scenario_file parameter is required for scenario_robot_publisher")

        self.robot_cfg = load_robot_config(scenario_file)
        self.init_pose = self.robot_cfg.get("init_pose")
        self.goal = self.robot_cfg.get("goal")
        self.publish_initialpose = bool(self.robot_cfg.get("publish_initialpose", False))
        self.publish_goal = bool(self.robot_cfg.get("publish_goal", False))

        self.initialpose_delay_sec = float(self.robot_cfg.get("initialpose_delay_sec", 5.0))
        self.goal_delay_sec = float(self.robot_cfg.get("goal_delay_sec", 8.0))

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.clicked_point_pub = self.create_publisher(PointStamped, "/clicked_point", 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self._start_time = time.monotonic()
        self._initialpose_messages_remaining = 5 if self.publish_initialpose and self.init_pose else 0
        self._goal_messages_remaining = 5 if self.publish_goal and self.goal else 0
        self._tick_timer = self.create_timer(1.0, self._tick)

        if self._initialpose_messages_remaining == 0 and self._goal_messages_remaining == 0:
            self.get_logger().info("Scenario has no robot initial pose/goal to publish.")

    def _tick(self) -> None:
        elapsed = time.monotonic() - self._start_time

        if self._initialpose_messages_remaining > 0 and elapsed >= self.initialpose_delay_sec:
            self._publish_initialpose()
            self._initialpose_messages_remaining -= 1

        if self._goal_messages_remaining > 0 and elapsed >= self.goal_delay_sec:
            self._publish_goal()
            self._goal_messages_remaining -= 1

        if self._initialpose_messages_remaining <= 0 and self._goal_messages_remaining <= 0:
            self.get_logger().info("Finished publishing scenario robot initial pose and goal.")
            self._tick_timer.cancel()

    def _publish_initialpose(self) -> None:
        assert self.init_pose is not None
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.init_pose["x"]
        msg.pose.pose.position.y = self.init_pose["y"]
        msg.pose.pose.position.z = self.init_pose["z"]

        qw, qz = yaw_to_quaternion(self.init_pose["h"])
        msg.pose.pose.orientation.w = qw
        msg.pose.pose.orientation.z = qz

        # Small planar covariance so AMCL accepts the message.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.initialpose_pub.publish(msg)

    def _publish_goal(self) -> None:
        assert self.goal is not None

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal["x"]
        goal_pose.pose.position.y = self.goal["y"]
        goal_pose.pose.position.z = self.goal["z"]

        qw, qz = yaw_to_quaternion(self.goal["h"])
        goal_pose.pose.orientation.w = qw
        goal_pose.pose.orientation.z = qz
        self.goal_pose_pub.publish(goal_pose)

        clicked_point = PointStamped()
        clicked_point.header.frame_id = "map"
        clicked_point.header.stamp = goal_pose.header.stamp
        clicked_point.point.x = self.goal["x"]
        clicked_point.point.y = self.goal["y"]
        clicked_point.point.z = self.goal["z"]
        self.clicked_point_pub.publish(clicked_point)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioRobotPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
