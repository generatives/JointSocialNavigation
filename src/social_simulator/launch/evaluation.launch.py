import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

from social_simulator.utils import load_robot_config

def launch_setup(context, *args, **kwargs):
    social_simulator_dir = get_package_share_directory("social_simulator")
    hunav_evaluator_dir = get_package_share_directory("hunav_evaluator")
    hunav_evaluator_launch_dir = os.path.join(hunav_evaluator_dir, "launch")

    scenario = LaunchConfiguration("scenario").perform(context).strip()
    scenario_path = os.path.join(social_simulator_dir, "scenarios", scenario)

    robot_cfg = load_robot_config(scenario_path)

    hunav_evaluator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(hunav_evaluator_launch_dir, "hunav_evaluator_launch.py")),
        launch_arguments={
            "metrics_file":"/workspaces/JointSocialNavigation/src/social_simulator/config/metrics.yaml"
        }.items(),
    )

    navigator_node = Node(
        package="social_navigation",
        executable="navigator",
        name="navigator",
        output="screen"
    )

    experiment_tag = LaunchConfiguration("experiment_tag")
    run_id = LaunchConfiguration("run_id")
    evaluation_goal_topic = LaunchConfiguration("evaluation_goal_topic")

    evaluation_runner_node = Node(
        package="social_simulator",
        executable="evaluation_runner",
        name="evaluation_runner",
        output="screen",
        parameters=[
            {
                'goal_x': robot_cfg['goal']['x'],
                'goal_y': robot_cfg['goal']['y'],
                'experiment_tag': experiment_tag,
                'run_id': run_id,
                'evaluation_goal_topic': evaluation_goal_topic,
            }
        ],
        on_exit=Shutdown()
    )

    start_evaluation_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=navigator_node,
            on_start=[TimerAction(period=2.0, actions=[evaluation_runner_node])],
        )
    )

    return [
        hunav_evaluator_launch,
        navigator_node,
        start_evaluation_handler,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="agents_doors_hallway.yaml",
                description="Scenario file in the social_simulator/scenarios directory.",
            ),
            DeclareLaunchArgument(
                "experiment_tag",
                default_value="untagged",
                description="Experiment tag to record results under",
            ),
            DeclareLaunchArgument(
                "run_id",
                default_value="-1",
                description="Run ID to record results under",
            ),
            DeclareLaunchArgument(
                "evaluation_goal_topic",
                default_value="/evaluation_goal_set",
                description="PoseStamped topic used to send the evaluation goal. " \
                "Choose /evaluation_goal_set for navigator or /goal_pose for Nav2",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
