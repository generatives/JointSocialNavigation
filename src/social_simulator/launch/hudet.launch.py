import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from hunavis.utils import goal_from_params


def launch_setup(context, *args, **kwargs):
    social_simulator_pkg = FindPackageShare("social_simulator")

    use_simulator = LaunchConfiguration("use_simulator")
    scenario_params_file = LaunchConfiguration("scenario_params_file")
    run_rviz = LaunchConfiguration("run_rviz")
    rviz_file = LaunchConfiguration("rviz_file")

    rviz_file_path = PathJoinSubstitution([social_simulator_pkg, "rviz", rviz_file])
    rviz_file_val = rviz_file_path.perform(context)
    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file_val],
        output={"both": "log"},
    )

    # Load list of human goals from the simulation parameters
    scenario_path = PathJoinSubstitution([social_simulator_pkg, "scenarios", scenario_params_file])
    scenario_params_file_val = scenario_path.perform(context)
    humans_goals_str = goal_from_params(scenario_params_file_val)

    people_visualizer_node = Node(
        package="hunavis",
        executable="people_visualizer",
        parameters=[
            {"use_simulator": use_simulator},
            {"goals": humans_goals_str},
        ],
    )

    return [
        people_visualizer_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulator",
                default_value="True",
                description="Whether to use simulator.",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "scenario_params_file",
                default_value="agents_doors_hallway.yaml",
                description=("Params file specifying scenario if use_simulator:=True."),
            ),
            DeclareLaunchArgument(
                "run_rviz",
                default_value="True",
                description="Whether to use rviz.",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "rviz_file",
                default_value="default_sim_view.yaml",
                description=("File containing rviz settings."),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
