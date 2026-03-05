"""
Starts human detection using a Zed2 camera. It launches
- rviz
- zed_wrapper_launch (starts Zed2 camera if use_simulator:=False)
- people_visualizer_node (shows detected humans as markers in rviz)

Arguments
 - use_simulator: 
    - If True, zed_wrapper_launch would not run. Instead, the scenario specified in 
      scenario_params_file will be simulated using hunavsim.
    - If False, zed_camera.launch.py (from zed_wrapper) would run, using launch
      arguments specified in the yaml file specified by zed_launch_args_file.
    - Note: For fixed cameras (e.g. detached from a robot), the tf_keyboard_publisher
            node from this repo can be run to adjust the map->camera tf in real time.
            Example parameters of that node are found in params/zed_common.yaml, 
            which is also the zed node parameters file.

- scenario_params_file: (ignored if use_simulator:=False)
    - Parameter file defining the simulation scenario. 
      Defaults to params/hunavsim.yaml

- zed_launch_args_file: (ignored if use_simulator:=True)
    - File containing the launch arguments to zed_camera.launch.py, including another
      parameter file that specifies the parameters of the zed node.
      Defaults to params/zed_launch_args.yaml

- run_rviz:
    - Whether to run rviz

- rviz_file:
    - File containing rviz settings
"""

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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, AndSubstitution, PythonExpression
from launch_ros.actions import Node

from hunavis.utils import goal_from_params


SCENARIO_PARAMS_FILE = "agents_doors_hallway.yaml"

DEFAULT_PARAMS_FILES = {
    "scenario": os.path.join(
        get_package_share_directory("social_simulator"),
        "scenarios",
        SCENARIO_PARAMS_FILE,
    ),
    "rviz": os.path.join(
        get_package_share_directory("social_simulator"), "rviz", "default_sim_view.yaml"
    ),
}


def launch_setup(context, *args, **kwargs):
    use_simulator = LaunchConfiguration("use_simulator")
    
    scenario_params_file = LaunchConfiguration("scenario_params_file")
    run_rviz = LaunchConfiguration("run_rviz")
    rviz_file = LaunchConfiguration("rviz_file")

    scenario_params_file_val = scenario_params_file.perform(context)
    rviz_file_val = rviz_file.perform(context)

    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d" + rviz_file_val],
        output={"both": "log"},
    )

    # Load list of human goals from the simulation parameters
    humans_goals_str = goal_from_params(scenario_params_file_val)

    people_visualizer_node = Node(
        package="hunavis",
        executable="people_visualizer",
        parameters=[
            {"use_simulator": use_simulator},
            {"goals": humans_goals_str},
        ],
    )
    p_mirrored_map_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_p_mirror",
        arguments=["0", "0", "0", "0", "3.14159", "0", "map", "p_mirrored_map"],
        output="screen",
    )
    rp_mirrored_map_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_rp_mirror",
        arguments=["0", "0", "0", "3.14159", "3.14159", "0", "map", "rp_mirrored_map"],
        output="screen",
    )
    r_mirrored_map_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_r_mirror",
        arguments=["0", "0", "0", "3.14159", "0", "0", "map", "r_mirrored_map"],
        output="screen",
    )
    return [
        people_visualizer_node,
        p_mirrored_map_tf_publisher,
        rp_mirrored_map_tf_publisher,
        r_mirrored_map_tf_publisher,
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
                default_value=DEFAULT_PARAMS_FILES["scenario"],
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
                default_value=DEFAULT_PARAMS_FILES["rviz"],
                description=("File containing rviz settings."),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
