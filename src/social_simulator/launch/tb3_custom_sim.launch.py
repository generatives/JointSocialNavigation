# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    social_simulator_pkg = FindPackageShare("social_simulator")
    social_simulator_dir = get_package_share_directory("social_simulator")
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    ))

    ld.add_action(DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    ))

    ld.add_action(DeclareLaunchArgument(
        "map",
        default_value="doors_hallway.yaml",
        description="Full path to map file to load",
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    ))

    ld.add_action(DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(social_simulator_dir, "params", "tb3_custom_sim.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    ))

    ld.add_action(DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    ))
    ld.add_action(DeclareLaunchArgument("x_pose", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("y_pose", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("yaw", default_value="0.0"))

    # Create the launch configuration variables
    slam = LaunchConfiguration("slam")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")

    rewritten_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "amcl.ros__parameters.initial_pose.x": x_pose,
            "amcl.ros__parameters.initial_pose.y": y_pose,
            "amcl.ros__parameters.initial_pose.yaw": yaw,
            "amcl.ros__parameters.set_initial_pose": "true",
        },
        convert_types=True,
    )

    # Launch configuration variables specific to simulation
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Using the TurtleBot3 Waffle model
    urdf = os.path.join(bringup_dir, "urdf", "turtlebot3_waffle.urdf")
    with open(urdf, "r") as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description}
        ],
        remappings=remappings,
    )

    map_yaml_path = PathJoinSubstitution([social_simulator_pkg, "maps", map_yaml_file])
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "map": map_yaml_path,
            "use_sim_time": use_sim_time,
            "params_file": rewritten_params,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    pub_robot_pose_node = Node(
        package="hunavis",
        executable="publish_global_pose",
        arguments=["--ros-args", "--log-level", "WARN"],
    )
 
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(pub_robot_pose_node)

    return ld
