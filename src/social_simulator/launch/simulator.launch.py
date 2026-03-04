from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, SetEnvironmentVariable,
    ExecuteProcess, RegisterEventHandler, LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def declare_args():
    return [
        DeclareLaunchArgument(
            "scenario",
            default_value="agents_doors_hallway.yaml",
            description="Specify scenario file name in the scenarios directory",
        ),
        DeclareLaunchArgument(
            "metrics_file",
            default_value="metrics.yaml",
            description="Specify the name of the metrics configuration file in the config directory",
        ),
        DeclareLaunchArgument(
            "base_world",
            default_value="doors_hallway.world",
            description="Specify world file name",
        ),
        DeclareLaunchArgument(
            "use_humans",
            default_value="true",
            description="Specify world file name",
        ),
        DeclareLaunchArgument(
            "use_gazebo_obs",
            default_value="true",
            description="Whether to fill the agents obstacles with closest Gazebo obstacle or not",
        ),
        DeclareLaunchArgument(
            "update_rate", 
            default_value="100.0", 
            description="Update rate of the plugin"
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="turtlebot3_waffle",
            description="Specify the name of the robot Gazebo model",
        ),
        DeclareLaunchArgument(
            "global_frame_to_publish",
            default_value="map",
            description="Name of the global frame in which the position of the agents are provided",
        ),
        DeclareLaunchArgument(
            "use_navgoal_to_start",
            default_value="false",
            description="Whether to start the agents movements when a navigation goal is received or not",
        ),
        DeclareLaunchArgument(
            "ignore_models",
            default_value="ground_plane",
            description="list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them",
        ),
        DeclareLaunchArgument(
            "generated_world",
            default_value="generatedWorld.world",
            description="World file created by the world generator",
        ),
        DeclareLaunchArgument(
            "robot_sdf",
            default_value="",
            description="Full path to robot sdf/model file for spawn_entity.py",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="true",
            description='Set "true" to increase messages written to terminal.',
        )
    ]

def make_cfg():
    return {
        "scenario": LaunchConfiguration("scenario"),
        "metrics_file": LaunchConfiguration("metrics_file"),
        "use_humans": LaunchConfiguration("use_humans"),
        "base_world": LaunchConfiguration("base_world"),
        "generated_world": LaunchConfiguration("generated_world"),
        "verbose": LaunchConfiguration("verbose"),
        "robot_name": LaunchConfiguration("robot_name"),
        "robot_sdf": LaunchConfiguration("robot_sdf"),
    }

def make_paths(cfg):
    pkg = FindPackageShare("social_simulator")
    return {
        "scenario_path": PathJoinSubstitution([pkg, "scenarios", cfg["scenario"]]),
        "base_world_path": PathJoinSubstitution([pkg, "worlds", cfg["base_world"]]),
        "generated_world_path": PathJoinSubstitution([pkg, "worlds", cfg["generated_world"]]),
    }

def gazebo_group(world_path, cfg):
    gzserver = ExecuteProcess(
        cmd=["gzserver", world_path, "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )
    gzclient = ExecuteProcess(cmd=["gzclient"], output="screen")

    return GroupAction([
        LogInfo(msg=["Starting Gazebo with world: ", world_path]),
        gzserver,
        gzclient,
        # return handles if you want to hook event handlers; otherwise keep local
    ])

def hunav_group(cfg, paths):
    loader = Node(
        package="hunav_agent_manager",
        executable="hunav_loader",
        output="screen",
        parameters=[paths["scenario_path"]],
    )

    worldgen = Node(
        package="hunav_gazebo_wrapper",
        executable="hunav_gazebo_world_generator",
        output="screen",
        parameters=[{"base_world": paths["base_world_path"]}],
    )

    return {"loader": loader, "worldgen": worldgen, "group": GroupAction([loader, worldgen])}


def generate_launch_description():
    ld = LaunchDescription()

    for a in declare_args():
        ld.add_action(a)

    cfg = make_cfg()
    paths = make_paths(cfg)

    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=[EnvironmentVariable("GAZEBO_RESOURCE_PATH"), ":", PathJoinSubstitution([FindPackageShare("hunav_gazebo_wrapper"), "models"])]
    ))

    hunav = hunav_group(cfg, paths)

    ld.add_action(GroupAction([hunav["group"]], condition=IfCondition(cfg["use_humans"])))

    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=hunav["worldgen"],
            on_exit=[
                LogInfo(msg="Worldgen finished. Launching Gazebo (generated world)."),
                gazebo_group(paths["generated_world_path"], cfg),
            ],
        )
    ))

    # Humans disabled: start gazebo immediately with base world
    ld.add_action(
        GroupAction(
            [gazebo_group(paths["base_world_path"], cfg)],
            condition=UnlessCondition(cfg["use_humans"]),
        )
    )

    # 5) spawn robot after gzserver starts (simple start-hook)
    # In practice you’d keep gzserver handle; if you need reliability, trigger on service availability.
    # Here’s the typical pattern when you *do* have a gzserver action handle:
    # ld.add_action(RegisterEventHandler(OnProcessStart(target_action=gzserver, on_start=[robot_spawn_group(cfg)])))

    # If you can’t easily hook it, a small delay is a fallback, but readiness is better.

    return ld
