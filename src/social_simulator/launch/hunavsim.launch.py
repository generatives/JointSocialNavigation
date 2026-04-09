import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find packages
    social_simulator_pkg = FindPackageShare("social_simulator")
    hunav_gazeob_wrapper_pkg = FindPackageShare("hunav_gazebo_wrapper")

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("world", default_value="doors_hallway.world"))
    ld.add_action(DeclareLaunchArgument("scenario", default_value="agents_doors_hallway.yaml"))
    ld.add_action(DeclareLaunchArgument("map_path", default_value="doors_hallway.yaml", description="Map path to use"))
    ld.add_action(DeclareLaunchArgument("use_humans", default_value="true"))
    ld.add_action(DeclareLaunchArgument("headless", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_gazebo_obs", default_value="true"))
    ld.add_action(DeclareLaunchArgument("update_rate", default_value="100.0"))
    ld.add_action(DeclareLaunchArgument("robot_name", default_value="turtlebot3_waffle"))
    ld.add_action(DeclareLaunchArgument("global_frame_to_publish", default_value="map"))
    ld.add_action(DeclareLaunchArgument("use_navgoal_to_start", default_value="false"))
    ld.add_action(DeclareLaunchArgument("navgoal_topic", default_value="/goal_pose"))
    ld.add_action(DeclareLaunchArgument("ignore_models", default_value="ground_plane"))
    ld.add_action(DeclareLaunchArgument("verbose", default_value="true"))
    ld.add_action(DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(get_package_share_directory("nav2_bringup"), "worlds", "waffle.model"),
    ))
    ld.add_action(DeclareLaunchArgument("namespace", default_value=""))
    ld.add_action(DeclareLaunchArgument("x_pose", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("y_pose", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("z_pose", default_value="0.01"))
    ld.add_action(DeclareLaunchArgument("roll", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("pitch", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("yaw", default_value="0.0"))

    world_file = LaunchConfiguration("world")
    scenario_file = LaunchConfiguration("scenario")
    use_humans = LaunchConfiguration("use_humans")
    headless = LaunchConfiguration("headless")

    base_world_path = PathJoinSubstitution([social_simulator_pkg, "worlds", world_file])
    generated_world_path = PathJoinSubstitution([social_simulator_pkg, "worlds", "generatedWorld.world"])
    scenario_path = PathJoinSubstitution([social_simulator_pkg, "scenarios", scenario_file])
    
    my_gazebo_models = PathJoinSubstitution([hunav_gazeob_wrapper_pkg, "models"])
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=[EnvironmentVariable("GAZEBO_RESOURCE_PATH"), my_gazebo_models],
    ))

    hunav_loader = Node(
        package="hunav_agent_manager",
        executable="hunav_loader",
        output="screen",
        parameters=[scenario_path],
        condition=IfCondition(use_humans),
    )

    hunav_manager = Node(
        package="hunav_agent_manager",
        executable="hunav_agent_manager",
        name="hunav_agent_manager",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_humans),
    )

    worldgen = Node(
        package="hunav_gazebo_wrapper",
        executable="hunav_gazebo_world_generator",
        output="screen",
        parameters=[
            {"base_world": base_world_path},
            {"use_gazebo_obs": LaunchConfiguration("use_gazebo_obs")},
            {"update_rate": LaunchConfiguration("update_rate")},
            {"robot_name": LaunchConfiguration("robot_name")},
            {"global_frame_to_publish": LaunchConfiguration("global_frame_to_publish")},
            {"use_navgoal_to_start": LaunchConfiguration("use_navgoal_to_start")},
            {"navgoal_topic": LaunchConfiguration("navgoal_topic")},
            {"ignore_models": LaunchConfiguration("ignore_models")},
        ],
        condition=IfCondition(use_humans),
    )

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            generated_world_path,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            "--ros-args",
            "--params-file", scenario_path,
        ],
        output="screen",
        on_exit=Shutdown(),
        condition=IfCondition(use_humans),
    )

    gzserver_nohumans = ExecuteProcess(
        cmd=[
            "gzserver",
            base_world_path,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
        on_exit=Shutdown(),
        condition=UnlessCondition(use_humans),
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        # cmd=["vglrun", "-d", os.environ.get("DISPLAY", ":1"), "gzclient"], # CUDA 13.1
        output="screen",
        on_exit=Shutdown(),
        condition=UnlessCondition(headless),
    )

    robot_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", LaunchConfiguration("robot_name"),
            "-file", LaunchConfiguration("robot_sdf"),
            "-robot_namespace", LaunchConfiguration("namespace"),
            "-x", LaunchConfiguration("x_pose"),
            "-y", LaunchConfiguration("y_pose"),
            "-z", LaunchConfiguration("z_pose"),
            "-R", LaunchConfiguration("roll"),
            "-P", LaunchConfiguration("pitch"),
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    ld.add_action(hunav_loader)
    ld.add_action(hunav_manager)

    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader,
            on_start=[TimerAction(period=2.0, actions=[worldgen])],
        )
    ))

    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=worldgen,
            on_start=[TimerAction(period=2.0, actions=[gzserver])],
        )
    ))
    ld.add_action(gzserver_nohumans)

    ld.add_action(gzclient)
    ld.add_action(TimerAction(period=4.0, actions=[robot_spawn]))

    return ld
