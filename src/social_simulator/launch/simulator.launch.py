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

from hunavis.utils import goal_from_params

def add_people_visualizer(context, social_simulator_dir, scenario_file, *args, **kwargs):
    scenario_params_file_val = scenario_file.perform(context)
    scenario_params_file_path = os.path.join(social_simulator_dir, "scenarios", scenario_params_file_val)

    humans_goals_str = goal_from_params(scenario_params_file_path)
    people_visualizer_node = Node(
        package="hunavis",
        executable="people_visualizer",
        parameters=[
            {"use_simulator": True},
            {"goals": humans_goals_str},
        ],
    )
    return [people_visualizer_node]

def add_rviz(context, social_simulator_dir, run_rviz, rviz_file, *args, **kwargs):
    rviz_file_val = rviz_file.perform(context)
    
    rviz_file_val_path = os.path.join(social_simulator_dir, "rviz", rviz_file_val)
    # rviz_file_val_path = "/workspaces/JointSocialNavigation/src/social_simulator/rviz/default_sim_view.rviz"
    print("#################### PATH: ", rviz_file_val_path)
    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file_val_path],
        output={"both": "log"},
    )

    return [rviz_node]

def add_map_server(context, world_dir_share, *args, **kwargs):
    map_file = LaunchConfiguration("map_path").perform(context)
    map_path_file = os.path.join(world_dir_share, "maps", map_file)

    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_path_file, "use_sim_time": True}],
    )

    lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    return [map_server_cmd, lifecycle_manager_cmd]

def generate_launch_description():
    world_dir = FindPackageShare("social_simulator")

    world_file = LaunchConfiguration("world")
    scenario_file = LaunchConfiguration("scenario")
    use_humans = LaunchConfiguration("use_humans")
    headless = LaunchConfiguration("headless")
    run_rviz = LaunchConfiguration("run_rviz")
    rviz_file = LaunchConfiguration("rviz_file")

    base_world_path = PathJoinSubstitution([world_dir, "worlds", world_file])
    generated_world_path = PathJoinSubstitution([world_dir, "worlds", "generatedWorld.world"])
    scenario_path = PathJoinSubstitution([FindPackageShare("social_simulator"), "scenarios", scenario_file])

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
    ld.add_action(DeclareLaunchArgument(
                "run_rviz",
                default_value="True",
                description="Whether to use rviz.",
                choices=["True", "False"]))
    ld.add_action(DeclareLaunchArgument(
                "rviz_file",
                default_value="default_test_view.rviz",
                description=("File containing rviz settings.")))

    my_gazebo_models = PathJoinSubstitution([FindPackageShare("hunav_gazebo_wrapper"), "models"])
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

    # social_simulator_dir_str = get_package_share_directory("social_simulator")
    # ld.add_action(OpaqueFunction(function=lambda ctx: add_map_server(ctx, social_simulator_dir_str)))

    # # Load list of human goals from the simulation parameters
    # ld.add_action(OpaqueFunction(function=lambda ctx: add_people_visualizer(ctx, social_simulator_dir_str, scenario_file)))
    # # ld.add_action(OpaqueFunction(function=lambda ctx: add_rviz(ctx, social_simulator_dir_str, run_rviz, rviz_file)))
    # ld.add_action(RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=gzserver,
    #         on_start=[
    #             TimerAction(
    #                 period=6.0,
    #                 actions=[
    #                     OpaqueFunction(
    #                         function=lambda ctx: add_rviz(ctx, social_simulator_dir_str, run_rviz, rviz_file)
    #                     )
    #                 ],
    #             )
    #         ],
    #     )
    # ))

    return ld