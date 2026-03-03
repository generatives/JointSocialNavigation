import os
from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessIO,
    OnProcessStart,
    OnShutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from scripts import GazeboRosPaths

def generate_launch_description():
    # World generation parameters
    world_file_name = LaunchConfiguration("base_world")
    gz_obs = LaunchConfiguration("use_gazebo_obs")
    rate = LaunchConfiguration("update_rate")
    robot_name = LaunchConfiguration("robot_name")
    global_frame = LaunchConfiguration("global_frame_to_publish")
    use_navgoal = LaunchConfiguration("use_navgoal_to_start")
    ignore_models = LaunchConfiguration("ignore_models")

    # agent configuration file
    params_file = PathJoinSubstitution(
        [
            FindPackageShare("hunavis"),
            "params",
            LaunchConfiguration("configuration_file"),
        ]
    )

    # Read the yaml file and load the parameters
    hunav_loader_node = Node(
        package="hunav_agent_manager",
        executable="hunav_loader",
        output="screen",
        parameters=[params_file],
    )

    # world base file
    # NOTE: hunav_gazebo_wrapper will generate a new world file
    #       in FindPackageShare(WORLD_DIR)/'worlds',
    #       ensure to change world_path
    world_file = PathJoinSubstitution(
        [FindPackageShare(WORLD_DIR), "worlds", world_file_name]
    )

    # the node looks for the base_world file in the directory 'worlds'
    # of the package hunav_gazebo_plugin direclty. So we do not need to
    # indicate the path
    hunav_gazebo_worldgen_node = Node(
        package="hunav_gazebo_wrapper",
        executable="hunav_gazebo_world_generator",
        output="screen",
        parameters=[
            {"base_world": world_file},
            {"use_gazebo_obs": gz_obs},
            {"update_rate": rate},
            {"robot_name": robot_name},
            {"global_frame_to_publish": global_frame},
            {"use_navgoal_to_start": use_navgoal},
            {"ignore_models": ignore_models},
        ],
        # arguments=['--ros-args', '--params-file', conf_file]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(
                    msg="HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds..."
                ),
                TimerAction(
                    period=2.0,
                    actions=[hunav_gazebo_worldgen_node],
                ),
            ],
        )
    )

    # Then, launch the generated world in Gazebo
    my_gazebo_models = PathJoinSubstitution(
        [
            FindPackageShare("hunav_gazebo_wrapper"),
            "models",
        ]
    )

    set_env_gazebo_resource = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=[EnvironmentVariable("GAZEBO_RESOURCE_PATH"), my_gazebo_models],
    )

    # the world generator will create this world
    # in this path
    world_path = PathJoinSubstitution(
        [FindPackageShare(WORLD_DIR), "worlds", "generatedWorld.world"]
    )

    # Gazebo server
    gzserver_cmd = [
        "gzserver ",
        #'--pause ',
        world_path,
        _boolean_command("verbose"),
        "",
        "-s ",
        "libgazebo_ros_init.so",
        "-s ",
        "libgazebo_ros_factory.so",
        "--ros-args",
        "--params-file",
        params_file,
    ]

    # Gazebo client
    gzclient_cmd = [
        "gzclient",
        _boolean_command("verbose"),
        " ",
    ]

    gzserver_process = ExecuteProcess(
        cmd=gzserver_cmd,
        output="screen",
        shell=True,
        on_exit=Shutdown(),
    )

    gzclient_process = ExecuteProcess(
        cmd=gzclient_cmd,
        output="screen",
        shell=True,
        on_exit=Shutdown(),
    )

    # Do not launch Gazebo until the world has been generated
    ordered_launch_event2 = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(
                    msg="GenerateWorld started, launching Gazebo after 2 seconds..."
                ),
                TimerAction(
                    period=2.0,
                    actions=[gzserver_process, gzclient_process],
                ),
            ],
        )
    )

    # hunav_manager node
    hunav_manager_node = Node(
        package="hunav_agent_manager",
        executable="hunav_agent_manager",
        name="hunav_agent_manager",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    metrics_file = PathJoinSubstitution(
        [
            FindPackageShare("hunav_evaluator"),
            "config",
            LaunchConfiguration("metrics_file"),
        ]
    )

    hunav_evaluator_node = Node(
        package="hunav_evaluator",
        executable="hunav_evaluator_node",
        output="screen",
        parameters=[metrics_file],
    )

    # DO NOT Launch this if any robot localization is launched
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        # other option: arguments = "0 0 0 0 0 0 pmb2 base_footprint".split(' ')
    )

    declare_agents_conf_file = DeclareLaunchArgument(
        "configuration_file",
        default_value="/workspaces/JointSocialNavigation/src/simulator/scenarios/agents_doors_hallway.yaml",
        description="Specify configuration file name in the config directory",
    )

    declare_metrics_conf_file = DeclareLaunchArgument(
        "metrics_file",
        default_value="metrics.yaml",
        description="Specify the name of the metrics configuration file in the config directory",
    )

    declare_arg_world = DeclareLaunchArgument(
        "base_world",
        default_value="empty_room.world",
        description="Specify world file name",
    )
    declare_gz_obs = DeclareLaunchArgument(
        "use_gazebo_obs",
        default_value="true",
        description="Whether to fill the agents obstacles with closest Gazebo obstacle or not",
    )
    declare_update_rate = DeclareLaunchArgument(
        "update_rate", default_value="100.0", description="Update rate of the plugin"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="turtlebot3_waffle",
        description="Specify the name of the robot Gazebo model",
    )
    declare_frame_to_publish = DeclareLaunchArgument(
        "global_frame_to_publish",
        default_value="map",
        description="Name of the global frame in which the position of the agents are provided",
    )
    declare_use_navgoal = DeclareLaunchArgument(
        "use_navgoal_to_start",
        default_value="false",
        description="Whether to start the agents movements when a navigation goal is received or not",
    )
    declare_ignore_models = DeclareLaunchArgument(
        "ignore_models",
        default_value="ground_plane",
        description="list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them",
    )
    declare_arg_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="true",
        description='Set "true" to increase messages written to terminal.',
    )

    ld = LaunchDescription()

    # set environment variables
    ld.add_action(set_env_gazebo_resource)

    # Declare the launch arguments
    ld.add_action(declare_agents_conf_file)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(declare_arg_world)
    ld.add_action(declare_gz_obs)
    ld.add_action(declare_update_rate)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_ignore_models)
    ld.add_action(declare_arg_verbose)

    # Generate the world with the agents
    # launch hunav_loader and the WorldGenerator
    # 2 seconds later
    ld.add_action(hunav_loader_node)
    ld.add_action(ordered_launch_event)

    # hunav behavior manager node
    ld.add_action(hunav_manager_node)
    # hunav evaluator
    ld.add_action(hunav_evaluator_node)

    ld.add_action(ordered_launch_event2)

    bringup_dir = get_package_share_directory("nav2_bringup")
    robot_sdf = LaunchConfiguration("robot_sdf")
    pose = {
        "x": LaunchConfiguration("x_pose", default="0.0"),
        "y": LaunchConfiguration("y_pose", default="0.0"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.0"),
        "P": LaunchConfiguration("pitch", default="0.0"),
        "Y": LaunchConfiguration("yaw", default="0.0"),
    }
    namespace = LaunchConfiguration("namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(bringup_dir, "worlds", "waffle.model"),
        description="Full path to robot sdf file to spawn the robot in gazebo",
    )

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    start_gazebo_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            robot_name,
            "-file",
            robot_sdf,
            "-robot_namespace",
            namespace,
            "-x",
            pose["x"],
            "-y",
            pose["y"],
            "-z",
            pose["z"],
            "-R",
            pose["R"],
            "-P",
            pose["P"],
            "-Y",
            pose["Y"],
        ],
    )

    # spawn robot in Gazebo
    ld.add_action(start_gazebo_spawner_cmd)

    return ld


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
