import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from social_simulator.utils import load_robot_config, load_scenario_parameters

def launch_setup(context, *args, **kwargs):
    social_simulator_dir = get_package_share_directory("social_simulator")
    launch_dir = os.path.join(social_simulator_dir, "launch")

    scenario = LaunchConfiguration("scenario").perform(context).strip()
    scenario_path = os.path.join(social_simulator_dir, "scenarios", scenario)

    scenario_params = load_scenario_parameters(scenario_path)
    robot_cfg = load_robot_config(scenario_path)
    init_pose = robot_cfg.get("init_pose") or {}

    world = f"{scenario_params.get('world')}.world"
    map_file = f"{scenario_params.get('map')}.yaml" 

    x_pose = str(init_pose.get("x", 0.0))
    y_pose = str(init_pose.get("y", 0.0))
    yaw = str(init_pose.get("yaw", 0.0))

    headless = LaunchConfiguration("headless").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_sdf = LaunchConfiguration("robot_sdf").perform(context)
    use_humans = LaunchConfiguration("use_humans").perform(context)
    use_gazebo_obs = LaunchConfiguration("use_gazebo_obs").perform(context)
    update_rate = LaunchConfiguration("update_rate").perform(context)
    global_frame_to_publish = LaunchConfiguration("global_frame_to_publish").perform(
        context
    )
    use_navgoal_to_start = LaunchConfiguration("use_navgoal_to_start").perform(context)
    navgoal_topic = LaunchConfiguration("navgoal_topic").perform(context)
    ignore_models = LaunchConfiguration("ignore_models").perform(context)
    use_namespace = LaunchConfiguration("use_namespace").perform(context)
    slam = LaunchConfiguration("slam").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    use_composition = LaunchConfiguration("use_composition").perform(context)
    use_respawn = LaunchConfiguration("use_respawn").perform(context)
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub").perform(context)
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)

    hunavsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "hunavsim.launch.py")),
        launch_arguments={
            "world": world,
            "scenario": scenario,
            "use_humans": use_humans,
            "headless": headless,
            "use_gazebo_obs": use_gazebo_obs,
            "update_rate": update_rate,
            "robot_name": robot_name,
            "global_frame_to_publish": global_frame_to_publish,
            "use_navgoal_to_start": use_navgoal_to_start,
            "navgoal_topic": navgoal_topic,
            "ignore_models": ignore_models,
            "robot_sdf": robot_sdf,
            "namespace": namespace,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "yaw" : yaw,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "tb3_custom_sim.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "map": map_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "use_robot_state_pub": use_robot_state_pub,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "yaw" : yaw,
        }.items(),
    )

    hudet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "hudet.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_simulator": "True",
            "scenario_params_file": scenario,
            "run_rviz": use_rviz,
            "rviz_file": rviz_config_file,
        }.items(),
    )

    return [
        hunavsim_launch,
        nav2_launch,
        hudet_launch,
    ]


def generate_launch_description():
    social_simulator_dir = get_package_share_directory("social_simulator")
    bringup_dir = get_package_share_directory("nav2_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="agents_doors_hallway.yaml",
                description="Scenario file in the social_simulator/scenarios directory.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="False",
                description="Whether to skip the Gazebo client.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="True",
                description="Whether to launch RViz through hudet.",
            ),
            DeclareLaunchArgument(
                "use_humans",
                default_value="true",
                description="Whether to spawn the human agents from the scenario.",
            ),
            DeclareLaunchArgument(
                "use_gazebo_obs",
                default_value="true",
                description="Whether to expose Gazebo obstacles through the wrapper.",
            ),
            DeclareLaunchArgument(
                "update_rate",
                default_value="100.0",
                description="World generator update rate.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="turtlebot3_waffle",
                description="Gazebo entity name for the robot.",
            ),
            DeclareLaunchArgument(
                "robot_sdf",
                default_value=os.path.join(bringup_dir, "worlds", "waffle.model"),
                description="Robot model used when spawning the robot.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Top-level namespace.",
            ),
            DeclareLaunchArgument(
                "global_frame_to_publish",
                default_value="map",
                description="Global frame used by the human world generator.",
            ),
            DeclareLaunchArgument(
                "use_navgoal_to_start",
                default_value="false",
                description="Whether to delay humans until a nav goal is sent.",
            ),
            DeclareLaunchArgument(
                "navgoal_topic",
                default_value="/goal_pose",
                description="Whether to delay humans until a nav goal is sent.",
            ),
            DeclareLaunchArgument(
                "ignore_models",
                default_value="ground_plane",
                description="Gazebo models ignored by the human wrapper.",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="False",
                description="Whether to run SLAM instead of localization.",
            ),
            DeclareLaunchArgument(
                "use_namespace",
                default_value="false",
                description="Whether to apply the namespace to the navigation stack.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Whether to use simulated time.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    social_simulator_dir, "params", "tb3_custom_sim.yaml"
                ),
                description="Nav2 params file.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Whether to autostart the Nav2 stack.",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Whether to use composed Nav2 bringup.",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="False",
                description="Whether to respawn Nav2 nodes when composition is disabled.",
            ),
            DeclareLaunchArgument(
                "use_robot_state_pub",
                default_value="True",
                description="Whether to start robot_state_publisher.",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(
                    social_simulator_dir, "rviz", "default_sim_view.rviz"
                ),
                description="RViz config file to use when use_rviz is true.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
