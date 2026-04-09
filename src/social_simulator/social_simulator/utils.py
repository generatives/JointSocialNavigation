import yaml

def load_scenario_parameters(scenario_file: str):
    with open(scenario_file, "r", encoding="utf-8") as stream:
        yaml_file = yaml.safe_load(stream) or {}
    return yaml_file.get("hunav_loader", {}).get("ros__parameters", {})

def load_robot_config(scenario_file: str) -> dict:
    scenario_params = load_scenario_parameters(scenario_file)
    robot_cfg = scenario_params["robot"]

    init_cfg = robot_cfg["init_pose"]
    init_pose = {
        "x": float(init_cfg["x"]),
        "y": float(init_cfg["y"]),
        "yaw": float(init_cfg["yaw"]),
    }

    goal_cfg = robot_cfg["goal"]
    goal = {
        "x": float(goal_cfg["x"]),
        "y": float(goal_cfg["y"]),
    }

    return {
        "init_pose": init_pose,
        "goal": goal,
    }
