import math
from typing import Any

import yaml


def _coerce_float(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    return float(value)


def _coerce_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)


def load_scenario_parameters(scenario_file: str) -> dict[str, Any]:
    with open(scenario_file, "r", encoding="utf-8") as stream:
        yaml_file = yaml.safe_load(stream) or {}
    return yaml_file.get("hunav_loader", {}).get("ros__parameters", {})


def _resolve_facing_target(
    pose_cfg: dict[str, Any], scenario_params: dict[str, Any]
) -> tuple[float, float] | None:
    if "face_agent" in pose_cfg:
        agent_name = pose_cfg["face_agent"]
        agent_cfg = scenario_params.get(agent_name)
        if not isinstance(agent_cfg, dict) or "init_pose" not in agent_cfg:
            raise ValueError(
                f"Scenario references face_agent '{agent_name}', but that agent has no init_pose."
            )
        agent_pose = agent_cfg["init_pose"]
        return (
            _coerce_float(agent_pose.get("x")),
            _coerce_float(agent_pose.get("y")),
        )

    face_point = pose_cfg.get("face_point")
    if isinstance(face_point, dict):
        return (
            _coerce_float(face_point.get("x")),
            _coerce_float(face_point.get("y")),
        )

    return None


def resolve_pose(
    pose_cfg: dict[str, Any] | None, scenario_params: dict[str, Any]
) -> dict[str, float] | None:
    if not isinstance(pose_cfg, dict):
        return None

    x = _coerce_float(pose_cfg.get("x"))
    y = _coerce_float(pose_cfg.get("y"))
    z = _coerce_float(pose_cfg.get("z"))

    if "h" in pose_cfg and pose_cfg.get("h") is not None:
        h = _coerce_float(pose_cfg.get("h"))
    else:
        target = _resolve_facing_target(pose_cfg, scenario_params)
        h = math.atan2(target[1] - y, target[0] - x) if target else 0.0

    return {"x": x, "y": y, "z": z, "h": h}


def load_robot_config(scenario_file: str) -> dict[str, Any]:
    scenario_params = load_scenario_parameters(scenario_file)
    robot_cfg = scenario_params.get("robot")
    if not isinstance(robot_cfg, dict):
        return {}

    init_pose = resolve_pose(robot_cfg.get("init_pose"), scenario_params)
    goal = resolve_pose(robot_cfg.get("goal"), scenario_params)

    return {
        "init_pose": init_pose,
        "goal": goal,
        "publish_initialpose": _coerce_bool(
            robot_cfg.get("publish_initialpose"), init_pose is not None
        ),
        "publish_goal": _coerce_bool(robot_cfg.get("publish_goal"), goal is not None),
        "initialpose_delay_sec": _coerce_float(
            robot_cfg.get("initialpose_delay_sec"), 5.0
        ),
        "goal_delay_sec": _coerce_float(robot_cfg.get("goal_delay_sec"), 8.0),
    }
