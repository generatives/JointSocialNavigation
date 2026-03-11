from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False

# Default config path: <repo_root>/config/simulator.yaml
# ui.py is at  src/social_navigation/social_navigation/simulator/ui.py
# parents[4]  = <repo_root>
_DEFAULT_CONFIG_PATH = Path(__file__).parents[4] / "config" / "simulator.yaml"


@dataclass
class SimConfig:
    # Map
    map: str = "hallway_crossing"
    # Robot
    robot_theta: float = 0.0
    # Crowd
    max_humans: int = 1
    max_spawns: int | None = 1
    spawn_rate_per_sec: float = 0.2
    pref_speed_min: float = 1.0
    pref_speed_max: float = 1.7
    # Control
    default_mode: str = "ROBOT_AI"
    # Joint A*
    robot_move_penalty: float = 0.05
    human_detour_penalty: float = 0.5
    blocking_penalty: float = 2.0
    proximity_penalty: float = 3.5
    proximity_threshold: int = 4
    replan_period: float = 0.25
    # Display
    cell_px: int = 72
    warmup_seconds: float = 6.0


def load_config(path: str | Path | None = None) -> SimConfig:
    """Load a SimConfig from a YAML file.

    Falls back to built-in defaults for any key not present in the file.
    If *path* is None the default location (<repo_root>/config/simulator.yaml)
    is tried; missing file is silently ignored.
    """
    search = Path(path) if path is not None else _DEFAULT_CONFIG_PATH

    data: dict[str, Any] = {}
    if _YAML_AVAILABLE and search.exists():
        with search.open() as f:
            data = yaml.safe_load(f) or {}

    def get(section: str, key: str, default: Any) -> Any:
        return data.get(section, {}).get(key, default)

    max_spawns_raw = get("crowd", "max_spawns", 1)
    max_spawns = None if max_spawns_raw is None else int(max_spawns_raw)

    return SimConfig(
        map=data.get("map", "hallway_crossing"),
        robot_theta=get("robot", "theta", 0.0),
        max_humans=get("crowd", "max_humans", 1),
        max_spawns=max_spawns,
        spawn_rate_per_sec=get("crowd", "spawn_rate_per_sec", 0.2),
        pref_speed_min=get("crowd", "pref_speed_min", 1.0),
        pref_speed_max=get("crowd", "pref_speed_max", 1.7),
        default_mode=get("control", "default_mode", "ROBOT_AI"),
        robot_move_penalty=get("joint_astar", "robot_move_penalty", 0.05),
        human_detour_penalty=get("joint_astar", "human_detour_penalty", 0.5),
        blocking_penalty=get("joint_astar", "blocking_penalty", 2.0),
        proximity_penalty=get("joint_astar", "proximity_penalty", 3.5),
        proximity_threshold=get("joint_astar", "proximity_threshold", 4),
        replan_period=get("joint_astar", "replan_period", 0.25),
        cell_px=get("display", "cell_px", 72),
        warmup_seconds=get("display", "warmup_seconds", 6.0),
    )
