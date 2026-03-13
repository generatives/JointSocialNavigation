from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False

# Default config path: src/social_navigation/config/simulator.yaml
# This file is at:    src/social_navigation/social_navigation/simulator/config.py
# parents[0] = simulator/
# parents[1] = social_navigation/  (inner package)
# parents[2] = social_navigation/  (outer package)
# parents[3] = src/
_DEFAULT_CONFIG_PATH = Path(__file__).parents[2] / "config" / "simulator.yaml"


@dataclass
class SimConfig:
    # Map
    map: str = "hallway_crossing"
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
    # Belief
    belief_sigma: float = 0.5
    belief_yield_threshold: float = 0.5
    belief_lookahead: float = 1.5
    belief_discount_blocking: float = 0.5
    awareness_sigma: float = 0.6
    # SFM
    human_human_amplitude: float = 6.0
    human_human_decay: float = 0.7
    robot_human_amplitude: float = 10.0
    robot_human_decay: float = 0.6
    obstacle_amplitude: float = 1.0
    obstacle_decay: float = 0.7
    # Display
    cell_px: int = 72
    warmup_seconds: float = 6.0


def load_config(path: str | Path | None = None) -> SimConfig:
    """Load a SimConfig from a YAML file.

    Falls back to built-in defaults for any key not present in the file.
    If *path* is None the default location is tried; a missing file is
    silently ignored and all defaults are used.
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
        belief_sigma=get("belief", "belief_sigma", 0.5),
        belief_yield_threshold=get("belief", "belief_yield_threshold", 0.5),
        belief_lookahead=get("belief", "belief_lookahead", 1.5),
        belief_discount_blocking=get("belief", "belief_discount_blocking", 0.5),
        awareness_sigma=get("belief", "awareness_sigma", 0.6),
        human_human_amplitude=get("sfm", "human_human_amplitude", 6.0),
        human_human_decay=get("sfm", "human_human_decay", 0.7),
        robot_human_amplitude=get("sfm", "robot_human_amplitude", 10.0),
        robot_human_decay=get("sfm", "robot_human_decay", 0.6),
        obstacle_amplitude=get("sfm", "obstacle_amplitude", 1.0),
        obstacle_decay=get("sfm", "obstacle_decay", 0.7),
        cell_px=get("display", "cell_px", 72),
        warmup_seconds=get("display", "warmup_seconds", 6.0),
    )
