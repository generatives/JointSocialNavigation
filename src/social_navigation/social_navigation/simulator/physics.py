from __future__ import annotations

import math

import numpy as np

from .constants import WALL
from .scenario_map import ScenarioMap


def _world_bounds(scenario: ScenarioMap) -> tuple[float, float, float, float]:
    min_x = float(scenario.origin_x)
    min_y = float(scenario.origin_y)
    max_x = min_x + float(scenario.width) * float(scenario.resolution)
    max_y = min_y + float(scenario.height) * float(scenario.resolution)
    return min_x, min_y, max_x, max_y


def _cell_index(position: np.ndarray, scenario: ScenarioMap) -> tuple[int, int]:
    resolution = float(scenario.resolution)
    cx = int(math.floor((float(position[0]) - float(scenario.origin_x)) / resolution))
    cy = int(math.floor((float(position[1]) - float(scenario.origin_y)) / resolution))
    cx = max(0, min(scenario.width - 1, cx))
    cy = max(0, min(scenario.height - 1, cy))
    return cx, cy


def collides_with_walls(position: np.ndarray, radius: float, scenario: ScenarioMap) -> bool:
    min_x, min_y, max_x, max_y = _world_bounds(scenario)
    if position[0] < min_x + radius or position[1] < min_y + radius:
        return True
    if position[0] > max_x - radius or position[1] > max_y - radius:
        return True

    cx, cy = _cell_index(position, scenario)
    if scenario.grid[cy, cx] == WALL:
        return True
    return bool(scenario.wall_distance_field[cy, cx] <= radius)


def distance_to_nearest_wall(
    position: np.ndarray,
    scenario: ScenarioMap,
    max_search_distance: float | None = None,
) -> float:
    min_x, min_y, max_x, max_y = _world_bounds(scenario)
    boundary_clearance = min(
        float(position[0]) - min_x,
        max_x - float(position[0]),
        float(position[1]) - min_y,
        max_y - float(position[1]),
    )

    cx, cy = _cell_index(position, scenario)
    interior_clearance = float(scenario.wall_distance_field[cy, cx])
    return max(0.0, min(boundary_clearance, interior_clearance))
