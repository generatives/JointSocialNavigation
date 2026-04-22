from __future__ import annotations

import math

import numpy as np

from .constants import WALL
from .scenario_map import ScenarioMap


# Temporary fix:
# Add extra bounds to the map so that
# when an agent is out of bounds, MCTS continues planning
WORLD_BOUNDS_MARGIN_METERS = 5.0
def _world_bounds(scenario: ScenarioMap) -> tuple[float, float, float, float]:
    min_x = float(scenario.origin_x) - WORLD_BOUNDS_MARGIN_METERS
    min_y = float(scenario.origin_y) - WORLD_BOUNDS_MARGIN_METERS
    max_x = float(scenario.origin_x) + float(scenario.width) * float(scenario.resolution) + WORLD_BOUNDS_MARGIN_METERS
    max_y = float(scenario.origin_y) + float(scenario.height) * float(scenario.resolution) + WORLD_BOUNDS_MARGIN_METERS
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
    # Use the precomputed distance to wall 
    return bool(scenario.wall_distance_field[cy, cx] <= radius)

