from __future__ import annotations

import math

import numpy as np

from .constants import WALL
from .map import ScenarioMap


def collides_with_walls(position: np.ndarray, radius: float, scenario: ScenarioMap) -> bool:
    if position[0] < radius or position[1] < radius:
        return True
    if position[0] > scenario.width - radius or position[1] > scenario.height - radius:
        return True

    min_x = int(math.floor(position[0] - radius))
    max_x = int(math.ceil(position[0] + radius))
    min_y = int(math.floor(position[1] - radius))
    max_y = int(math.ceil(position[1] + radius))
    for y in range(min_y, max_y + 1):
        for x in range(min_x, max_x + 1):
            if x < 0 or y < 0 or x >= scenario.width or y >= scenario.height:
                continue
            if scenario.grid[y, x] != WALL:
                continue
            near_x = max(float(x), min(float(position[0]), float(x + 1)))
            near_y = max(float(y), min(float(position[1]), float(y + 1)))
            dx = float(position[0]) - near_x
            dy = float(position[1]) - near_y
            if dx * dx + dy * dy <= radius * radius:
                return True
    return False
