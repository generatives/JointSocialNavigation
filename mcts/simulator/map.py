from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .constants import FREE, WALL


@dataclass(slots=True)
class ScenarioMap:
    grid: np.ndarray
    human_starts: list[tuple[int, int]]
    human_ends: list[tuple[int, int]]
    robot_start: tuple[int, int]
    robot_goals: list[tuple[int, int]]

    @staticmethod
    def build_default() -> "ScenarioMap":
        height, width = 9, 18
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL
        grid[3:6, :] = FREE
        grid[:, 10:13] = FREE

        human_starts = [(17, 3), (0, 3)]
        human_ends = [(10, 0), (10, 8)]
        robot_start = (0, 5)
        robot_goals = []
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @property
    def shape(self) -> tuple[int, int]:
        return self.grid.shape

    @property
    def width(self) -> int:
        return self.grid.shape[1]

    @property
    def height(self) -> int:
        return self.grid.shape[0]

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: tuple[int, int]) -> bool:
        x, y = cell
        return self.in_bounds(cell) and self.grid[y, x] == FREE
    
    def position_is_free(self, position: np.ndarray) -> bool:
        cell = self.world_to_cell(position)
        return self.is_free(cell)

    def world_to_cell(self, position: np.ndarray) -> tuple[int, int]:
        x = int(math.floor(float(position[0])))
        y = int(math.floor(float(position[1])))
        x = int(np.clip(x, 0, self.width - 1))
        y = int(np.clip(y, 0, self.height - 1))
        return x, y

    def nearest_free(self, cell: tuple[int, int], max_radius: int = 4) -> tuple[int, int] | None:
        if self.is_free(cell):
            return cell
        for radius in range(1, max_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if self.is_free(candidate):
                        return candidate
        return None

    def cell_to_world(self, cell: tuple[int, int]) -> np.ndarray:
        return np.array([float(cell[0]) + 0.5, float(cell[1]) + 0.5], dtype=np.float32)
