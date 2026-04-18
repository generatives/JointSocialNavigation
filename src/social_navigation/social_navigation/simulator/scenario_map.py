from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .constants import FREE, WALL

def _edt_squared_1d(f: np.ndarray) -> np.ndarray:
    """Felzenszwalb-Huttenlocher 1D squared Euclidean distance transform.
    Input: 1D array of parabola heights (0 at sources, +inf elsewhere).
    Output: squared distance from each index to the nearest source."""
    n = f.shape[0]

    first_finite = -1
    for i in range(n):
        if np.isfinite(f[i]):
            first_finite = i
            break
    if first_finite == -1:
        return np.full(n, np.inf, dtype=np.float64)

    v = np.empty(n, dtype=np.int64)
    z = np.empty(n + 1, dtype=np.float64)
    v[0] = first_finite
    z[0] = -np.inf
    z[1] = np.inf
    k = 0

    for q in range(first_finite + 1, n):
        fq = f[q]
        if not np.isfinite(fq):
            continue
        while True:
            vk = v[k]
            s = ((fq + q * q) - (f[vk] + vk * vk)) / (2.0 * (q - vk))
            if s > z[k]:
                k += 1
                v[k] = q
                z[k] = s
                z[k + 1] = np.inf
                break
            if k == 0:
                v[0] = q
                z[1] = np.inf
                break
            k -= 1

    d = np.empty(n, dtype=np.float64)
    j = 0
    for q in range(n):
        while z[j + 1] < q:
            j += 1
        vj = v[j]
        d[q] = (q - vj) * (q - vj) + f[vj]
    return d


def _edt_squared_2d(wall_mask: np.ndarray) -> np.ndarray:
    """Separable 2D squared EDT: 0 at wall cells, squared distance elsewhere."""
    h, w = wall_mask.shape
    f = np.where(wall_mask, 0.0, np.inf)
    # column pass
    for x in range(w):
        f[:, x] = _edt_squared_1d(f[:, x])
    # row pass
    for y in range(h):
        f[y, :] = _edt_squared_1d(f[y, :])
    return f


def inflate_grid(grid, cells):
    cells = max(0, int(cells))
    if cells == 0:
        return grid.copy()

    inflated_grid = grid.copy()
    occupied_rows, occupied_cols = np.nonzero(grid == WALL)
    for row, col in zip(occupied_rows, occupied_cols):
        row_start = max(0, row - cells)
        row_end = min(grid.shape[0], row + cells + 1)
        col_start = max(0, col - cells)
        col_end = min(grid.shape[1], col + cells + 1)
        inflated_grid[row_start:row_end, col_start:col_end] = WALL

    return inflated_grid

@dataclass(slots=True)
class ScenarioMap:
    grid: np.ndarray
    human_starts: list[tuple[int, int]]
    human_ends: list[tuple[int, int]]
    robot_start: tuple[int, int]
    robot_goals: list[tuple[int, int]]
    resolution: float = 1.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    _wall_distance_field: np.ndarray | None = None

    @property
    def wall_distance_field(self) -> np.ndarray:
        """Per-cell distance (meters) from the cell centre to the nearest
        wall-cell edge. Computed lazily via a separable Euclidean distance
        transform so wall proximity queries during MCTS become O(1) lookups
        instead of O(radius²) cell scans."""
        if self._wall_distance_field is None:
            wall_mask = self.grid == WALL
            if not np.any(wall_mask):
                edt_cells = np.full(self.grid.shape, np.inf, dtype=np.float64)
            else:
                edt_cells = _edt_squared_2d(wall_mask)
                edt_cells = np.sqrt(edt_cells)

            field = np.maximum(edt_cells * float(self.resolution) - 0.5 * float(self.resolution), 0.0)
            self._wall_distance_field = field.astype(np.float32)
        return self._wall_distance_field

    @staticmethod
    def build_empty() -> "ScenarioMap":
        height, width = 20, 20
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = FREE

        human_starts = []
        human_ends = []
        robot_start = (0, 0)
        robot_goals = []
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @staticmethod
    def build_default() -> "ScenarioMap":
        height, width = 18, 36
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL
        grid[6:12, :] = FREE
        grid[:, 20:26] = FREE

        human_starts = [(35, 6), (0, 6)]
        human_ends = [(20, 0), (20, 17)]
        robot_start = (0, 10)
        robot_goals = []
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @staticmethod
    def build_hallway_crossing() -> "ScenarioMap":
        height, width = 18, 18
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL
        grid[6:12, :] = FREE
        grid[:, 6:12] = FREE

        human_starts = [(8, 0)]
        human_ends = [(8, 17)]
        robot_start = (0, 8)
        robot_goals = []
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)
    
    @staticmethod
    def build_hallway_collision() -> "ScenarioMap":
        height, width = 18, 18
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL
        grid[8:10, :] = FREE

        human_starts = [(17, 8)]
        human_ends = [(0, 8)]
        robot_start = (0, 8)
        robot_goals = []
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)
    
    @staticmethod
    def build_hallway_tradeoff() -> "ScenarioMap":
        height, width = 18, 18
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL
        grid[6:14, :] = FREE

        human_starts = [(11, 8), (11, 11)]
        human_ends = [(0, 8)]
        robot_start = (0, 8)
        robot_goals = [(16, 10), (0, 8)]
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @staticmethod
    def build_from_occupancy_data(
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: list[int] | np.ndarray,
        occupied_threshold: int = 50,
        treat_unknown_as_wall: bool = True,
    ) -> "ScenarioMap":
        occupancy = np.asarray(data, dtype=np.int16).reshape(height, width)
        blocked = occupancy >= occupied_threshold
        if treat_unknown_as_wall:
            blocked |= occupancy < 0

        grid = np.where(blocked, WALL, FREE).astype(np.int8)
        human_starts: list[tuple[int, int]] = []
        human_ends: list[tuple[int, int]] = []
        robot_start = (0, 0)
        robot_goals: list[tuple[int, int]] = []
        return ScenarioMap(
            grid=grid,
            human_starts=human_starts,
            human_ends=human_ends,
            robot_start=robot_start,
            robot_goals=robot_goals,
            resolution=float(resolution),
            origin_x=float(origin_x),
            origin_y=float(origin_y),
        )

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
    
    def wall_cells_in_range(self, start: tuple[int, int], end: tuple[int, int]) -> int:
        sx, sy = start
        ex, ey = end

        return np.count_nonzero(self.grid[sy:ey, sx:ex])
    
    def position_is_free(self, position: np.ndarray) -> bool:
        cell = self.world_to_cell(position)
        return self.is_free(cell)

    def world_to_cell(self, position: np.ndarray) -> tuple[int, int]:
        x = int(math.floor((float(position[0]) - self.origin_x) / self.resolution))
        y = int(math.floor((float(position[1]) - self.origin_y) / self.resolution))
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
        return np.array(
            [
                self.origin_x + (float(cell[0]) + 0.5) * self.resolution,
                self.origin_y + (float(cell[1]) + 0.5) * self.resolution,
            ],
            dtype=np.float32,
        )
