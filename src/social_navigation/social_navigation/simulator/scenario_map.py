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
    robot_start: tuple[int, int, float]   # (x, y, theta_radians)
    robot_goals: list[tuple[int, int]]

    @staticmethod
    def build_empty() -> "ScenarioMap":
        height, width = 20, 20
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = FREE

        human_starts = []
        human_ends = []
        robot_start = (0, 0, 0.0)
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
        robot_start = (0, 10, 0.0)
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
        robot_start = (0, 8, 0.0)
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
        robot_start = (0, 8, 0.0)
        robot_goals = []
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @staticmethod
    def build_narrow_hallway() -> "ScenarioMap":
        height, width = 18, 18
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL
        midpoint = height//2
        endpoint = width - 1

        l = midpoint
        r = midpoint+1
        grid[l:r, :] = FREE
        grid[10, 12] = FREE

        human_starts = [(0, midpoint)]
        human_ends = [(endpoint, midpoint)]
        robot_start = (endpoint, midpoint, math.pi)   # faces left toward oncoming human
        robot_goals = []
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @staticmethod
    def build_narrow_hallway2() -> "ScenarioMap":
        height, width = 18, 36
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL
        midpoint = height//2
        endpoint = width - 1

        l = midpoint
        r = midpoint+1
        grid[l:r, :] = FREE
        grid[10:12, 30:32] = FREE
        grid[10:12, 25:27] = FREE

        human_starts = [(0, midpoint)]
        human_ends = [(endpoint, midpoint)]
        robot_start = (endpoint, midpoint, math.pi)   # faces left toward oncoming human
        robot_goals = []
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @staticmethod
    def build_t_junction() -> "ScenarioMap":
        """T-junction: robot climbs the stem and must choose left or right.

        Belief-update demonstration
        ---------------------------
        While the robot travels up the vertical stem its heading direction
        is almost equidistant (≈ ±90°) from both candidate goals, so the
        human's belief stays near 50/50.  The moment the robot exits the
        stem and turns, the heading aligns with one goal and the belief
        snaps instantly (likelihood ratio ~1000:1 from the Gaussian model).

        With anticipatory belief forces the human starts yielding as soon
        as the robot commits to the branch heading toward them — several
        cell-lengths before physical contact.  Without belief forces the
        human only reacts via SFM when the robot is already close.

        Layout (16 × 22 cells)
        ----------------------
          G_left(1,6)────────horizontal bar (y=6..7)────────G_right(20,6)
                                    │
                                  stem
                               (x=10..11, y=7..14)
                                    │
                               Robot start (10,13) ↑

        Robot goals  : G_left=(1,6)  and  G_right=(20,6)
        Human starts : G_left end (1,6)  →  G_right end (20,6)
        """
        height, width = 16, 22
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL

        # Horizontal bar
        grid[6:8, :] = FREE
        # Vertical stem (connects to bar at y=7)
        grid[7:15, 10:12] = FREE

        human_starts = [(1, 6)]
        human_ends = [(20, 6)]
        robot_start = (10, 13, -math.pi / 2)   # bottom of stem, facing up
        robot_goals = [(1, 6), (20, 6)]
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @staticmethod
    def build_belief_corridor() -> "ScenarioMap":
        """Narrow corridor with alcove — belief coordination demo.

        Layout (16 × 40 cells)
        ----------------------
        A long 1-cell-wide hallway spanning the full width with a small
        alcove (2 cells tall × 3 cells long) roughly one-third from the
        human's start.

        Human starts at the left end, robot at the right.  They walk
        head-on toward each other.

        Without belief the human plows forward and relies on reactive SFM
        forces — typically producing an awkward jitter dance.

        With belief the human observes the robot approaching, builds
        confidence about its heading, and preemptively steps into the
        alcove.  The JointAStarRobotAI sees the human's strong belief
        and discounts the blocking penalty, continuing straight rather
        than detouring into the alcove itself.

               alcove
              ┌─────┐
              │ · · ·│   row 7
        ══════╧═════╧══════════════════════════  row 8  (corridor)
        H→                                  ←R
        x=0                                x=39

        Alcove: rows 7, columns 12–14  (corridor widens to 2 cells)
        """
        height, width = 16, 40
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL
        midpoint = 8

        # 1-cell-wide corridor spanning full width
        grid[midpoint, :] = FREE

        # Alcove: widen corridor by 1 row for 3 columns (roughly 1/3 from human start)
        grid[midpoint - 1, 12:15] = FREE

        human_starts = [(0, midpoint)]
        human_ends = [(width - 1, midpoint)]
        robot_start = (width - 1, midpoint, math.pi)  # faces left toward human
        robot_goals = [(0, midpoint), (width - 1, midpoint)]
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @staticmethod
    def build_narrow_hallway3() -> "ScenarioMap":
        height, width = 18, 25
        grid = np.zeros((height, width), dtype=np.int8)
        grid[:, :] = WALL
        midpoint = height//2
        endpoint = width - 1

        l = midpoint
        r = midpoint+1
        grid[l:r, :] = FREE
        grid[10:12, 20:22] = FREE

        human_starts = [(0, midpoint)]
        human_ends = [(endpoint, midpoint)]
        robot_start = (endpoint-10, midpoint, math.pi)   # faces left toward oncoming human
        robot_goals = []
        return ScenarioMap(grid, human_starts, human_ends, robot_start, robot_goals)

    @property
    def robot_goal_candidates(self) -> list[tuple[int, int]]:
        """Candidate destinations a human might infer the robot is heading to.

        Used by the belief system (Bayesian goal inference) and by
        ``_find_yield_cell`` to define which cells are "main corridor."

        If the scenario defines explicit ``robot_goals``, those are used.
        Otherwise falls back to ``human_starts ∪ human_ends`` (the legacy
        heuristic).  The result is always deduplicated.
        """
        if self.robot_goals:
            return list(dict.fromkeys(self.robot_goals))
        return list(dict.fromkeys([*self.human_starts, *self.human_ends]))

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
