from __future__ import annotations

import heapq
import math

import numpy as np

from .constants import WALL


def a_star(grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    if start == goal:
        return [start]

    h, w = grid.shape
    if not (0 <= start[0] < w and 0 <= start[1] < h):
        return []
    if not (0 <= goal[0] < w and 0 <= goal[1] < h):
        return []
    if grid[start[1], start[0]] == WALL or grid[goal[1], goal[0]] == WALL:
        return []

    open_heap: list[tuple[float, tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, start))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}

    def heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dx, dy in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt[0] < 0 or nxt[0] >= w or nxt[1] < 0 or nxt[1] >= h:
                continue
            if grid[nxt[1], nxt[0]] == WALL:
                continue

            tentative = g_score[current] + 1.0
            if tentative < g_score.get(nxt, math.inf):
                came_from[nxt] = current
                g_score[nxt] = tentative
                f = tentative + heuristic(nxt, goal)
                heapq.heappush(open_heap, (f, nxt))

    return []
