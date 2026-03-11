from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

# (xr, yr, xh, yh) — spatial joint state, no time dimension.
# Dropping time is safe because g increases monotonically (each step >= 1.0),
# so re-reaching the same spatial state at a later timestep always has higher
# cost and will be pruned by the visited check.
JointCell = Tuple[int, int, int, int]


@dataclass(slots=True)
class JointPlanResult:
    robot_path: List[Tuple[int, int]]
    human_path: List[Tuple[int, int]]
    joint_path: List[JointCell]
    cost: float


def manhattan(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def joint_a_star(
    scenario,
    robot_start: Tuple[int, int],
    robot_goal: Tuple[int, int],
    human_start: Tuple[int, int],
    human_goal: Tuple[int, int],
    robot_move_penalty: float = 0.05,
    human_detour_penalty: float = 0.5,
    blocking_penalty: float = 2.0,
    proximity_penalty: float = 3.5,
    proximity_threshold: int = 4,
) -> Optional[JointPlanResult]:
    """
    Joint A* over the 4D spatial state (xr, yr, xh, yh).

    Both agents can wait or step in one of four cardinal directions each tick.

    Collision constraints
    --------------------
    - Same-cell occupancy is forbidden.
    - Edge-swap (head-on pass through a shared edge) is forbidden.

    Cost structure
    --------------
    Each joint step costs 1.0 plus optional penalties:
    - ``robot_move_penalty``   added when the robot moves (default 0.05).
    - ``human_detour_penalty`` added when the human moves *away* from its goal
                               (default 0.5).
    - ``blocking_penalty``     added, scaled by 1/distance, whenever the robot's
                               next position lies on the human's optimal path
                               (default 2.0).  The closer the human, the more
                               expensive it is to stay in the way, so the robot
                               commits to the alcove early as a social signal.
    - ``proximity_penalty``    added, linearly graded, whenever the robot moves
                               and the resulting robot–human distance is less than
                               ``proximity_threshold`` cells (default 1.5 / 4).
                               This enforces a clearance buffer so the robot does
                               not exit the alcove while the human is still nearby.

    Returns None if no solution exists.
    """
    if robot_start == human_start:
        return None

    moves = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]

    start: JointCell = (robot_start[0], robot_start[1], human_start[0], human_start[1])

    def is_goal(s: JointCell) -> bool:
        return (s[0], s[1]) == robot_goal and (s[2], s[3]) == human_goal

    def heuristic(s: JointCell) -> float:
        # max is admissible: one joint step can advance both agents by <= 1 cell.
        hr = manhattan((s[0], s[1]), robot_goal)
        hh = manhattan((s[2], s[3]), human_goal)
        return float(max(hr, hh))

    # heap entries: (f, g, state)
    open_heap: List[Tuple[float, float, JointCell]] = []
    heapq.heappush(open_heap, (heuristic(start), 0.0, start))

    g_cost: Dict[JointCell, float] = {start: 0.0}
    parents: Dict[JointCell, Optional[JointCell]] = {start: None}

    while open_heap:
        _, g, curr = heapq.heappop(open_heap)

        # Stale-entry guard: a cheaper path was already found.
        if g > g_cost.get(curr, float("inf")):
            continue

        if is_goal(curr):
            return _reconstruct(parents, curr, g)

        xr, yr, xh, yh = curr

        for drx, dry in moves:
            nr = (xr + drx, yr + dry)
            if not scenario.is_free(nr):
                continue

            for dhx, dhy in moves:
                nh = (xh + dhx, yh + dhy)
                if not scenario.is_free(nh):
                    continue

                # Forbid same-cell occupancy.
                if nr == nh:
                    continue

                # Forbid edge-swap (agents passing through each other).
                if nr == (xh, yh) and nh == (xr, yr):
                    continue

                nxt: JointCell = (nr[0], nr[1], nh[0], nh[1])

                step_cost = 1.0

                # Robot yields: penalize robot motion to prefer robot waiting.
                if (drx, dry) != (0, 0):
                    step_cost += robot_move_penalty

                # Human right-of-way: penalize human detouring away from goal.
                prev_h_dist = manhattan((xh, yh), human_goal)
                new_h_dist = manhattan((nh[0], nh[1]), human_goal)
                if new_h_dist > prev_h_dist:
                    step_cost += human_detour_penalty

                # Proximity buffer: when the robot moves and would end up close
                # to the human, add a flat penalty so the robot waits for a
                # safe clearance gap before exiting the alcove.
                # The penalty must exceed 1.0 (the heuristic gain from moving
                # one cell closer to the robot's goal), otherwise the A* search
                # still prefers moving even when the human is nearby.
                if (drx, dry) != (0, 0):
                    h_to_r_after = manhattan(nh, nr)
                    if h_to_r_after < proximity_threshold:
                        step_cost += proximity_penalty

                # Early-yield signal: penalize robot for staying on the human's
                # optimal path.  A cell nr is on the human's shortest path from
                # nh to human_goal when the triangle inequality is tight:
                #   d(nh, nr) + d(nr, human_goal) == d(nh, human_goal)
                # The penalty scales as 1/distance so urgency grows as the
                # human approaches, pushing the robot into the alcove early.
                h_to_r = manhattan(nh, nr)
                if h_to_r > 0:
                    r_to_hg = manhattan(nr, human_goal)
                    h_to_hg = manhattan(nh, human_goal)
                    if h_to_r + r_to_hg == h_to_hg:
                        step_cost += blocking_penalty / h_to_r

                new_g = g + step_cost

                if new_g < g_cost.get(nxt, float("inf")):
                    g_cost[nxt] = new_g
                    parents[nxt] = curr
                    f = new_g + heuristic(nxt)
                    heapq.heappush(open_heap, (f, new_g, nxt))

    return None


def _reconstruct(
    parents: Dict[JointCell, Optional[JointCell]],
    goal: JointCell,
    cost: float,
) -> JointPlanResult:
    joint_path: List[JointCell] = []
    s: Optional[JointCell] = goal
    while s is not None:
        joint_path.append(s)
        s = parents[s]
    joint_path.reverse()

    robot_path = [(s[0], s[1]) for s in joint_path]
    human_path = [(s[2], s[3]) for s in joint_path]
    return JointPlanResult(
        robot_path=robot_path,
        human_path=human_path,
        joint_path=joint_path,
        cost=cost,
    )
