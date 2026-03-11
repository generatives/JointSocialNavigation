"""
Replanning demo: Joint State Space A* on the narrow-hallway map.

At every discrete step the planner is called fresh from the agents' current
positions.  Only the *first* joint move of the returned plan is executed, then
the loop repeats — simulating a closed-loop receding-horizon planner.

The narrow hallway is a single-cell-wide corridor (y=9) with one alcove cell
at (30, 10).  The robot starts at the right end (35, 9) and must reach (0, 9);
the human starts at (0, 9) and must reach (35, 9).

Controls
--------
SPACE      pause / resume
RIGHT      advance one step manually (while paused)
R          reset to start positions
Q / Esc    quit
"""
from __future__ import annotations

import time
from typing import Optional

import pygame

from social_navigation.joint_astar.joint_astar import JointPlanResult, joint_a_star
from social_navigation.simulator.constants import WALL
from social_navigation.simulator.map import ScenarioMap

# ── display ──────────────────────────────────────────────────────────────────

CELL_PX = 72
STEP_PERIOD = 0.45   # seconds between automatic steps

BG_COLOR       = (237, 242, 245)
WALL_COLOR     = (37,  41,  44)
FREE_COLOR     = (220, 226, 230)
GRID_COLOR     = (190, 196, 200)
ROBOT_COLOR    = (212, 63,  44)
HUMAN_COLOR    = (58,  138, 246)
ROBOT_GOAL_CLR = (235, 120, 50)
HUMAN_GOAL_CLR = (61,  184, 112)
TEXT_COLOR     = (12,  12,  12)


# ── drawing helpers ───────────────────────────────────────────────────────────

def cell_center(cell: tuple[int, int]) -> tuple[int, int]:
    return (cell[0] * CELL_PX + CELL_PX // 2, cell[1] * CELL_PX + CELL_PX // 2)


def draw_map(screen: pygame.Surface, scenario: ScenarioMap) -> None:
    for y in range(scenario.height):
        for x in range(scenario.width):
            rect = pygame.Rect(x * CELL_PX, y * CELL_PX, CELL_PX, CELL_PX)
            if scenario.grid[y, x] == WALL:
                pygame.draw.rect(screen, WALL_COLOR, rect)
            else:
                pygame.draw.rect(screen, FREE_COLOR, rect)
                pygame.draw.rect(screen, GRID_COLOR, rect, 1)


def draw_goal_ring(screen: pygame.Surface, cell: tuple[int, int], color: tuple) -> None:
    pygame.draw.circle(screen, color, cell_center(cell), CELL_PX // 3, 3)


def draw_planned_path(
    screen: pygame.Surface,
    path: list[tuple[int, int]],
    color: tuple[int, int, int],
) -> None:
    """Fading dots showing the current plan from position 1 onward."""
    for i, cell in enumerate(path):
        if i == 0:
            continue   # current position drawn as agent circle
        alpha = max(20, 180 - i * 6)
        s = pygame.Surface((CELL_PX, CELL_PX), pygame.SRCALPHA)
        pygame.draw.circle(s, (*color, alpha), (CELL_PX // 2, CELL_PX // 2), CELL_PX // 5)
        screen.blit(s, (cell[0] * CELL_PX, cell[1] * CELL_PX))


def draw_history_trail(
    screen: pygame.Surface,
    history: list[tuple[int, int]],
    color: tuple[int, int, int],
) -> None:
    """Small breadcrumb dots for cells the agent has already visited."""
    for i, cell in enumerate(history):
        alpha = max(15, 80 - (len(history) - i) * 3)
        s = pygame.Surface((CELL_PX, CELL_PX), pygame.SRCALPHA)
        pygame.draw.circle(s, (*color, alpha), (CELL_PX // 2, CELL_PX // 2), CELL_PX // 8)
        screen.blit(s, (cell[0] * CELL_PX, cell[1] * CELL_PX))


def draw_agent(
    screen: pygame.Surface,
    cell: tuple[int, int],
    color: tuple[int, int, int],
    label: str,
    font: pygame.font.Font,
) -> None:
    center = cell_center(cell)
    pygame.draw.circle(screen, color, center, CELL_PX // 3)
    lbl = font.render(label, True, (255, 255, 255))
    screen.blit(lbl, lbl.get_rect(center=center))


def draw_hud(
    screen: pygame.Surface,
    font: pygame.font.Font,
    *,
    step: int,
    paused: bool,
    done: bool,
    replan_ms: float,
    robot_pos: tuple[int, int],
    human_pos: tuple[int, int],
    robot_goal: tuple[int, int],
    human_goal: tuple[int, int],
    plan_len: int,
) -> None:
    status = "[DONE]" if done else ("[PAUSED]" if paused else "[PLAYING]")
    lines = [
        f"Step {step:3d}  {status}  replan: {replan_ms:.1f} ms  plan horizon: {plan_len} steps",
        f"Robot  {robot_pos} → {robot_goal}",
        f"Human  {human_pos} → {human_goal}",
        "SPACE pause  RIGHT step  R reset  Q quit",
    ]
    padding = 6
    line_h = font.get_linesize()
    box_h = len(lines) * line_h + padding * 2
    box_w = max(font.size(l)[0] for l in lines) + padding * 2
    box = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
    box.fill((255, 255, 255, 210))
    screen.blit(box, (8, 8))
    for i, line in enumerate(lines):
        screen.blit(font.render(line, True, TEXT_COLOR), (8 + padding, 8 + padding + i * line_h))


# ── simulation state ──────────────────────────────────────────────────────────

class ReplanningState:
    def __init__(self, scenario: ScenarioMap) -> None:
        self.scenario = scenario
        self.robot_goal = scenario.human_starts[0]   # (0, 9)
        self.human_goal = scenario.robot_start[:2]    # (x, y)
        self.reset()

    def reset(self) -> None:
        self.robot_pos: tuple[int, int] = self.scenario.robot_start[:2]
        self.human_pos: tuple[int, int] = self.scenario.human_starts[0]
        self.robot_history: list[tuple[int, int]] = []
        self.human_history: list[tuple[int, int]] = []
        self.step = 0
        self.last_plan: Optional[JointPlanResult] = None
        self.last_replan_ms: float = 0.0
        self._replan()

    def _replan(self) -> None:
        t0 = time.perf_counter()
        self.last_plan = joint_a_star(
            self.scenario,
            self.robot_pos, self.robot_goal,
            self.human_pos, self.human_goal,
        )
        self.last_replan_ms = (time.perf_counter() - t0) * 1000.0

    @property
    def done(self) -> bool:
        return self.robot_pos == self.robot_goal and self.human_pos == self.human_goal

    def advance(self) -> None:
        """Execute the first move of the current plan, then replan."""
        if self.done or self.last_plan is None or len(self.last_plan.joint_path) < 2:
            return

        self.robot_history.append(self.robot_pos)
        self.human_history.append(self.human_pos)

        next_state = self.last_plan.joint_path[1]
        self.robot_pos = (next_state[0], next_state[1])
        self.human_pos = (next_state[2], next_state[3])
        self.step += 1

        self._replan()


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    scenario = ScenarioMap.build_narrow_hallway2()

    pygame.init()
    screen = pygame.display.set_mode((scenario.width * CELL_PX, scenario.height * CELL_PX))
    pygame.display.set_caption("Joint A* Replanning — Narrow Hallway")
    font = pygame.font.SysFont("consolas", 14)
    clock = pygame.time.Clock()

    state = ReplanningState(scenario)
    paused = False
    step_timer = 0.0

    while True:
        dt = clock.tick(60) / 1000.0

        # ── events ────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    pygame.quit()
                    return
                if event.key == pygame.K_SPACE:
                    paused = not paused
                    step_timer = 0.0
                if event.key == pygame.K_RIGHT and paused:
                    state.advance()
                if event.key == pygame.K_r:
                    state.reset()
                    paused = False
                    step_timer = 0.0

        # ── auto-advance ──────────────────────────────────────────────────
        if not paused and not state.done:
            step_timer += dt
            if step_timer >= STEP_PERIOD:
                step_timer -= STEP_PERIOD
                state.advance()

        # ── draw ──────────────────────────────────────────────────────────
        screen.fill(BG_COLOR)
        draw_map(screen, scenario)

        draw_goal_ring(screen, state.robot_goal, ROBOT_GOAL_CLR)
        draw_goal_ring(screen, state.human_goal, HUMAN_GOAL_CLR)

        # History breadcrumbs
        draw_history_trail(screen, state.robot_history, ROBOT_COLOR)
        draw_history_trail(screen, state.human_history, HUMAN_COLOR)

        # Current plan horizon (from the most recent replan)
        if state.last_plan is not None:
            draw_planned_path(screen, state.last_plan.robot_path, ROBOT_COLOR)
            draw_planned_path(screen, state.last_plan.human_path, HUMAN_COLOR)

        # Agents
        draw_agent(screen, state.robot_pos, ROBOT_COLOR, "R", font)
        draw_agent(screen, state.human_pos, HUMAN_COLOR, "H", font)

        plan_len = len(state.last_plan.joint_path) if state.last_plan else 0
        draw_hud(
            screen, font,
            step=state.step,
            paused=paused,
            done=state.done,
            replan_ms=state.last_replan_ms,
            robot_pos=state.robot_pos,
            human_pos=state.human_pos,
            robot_goal=state.robot_goal,
            human_goal=state.human_goal,
            plan_len=plan_len,
        )

        pygame.display.flip()


if __name__ == "__main__":
    main()
