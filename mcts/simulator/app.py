from __future__ import annotations

import math

import numpy as np
import pygame

from .agents import Crowd, MCTSRobotAI, Robot, RobotAI
from .constants import WALL
from .map import ScenarioMap


class Simulator:
    def __init__(self) -> None:
        self.scenario = ScenarioMap.build_default()
        robot_start = self.scenario.cell_to_world(self.scenario.robot_start)
        self.robot = Robot(position=robot_start.copy(), theta=0.0)
        self.robot_ai = RobotAI(self.scenario)
        self.crowd = Crowd(max_humans=220, scenario=self.scenario)
        self.mcts_robot_ai = MCTSRobotAI(self.scenario, self.crowd)
        self.control_modes = ("MANUAL", "ROBOT_AI", "MCTS_ROBOT_AI")
        self.control_mode_idx = 1

        pygame.init()
        self.cell_px = 24
        self.screen = pygame.display.set_mode(
            (self.scenario.width * self.cell_px, self.scenario.height * self.cell_px)
        )
        pygame.display.set_caption("Crowded Navigation Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 18)

    def run(self) -> None:

        # Run for a few seconds to get some people in the scene
        base_dt = 1.0 / 30.0
        for i in range(int(1.0 / base_dt * 6)):
            self._update(base_dt)

        running = True
        while running:
            dt = self.clock.tick(30) / 1000.0
            dt = min(dt, 1.0 / 30.0)
            running = self._handle_events()
            self._update(dt)
            self._draw()
        pygame.quit()

    def _handle_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_TAB:
                prev_mode = self.control_modes[self.control_mode_idx]
                self.control_mode_idx = (self.control_mode_idx + 1) % len(self.control_modes)
                next_mode = self.control_modes[self.control_mode_idx]
                if prev_mode == "ROBOT_AI" and next_mode != "ROBOT_AI":
                    self.robot.path = []
                    self.robot.path_ptr = 0
            if event.type == pygame.MOUSEBUTTONDOWN and self.control_modes[self.control_mode_idx] != "MANUAL":
                clicked = self._px_to_cell(event.pos)
                if event.button == 1 and self.scenario.is_free(clicked):
                    self.robot_ai.set_manual_goal(clicked)
                    self.mcts_robot_ai.set_manual_goal(clicked)
                    self.robot.path = []
                    self.robot.path_ptr = 0
                if event.button == 3:
                    self.robot_ai.clear_manual_goal()
                    self.mcts_robot_ai.clear_manual_goal()
                    self.robot.path = []
                    self.robot.path_ptr = 0
        return True

    def _update(self, dt: float) -> None:
        keys = pygame.key.get_pressed()
        mode = self.control_modes[self.control_mode_idx]
        if mode == "ROBOT_AI":
            self.robot_ai.update(self.robot, dt)
        elif mode == "MCTS_ROBOT_AI":
            self.mcts_robot_ai.update(self.robot, dt)
        else:
            v = 0.0
            w = 0.0
            if keys[pygame.K_UP]:
                v += 2.0
            if keys[pygame.K_DOWN]:
                v -= 1.0
            if keys[pygame.K_LEFT]:
                w -= 2.5
            if keys[pygame.K_RIGHT]:
                w += 2.5
            self.robot.command_v = v
            self.robot.command_w = w

        self.robot.step(dt, self.scenario)
        self.crowd.update(dt, self.robot)

    def _draw(self) -> None:
        self.screen.fill((237, 242, 245))
        for y in range(self.scenario.height):
            for x in range(self.scenario.width):
                rect = pygame.Rect(x * self.cell_px, y * self.cell_px, self.cell_px, self.cell_px)
                if self.scenario.grid[y, x] == WALL:
                    pygame.draw.rect(self.screen, (37, 41, 44), rect)
                else:
                    pygame.draw.rect(self.screen, (220, 226, 230), rect, width=1)

        self._draw_markers(self.scenario.human_starts, (44, 120, 230))
        self._draw_markers(self.scenario.human_ends, (61, 184, 112))
        goal = self.robot_ai.manual_goal if self.robot_ai.manual_goal is not None else self.mcts_robot_ai.manual_goal
        if goal is not None:
            self._draw_markers([goal], (235, 120, 50))

        active_idxs = np.flatnonzero(self.crowd.active)
        for idx in active_idxs:
            self._draw_circle(self.crowd.positions[idx], float(self.crowd.radius[idx]), (58, 138, 246))

        if self.robot.path and len(self.robot.path) >= 2:
            points = [self._to_px(self.scenario.cell_to_world(c)) for c in self.robot.path]
            pygame.draw.lines(self.screen, (201, 85, 73), False, points, width=2)

        self._draw_circle(self.robot.position, self.robot.radius, (212, 63, 44))
        heading = self.robot.position + self.robot.forward() * (self.robot.radius + 0.45)
        pygame.draw.line(self.screen, (24, 27, 28), self._to_px(self.robot.position), self._to_px(heading), 3)

        mode = self.control_modes[self.control_mode_idx]
        manual_goal = "None" if goal is None else f"{goal[0]},{goal[1]}"
        text = self.font.render(
            f"Mode: {mode} | TAB cycle | LMB set AI goal | RMB clear | goal: {manual_goal}",
            True,
            (12, 12, 12),
        )
        self.screen.blit(text, (10, 8))
        pygame.display.flip()

    def _draw_markers(self, cells: list[tuple[int, int]], color: tuple[int, int, int]) -> None:
        for cell in cells:
            center = self._to_px(self.scenario.cell_to_world(cell))
            pygame.draw.circle(self.screen, color, center, self.cell_px // 4, width=2)

    def _draw_circle(self, world_pos: np.ndarray, world_radius: float, color: tuple[int, int, int]) -> None:
        pygame.draw.circle(
            self.screen,
            color,
            self._to_px(world_pos),
            max(2, int(round(world_radius * self.cell_px))),
        )

    def _to_px(self, world_pos: np.ndarray) -> tuple[int, int]:
        return int(round(world_pos[0] * self.cell_px)), int(round(world_pos[1] * self.cell_px))

    def _px_to_cell(self, px_pos: tuple[int, int]) -> tuple[int, int]:
        x = int(math.floor(px_pos[0] / self.cell_px))
        y = int(math.floor(px_pos[1] / self.cell_px))
        x = int(np.clip(x, 0, self.scenario.width - 1))
        y = int(np.clip(y, 0, self.scenario.height - 1))
        return x, y
