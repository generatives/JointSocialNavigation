from __future__ import annotations

from dataclasses import asdict, dataclass
import multiprocessing as mp
import random
from typing import Sequence

import numpy as np

from .simulation import NavigationSimulation

GoalCell = tuple[int, int]


@dataclass(slots=True)
class RunConfig:
    seed: int
    goals: tuple[GoalCell, ...]
    control_mode: str = "ROBOT_AI"
    dt: float = 1.0 / 30.0
    warmup_seconds: float = 6.0
    max_steps_total: int = 1200
    max_humans: int = 220


@dataclass(slots=True)
class RunResult:
    seed: int
    control_mode: str
    goals_total: int
    goals_reached: int
    timed_out_goals: int
    wall_collisions: int
    human_collisions: int
    robot_social_force_generated: float
    distance_travelled: float
    total_time_to_destination: float
    average_time_to_destination: float
    time_to_destination: list[float]


def run_single(config: RunConfig) -> RunResult:
    random.seed(config.seed)
    np.random.seed(config.seed)

    simulation = NavigationSimulation(control_mode=config.control_mode, max_humans=config.max_humans)
    simulation.warmup(config.warmup_seconds, dt=config.dt)

    goals_reached = 0
    timed_out_goals = 0
    wall_collisions = 0
    human_collisions = 0
    robot_social_force_generated = 0.0
    distance_travelled = 0.0
    total_time_to_destination = 0.0
    time_to_destination: list[float] = []
    remaining_steps = config.max_steps_total

    for goal in config.goals:
        if remaining_steps <= 0:
            timed_out_goals += 1
            time_to_destination.append(float("inf"))
            continue
        simulation.set_goal(goal)
        start_time = simulation.sim_time

        reached = False
        while remaining_steps > 0:
            metrics = simulation.update(config.dt)
            remaining_steps -= 1
            wall_collisions += int(metrics.collided_with_wall)
            human_collisions += metrics.robot_human_collisions
            robot_social_force_generated += metrics.robot_social_force_generated
            distance_travelled += metrics.distance_travelled
            if simulation.goal_reached(goal):
                reached = True
                break

        if reached:
            goals_reached += 1
            elapsed = simulation.sim_time - start_time
            total_time_to_destination += elapsed
            time_to_destination.append(elapsed)
        else:
            timed_out_goals += 1
            time_to_destination.append(float("inf"))

    average_time_to_destination = (
        total_time_to_destination / goals_reached if goals_reached > 0 else float("inf")
    )
    return RunResult(
        seed=config.seed,
        control_mode=config.control_mode,
        goals_total=len(config.goals),
        goals_reached=goals_reached,
        timed_out_goals=timed_out_goals,
        wall_collisions=wall_collisions,
        human_collisions=human_collisions,
        robot_social_force_generated=robot_social_force_generated,
        distance_travelled=distance_travelled,
        total_time_to_destination=total_time_to_destination,
        average_time_to_destination=average_time_to_destination,
        time_to_destination=time_to_destination,
    )


def run_parallel(
    goals: Sequence[GoalCell],
    *,
    control_mode: str = "ROBOT_AI",
    num_runs: int = 8,
    base_seed: int = 7,
    processes: int | None = None,
    dt: float = 1.0 / 30.0,
    warmup_seconds: float = 6.0,
    max_steps_total: int = 1200,
    max_humans: int = 220,
) -> dict[str, object]:
    configs = [
        RunConfig(
            seed=base_seed + idx,
            goals=tuple(goals),
            control_mode=control_mode,
            dt=dt,
            warmup_seconds=warmup_seconds,
            max_steps_total=max_steps_total,
            max_humans=max_humans,
        )
        for idx in range(num_runs)
    ]

    if processes == 1 or num_runs == 1:
        results = [run_single(config) for config in configs]
    else:
        ctx = mp.get_context("spawn")
        with ctx.Pool(processes=processes) as pool:
            results = pool.map(run_single, configs)

    total_goals = sum(r.goals_total for r in results)
    total_reached = sum(r.goals_reached for r in results)
    valid_times = [r.average_time_to_destination for r in results if np.isfinite(r.average_time_to_destination)]

    aggregate = {
        "runs": num_runs,
        "control_mode": control_mode,
        "total_goals": total_goals,
        "goals_reached": total_reached,
        "goal_reach_rate": (total_reached / total_goals) if total_goals > 0 else 0.0,
        "wall_collisions": int(sum(r.wall_collisions for r in results)),
        "human_collisions": int(sum(r.human_collisions for r in results)),
        "robot_social_force_generated": float(sum(r.robot_social_force_generated for r in results)),
        "distance_travelled": float(sum(r.distance_travelled for r in results)),
        "total_time_to_destination": float(sum(r.total_time_to_destination for r in results)),
        "mean_time_to_destination": (
            float(sum(valid_times) / len(valid_times)) if valid_times else float("inf")
        ),
        "per_run": [asdict(r) for r in results],
    }
    return aggregate
