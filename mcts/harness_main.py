from __future__ import annotations

import argparse
import json

from simulator.harness import run_parallel


def _parse_goals(goals_raw: str) -> list[tuple[int, int]]:
    goals: list[tuple[int, int]] = []
    for goal_raw in goals_raw.split(";"):
        goal_raw = goal_raw.strip()
        if not goal_raw:
            continue
        x_raw, y_raw = goal_raw.split(",")
        goals.append((int(x_raw), int(y_raw)))
    if not goals:
        raise ValueError("At least one goal is required, format: \"x1,y1;x2,y2\"")
    return goals


def main() -> None:
    parser = argparse.ArgumentParser(description="Headless navigation test harness")
    parser.add_argument("--goals", required=True, help="Goal script, e.g. \"17,5;12,4;2,3\"")
    parser.add_argument("--mode", default="ROBOT_AI", choices=["ROBOT_AI", "MCTS_ROBOT_AI"])
    parser.add_argument("--runs", type=int, default=8)
    parser.add_argument("--base-seed", type=int, default=7)
    parser.add_argument("--processes", type=int, default=None)
    parser.add_argument("--dt", type=float, default=1.0 / 30.0)
    parser.add_argument("--warmup-seconds", type=float, default=6.0)
    parser.add_argument("--max-steps-total", type=int, default=1200)
    parser.add_argument("--max-humans", type=int, default=220)
    args = parser.parse_args()

    goals = _parse_goals(args.goals)
    result = run_parallel(
        goals=goals,
        control_mode=args.mode,
        num_runs=args.runs,
        base_seed=args.base_seed,
        processes=args.processes,
        dt=args.dt,
        warmup_seconds=args.warmup_seconds,
        max_steps_total=args.max_steps_total,
        max_humans=args.max_humans,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
