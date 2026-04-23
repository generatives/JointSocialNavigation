#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import re
from collections import defaultdict
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
except ModuleNotFoundError as exc:
    raise SystemExit(
        "matplotlib is required to generate the completion time chart. "
        "Install it in the active Python environment and rerun the script."
    ) from exc

KNOWN_METHODS = ("nav2", "mcts")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot the time taken by MCTS and Nav2 to finish each scenario "
            "from results/metrics.csv."
        )
    )
    parser.add_argument(
        "--results-file",
        default="results/metrics.csv",
        help="Summary CSV containing experiment_tag and time_to_reach_goal columns.",
    )
    parser.add_argument(
        "--output",
        default="results/completion_time_comparison.png",
        help="Path to save the generated chart.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the plot interactively after saving it.",
    )
    return parser.parse_args()


def normalize_token(text: str) -> str:
    normalized = re.sub(r"[\s-]+", "_", text.strip().lower())
    return re.sub(r"_+", "_", normalized).strip("_")


def natural_key(text: str) -> tuple[object, ...]:
    parts = re.split(r"(\d+)", text)
    key: list[object] = []
    for part in parts:
        if not part:
            continue
        if part.isdigit():
            key.append((0, int(part)))
        else:
            key.append((1, part))
    return tuple(key)


def normalize_scenario_label(scenario: str) -> str:
    normalized = normalize_token(scenario)
    normalized = re.sub(r"_(\d+)_agents_group$", r"_\1_agent_group", normalized)
    return normalized


def parse_experiment_tag(experiment_tag: str) -> tuple[str | None, str]:
    normalized = normalize_token(experiment_tag)
    for method in KNOWN_METHODS:
        prefix = f"{method}_"
        if normalized.startswith(prefix):
            return method, normalize_scenario_label(normalized[len(prefix) :])
    return None, normalize_scenario_label(normalized)


def pretty_scenario_name(scenario: str) -> str:
    label = scenario.replace("_", " ")
    label = re.sub(r"\b(\d+) agent group\b", r"\1-agent group", label)
    return label.title()


def parse_completed(value: str | None) -> bool:
    if value is None:
        return True
    return normalize_token(value) in {"true", "1", "yes"}


def load_completion_times(results_file: Path) -> list[dict[str, object]]:
    with results_file.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError(f"{results_file} is empty")

        required_columns = {"experiment_tag", "time_to_reach_goal"}
        missing_columns = required_columns - set(reader.fieldnames)
        if missing_columns:
            missing = ", ".join(sorted(missing_columns))
            raise ValueError(f"Missing required columns in {results_file}: {missing}")

        grouped_rows: dict[tuple[str, str], list[dict[str, object]]] = defaultdict(list)
        for row in reader:
            method, scenario = parse_experiment_tag(row.get("experiment_tag", ""))
            if method not in KNOWN_METHODS or not scenario:
                continue

            try:
                time_to_reach_goal = float(row["time_to_reach_goal"])
            except (TypeError, ValueError):
                continue

            grouped_rows[(scenario, method)].append(
                {
                    "time_to_reach_goal": time_to_reach_goal,
                    "completed": parse_completed(row.get("completed")),
                }
            )

    if not grouped_rows:
        raise ValueError(f"No MCTS/Nav2 completion rows found in {results_file}")

    aggregated_rows: list[dict[str, object]] = []
    for (scenario, method), rows in sorted(grouped_rows.items()):
        times = [row["time_to_reach_goal"] for row in rows]
        aggregated_rows.append(
            {
                "scenario": scenario,
                "method": method,
                "time_to_reach_goal": sum(times) / len(times),
                "completed": all(bool(row["completed"]) for row in rows),
                "run_count": len(rows),
            }
        )

    return aggregated_rows


def plot_completion_times(data: list[dict[str, object]], output_path: Path, show: bool) -> None:
    scenarios = sorted({str(row["scenario"]) for row in data}, key=natural_key)
    width = 0.35
    colors = {"nav2": "#1f77b4", "mcts": "#ff7f0e"}

    fig, ax = plt.subplots(figsize=(max(10, len(scenarios) * 1.4), 6))
    incomplete_labeled = False

    row_lookup = {
        (str(row["scenario"]), str(row["method"])): row
        for row in data
    }

    for offset, method in [(-width / 2, "mcts"), (width / 2, "nav2")]:
        heights: list[float] = []
        completed: list[bool] = []
        x_positions = [index + offset for index in range(len(scenarios))]
        for scenario in scenarios:
            row = row_lookup.get((scenario, method))
            if row is None:
                heights.append(math.nan)
                completed.append(False)
                continue
            heights.append(float(row["time_to_reach_goal"]))
            completed.append(bool(row["completed"]))

        bars = ax.bar(
            x_positions,
            heights,
            width=width,
            label=method.upper(),
            color=colors[method],
            alpha=0.9,
        )

        for bar, is_completed in zip(bars, completed):
            if not math.isfinite(bar.get_height()):
                bar.set_visible(False)
                continue
            if not is_completed:
                bar.set_hatch("//")
                bar.set_alpha(0.45)
                if not incomplete_labeled:
                    bar.set_label("Incomplete run")
                    incomplete_labeled = True

    for row in data:
        method_offset = -width / 2 if row["method"] == "mcts" else width / 2
        xpos = scenarios.index(str(row["scenario"])) + method_offset
        label = f"{float(row['time_to_reach_goal']):.1f}s"
        if not bool(row["completed"]):
            label += " *"
        ax.text(
            xpos,
            float(row["time_to_reach_goal"]) + 0.35,
            label,
            ha="center",
            va="bottom",
            rotation=90,
            fontsize=8,
        )

    ax.set_title("Scenario Completion Time: MCTS vs Nav2")
    ax.set_ylabel("Time to Reach Goal (s)")
    ax.set_xlabel("Scenario")
    ax.set_xticks(list(range(len(scenarios))))
    ax.set_xticklabels([pretty_scenario_name(scenario) for scenario in scenarios], rotation=25, ha="right")
    ax.grid(axis="y", linestyle="--", alpha=0.3)
    legend_handles = [
        Patch(facecolor=colors["mcts"], edgecolor="black", alpha=0.9, label="MCTS"),
        Patch(facecolor=colors["nav2"], edgecolor="black", alpha=0.9, label="NAV2"),
    ]
    if not all(bool(row["completed"]) for row in data):
        legend_handles.append(
            Patch(
                facecolor="white",
                edgecolor="black",
                hatch="//",
                alpha=0.45,
                label="Incomplete run",
            )
        )
    ax.legend(handles=legend_handles)

    if not all(bool(row["completed"]) for row in data):
        ax.text(
            0.99,
            0.98,
            "* incomplete or timed-out run",
            transform=ax.transAxes,
            ha="right",
            va="top",
            fontsize=9,
        )

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    args = parse_args()
    results_file = Path(args.results_file)
    output_path = Path(args.output)

    data = load_completion_times(results_file)
    plot_completion_times(data, output_path, args.show)
    print(f"Saved completion time chart to {output_path}")


if __name__ == "__main__":
    main()
