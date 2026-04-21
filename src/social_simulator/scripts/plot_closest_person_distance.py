#!/usr/bin/env python3

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

AVG_DISTANCE_COLUMN = "avg_distance_to_closest_person"
TIME_COLUMN_CANDIDATES = ("time_stamps.1", "time_stamps")
GENERIC_SUMMARY_STEMS = {"metrics", "metrics_nav2"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Scan result CSVs, normalize scenario and method labels, and plot the "
            "average distance to the closest person."
        )
    )
    parser.add_argument(
        "--results-dir",
        default="results",
        help="Directory that contains the result folders.",
    )
    parser.add_argument(
        "--output-dir",
        default="results/closest_person_distance",
        help="Directory where organized CSVs and plots will be written.",
    )
    return parser.parse_args()


def looks_like_int(token: str) -> bool:
    return re.fullmatch(r"\d+", token) is not None


def looks_like_float(token: str) -> bool:
    return re.fullmatch(r"\d+(?:\.\d+)?", token) is not None


def natural_key(text: str) -> tuple[object, ...]:
    parts = re.split(r"(\d+(?:\.\d+)?)", text)
    key: list[object] = []
    for part in parts:
        if not part:
            continue
        if looks_like_float(part):
            key.append((0, float(part)))
        else:
            key.append((1, part.lower()))
    return tuple(key)


def pretty_name(text: str) -> str:
    return text.replace("_", " ")


def sanitize_name(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", text.strip()).strip("_") or "unknown"


def normalize_experiment_name(experiment: str, method_path: str) -> str:
    if method_path.startswith("nav2/") or method_path == "nav2":
        if experiment.startswith("nav2_"):
            return experiment[len("nav2_") :]
    if method_path.startswith("mcts/") or method_path == "mcts":
        if experiment.startswith("mcts_"):
            return experiment[len("mcts_") :]
    return experiment


def parse_summary_config(file_path: Path) -> str | None:
    match = re.fullmatch(r"metrics_(\d+(?:\.\d+)?)", file_path.stem)
    if match:
        return match.group(1)
    return None


def format_method_label(method_path: str, config: str | None) -> str:
    if config:
        return f"{method_path} ({config})"
    return method_path


def parse_step_metadata(results_dir: Path, file_path: Path) -> dict[str, object]:
    rel_path = file_path.relative_to(results_dir)
    scenario_group = rel_path.parts[0]
    method_path = "/".join(rel_path.parts[1:-1])

    stem = file_path.stem
    remainder = stem[len("metrics_steps_") :] if stem.startswith("metrics_steps_") else stem
    tokens = remainder.split("_")

    config = None
    run_id = None
    if len(tokens) >= 2 and looks_like_float(tokens[-1]) and looks_like_int(tokens[-2]):
        config = tokens[-1]
        run_id = int(tokens[-2])
        experiment_tokens = tokens[:-2]
    elif tokens and looks_like_int(tokens[-1]):
        run_id = int(tokens[-1])
        experiment_tokens = tokens[:-1]
    else:
        experiment_tokens = tokens

    experiment = normalize_experiment_name("_".join(experiment_tokens), method_path)
    method_label = format_method_label(method_path, config)

    return {
        "scenario_group": scenario_group,
        "method_path": method_path,
        "method_label": method_label,
        "experiment": experiment,
        "config": config,
        "run_id": run_id,
        "source_file": str(rel_path),
    }


def load_step_series(results_dir: Path) -> pd.DataFrame:
    series_frames: list[pd.DataFrame] = []
    for file_path in sorted(results_dir.rglob("metrics_steps*.csv")):
        meta = parse_step_metadata(results_dir, file_path)
        df = pd.read_csv(file_path)

        if AVG_DISTANCE_COLUMN not in df.columns:
            continue

        time_column = next(
            (candidate for candidate in TIME_COLUMN_CANDIDATES if candidate in df.columns),
            None,
        )
        if time_column is None:
            continue

        series = pd.DataFrame(
            {
                "scenario_group": meta["scenario_group"],
                "method_path": meta["method_path"],
                "method_label": meta["method_label"],
                "experiment": meta["experiment"],
                "raw_experiment": meta["experiment"],
                "config": meta["config"],
                "run_id": meta["run_id"],
                "source_file": meta["source_file"],
                "scenario_harmonized": False,
                "time_sec": pd.to_numeric(df[time_column], errors="coerce"),
                AVG_DISTANCE_COLUMN: pd.to_numeric(df[AVG_DISTANCE_COLUMN], errors="coerce"),
            }
        ).dropna(subset=["time_sec", AVG_DISTANCE_COLUMN])

        series_frames.append(series)

    if not series_frames:
        return pd.DataFrame(
            columns=[
                "scenario_group",
                "method_path",
                "method_label",
                "experiment",
                "raw_experiment",
                "config",
                "run_id",
                "source_file",
                "scenario_harmonized",
                "time_sec",
                AVG_DISTANCE_COLUMN,
            ]
        )

    return pd.concat(series_frames, ignore_index=True)


def load_summary_records(results_dir: Path) -> pd.DataFrame:
    records: list[dict[str, object]] = []

    for file_path in sorted(results_dir.rglob("metrics*.csv")):
        if file_path.name.startswith("metrics_steps"):
            continue

        rel_path = file_path.relative_to(results_dir)
        scenario_group = rel_path.parts[0]
        method_path = "/".join(rel_path.parts[1:-1])
        config = parse_summary_config(file_path)
        method_label = format_method_label(method_path, config)

        df = pd.read_csv(file_path)
        if AVG_DISTANCE_COLUMN not in df.columns:
            continue

        for row_index, row in df.iterrows():
            experiment = normalize_experiment_name(
                str(row.get("experiment_tag", "")).strip(),
                method_path,
            )

            records.append(
                {
                    "scenario_group": scenario_group,
                    "method_path": method_path,
                    "method_label": method_label,
                    "experiment": experiment,
                    "raw_experiment": experiment,
                    "config": config,
                    "run_id": row.get("run_id"),
                    "source_file": str(rel_path),
                    "source_stem": file_path.stem,
                    "summary_row_index": row_index,
                    AVG_DISTANCE_COLUMN: pd.to_numeric(
                        row.get(AVG_DISTANCE_COLUMN), errors="coerce"
                    ),
                    "experiment_inferred": False,
                    "scenario_harmonized": False,
                }
            )

    if not records:
        return pd.DataFrame(
            columns=[
                "scenario_group",
                "method_path",
                "method_label",
                "experiment",
                "raw_experiment",
                "config",
                "run_id",
                "source_file",
                "source_stem",
                "summary_row_index",
                AVG_DISTANCE_COLUMN,
                "experiment_inferred",
                "scenario_harmonized",
            ]
        )

    summary_df = pd.DataFrame.from_records(records)
    return summary_df.dropna(subset=[AVG_DISTANCE_COLUMN]).copy()


def reconcile_summary_experiments(
    summary_df: pd.DataFrame, step_df: pd.DataFrame
) -> pd.DataFrame:
    if summary_df.empty or step_df.empty:
        return summary_df

    step_experiments_by_key = (
        step_df.groupby(["scenario_group", "method_label"])["experiment"]
        .apply(lambda values: sorted(set(values)))
        .to_dict()
    )

    updated = summary_df.copy()

    for (scenario_group, method_label), group_index in updated.groupby(
        ["scenario_group", "method_label"]
    ).groups.items():
        step_experiments = step_experiments_by_key.get((scenario_group, method_label), [])
        if len(step_experiments) <= 1:
            continue

        group = updated.loc[group_index]
        generic_mask = (
            group["source_stem"].isin(GENERIC_SUMMARY_STEMS)
            & (group["experiment"] == scenario_group)
        )
        generic_index = group.index[generic_mask].tolist()
        if not generic_index:
            continue

        explicit_experiments = set(group.loc[~generic_mask, "experiment"])
        unmatched_experiments = [
            experiment
            for experiment in step_experiments
            if experiment not in explicit_experiments
        ]

        if len(generic_index) == 1 and len(unmatched_experiments) == 1:
            updated.loc[generic_index[0], "experiment"] = unmatched_experiments[0]
            updated.loc[generic_index[0], "experiment_inferred"] = True

    return updated


def summarize_average_distances(summary_df: pd.DataFrame) -> pd.DataFrame:
    if summary_df.empty:
        return pd.DataFrame(
            columns=[
                "scenario_group",
                "experiment",
                "method_label",
                "record_count",
                AVG_DISTANCE_COLUMN,
            ]
        )

    aggregated = (
        summary_df.groupby(["scenario_group", "experiment", "method_label"], as_index=False)
        .agg(
            record_count=(AVG_DISTANCE_COLUMN, "size"),
            avg_distance_to_closest_person=(AVG_DISTANCE_COLUMN, "mean"),
        )
        .sort_values(
            by=["scenario_group", "experiment", "method_label"],
            key=lambda column: column.map(lambda value: natural_key(str(value))),
        )
    )
    return aggregated


def harmonize_single_experiment_scenarios(
    summary_df: pd.DataFrame, step_df: pd.DataFrame
) -> tuple[pd.DataFrame, pd.DataFrame]:
    updated_summary = summary_df.copy()
    updated_step = step_df.copy()

    scenario_groups = sorted(
        set(updated_summary["scenario_group"]).union(updated_step["scenario_group"]),
        key=natural_key,
    )

    for scenario_group in scenario_groups:
        method_experiments: dict[str, set[str]] = {}

        scenario_summary = updated_summary[updated_summary["scenario_group"] == scenario_group]
        for method_label, group in scenario_summary.groupby("method_label"):
            method_experiments.setdefault(method_label, set()).update(group["experiment"])

        scenario_step = updated_step[updated_step["scenario_group"] == scenario_group]
        for method_label, group in scenario_step.groupby("method_label"):
            method_experiments.setdefault(method_label, set()).update(group["experiment"])

        if method_experiments and all(len(experiments) == 1 for experiments in method_experiments.values()):
            updated_summary.loc[
                updated_summary["scenario_group"] == scenario_group, "experiment"
            ] = scenario_group
            updated_summary.loc[
                updated_summary["scenario_group"] == scenario_group, "scenario_harmonized"
            ] = True

            updated_step.loc[
                updated_step["scenario_group"] == scenario_group, "experiment"
            ] = scenario_group
            updated_step.loc[
                updated_step["scenario_group"] == scenario_group, "scenario_harmonized"
            ] = True

    return updated_summary, updated_step


def method_sort_key(method_label: str) -> tuple[object, ...]:
    base_method = method_label.split(" (", 1)[0]
    config_match = re.search(r"\(([^)]+)\)$", method_label)
    config_value = float(config_match.group(1)) if config_match and looks_like_float(config_match.group(1)) else -1.0
    base_order = {
        "nav2": 0,
        "mcts": 1,
        "mcts/tuning": 2,
        "mcts/main": 3,
    }.get(base_method, 99)
    return (base_order, natural_key(base_method), config_value, natural_key(method_label))


def build_method_colors(method_labels: list[str]) -> dict[str, tuple[float, float, float, float]]:
    cmap = plt.get_cmap("tab10")
    return {
        method: cmap(index % cmap.N)
        for index, method in enumerate(sorted(method_labels, key=method_sort_key))
    }


def plot_grouped_bars(
    plot_df: pd.DataFrame,
    output_path: Path,
    title: str,
    category_column: str,
    category_label_fn,
    method_colors: dict[str, tuple[float, float, float, float]],
) -> None:
    if plot_df.empty:
        return

    categories = sorted(plot_df[category_column].unique(), key=natural_key)
    methods = sorted(plot_df["method_label"].unique(), key=method_sort_key)

    width = 0.8 / max(len(methods), 1)
    positions = list(range(len(categories)))

    fig, ax = plt.subplots(figsize=(max(8, len(categories) * 1.3), 5.5))
    for offset_index, method in enumerate(methods):
        method_values = []
        for category in categories:
            row = plot_df[
                (plot_df[category_column] == category)
                & (plot_df["method_label"] == method)
            ]
            method_values.append(
                row[AVG_DISTANCE_COLUMN].iloc[0] if not row.empty else float("nan")
            )

        x_positions = [
            position - 0.4 + width / 2 + (offset_index * width)
            for position in positions
        ]
        ax.bar(
            x_positions,
            method_values,
            width=width,
            label=method,
            color=method_colors[method],
        )

    ax.set_title(title)
    ax.set_ylabel("Avg distance to closest person (m)")
    ax.set_xticks(positions)
    ax.set_xticklabels([category_label_fn(category) for category in categories], rotation=20, ha="right")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend(title="Method")
    fig.tight_layout()
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_time_series(
    step_df: pd.DataFrame,
    output_dir: Path,
    method_colors: dict[str, tuple[float, float, float, float]],
) -> list[Path]:
    written_files: list[Path] = []
    if step_df.empty:
        return written_files

    for (scenario_group, experiment), group in step_df.groupby(
        ["scenario_group", "experiment"]
    ):
        fig, ax = plt.subplots(figsize=(9, 5))

        methods = sorted(group["method_label"].unique(), key=method_sort_key)
        for method in methods:
            method_group = (
                group[group["method_label"] == method]
                .groupby("time_sec", as_index=False)[AVG_DISTANCE_COLUMN]
                .mean()
                .sort_values("time_sec")
            )
            ax.plot(
                method_group["time_sec"],
                method_group[AVG_DISTANCE_COLUMN],
                label=method,
                color=method_colors[method],
                linewidth=2,
            )

        ax.set_title(
            f"Closest Person Distance Over Time: {pretty_name(scenario_group)} / {pretty_name(experiment)}"
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance (m)")
        ax.grid(alpha=0.3)
        ax.legend(title="Method")
        fig.tight_layout()

        scenario_dir = output_dir / sanitize_name(scenario_group)
        scenario_dir.mkdir(parents=True, exist_ok=True)
        output_path = scenario_dir / f"{sanitize_name(experiment)}.png"
        fig.savefig(output_path, dpi=200, bbox_inches="tight")
        plt.close(fig)
        written_files.append(output_path)

    return written_files


def write_outputs(
    output_dir: Path,
    summary_df: pd.DataFrame,
    summary_plot_df: pd.DataFrame,
    step_df: pd.DataFrame,
) -> dict[str, object]:
    organized_dir = output_dir / "organized"
    plots_dir = output_dir / "plots"
    summary_plots_dir = plots_dir / "summary"
    time_series_dir = plots_dir / "time_series"

    shutil.rmtree(organized_dir, ignore_errors=True)
    shutil.rmtree(summary_plots_dir, ignore_errors=True)
    shutil.rmtree(time_series_dir, ignore_errors=True)

    organized_dir.mkdir(parents=True, exist_ok=True)
    summary_plots_dir.mkdir(parents=True, exist_ok=True)
    time_series_dir.mkdir(parents=True, exist_ok=True)

    summary_catalog_path = organized_dir / "summary_catalog.csv"
    summary_plot_path = organized_dir / "summary_plot_data.csv"
    step_series_path = organized_dir / "step_series.csv"

    summary_df.to_csv(summary_catalog_path, index=False)
    summary_plot_df.to_csv(summary_plot_path, index=False)
    step_df.to_csv(step_series_path, index=False)

    method_colors = build_method_colors(
        sorted(
            set(summary_plot_df["method_label"]).union(step_df["method_label"]),
            key=method_sort_key,
        )
    )

    written_summary_plots: list[Path] = []
    if not summary_plot_df.empty:
        overall_plot_path = summary_plots_dir / "all_scenarios.png"
        all_scenarios_df = summary_plot_df.copy()
        all_scenarios_df["scenario_experiment"] = all_scenarios_df.apply(
            lambda row: (
                row["scenario_group"]
                if row["experiment"] == row["scenario_group"]
                else f"{row['scenario_group']} / {row['experiment']}"
            ),
            axis=1,
        )
        plot_grouped_bars(
            all_scenarios_df,
            overall_plot_path,
            "Average Distance to Closest Person Across All Scenarios",
            "scenario_experiment",
            lambda category: pretty_name(category),
            method_colors,
        )
        written_summary_plots.append(overall_plot_path)

        for scenario_group, scenario_df in summary_plot_df.groupby("scenario_group"):
            scenario_plot_path = summary_plots_dir / f"{sanitize_name(scenario_group)}.png"
            plot_grouped_bars(
                scenario_df,
                scenario_plot_path,
                f"Average Distance to Closest Person: {pretty_name(scenario_group)}",
                "experiment",
                lambda category: pretty_name(category),
                method_colors,
            )
            written_summary_plots.append(scenario_plot_path)

    written_time_series = plot_time_series(step_df, time_series_dir, method_colors)

    return {
        "summary_catalog_path": summary_catalog_path,
        "summary_plot_data_path": summary_plot_path,
        "step_series_path": step_series_path,
        "summary_plot_paths": written_summary_plots,
        "time_series_paths": written_time_series,
    }


def main() -> None:
    args = parse_args()
    results_dir = Path(args.results_dir).resolve()
    output_dir = Path(args.output_dir).resolve()

    step_df = load_step_series(results_dir)
    summary_df = load_summary_records(results_dir)
    summary_df = reconcile_summary_experiments(summary_df, step_df)
    summary_df, step_df = harmonize_single_experiment_scenarios(summary_df, step_df)
    summary_plot_df = summarize_average_distances(summary_df)

    output_info = write_outputs(output_dir, summary_df, summary_plot_df, step_df)

    print(f"Wrote summary catalog: {output_info['summary_catalog_path']}")
    print(f"Wrote plot data: {output_info['summary_plot_data_path']}")
    print(f"Wrote step series: {output_info['step_series_path']}")
    print(f"Generated {len(output_info['summary_plot_paths'])} summary plot(s)")
    print(f"Generated {len(output_info['time_series_paths'])} time-series plot(s)")


if __name__ == "__main__":
    main()
