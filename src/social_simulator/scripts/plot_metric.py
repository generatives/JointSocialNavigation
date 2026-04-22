#!/usr/bin/env python3

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

DEFAULT_METRIC = "avg_distance_to_closest_person"
TIME_COLUMN_CANDIDATES = ("time_stamps.1", "time_stamps")
GENERIC_SUMMARY_STEMS = {"metrics", "metrics_nav2"}
KNOWN_BASE_METHODS = ("nav2", "mcts")
METRIC_ALIASES = {
    "closest_person_distance": DEFAULT_METRIC,
}
METRIC_SPECS = {
    DEFAULT_METRIC: {
        "display_name": "Average Distance to Closest Person",
        "summary_ylabel": "Avg distance to closest person (m)",
        "time_series_ylabel": "Distance (m)",
        "output_dir_name": "closest_person_distance",
    },
    "intimate_space_intrusions": {
        "display_name": "Intimate Space Intrusions",
        "summary_ylabel": "Intrusions",
        "time_series_ylabel": "Intrusions",
        "output_dir_name": "intimate_space_intrusions",
    },
    "personal_space_intrusions": {
        "display_name": "Personal Space Intrusions",
        "summary_ylabel": "Intrusions",
        "time_series_ylabel": "Intrusions",
        "output_dir_name": "personal_space_intrusions",
    },
    "social_space_intrusions": {
        "display_name": "Social Space Intrusions",
        "summary_ylabel": "Intrusions",
        "time_series_ylabel": "Intrusions",
        "output_dir_name": "social_space_intrusions",
    },
    "robot_on_person_collision": {
        "display_name": "Robot-on-Person Collisions",
        "summary_ylabel": "Collisions",
        "time_series_ylabel": "Collisions",
        "output_dir_name": "robot_on_person_collision",
    },
    "person_on_robot_collision": {
        "display_name": "Person-on-Robot Collisions",
        "summary_ylabel": "Collisions",
        "time_series_ylabel": "Collisions",
        "output_dir_name": "person_on_robot_collision",
    },
    "time_not_moving": {
        "display_name": "Time Not Moving",
        "summary_ylabel": "Time not moving (s)",
        "time_series_ylabel": "Time not moving (s)",
        "output_dir_name": "time_not_moving",
    },
    "social_force_on_agents": {
        "display_name": "Social Force on Agents",
        "summary_ylabel": "Social force on agents",
        "time_series_ylabel": "Social force on agents",
        "output_dir_name": "social_force_on_agents",
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Scan result CSVs, normalize scenario and method labels, and plot a "
            "selected metric."
        )
    )
    parser.add_argument(
        "--metric",
        default=DEFAULT_METRIC,
        help=(
            "Metric column to plot. Aliases such as "
            "'closest_person_distance' are also accepted."
        ),
    )
    parser.add_argument(
        "--results-dir",
        default="results",
        help="Directory that contains the result folders.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help=(
            "Directory where organized CSVs and plots will be written. "
            "Defaults to <results-dir>/<metric-name>."
        ),
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


def normalize_cli_token(text: str) -> str:
    normalized = re.sub(r"[\s-]+", "_", text.strip().lower())
    return re.sub(r"_+", "_", normalized).strip("_")


def build_metric_spec(metric_name: str) -> dict[str, str]:
    metric_column = METRIC_ALIASES.get(normalize_cli_token(metric_name), normalize_cli_token(metric_name))
    base_spec = METRIC_SPECS.get(metric_column, {})
    display_name = base_spec.get("display_name", pretty_name(metric_column).title())
    return {
        "column": metric_column,
        "display_name": display_name,
        "summary_ylabel": base_spec.get("summary_ylabel", display_name),
        "time_series_ylabel": base_spec.get("time_series_ylabel", display_name),
        "output_dir_name": base_spec.get("output_dir_name", sanitize_name(metric_column)),
    }


def base_method_name(method_path: str) -> str:
    return method_path.split("/", 1)[0] if method_path else ""


def scenario_match_sort_key(name: str) -> tuple[object, ...]:
    return (-len(name), *natural_key(name))


def discover_scenario_names(results_dir: Path) -> list[str]:
    scenario_names: set[str] = set()
    scenario_root = Path(__file__).resolve().parent.parent / "scenarios"

    if scenario_root.exists():
        for path in sorted(scenario_root.iterdir()):
            if path.is_dir():
                scenario_names.add(path.name)

    for file_path in sorted(results_dir.rglob("metrics*.csv")):
        rel_path = file_path.relative_to(results_dir)
        if len(rel_path.parts) >= 2 and rel_path.parts[1] in KNOWN_BASE_METHODS:
            scenario_names.add(rel_path.parts[0])

    return sorted(scenario_names, key=scenario_match_sort_key)


def strip_base_method_prefix(text: str, method_path: str) -> str:
    base_method = base_method_name(method_path)
    if not base_method:
        return text
    if text == base_method:
        return ""
    prefix = f"{base_method}_"
    if text.startswith(prefix):
        return text[len(prefix) :]
    return text


def split_config_and_run_id(metric_stem: str) -> tuple[str, str | None, int | None]:
    tokens = [token for token in metric_stem.split("_") if token]

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

    return "_".join(experiment_tokens), config, run_id


def split_scenario_and_experiment(
    experiment_text: str,
    scenario_names: list[str],
    scenario_hint: str | None = None,
) -> tuple[str | None, str]:
    normalized = normalize_cli_token(experiment_text)

    if scenario_hint:
        if not normalized or normalized == scenario_hint:
            return scenario_hint, scenario_hint
        prefix = f"{scenario_hint}_"
        if normalized.startswith(prefix):
            experiment = normalized[len(prefix) :]
            return scenario_hint, experiment or scenario_hint
        return scenario_hint, normalized

    for scenario_name in scenario_names:
        if normalized == scenario_name:
            return scenario_name, scenario_name
        prefix = f"{scenario_name}_"
        if normalized.startswith(prefix):
            experiment = normalized[len(prefix) :]
            return scenario_name, experiment or scenario_name

    return None, normalized or "unknown"


def normalize_experiment_label(experiment: str, scenario_group: str) -> str:
    normalized = normalize_cli_token(experiment)

    if not normalized:
        return scenario_group
    if normalized == f"agents_{scenario_group}":
        return scenario_group

    agent_group_match = re.fullmatch(r"(\d+)_agents?_group", normalized)
    if agent_group_match:
        return f"{agent_group_match.group(1)}_group"

    return normalized


def parse_summary_experiment(
    experiment_tag: str,
    scenario_names: list[str],
    method_hint: str | None = None,
    scenario_hint: str | None = None,
) -> tuple[str, str, str]:
    normalized_tag = normalize_cli_token(experiment_tag)
    method_path = method_hint or ""
    experiment_body = normalized_tag

    if method_path:
        experiment_body = strip_base_method_prefix(experiment_body, method_path)
    else:
        for base_method in KNOWN_BASE_METHODS:
            prefix = f"{base_method}_"
            if normalized_tag.startswith(prefix):
                method_path = base_method
                experiment_body = normalized_tag[len(prefix) :]
                break

    scenario_group, experiment = split_scenario_and_experiment(
        experiment_body,
        scenario_names,
        scenario_hint=scenario_hint,
    )

    if scenario_group is None:
        scenario_group = scenario_hint or experiment

    return scenario_group, method_path, normalize_experiment_label(experiment, scenario_group)


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


def parse_step_metadata(
    results_dir: Path,
    file_path: Path,
    scenario_names: list[str],
) -> dict[str, object]:
    rel_path = file_path.relative_to(results_dir)
    scenario_hint = None
    method_hint = None
    if len(rel_path.parts) >= 2 and rel_path.parts[1] in KNOWN_BASE_METHODS:
        scenario_hint = rel_path.parts[0]
        method_hint = "/".join(rel_path.parts[1:-1])

    stem = file_path.stem
    remainder = stem[len("metrics_steps_") :] if stem.startswith("metrics_steps_") else stem
    experiment_body, config, run_id = split_config_and_run_id(remainder)
    scenario_group, method_path, experiment = parse_summary_experiment(
        experiment_body,
        scenario_names,
        method_hint=method_hint,
        scenario_hint=scenario_hint,
    )
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


def load_step_series(results_dir: Path, metric_column: str) -> pd.DataFrame:
    series_frames: list[pd.DataFrame] = []
    scenario_names = discover_scenario_names(results_dir)
    for file_path in sorted(results_dir.rglob("metrics_steps*.csv")):
        meta = parse_step_metadata(results_dir, file_path, scenario_names)
        df = pd.read_csv(file_path)

        if metric_column not in df.columns:
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
                metric_column: pd.to_numeric(df[metric_column], errors="coerce"),
            }
        ).dropna(subset=["time_sec", metric_column])

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
                metric_column,
            ]
        )

    return pd.concat(series_frames, ignore_index=True)


def load_summary_records(results_dir: Path, metric_column: str) -> pd.DataFrame:
    records: list[dict[str, object]] = []
    scenario_names = discover_scenario_names(results_dir)
    central_summary_path = results_dir / "metrics.csv"
    summary_files: list[tuple[int, Path]] = []

    if central_summary_path.exists():
        summary_files.append((0, central_summary_path))

    for file_path in sorted(results_dir.rglob("metrics*.csv")):
        if file_path == central_summary_path or file_path.name.startswith("metrics_steps"):
            continue
        summary_files.append((1, file_path))

    for source_priority, file_path in summary_files:
        rel_path = file_path.relative_to(results_dir)
        scenario_hint = rel_path.parts[0] if len(rel_path.parts) >= 2 else None
        method_path = "/".join(rel_path.parts[1:-1]) if len(rel_path.parts) >= 2 else ""
        config = parse_summary_config(file_path)

        df = pd.read_csv(file_path)
        if metric_column not in df.columns:
            continue

        for row_index, row in df.iterrows():
            raw_experiment = str(row.get("experiment_tag", "")).strip()
            scenario_group, parsed_method_path, experiment = parse_summary_experiment(
                raw_experiment,
                scenario_names,
                method_hint=method_path or None,
                scenario_hint=scenario_hint,
            )
            method_label = format_method_label(parsed_method_path, config)

            records.append(
                {
                    "scenario_group": scenario_group,
                    "method_path": parsed_method_path,
                    "method_label": method_label,
                    "experiment": experiment,
                    "raw_experiment": raw_experiment,
                    "config": config,
                    "run_id": row.get("run_id"),
                    "source_file": str(rel_path),
                    "source_stem": file_path.stem,
                    "summary_row_index": row_index,
                    metric_column: pd.to_numeric(row.get(metric_column), errors="coerce"),
                    "experiment_inferred": False,
                    "scenario_harmonized": False,
                    "source_priority": source_priority,
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
                metric_column,
                "experiment_inferred",
                "scenario_harmonized",
                "source_priority",
            ]
        )

    summary_df = pd.DataFrame.from_records(records).dropna(subset=[metric_column]).copy()
    summary_df["run_id_dedup"] = summary_df["run_id"].astype("string").fillna("<missing>")
    summary_df = summary_df.sort_values(
        by=["source_priority", "scenario_group", "method_label", "experiment", "run_id_dedup"]
    ).drop_duplicates(
        subset=["scenario_group", "method_label", "experiment", "run_id_dedup"],
        keep="first",
    )
    return summary_df.drop(columns=["run_id_dedup", "source_priority"])


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


def summarize_metric(summary_df: pd.DataFrame, metric_column: str) -> pd.DataFrame:
    if summary_df.empty:
        return pd.DataFrame(
            columns=[
                "scenario_group",
                "experiment",
                "method_label",
                "record_count",
                metric_column,
            ]
        )

    aggregated = (
        summary_df.groupby(["scenario_group", "experiment", "method_label"], as_index=False)
        .agg(record_count=(metric_column, "size"), **{metric_column: (metric_column, "mean")})
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
    config_value = (
        float(config_match.group(1))
        if config_match and looks_like_float(config_match.group(1))
        else -1.0
    )
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
    metric_column: str,
    y_label: str,
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
                row[metric_column].iloc[0] if not row.empty else float("nan")
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
    ax.set_ylabel(y_label)
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
    metric_spec: dict[str, str],
) -> list[Path]:
    written_files: list[Path] = []
    metric_column = metric_spec["column"]
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
                .groupby("time_sec", as_index=False)[metric_column]
                .mean()
                .sort_values("time_sec")
            )
            ax.plot(
                method_group["time_sec"],
                method_group[metric_column],
                label=method,
                color=method_colors[method],
                linewidth=2,
            )

        ax.set_title(
            f"{metric_spec['display_name']} Over Time: "
            f"{pretty_name(scenario_group)} / {pretty_name(experiment)}"
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(metric_spec["time_series_ylabel"])
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
    metric_spec: dict[str, str],
) -> dict[str, object]:
    organized_dir = output_dir / "organized"
    plots_dir = output_dir / "plots"
    summary_plots_dir = plots_dir / "summary"
    time_series_dir = plots_dir / "time_series"
    metric_column = metric_spec["column"]

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
            f"{metric_spec['display_name']} Across All Scenarios",
            "scenario_experiment",
            lambda category: pretty_name(category),
            method_colors,
            metric_column,
            metric_spec["summary_ylabel"],
        )
        written_summary_plots.append(overall_plot_path)

        for scenario_group, scenario_df in summary_plot_df.groupby("scenario_group"):
            scenario_plot_path = summary_plots_dir / f"{sanitize_name(scenario_group)}.png"
            plot_grouped_bars(
                scenario_df,
                scenario_plot_path,
                f"{metric_spec['display_name']}: {pretty_name(scenario_group)}",
                "experiment",
                lambda category: pretty_name(category),
                method_colors,
                metric_column,
                metric_spec["summary_ylabel"],
            )
            written_summary_plots.append(scenario_plot_path)

    written_time_series = plot_time_series(step_df, time_series_dir, method_colors, metric_spec)

    return {
        "summary_catalog_path": summary_catalog_path,
        "summary_plot_data_path": summary_plot_path,
        "step_series_path": step_series_path,
        "summary_plot_paths": written_summary_plots,
        "time_series_paths": written_time_series,
    }


def main() -> None:
    args = parse_args()
    metric_spec = build_metric_spec(args.metric)
    results_dir = Path(args.results_dir).resolve()
    output_dir = (
        Path(args.output_dir).resolve()
        if args.output_dir
        else (results_dir / metric_spec["output_dir_name"]).resolve()
    )

    step_df = load_step_series(results_dir, metric_spec["column"])
    summary_df = load_summary_records(results_dir, metric_spec["column"])
    summary_df = reconcile_summary_experiments(summary_df, step_df)
    summary_df, step_df = harmonize_single_experiment_scenarios(summary_df, step_df)
    summary_plot_df = summarize_metric(summary_df, metric_spec["column"])

    output_info = write_outputs(output_dir, summary_df, summary_plot_df, step_df, metric_spec)

    print(f"Metric column: {metric_spec['column']}")
    print(f"Wrote summary catalog: {output_info['summary_catalog_path']}")
    print(f"Wrote plot data: {output_info['summary_plot_data_path']}")
    print(f"Wrote step series: {output_info['step_series_path']}")
    print(f"Generated {len(output_info['summary_plot_paths'])} summary plot(s)")
    print(f"Generated {len(output_info['time_series_paths'])} time-series plot(s)")


if __name__ == "__main__":
    main()
