#!/usr/bin/env python3
"""Aggregate Exp2 category-aware runs into tables and figures."""

import argparse
import csv
import math
from pathlib import Path
from statistics import mean, stdev

import yaml


CLASS_ORDER = ["box", "adult", "child_like", "cyclist"]
METHOD_ORDER = ["Fixed_margin", "Category_only", "SEESM_Ours"]


def read_csv(path):
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def parse_class(meta):
    classes = meta.get("obstacle_classes", "")
    if isinstance(classes, list):
        return classes[0] if classes else "unknown"
    text = str(classes).strip()
    if text.startswith("[") and text.endswith("]"):
        text = text[1:-1]
    return text.split(",")[0].strip() if text else "unknown"


def float_values(rows, field):
    values = []
    for row in rows:
        try:
            values.append(float(row[field]))
        except (KeyError, TypeError, ValueError):
            pass
    return values


def path_length(robot_rows):
    points = []
    for row in robot_rows:
        try:
            points.append((float(row["x"]), float(row["y"])))
        except (KeyError, TypeError, ValueError):
            pass
    if len(points) < 2:
        return 0.0
    return sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(points, points[1:]))


def travel_time(robot_rows):
    times = float_values(robot_rows, "t")
    return max(times) - min(times) if len(times) >= 2 else 0.0


def success(robot_rows, goal):
    if not robot_rows:
        return 0
    last = robot_rows[-1]
    try:
        dx = float(last["x"]) - float(goal[0])
        dy = float(last["y"]) - float(goal[1])
    except (KeyError, TypeError, ValueError, IndexError):
        return 0
    return int(math.hypot(dx, dy) <= 0.8)


def summarize_run(run_dir):
    meta_path = run_dir / "meta.yaml"
    if not meta_path.exists():
        return None
    meta = yaml.safe_load(meta_path.read_text(encoding="utf-8")) or {}
    if meta.get("experiment_id") != "Exp2_category_aware":
        return None

    guard_rows = read_csv(run_dir / "margin_guard_log.csv")
    robot_rows = read_csv(run_dir / "robot_log.csv")
    obs_rows = read_csv(run_dir / "obstacle_log.csv")

    beta = float_values(guard_rows, "beta_applied")
    d_clear = []
    for row in obs_rows:
        try:
            d_clear.append(float(row["d_i"]) - float(row["radius"]) - float(meta.get("robot_radius", 0.4)))
        except (KeyError, TypeError, ValueError):
            pass

    return {
        "run_id": run_dir.name,
        "method": meta.get("method", meta.get("baseline_id", "")),
        "baseline_id": meta.get("baseline_id", ""),
        "class": parse_class(meta),
        "D_min": min(d_clear) if d_clear else "",
        "beta_mean": mean(beta) if beta else "",
        "beta_max": max(beta) if beta else "",
        "path_length": path_length(robot_rows),
        "travel_time": travel_time(robot_rows),
        "success": success(robot_rows, meta.get("goal", [0.0, 0.0, 0.0])),
        "run_dir": str(run_dir),
    }


def fmt(value):
    return f"{value:.6f}" if isinstance(value, float) else value


def write_rows(path, rows, fields):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: fmt(row.get(field, "")) for field in fields})


def aggregate(rows):
    metrics = ["D_min", "beta_mean", "beta_max", "path_length", "travel_time", "success"]
    stats = []
    for method in METHOD_ORDER:
        for cls in CLASS_ORDER:
            group = [row for row in rows if row["method"] == method and row["class"] == cls]
            if not group:
                continue
            item = {"method": method, "class": cls, "n": len(group)}
            for metric in metrics:
                values = [row[metric] for row in group if isinstance(row[metric], float) or isinstance(row[metric], int)]
                if values:
                    item[f"{metric}_mean"] = mean(values)
                    item[f"{metric}_std"] = stdev(values) if len(values) > 1 else 0.0
            stats.append(item)
    return stats


def plot_outputs(rows, output_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        (output_dir / "plot_warning.txt").write_text(f"matplotlib unavailable: {exc}\n", encoding="utf-8")
        return []

    generated = []
    stats = aggregate(rows)
    by_key = {(row["method"], row["class"]): row for row in stats}

    for metric, ylabel, filename in [
        ("D_min_mean", "D_min (m)", "dmin_by_class.png"),
        ("beta_mean_mean", "beta mean (m)", "beta_by_class.png"),
    ]:
        x = range(len(CLASS_ORDER))
        width = 0.25
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for offset, method in enumerate(METHOD_ORDER):
            values = [by_key.get((method, cls), {}).get(metric, 0.0) for cls in CLASS_ORDER]
            ax.bar([i + (offset - 1) * width for i in x], values, width=width, label=method)
        ax.set_xticks(list(x))
        ax.set_xticklabels(CLASS_ORDER)
        ax.set_ylabel(ylabel)
        ax.legend()
        ax.grid(axis="y", alpha=0.25)
        fig.tight_layout()
        path = output_dir / filename
        fig.savefig(path, dpi=160)
        plt.close(fig)
        generated.append(path)

    fig, axes = plt.subplots(2, 2, figsize=(9, 7), sharex=True, sharey=True)
    for ax, cls in zip(axes.flat, CLASS_ORDER):
        candidates = [row for row in rows if row["class"] == cls and row["method"] == "SEESM_Ours"]
        if not candidates:
            ax.set_title(cls)
            continue
        run_dir = Path(candidates[0]["run_dir"])
        robot_rows = read_csv(run_dir / "robot_log.csv")
        obs_rows = read_csv(run_dir / "obstacle_log.csv")
        rx = float_values(robot_rows, "x")
        ry = float_values(robot_rows, "y")
        ox = float_values(obs_rows, "x")
        oy = float_values(obs_rows, "y")
        ax.plot(rx, ry, label="robot", linewidth=2)
        ax.plot(ox, oy, label="obstacle", linewidth=1.5)
        ax.set_title(cls)
        ax.grid(alpha=0.25)
    axes.flat[0].legend()
    fig.supxlabel("x (m)")
    fig.supylabel("y (m)")
    fig.tight_layout()
    path = output_dir / "trajectory_by_class.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    generated.append(path)
    return generated


def main():
    parser = argparse.ArgumentParser(description="Analyze Exp2 category-aware runs")
    parser.add_argument("run_root", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    output_dir = args.output_dir or args.run_root / "analysis"
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for meta_path in sorted(args.run_root.glob("**/meta.yaml")):
        row = summarize_run(meta_path.parent)
        if row is not None:
            rows.append(row)
    if not rows:
        raise SystemExit(f"No Exp2 runs found under {args.run_root}")

    metric_fields = [
        "run_id", "method", "baseline_id", "class", "D_min", "beta_mean",
        "beta_max", "path_length", "travel_time", "success", "run_dir",
    ]
    stat_fields = ["method", "class", "n"]
    for metric in ["D_min", "beta_mean", "beta_max", "path_length", "travel_time", "success"]:
        stat_fields.extend([f"{metric}_mean", f"{metric}_std"])

    write_rows(output_dir / "exp2_run_metrics.csv", rows, metric_fields)
    stats = aggregate(rows)
    write_rows(output_dir / "exp2_summary_stats.csv", stats, stat_fields)
    generated = plot_outputs(rows, output_dir)

    print(f"Exp2 runs analyzed: {len(rows)}")
    print(f"Wrote: {output_dir / 'exp2_run_metrics.csv'}")
    print(f"Wrote: {output_dir / 'exp2_summary_stats.csv'}")
    for path in generated:
        print(f"Wrote: {path}")


if __name__ == "__main__":
    main()
