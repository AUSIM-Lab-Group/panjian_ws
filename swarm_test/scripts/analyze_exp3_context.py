#!/usr/bin/env python3
"""Aggregate Exp3 context-modulation runs and plot beta curves."""

import argparse
import csv
from pathlib import Path
from statistics import mean, stdev

import yaml


CONTEXT_ORDER = ["static", "same_direction", "crossing", "frontal_approaching"]


def read_csv(path):
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def values(rows, field):
    out = []
    for row in rows:
        try:
            out.append(float(row[field]))
        except (KeyError, TypeError, ValueError):
            pass
    return out


def infer_context(meta):
    scenario = str(meta.get("scenario", ""))
    prefix = "Exp3_context_"
    if scenario.startswith(prefix):
        return scenario[len(prefix):]
    return str(meta.get("context", "unknown"))


def summarize_run(run_dir):
    meta_path = run_dir / "meta.yaml"
    if not meta_path.exists():
        return None
    meta = yaml.safe_load(meta_path.read_text(encoding="utf-8")) or {}
    if meta.get("experiment_id") != "Exp3_context_modulation":
        return None

    rows = read_csv(run_dir / "margin_guard_log.csv")
    if not rows:
        return None

    beta = values(rows, "beta_applied")
    beta_hat = values(rows, "beta_requested")
    mu = values(rows, "mu")
    cos_delta = values(rows, "cos_delta")
    ttc = values(rows, "ttc")
    ttc_norm = values(rows, "ttc_norm")
    rho_norm = values(rows, "rho_norm")
    h_ee = values(rows, "h_ee")
    h_see = values(rows, "h_see")

    return {
        "run_id": run_dir.name,
        "context": infer_context(meta),
        "n": len(rows),
        "mu_mean": mean(mu) if mu else "",
        "mu_max": max(mu) if mu else "",
        "beta_hat_mean": mean(beta_hat) if beta_hat else "",
        "beta_hat_max": max(beta_hat) if beta_hat else "",
        "beta_mean": mean(beta) if beta else "",
        "beta_max": max(beta) if beta else "",
        "cos_delta_mean": mean(cos_delta) if cos_delta else "",
        "ttc_mean": mean(ttc) if ttc else "",
        "ttc_norm_mean": mean(ttc_norm) if ttc_norm else "",
        "rho_norm_mean": mean(rho_norm) if rho_norm else "",
        "h_EE_min": min(h_ee) if h_ee else "",
        "h_SEE_min": min(h_see) if h_see else "",
        "run_dir": str(run_dir),
    }


def aggregate(rows):
    metrics = [
        "mu_mean", "mu_max", "beta_hat_mean", "beta_hat_max", "beta_mean",
        "beta_max", "cos_delta_mean", "ttc_mean", "ttc_norm_mean",
        "rho_norm_mean", "h_EE_min", "h_SEE_min",
    ]
    stats = []
    for context in CONTEXT_ORDER:
        group = [row for row in rows if row["context"] == context]
        if not group:
            continue
        item = {"context": context, "runs": len(group), "records": sum(int(row["n"]) for row in group)}
        for metric in metrics:
            vals = [row[metric] for row in group if isinstance(row[metric], float) or isinstance(row[metric], int)]
            if vals:
                item[f"{metric}_mean"] = mean(vals)
                item[f"{metric}_std"] = stdev(vals) if len(vals) > 1 else 0.0
        stats.append(item)
    return stats


def write_rows(path, rows, fields):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: format_value(row.get(field, "")) for field in fields})


def format_value(value):
    if isinstance(value, float):
        return f"{value:.6f}"
    return value


def rel_time(rows):
    times = values(rows, "time")
    if not times:
        return []
    t0 = times[0]
    return [t - t0 for t in times]


def plot_curves(rows, output_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        (output_dir / "plot_warning.txt").write_text(f"matplotlib unavailable: {exc}\n", encoding="utf-8")
        return []

    generated = []
    exemplar = {}
    for row in rows:
        exemplar.setdefault(row["context"], Path(row["run_dir"]))

    fig, axes = plt.subplots(3, 1, figsize=(8.5, 8), sharex=True)
    for context in CONTEXT_ORDER:
        run_dir = exemplar.get(context)
        if not run_dir:
            continue
        guard_rows = read_csv(run_dir / "margin_guard_log.csv")
        t = rel_time(guard_rows)
        if not t:
            continue
        axes[0].plot(t, values(guard_rows, "mu"), label=context)
        axes[1].plot(t, values(guard_rows, "beta_requested"), label=context)
        axes[2].plot(t, values(guard_rows, "beta_applied"), label=context)
    axes[0].set_ylabel("mu")
    axes[1].set_ylabel("beta_hat (m)")
    axes[2].set_ylabel("beta (m)")
    axes[2].set_xlabel("time (s)")
    for ax in axes:
        ax.grid(alpha=0.25)
        ax.legend()
    fig.tight_layout()
    path = output_dir / "context_mu_beta_curves.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    generated.append(path)

    stats = aggregate(rows)
    by_context = {row["context"]: row for row in stats}
    x = range(len(CONTEXT_ORDER))
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    ax.bar(
        list(x),
        [by_context.get(context, {}).get("beta_mean_mean", 0.0) for context in CONTEXT_ORDER],
        color=["#6b7280", "#3b82f6", "#f59e0b", "#ef4444"],
    )
    ax.set_xticks(list(x))
    ax.set_xticklabels(CONTEXT_ORDER, rotation=15, ha="right")
    ax.set_ylabel("beta mean (m)")
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    path = output_dir / "context_beta_bar.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    generated.append(path)
    return generated


def main():
    parser = argparse.ArgumentParser(description="Analyze Exp3 context-modulation runs")
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
        raise SystemExit(f"No Exp3 runs found under {args.run_root}")

    run_fields = [
        "run_id", "context", "n", "mu_mean", "mu_max", "beta_hat_mean",
        "beta_hat_max", "beta_mean", "beta_max", "cos_delta_mean",
        "ttc_mean", "ttc_norm_mean", "rho_norm_mean", "h_EE_min",
        "h_SEE_min", "run_dir",
    ]
    stat_fields = ["context", "runs", "records"]
    for metric in [
        "mu_mean", "mu_max", "beta_hat_mean", "beta_hat_max", "beta_mean",
        "beta_max", "cos_delta_mean", "ttc_mean", "ttc_norm_mean",
        "rho_norm_mean", "h_EE_min", "h_SEE_min",
    ]:
        stat_fields.extend([f"{metric}_mean", f"{metric}_std"])

    write_rows(output_dir / "exp3_run_metrics.csv", rows, run_fields)
    stats = aggregate(rows)
    write_rows(output_dir / "exp3_summary_stats.csv", stats, stat_fields)
    generated = plot_curves(rows, output_dir)

    print(f"Exp3 runs analyzed: {len(rows)}")
    print(f"Wrote: {output_dir / 'exp3_run_metrics.csv'}")
    print(f"Wrote: {output_dir / 'exp3_summary_stats.csv'}")
    for path in generated:
        print(f"Wrote: {path}")


if __name__ == "__main__":
    main()
