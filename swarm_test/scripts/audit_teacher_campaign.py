#!/usr/bin/env python3
"""Fail-closed audit for one Teacher-v1 smoke or formal campaign."""

import argparse
from collections import defaultdict
import csv
import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
RUNNER_PATH = SCRIPT_DIR / "run_secbf_sim_experiments.py"
CHECKER_PATH = SCRIPT_DIR / "check_experiment_csv_fields.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("teacher_campaign_runner", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def first_csv_row(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return next(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--campaign", required=True)
    parser.add_argument("--tier", choices=("smoke", "formal"), required=True)
    args = parser.parse_args()

    runner = load_runner()
    profile = runner.CAMPAIGN_PROFILES.get(args.campaign)
    if profile is None:
        raise SystemExit(f"unknown campaign: {args.campaign}")
    expected_trials = profile[f"{args.tier}_trials"]
    expected_methods = {
        runner.resolve_baseline_alias(method) for method in profile["methods"]
    }
    expected_groups = expected_trials // len(expected_methods)

    root = args.root.resolve()
    run_dirs = sorted(
        path for path in root.iterdir()
        if path.is_dir() and not path.name.startswith(".")
    )
    if len(run_dirs) != expected_trials:
        raise SystemExit(
            f"expected {expected_trials} run directories, found {len(run_dirs)}"
        )

    paired = defaultdict(list)
    for run_dir in run_dirs:
        subprocess.run(
            [sys.executable, str(CHECKER_PATH), str(run_dir)],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        summary = first_csv_row(run_dir / "summary.csv")
        if str(summary.get("trial_valid", "")).strip() not in {"1", "true", "True"}:
            raise SystemExit(f"invalid trial contract: {run_dir}")
        if summary.get("termination_reason") == "invalid":
            raise SystemExit(f"invalid termination reason: {run_dir}")

        meta = yaml.safe_load((run_dir / "meta.yaml").read_text(encoding="utf-8"))
        if meta.get("campaign") != args.campaign:
            raise SystemExit(f"campaign metadata mismatch: {run_dir}")
        if meta.get("execution_tier") != args.tier:
            raise SystemExit(f"execution tier mismatch: {run_dir}")
        trial_id = meta["trial_manifest"]["trial_id"]
        digest = hashlib.sha256(
            (run_dir / "obstacles_param.yaml").read_bytes()
        ).hexdigest()
        paired[(meta["scenario"], trial_id)].append(
            (meta["baseline_id"], digest)
        )

    if len(paired) != expected_groups:
        raise SystemExit(
            f"expected {expected_groups} paired groups, found {len(paired)}"
        )
    for key, values in paired.items():
        if {baseline for baseline, _ in values} != expected_methods:
            raise SystemExit(f"incomplete method group {key}: {values}")
        if len({digest for _, digest in values}) != 1:
            raise SystemExit(f"paired obstacle hash mismatch {key}: {values}")

    sentinel = root / f"{args.campaign.upper()}_{args.tier.upper()}_AUDIT_PASS.txt"
    sentinel.write_text(
        f"{expected_trials}/{expected_trials} valid field contracts passed\n"
        f"{expected_groups}/{expected_groups} paired obstacle hashes passed\n",
        encoding="utf-8",
    )
    print(sentinel)


if __name__ == "__main__":
    main()
