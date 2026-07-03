#!/usr/bin/env python3
"""
Safety Bound Verification Script
Verifies: inf_t h(X_t, obs_i) >= -(eps_max + delta_bar_beta) / gamma

Usage:
    python3 verify_safety_bound.py --csv guard_log.csv --gamma 0.35 --eps_max 0.05 --delta_bar_beta 0.3
"""
import argparse
import pandas as pd
import numpy as np
import sys
import os


def positive_delta_beta(df):
    """Return max positive beta increment per obstacle from the guard log."""
    if "beta_applied" not in df.columns:
        return 0.0
    id_col = "obs_id" if "obs_id" in df.columns else None
    if id_col is None:
        beta = df["beta_applied"].astype(float).values
        if len(beta) < 2:
            return 0.0
        return float(np.maximum(np.diff(beta), 0.0).max())

    max_delta = 0.0
    for _, group in df.sort_values(["obs_id", "time"]).groupby("obs_id"):
        beta = group["beta_applied"].astype(float).values
        if len(beta) < 2:
            continue
        max_delta = max(max_delta, float(np.maximum(np.diff(beta), 0.0).max()))
    return max_delta


def verify_safety(csv_path, gamma, eps_max, delta_bar_beta, h_column):
    """Verify the practical safety lower bound from guard log data."""
    if not os.path.exists(csv_path):
        print(f"ERROR: File not found: {csv_path}")
        return False

    df = pd.read_csv(csv_path)
    if df.empty:
        print("ERROR: CSV file is empty")
        return False

    print(f"{'='*60}")
    print(f"Safety Bound Verification")
    print(f"{'='*60}")
    print(f"Data file: {csv_path}")
    print(f"Total records: {len(df)}")
    print(f"Time span: {df['time'].max() - df['time'].min():.2f}s")
    print(f"")

    if h_column not in df.columns:
        print(f"ERROR: CSV does not contain requested h column: {h_column}")
        print(f"Available columns: {', '.join(df.columns)}")
        return False

    # Parameters
    theoretical_bound = -(eps_max + delta_bar_beta) / gamma
    observed_delta_beta = positive_delta_beta(df)
    print(f"Parameters:")
    print(f"  gamma = {gamma}")
    print(f"  eps_max = {eps_max}")
    print(f"  delta_bar_beta = {delta_bar_beta}")
    print(f"  observed max positive delta_beta = {observed_delta_beta:.4f}")
    print(f"  Theoretical bound = -(eps_max + delta_bar_beta) / gamma = {theoretical_bound:.4f}")
    print(f"")

    # Statistics
    h_values = df[h_column].values
    h_min = h_values.min()
    h_mean = h_values.mean()
    h_std = h_values.std()

    print(f"{h_column} Statistics:")
    print(f"  min({h_column})  = {h_min:.4f}")
    print(f"  mean({h_column}) = {h_mean:.4f}")
    print(f"  std({h_column})  = {h_std:.4f}")
    print(f"")

    # Guard statistics
    guard_passed = df['guard_passed'].sum()
    guard_failed = len(df) - guard_passed
    print(f"Guard Statistics:")
    print(f"  Total checks: {len(df)}")
    print(f"  Passed: {guard_passed} ({100*guard_passed/len(df):.1f}%)")
    print(f"  Rollbacks: {guard_failed} ({100*guard_failed/len(df):.1f}%)")
    print(f"")

    # Beta statistics
    if 'beta_requested' in df.columns and 'beta_applied' in df.columns:
        beta_req = df['beta_requested'].values
        beta_app = df['beta_applied'].values
        print(f"Beta Statistics:")
        print(f"  beta_requested: mean={beta_req.mean():.4f}, max={beta_req.max():.4f}")
        print(f"  beta_applied:   mean={beta_app.mean():.4f}, max={beta_app.max():.4f}")
        print(f"  max positive delta_beta: {observed_delta_beta:.4f}")
        print(f"  max |beta_requested - beta_applied|: {np.abs(beta_req - beta_app).max():.4f}")
        print(f"")

    # Verification
    safety_satisfied = h_min > theoretical_bound
    print(f"{'='*60}")
    print(f"VERIFICATION RESULT:")
    print(f"  min({h_column}) = {h_min:.4f}")
    print(f"  Theoretical bound = {theoretical_bound:.4f}")
    print(f"  min({h_column}) > bound? {'YES' if safety_satisfied else 'NO'}")
    print(f"")

    if safety_satisfied:
        print(f"  ✅ SAFETY GUARANTEED: Practical safety lower bound is satisfied.")
        margin = h_min - theoretical_bound
        print(f"     Margin: {margin:.4f}m")
    else:
        print(f"  ❌ SAFETY VIOLATED: h went below theoretical bound!")
        violation = theoretical_bound - h_min
        print(f"     Violation depth: {violation:.4f}m")
        # Find when violation occurred
        violations = df[df[h_column] < theoretical_bound]
        print(f"     Number of violating timesteps: {len(violations)}")
        if not violations.empty:
            print(f"     First violation at t={violations.iloc[0]['time']:.3f}s")

    print(f"{'='*60}")

    # Per-class breakdown
    if 'class' in df.columns:
        print(f"\nPer-class {h_column} minimum:")
        for cls in df['class'].unique():
            cls_data = df[df['class'] == cls]
            print(f"  {cls:12s}: min(h)={cls_data[h_column].min():.4f}, "
                  f"mean(beta)={cls_data['beta_applied'].mean():.4f}, "
                  f"n={len(cls_data)}")

    return safety_satisfied


def main():
    parser = argparse.ArgumentParser(description="Verify safety bound from guard log")
    parser.add_argument("--csv", type=str, required=True, help="Path to guard_log.csv")
    parser.add_argument("--gamma", type=float, default=0.35, help="CBF decay rate")
    parser.add_argument("--eps_max", type=float, default=0.05, help="Max discretization error")
    parser.add_argument("--delta_bar_beta", type=float, default=0.3, help="Max single-step beta change")
    parser.add_argument("--h-column", default="h_see", help="Guard-log safety column to verify")
    args = parser.parse_args()

    success = verify_safety(args.csv, args.gamma, args.eps_max, args.delta_bar_beta, args.h_column)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
