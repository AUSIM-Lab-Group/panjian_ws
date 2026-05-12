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


def verify_safety(csv_path, gamma, eps_max, delta_bar_beta):
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

    # Parameters
    theoretical_bound = -(eps_max + delta_bar_beta) / gamma
    print(f"Parameters:")
    print(f"  gamma = {gamma}")
    print(f"  eps_max = {eps_max}")
    print(f"  delta_bar_beta = {delta_bar_beta}")
    print(f"  Theoretical bound = -(eps_max + delta_bar_beta) / gamma = {theoretical_bound:.4f}")
    print(f"")

    # Statistics
    h_values = df['h_ee'].values
    h_min = h_values.min()
    h_mean = h_values.mean()
    h_std = h_values.std()

    print(f"h_EE Statistics:")
    print(f"  min(h_EE)  = {h_min:.4f}")
    print(f"  mean(h_EE) = {h_mean:.4f}")
    print(f"  std(h_EE)  = {h_std:.4f}")
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
        print(f"  max |delta_beta|: {np.abs(beta_req - beta_app).max():.4f}")
        print(f"")

    # Verification
    safety_satisfied = h_min > theoretical_bound
    print(f"{'='*60}")
    print(f"VERIFICATION RESULT:")
    print(f"  min(h_EE) = {h_min:.4f}")
    print(f"  Theoretical bound = {theoretical_bound:.4f}")
    print(f"  min(h_EE) > bound? {'YES' if safety_satisfied else 'NO'}")
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
        violations = df[df['h_ee'] < theoretical_bound]
        print(f"     Number of violating timesteps: {len(violations)}")
        if not violations.empty:
            print(f"     First violation at t={violations.iloc[0]['time']:.3f}s")

    print(f"{'='*60}")

    # Per-class breakdown
    if 'class' in df.columns:
        print(f"\nPer-class h_EE minimum:")
        for cls in df['class'].unique():
            cls_data = df[df['class'] == cls]
            print(f"  {cls:12s}: min(h)={cls_data['h_ee'].min():.4f}, "
                  f"mean(beta)={cls_data['beta_applied'].mean():.4f}, "
                  f"n={len(cls_data)}")

    return safety_satisfied


def main():
    parser = argparse.ArgumentParser(description="Verify safety bound from guard log")
    parser.add_argument("--csv", type=str, required=True, help="Path to guard_log.csv")
    parser.add_argument("--gamma", type=float, default=0.35, help="CBF decay rate")
    parser.add_argument("--eps_max", type=float, default=0.05, help="Max discretization error")
    parser.add_argument("--delta_bar_beta", type=float, default=0.3, help="Max single-step beta change")
    args = parser.parse_args()

    success = verify_safety(args.csv, args.gamma, args.eps_max, args.delta_bar_beta)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
