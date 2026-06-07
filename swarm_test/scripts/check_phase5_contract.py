#!/usr/bin/env python3
"""Static checks for the Phase 5 experiment contract."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def read(rel_path: str) -> str:
    return (ROOT / rel_path).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def check_guard_source(rel_path: str) -> None:
    src = read(rel_path)
    for status in ['"accept"', '"project"', '"fallback"', '"zero"']:
        require(status in src, f"{rel_path}: missing guard status {status}")
    for obsolete in ['"rollback"', '"rate_limited"']:
        require(obsolete not in src, f"{rel_path}: obsolete guard status {obsolete} still present")
    require(
        "std::min(beta_bar_val, std::max(0.0, h_ee - eta_))" in src,
        f"{rel_path}: guard_upper_bound must be b_i^eta = min(beta_bar, max(0, h_EE - eta))",
    )
    require(
        "h_see = h_ee - beta_final" in src,
        f"{rel_path}: h_SEE must be logged as h_EE - beta_final",
    )
    require(
        "r_sem = r_base + beta_final" in src,
        f"{rel_path}: semantic radius must be physical radius plus beta",
    )


def check_mpc_source() -> None:
    src = read("planner/mpc_secbf/src/mpc_secbf.cpp")
    require("safe_dist" not in src, "mpc_secbf.cpp: safe_dist should not be used")
    require("R_safe" not in src, "mpc_secbf.cpp: R_safe should not be used")
    require(
        "obs_radius - robot_radius_" in src,
        "mpc_secbf.cpp: h_EE must subtract obstacle radius and robot radius",
    )
    require(
        "h_SEE = h_EE - beta_i" in src,
        "mpc_secbf.cpp: h_SEE must subtract beta_i from h_EE",
    )


def main() -> int:
    checks = [
        lambda: check_guard_source("planner/semantic_guard/src/beta_guard_node.cpp"),
        lambda: check_guard_source("planner/semantic_guard/src/beta_ground_truth_node.cpp"),
        check_mpc_source,
    ]
    for check in checks:
        check()
    print("Phase 5 contract checks passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
