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
    for param in [
        "semantic_mode_",
        "enable_rate_limit_",
        "enable_available_projection_",
        "enable_guard_fallback_",
        "fixed_beta_",
    ]:
        require(param in src, f"{rel_path}: missing ablation parameter {param}")
    for field in ["semantic_mode", "delta_beta", "rate_limit_active", "projection_active"]:
        require(field in src, f"{rel_path}: margin guard CSV missing field {field}")
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
    header = read("planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h")
    node = read("planner/mpc_secbf/src/mpc_secbf_node.cpp")
    require("safe_dist" not in src, "mpc_secbf.cpp: safe_dist should not be used")
    require("R_safe" not in src, "mpc_secbf.cpp: R_safe should not be used")
    require(
        "obs_radius - robot_radius_" in src,
        "mpc_secbf.cpp: h_EE must subtract obstacle radius and robot radius",
    )
    require(
        "dynamicTauCasadi" in src,
        "mpc_secbf.cpp: dynamic h_cbf must use dynamicTauCasadi",
    )
    require(
        "if (!config_valid)" in src and "return casadi::MX(0.0);" in src,
        "mpc_secbf.cpp: invalid dynamic tau config must return symbolic zero early",
    )
    require(
        "casadi::MX lookahead_x = lx + tau * vx" in src and
        "casadi::MX lookahead_y = ly + tau * vy" in src,
        "mpc_secbf.cpp: dynamic h_cbf must use l + tau*v lookahead",
    )
    require(
        "- obs_radius - robot_radius_ - beta_i" in src,
        "mpc_secbf.cpp: dynamic h_cbf must subtract both radii and beta_i",
    )
    require(
        "return casadi::MX::sqrt(lx * lx + ly * ly) - obs_radius - robot_radius_ - beta_i;" in src,
        "mpc_secbf.cpp: instantaneous h_cbf must subtract beta_i",
    )
    require(
        "h_SEE = h_EE - beta_i" not in src,
        "mpc_secbf.cpp: obsolete h_SEE = h_EE - beta_i literal remains",
    )
    for symbol in ["last_slack_sum", "last_slack_mean", "last_slack_max"]:
        require(symbol in header or symbol in src, f"mpc_secbf: missing slack metric {symbol}")
    require("epsilon" in src, "mpc_secbf.cpp: SECBF constraints must use explicit slack variables")
    for field in ["first_attempt_status", "final_status", "accepted_beta_source", "slack_max"]:
        require(field in node, f"mpc_secbf_node.cpp: planner CSV missing field {field}")
    require(
        "const bool obstacle_contract_valid = validateObstacleContractLocked();" in node and
        "const int constrained_obs_count = obstacle_contract_valid" in node and
        "!obstacle_contract_valid" in node,
        "mpc_secbf_node.cpp: audit must reject stale solver obstacle state on contract mismatch",
    )
    require(
        "resetAuditMetrics" in header and "solver_.resetAuditMetrics();" in node,
        "mpc_secbf: audit metrics must be reset before invalid-cycle logging",
    )
    require(
        "accepted_beta_ids_ == obstacle_ids_" in node and
        "accepted_beta_ids_ = obstacle_ids_" in node and
        "accepted_beta_list_.clear()" in node,
        "mpc_secbf_node.cpp: previous beta fallback must be bound to obstacle IDs",
    )
    require(
        "std::isfinite" in node,
        "mpc_secbf_node.cpp: payload validation must reject non-finite values",
    )


def check_global_csv_source() -> None:
    src = read("planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp")
    header_chunks = [
        '"t,replan_id,global_seesm_enable,obs_id,beta_applied,accepted_source,"',
        '"margin_age_ms,h_ee,h_see,primitive_rejected,shot_rejected,reason,global_replan_ms,"',
        '"tau,T_i,f_r,f_v,f_T,tau_valid,tau_reason\\n"',
    ]
    for chunk in header_chunks:
        require(chunk in src, f"obs_manager.hpp: global SEESM CSV missing header chunk {chunk}")
    for field in [
        "tau_result.tau", "tau_result.T_i", "tau_result.f_r", "tau_result.f_v",
        "tau_result.f_T", "tau_result.valid",
    ]:
        require(field in src, f"obs_manager.hpp: global SEESM CSV missing {field}")
    for expression in [
        "sanitizeCsvField(accepted_source)",
        "sanitizeCsvField(reason)",
        "sanitizeCsvField(tau_result.reason)",
    ]:
        require(expression in src, f"obs_manager.hpp: CSV field must use {expression}")


def check_safety_verifier() -> None:
    src = read("swarm_test/scripts/verify_safety_bound.py")
    require("h_see" in src, "verify_safety_bound.py: theoretical bound must check h_see/H_i,t")
    require("positive_delta_beta" in src or "delta_beta_pos" in src,
            "verify_safety_bound.py: must compute positive beta increments")


def main() -> int:
    checks = [
        lambda: check_guard_source("planner/semantic_guard/src/beta_guard_node.cpp"),
        lambda: check_guard_source("planner/semantic_guard/src/beta_ground_truth_node.cpp"),
        check_mpc_source,
        check_global_csv_source,
        check_safety_verifier,
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
