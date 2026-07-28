#!/usr/bin/env python3
"""Generate the eight campaign/tier execution freezes used by Teacher-v1."""

import hashlib
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "config/experiment_freezes"
TEACHER_ROOT = ROOT.parents[1]
SOURCE_FREEZE_MANIFEST = (
    TEACHER_ROOT
    / "seesm_social_navigation/新计划实验输出目录/05_formal_preparation/"
      "20260728_regression_passed_source_freeze/SOURCE_FREEZE_MANIFEST.json"
)
COMMON_EVALUATOR = ROOT / "config/common_offline_evaluation_v1.yaml"

CAMPAIGNS = {
    "main": {
        "scenarios": [
            "head_on_context_bl", "head_on_context_int", "head_on_context_ext",
            "crossing_context_bl", "crossing_context_int", "crossing_context_ext",
            "local_crowding_context_bl", "local_crowding_context_int",
            "local_crowding_context_ext",
        ],
        "methods": [
            "Standard_MPC_CBF", "EESM_MPC_ECBF", "SEESM_Without_FPU",
            "Proposed_MPC_SECBF",
        ],
        "smoke_trials": 72,
        "formal_trials": 1080,
        "semantic_overrides": False,
        "solver": {"epsilon_max": 0.05, "slack_weight": 1000.0},
    },
    "ablation": {
        "scenarios": [
            "head_on_context_int", "crossing_context_int",
            "local_crowding_context_int",
        ],
        "methods": [
            "No_semantic", "Category_only", "Unguarded_SEESM", "No_J_side",
            "SEESM_Ours",
        ],
        "smoke_trials": 30,
        "formal_trials": 450,
        "semantic_overrides": False,
        "solver": {"epsilon_max": 0.05, "slack_weight": 1000.0},
    },
    "stress": {
        "scenarios": [
            "stress_high_candidate_margin", "stress_short_ttc",
            "stress_local_crowding",
        ],
        "methods": ["Unguarded_SEESM", "SEESM_Ours"],
        "smoke_trials": 12,
        "formal_trials": 180,
        "semantic_overrides": True,
        "solver": {"epsilon_max": 0.01, "slack_weight": 5000.0},
    },
    "runtime": {
        "scenarios": [
            "runtime_scaling_n1", "runtime_scaling_n2", "runtime_scaling_n4",
            "runtime_scaling_n6",
        ],
        "methods": ["EESM_MPC_ECBF", "Proposed_MPC_SECBF"],
        "smoke_trials": 16,
        "formal_trials": 240,
        "semantic_overrides": False,
        "solver": {"epsilon_max": 0.05, "slack_weight": 1000.0},
    },
}

BETA_BAR = {
    "box": 0.20,
    "adult": 0.75,
    "pedestrian": 0.75,
    "child": 1.05,
    "child_like": 1.05,
    "cyclist": 0.90,
    "vehicle": 0.80,
    "unknown": 0.75,
}


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def payload(campaign, tier, profile):
    seeds = 2 if tier == "smoke" else 30
    validated_recovery_profile = (
        tier == "formal" or
        (tier == "smoke" and campaign in {"main", "stress"})
    )
    freeze_version = "v2_20260728" if tier == "formal" else "v1_20260723"
    result = {
        "freeze_id": f"teacher_v1_{campaign}_{tier}_{freeze_version}",
        "status": f"{tier}_frozen",
        "execution_tier": tier,
        "campaign": campaign,
        "created_at": "2026-07-28" if tier == "formal" else "2026-07-23",
        "authority": {
            "definitions": "老师发的实验设置/最新指示/draft_V7_071.tex (non-red text)",
            "experiment_placeholders": "老师发的实验设置/最新指示/draft_V6.tex (Experimental Validation red text)",
            "experiment_design": "老师发的实验设置/最新指示/lu-xuran-mpc-secbf-experiment-design.md",
            "overall_plan": "老师发的实验设置/最新指示/整体计划.md",
        },
        "matrix": {
            "scenarios": profile["scenarios"],
            "requested_methods": profile["methods"],
            "paired_seeds_per_condition": seeds,
            "expected_trials": profile[f"{tier}_trials"],
            "duration_sec": 30,
            "preserve_scenario_semantic_overrides": profile["semantic_overrides"],
            "common_seed_policy": "same scenario/trial obstacle realization across all compared methods",
        },
        "robot_and_mpc": {
            "robot_radius_m": 0.4,
            "obstacle_radius_m": 0.4,
            "control_frequency_hz": 10.0,
            "control_period_sec": 0.10,
            "prediction_step_sec": 0.20,
            "horizon_steps": 20,
            "v_min_mps": -0.2,
            "v_max_mps": 1.5,
            "omega_max_radps": 0.8,
            "gamma": 0.35,
            "qf_scale": 1.1,
            "delta_u_weight": 0.02,
            "delta_u_max": 0.4,
            "max_cbf_obstacles": 6,
            "active_set_distance_m": 8.0,
            "graph_cache_enabled": validated_recovery_profile,
            **({"solver_max_cpu_time_ms": 120.0} if validated_recovery_profile else {}),
        },
        "semantic_margin": {
            "phi_status": f"explicit_linear_current_mapping_frozen_for_{tier}",
            "phi_weights": {
                "bias": 0.6, "head_on": 0.2, "ttc_norm": 0.15,
                "density_norm": 0.1,
            },
            "beta_bar_m": BETA_BAR,
            "beta_max_m": BETA_BAR,
            "h_min_m": 0.10,
            "delta_beta_positive_m_per_cycle": 0.30,
            "dynamic_tau": {
                "mode": "teacher_tca", "delta_tau": 1.0e-6,
                "max_tau_sec": 2.0,
            },
        },
        "guard_and_failure_policy": {
            "feasibility_guard_enabled": True,
            "kappa": 0.5,
            "max_backtracks_q": 6,
            "time_budget_ms": 200.0 if validated_recovery_profile else 500.0,
            **({"recovery_solver_max_cpu_time_ms": 70.0} if validated_recovery_profile else {}),
            "guard_binary_search": False,
            **({"bounded_midpoint_then_zero": True} if validated_recovery_profile else {}),
            "risk_order": "beta_tilde_descending_then_obstacle_id",
            "terminal_action": "emergency_cbf" if validated_recovery_profile else "safe_stop",
            "terminal_status": (
                "infeasible_emergency_cbf"
                if validated_recovery_profile else "infeasible_safe_stop"
            ),
            **({
                "emergency_cbf": {
                    "alpha": 1.5,
                    "extra_margin_m": 0.10,
                    "v_max_mps": 0.35,
                    "turn_gain": 1.5,
                    "activation_distance_m": 4.0,
                    "progress_v_mps": 0.20,
                }
            } if validated_recovery_profile else {}),
        },
        "scenario_specific_solver_settings": {
            "invariant_across_context_levels": True,
            "all_context_levels": profile["solver"],
        },
        "trial_outcome": {
            "goal_tolerance_m": 0.55,
            "timeout_sec": 30.0,
            "deadlock_speed_threshold_mps": 0.05,
            "deadlock_hold_sec": 2.0,
            "deadlock_requires_final_goal_distance_gt_tolerance": True,
        },
        "stability": {
            "tracking_error_threshold_m": 0.30,
            "tracking_hold_sec": 1.0,
            "settling_deadline_sec": 5.0,
            "final_goal_error_threshold_m": 0.55,
            "persistent_slack_threshold": 0.005,
            "persistent_slack_hold_sec": 1.0,
            "runtime_control_period_ms": 100.0,
        },
        "j_side": {
            "status": "optional_weak_preference_not_safety_metric",
            "enabled_for": [
                "Category_only", "SEESM_Without_FPU", "Unguarded_SEESM",
                "Proposed_MPC_SECBF", "SEESM_Ours",
            ],
            "disabled_for": [
                "Standard_MPC_CBF", "EESM_MPC_ECBF", "No_semantic",
                "No_J_side",
            ],
            "weight": 0.05,
            "activation_distance_m": 3.0,
        },
    }
    if tier == "formal":
        result["validated_recovery_implementation_contract"] = {
            "source_freeze_manifest": str(
                SOURCE_FREEZE_MANIFEST.relative_to(TEACHER_ROOT)
            ),
            "source_freeze_manifest_sha256": sha256_file(SOURCE_FREEZE_MANIFEST),
            "common_offline_evaluation": str(
                COMMON_EVALUATOR.relative_to(TEACHER_ROOT)
            ),
            "common_offline_evaluation_sha256": sha256_file(COMMON_EVALUATOR),
            "regression_evidence": (
                "seesm_social_navigation/新计划实验输出目录/04_fix_validation/"
                "local_crowding_recovery_adaptive_budget_regression_20260728"
            ),
            "main_solver_warm_start": (
                "retain_and_shift_last_successful_full_candidate_after_failure"
            ),
            "guard_solver_isolation": (
                "guard_solution_never_overwrites_main_full_candidate_warm_start"
            ),
            "guard_solver_launch_reserve_ms": {
                "single_obstacle": 5.0,
                "multiple_obstacles": 100.0,
            },
            "crossing_context_ext_static_box": False,
            "csv_audit_async_zero_kappa_feedback": "accepted_without_lifecycle_rebirth",
        }
    return result


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for campaign, profile in CAMPAIGNS.items():
        for tier in ("smoke", "formal"):
            path = OUTPUT / f"{campaign}_{tier}.yaml"
            rendered = yaml.safe_dump(
                payload(campaign, tier, profile),
                sort_keys=False,
                allow_unicode=True,
            )
            if tier == "smoke" and path.exists():
                existing = path.read_text(encoding="utf-8")
                if yaml.safe_load(existing) == yaml.safe_load(rendered):
                    print(f"{path} (unchanged)")
                    continue
            path.write_text(rendered, encoding="utf-8")
            print(path)


if __name__ == "__main__":
    main()
