#!/usr/bin/env python3
"""Run the deterministic SEESM component-validation protocol from the paper."""

import argparse
import csv
import json
import math
from pathlib import Path


BETA_PRIORS = {
    "box": 0.20,
    "pedestrian": 0.75,
    "vehicle": 0.80,
    "cyclist": 0.90,
}
WEIGHTS = {"bias": 0.60, "heading": 0.20, "ttc": 0.15, "density": 0.10}
ETA = 0.10
MAX_DELTA_BETA = 0.30
L_REL = (-4.0, 0.0)
V_REL = (1.0, 0.0)
TAU = 1.0
ROBOT_RADIUS = 0.40
OBSTACLE_RADIUS = 0.40
DIRECTION_CONDITIONS = (
    ("moving-away", 180.0),
    ("crossing", 90.0),
    ("head-on", 0.0),
)


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


def fixed_h_eesm():
    shifted_x = L_REL[0] + TAU * V_REL[0]
    shifted_y = L_REL[1] + TAU * V_REL[1]
    return math.hypot(shifted_x, shifted_y) - ROBOT_RADIUS - OBSTACLE_RADIUS


def h_eesm_for_geometry(l_rel, v_rel):
    shifted_x = l_rel[0] + TAU * v_rel[0]
    shifted_y = l_rel[1] + TAU * v_rel[1]
    return math.hypot(shifted_x, shifted_y) - ROBOT_RADIUS - OBSTACLE_RADIUS


def direction_alignment(level, raw_value):
    canonical = {
        "moving-away": -1.0,
        "receding": -1.0,
        "crossing": 0.0,
        "ideal-crossing": 0.0,
        "tangential-crossing": 0.0,
        "head-on": 1.0,
        "frontal-approach": 1.0,
    }
    return canonical.get(level, float(raw_value))


def fixed_kernel_direction_geometry(raw_alignment):
    """Construct vectors with -l_hat dot v_hat=a and h_EESM=2.2 m.

    The relative-speed norm and tau are one.  With the target shifted-distance
    norm D=3 m, choose l=(r,0), v=(-a,sqrt(1-a^2)), where
    r=a+sqrt(a^2+D^2-1).  This keeps ||l+tau*v||=D for every a.
    """
    alignment = clamp(raw_alignment, -1.0, 1.0)
    target_shifted_norm = 2.2 + ROBOT_RADIUS + OBSTACLE_RADIUS
    radius = alignment + math.sqrt(
        alignment * alignment + target_shifted_norm * target_shifted_norm - 1.0
    )
    l_rel = (radius, 0.0)
    v_rel = (-alignment, math.sqrt(max(0.0, 1.0 - alignment * alignment)))
    return l_rel, v_rel


def head_on_factor(approach_angle_deg):
    """Return [cos(alpha)]+ for alpha measured from direct frontal approach."""
    value = max(0.0, math.cos(math.radians(approach_angle_deg)))
    return 0.0 if value <= 1e-12 else value


def conditions():
    rows = []
    for category in ("box", "pedestrian", "vehicle", "cyclist"):
        rows.append(("category", category, category, 0.0, 0.0, 0.0, ""))

    for level, approach_angle_deg in DIRECTION_CONDITIONS:
        rows.append((
            "direction",
            level,
            "pedestrian",
            head_on_factor(approach_angle_deg),
            0.0,
            0.0,
            approach_angle_deg,
        ))
    for level, ttc, value in (("long", 5.0, 0.0), ("medium", 2.5, 0.5), ("short", 1.0, 0.8)):
        rows.append(("ttc", level, "pedestrian", 0.0, value, 0.0, ttc))
    for level, count, value in (("N=1", 1, 0.0), ("N=3", 3, 0.4), ("N=6", 6, 1.0)):
        rows.append(("crowding", level, "pedestrian", 0.0, 0.0, value, count))
    return rows


def continuous_conditions():
    """Return dense one-factor sweeps while retaining canonical anchors."""
    rows = []
    for category in ("box", "pedestrian", "vehicle", "cyclist"):
        rows.append(("category", category, category, 0.0, 0.0, 0.0, ""))

    for index in range(41):
        raw_alignment = -1.0 + 0.05 * index
        if abs(raw_alignment + 1.0) <= 1e-12:
            level = "moving-away"
        elif abs(raw_alignment) <= 1e-12:
            level = "ideal-crossing"
        elif abs(raw_alignment - 1.0) <= 1e-12:
            level = "head-on"
        else:
            level = f"alignment_{raw_alignment:+.2f}"
        rows.append((
            "direction",
            level,
            "pedestrian",
            max(0.0, raw_alignment),
            0.0,
            0.0,
            raw_alignment,
        ))

    for index in range(23):
        ttc = 0.5 + 0.25 * index
        if abs(ttc - 5.0) <= 1e-12:
            level = "long"
        elif abs(ttc - 2.5) <= 1e-12:
            level = "medium"
        elif abs(ttc - 1.0) <= 1e-12:
            level = "short"
        else:
            level = f"ttc_{ttc:.2f}s"
        ttc_norm = clamp(1.0 - ttc / 5.0, 0.0, 1.0)
        rows.append(("ttc", level, "pedestrian", 0.0, ttc_norm, 0.0, ttc))

    for count in range(1, 7):
        rho_norm = min(1.0, (count - 1) / 5.0)
        rows.append((
            "crowding",
            f"N={count}",
            "pedestrian",
            0.0,
            0.0,
            rho_norm,
            count,
        ))
    return rows


def run_condition(family, level, category, f_head, ttc_norm, rho_norm, raw_value, cycles):
    l_rel = L_REL
    v_rel = V_REL
    if family == "direction":
        alignment = direction_alignment(level, raw_value)
        l_rel, v_rel = fixed_kernel_direction_geometry(alignment)
    h_eesm = h_eesm_for_geometry(l_rel, v_rel)
    beta_bar = BETA_PRIORS[category]
    mu = clamp(
        WEIGHTS["bias"]
        + WEIGHTS["heading"] * f_head
        + WEIGHTS["ttc"] * ttc_norm
        + WEIGHTS["density"] * rho_norm,
        0.0,
        1.0,
    )
    beta_candidate = beta_bar * mu
    beta_previous = 0.0
    trial_rows = []

    for cycle in range(cycles):
        beta_upper = min(beta_bar, beta_previous + MAX_DELTA_BETA, max(0.0, h_eesm - ETA))
        beta_pre_guard = min(beta_candidate, beta_upper)

        # This deterministic component check only verifies the analytical
        # available-clearance condition. Actual MPC acceptance and retry branches
        # are required as separate integrated ROS audit inputs to this script.
        clearance_feasible = h_eesm - beta_pre_guard >= ETA - 1e-12
        beta_accepted = beta_pre_guard if clearance_feasible else min(beta_previous, beta_upper)
        guard_status = "component_accept" if clearance_feasible else "component_fallback"
        h_seesm = h_eesm - beta_accepted

        trial_rows.append({
            "condition_id": f"{family}_{level}",
            "test_family": family,
            "level": level,
            "cycle": cycle,
            "class": category,
            "raw_value": raw_value,
            "l_x": l_rel[0],
            "l_y": l_rel[1],
            "v_rel_x": v_rel[0],
            "v_rel_y": v_rel[1],
            "tau": TAU,
            "robot_radius": ROBOT_RADIUS,
            "obstacle_radius": OBSTACLE_RADIUS,
            "f_head": f_head,
            "ttc_norm": ttc_norm,
            "rho_norm": rho_norm,
            "beta_bar": beta_bar,
            "mu": mu,
            "beta_candidate": beta_candidate,
            "beta_upper_bound": beta_upper,
            "beta_pre_guard": beta_pre_guard,
            "beta_accepted": beta_accepted,
            "component_clearance_feasible": int(clearance_feasible),
            "guard_status": guard_status,
            "h_eesm": h_eesm,
            "h_seesm": h_seesm,
        })
        beta_previous = beta_accepted
    return trial_rows


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def true_value(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def audit_integrated_guard(candidate_dir, retry_dir):
    candidate_rows = read_csv(candidate_dir / "mpc_margin_log.csv")
    retry_rows = read_csv(retry_dir / "mpc_margin_log.csv")
    if not candidate_rows or not retry_rows:
        raise ValueError("integrated Guard audit logs must be non-empty")

    candidate_accept_rows = [
        row for row in candidate_rows
        if true_value(row["mpc_feasibility_guard_enabled"])
        and true_value(row["candidate_feasibility_checked"])
        and row["first_attempt_status"] == "success"
        and row["final_status"] == "success"
        and row["accepted_beta_source"] == "candidate"
        and abs(float(row["beta_pre_guard"]) - float(row["beta_applied"])) <= 1e-8
    ]
    if len(candidate_accept_rows) != len(candidate_rows):
        raise ValueError("candidate-accept audit contains a row that does not prove MPC acceptance")

    retry_used_rows = [
        row for row in retry_rows
        if true_value(row["mpc_feasibility_guard_enabled"])
        and true_value(row["candidate_feasibility_checked"])
        and true_value(row["mpc_feasibility_guard_used"])
        and row["first_attempt_status"] == "infeasible"
    ]
    if not retry_used_rows:
        raise ValueError("retry audit does not contain a naturally triggered MPC Guard retry")

    return {
        "candidate_accept_dir": str(candidate_dir.resolve()),
        "candidate_accept_rows": len(candidate_accept_rows),
        "retry_dir": str(retry_dir.resolve()),
        "retry_total_rows": len(retry_rows),
        "retry_used_rows": len(retry_used_rows),
        "retry_sources": {
            source: sum(row["accepted_beta_source"] == source for row in retry_used_rows)
            for source in ("previous", "zero", "no_cbf")
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument(
        "--sweep",
        choices=("canonical", "continuous"),
        default="canonical",
        help="use the original 13 anchors or dense direction/TTC/crowding sweeps",
    )
    parser.add_argument("--candidate-accept-audit-dir", type=Path, required=True)
    parser.add_argument("--retry-audit-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.cycles < 3:
        parser.error("--cycles must be at least 3 so the rate-limit transient can settle")

    try:
        guard_audit = audit_integrated_guard(
            args.candidate_accept_audit_dir, args.retry_audit_dir
        )
    except (OSError, KeyError, TypeError, ValueError) as exc:
        parser.error(f"integrated Guard audit failed: {exc}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    all_rows = []
    summary_rows = []
    protocol_conditions = (
        continuous_conditions() if args.sweep == "continuous" else conditions()
    )
    for condition in protocol_conditions:
        rows = run_condition(*condition, cycles=args.cycles)
        all_rows.extend(rows)
        steady = dict(rows[-1])
        steady["candidate_error"] = steady["beta_accepted"] - steady["beta_candidate"]
        steady["fixed_h_eesm_error"] = steady["h_eesm"] - 2.2
        steady["passed"] = int(
            abs(steady["candidate_error"]) <= 1e-12
            and abs(steady["fixed_h_eesm_error"]) <= 1e-12
            and steady["guard_status"] == "component_accept"
        )
        summary_rows.append(steady)

    write_csv(args.output_dir / "component_trials.csv", all_rows)
    write_csv(args.output_dir / "component_summary.csv", summary_rows)
    metadata = {
        "protocol": "paper_B_SEESM_component_validation",
        "deterministic": True,
        "sweep_mode": args.sweep,
        "cycles_per_condition": args.cycles,
        "condition_count": len(summary_rows),
        "parameters": {
            "beta_priors": BETA_PRIORS,
            "weights": WEIGHTS,
            "eta": ETA,
            "max_delta_beta": MAX_DELTA_BETA,
            "direction_conditions": [
                {
                    "level": level,
                    "approach_angle_deg": angle,
                    "f_head": head_on_factor(angle),
                }
                for level, angle in DIRECTION_CONDITIONS
            ],
            "continuous_sweep": {
                "raw_alignment": {"min": -1.0, "max": 1.0, "step": 0.05},
                "ttc_seconds": {"min": 0.5, "max": 6.0, "step": 0.25},
                "crowding_count": {"min": 1, "max": 6, "step": 1},
            } if args.sweep == "continuous" else None,
            "direction_geometry": {
                "construction": "fixed-kernel vectors satisfying -l_hat_dot_v_hat=raw_alignment",
                "relative_speed_norm": 1.0,
                "target_shifted_distance_norm": 3.0,
                "expected_h_eesm": 2.2,
            },
            "l_rel": L_REL,
            "v_rel": V_REL,
            "tau": TAU,
            "robot_radius": ROBOT_RADIUS,
            "obstacle_radius": OBSTACLE_RADIUS,
        },
        "integrated_mpc_guard_audit": guard_audit,
    }
    (args.output_dir / "meta.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    passed = sum(row["passed"] for row in summary_rows)
    lines = [
        "# SEESM Component Validation Protocol Result",
        "",
        f"- Conditions passed: {passed}/{len(summary_rows)}",
        f"- Fixed h_EESM: {fixed_h_eesm():.6f} m",
        f"- Cycles per condition: {args.cycles}",
        f"- Sweep mode: {args.sweep}",
        "- Category used for interaction tests: pedestrian",
        "- Controlled stages logged: candidate, upper bound, pre-guard, clearance check, accepted margin",
        f"- Integrated candidate-accept audit rows: {guard_audit['candidate_accept_rows']}",
        f"- Integrated MPC Guard retry rows: {guard_audit['retry_used_rows']}",
        "",
        "| Family | Level | Candidate | Pre-guard | Accepted | h_SEESM | Guard |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    for row in summary_rows:
        lines.append(
            f"| {row['test_family']} | {row['level']} | {row['beta_candidate']:.6f} | "
            f"{row['beta_pre_guard']:.6f} | {row['beta_accepted']:.6f} | "
            f"{row['h_seesm']:.6f} | {row['guard_status']} |"
        )
    lines.extend(["", "See meta.json for the bound integrated-MPC Guard audit paths and counts.", ""])
    (args.output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    if passed != len(summary_rows):
        raise SystemExit(f"component protocol failed: {passed}/{len(summary_rows)}")
    print(f"component protocol passed: {passed}/{len(summary_rows)}")
    print(args.output_dir)


if __name__ == "__main__":
    main()
