#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace semantic_guard {

// Runtime/offline audit of the one-step inequality in Teacher-v1 Theorem 1.
// The helper deliberately does not certify the theorem: callers must supply
// the first-step CBF/slack and backup provenance flags.  It only recomputes
// the measurable terms and exposes an explicit residual for the CSV contract.
struct SafetyRecurrenceInput {
    double gamma = std::numeric_limits<double>::quiet_NaN();
    double h_eesm_t = std::numeric_limits<double>::quiet_NaN();
    double beta_t = std::numeric_limits<double>::quiet_NaN();
    double h_eesm_pred_next = std::numeric_limits<double>::quiet_NaN();
    double h_eesm_next = std::numeric_limits<double>::quiet_NaN();
    double beta_next = std::numeric_limits<double>::quiet_NaN();
    double epsilon_t = std::numeric_limits<double>::quiet_NaN();
    double epsilon_max = std::numeric_limits<double>::quiet_NaN();
    double delta_bar = std::numeric_limits<double>::quiet_NaN();
    double delta_beta_bar = std::numeric_limits<double>::quiet_NaN();
    bool cbf_executed = false;
    bool backup_used = false;
    bool baseline_infeasible = false;
};

struct SafetyRecurrenceAudit {
    bool finite = false;
    bool theorem1_applicable = false;
    double H_t = std::numeric_limits<double>::quiet_NaN();
    double H_pred_next = std::numeric_limits<double>::quiet_NaN();
    double H_next = std::numeric_limits<double>::quiet_NaN();
    double delta = std::numeric_limits<double>::quiet_NaN();
    double delta_beta_plus = std::numeric_limits<double>::quiet_NaN();
    double recursion_rhs = std::numeric_limits<double>::quiet_NaN();
    double one_step_residual = std::numeric_limits<double>::quiet_NaN();
    double bar_w = std::numeric_limits<double>::quiet_NaN();
    double asymptotic_bound = std::numeric_limits<double>::quiet_NaN();
};

// The finite-horizon lower bound from Eq. (closed_loop_practical_safety_bound).
// Returns NaN for invalid inputs rather than silently producing a bound from
// an inapplicable cycle.
inline double practicalSafetyBound(double H_t, double gamma, double bar_w,
                                   std::size_t horizon_steps) {
    if (!std::isfinite(H_t) || !std::isfinite(gamma) || gamma <= 0.0 ||
        gamma > 1.0 || !std::isfinite(bar_w) || bar_w < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double contraction = std::pow(1.0 - gamma,
                                        static_cast<double>(horizon_steps));
    return contraction * H_t -
           (1.0 - contraction) / gamma * bar_w;
}

inline SafetyRecurrenceAudit auditSafetyRecurrence(
    const SafetyRecurrenceInput& input) {
    SafetyRecurrenceAudit audit;
    const bool finite =
        std::isfinite(input.gamma) && input.gamma > 0.0 && input.gamma <= 1.0 &&
        std::isfinite(input.h_eesm_t) && std::isfinite(input.beta_t) &&
        std::isfinite(input.h_eesm_pred_next) && std::isfinite(input.h_eesm_next) &&
        std::isfinite(input.beta_next) && std::isfinite(input.epsilon_t) &&
        std::isfinite(input.epsilon_max) && input.epsilon_max >= 0.0 &&
        std::isfinite(input.delta_bar) && input.delta_bar >= 0.0 &&
        std::isfinite(input.delta_beta_bar) && input.delta_beta_bar >= 0.0;
    audit.finite = finite;
    if (!finite) return audit;

    // H_t = h_EESM(X_t)-beta_t; H_{t+1|t} uses the margin frozen during
    // the current MPC solve, while H_{t+1} uses the next cycle's accepted
    // margin.  These are exactly the quantities in draft_V7_071.tex.
    audit.H_t = input.h_eesm_t - input.beta_t;
    audit.H_pred_next = input.h_eesm_pred_next - input.beta_t;
    audit.H_next = input.h_eesm_next - input.beta_next;
    audit.delta = std::abs(input.h_eesm_next - input.h_eesm_pred_next);
    audit.delta_beta_plus = std::max(input.beta_next - input.beta_t, 0.0);
    audit.recursion_rhs = (1.0 - input.gamma) * audit.H_t - input.epsilon_t -
                          audit.delta_beta_plus - audit.delta;
    audit.one_step_residual = audit.H_next - audit.recursion_rhs;
    audit.bar_w = input.epsilon_max + input.delta_beta_bar + input.delta_bar;
    audit.asymptotic_bound = -audit.bar_w / input.gamma;
    audit.theorem1_applicable =
        input.cbf_executed && !input.backup_used && !input.baseline_infeasible &&
        input.epsilon_t >= 0.0 && input.epsilon_t <= input.epsilon_max + 1e-12 &&
        audit.delta_beta_plus <= input.delta_beta_bar + 1e-12 &&
        audit.delta <= input.delta_bar + 1e-12;
    return audit;
}

}  // namespace semantic_guard
