#pragma once

#include <cmath>
#include <string>

namespace semantic_guard {

inline bool validateAppliedMarginFeedback(
    const std::string& source,
    double applied,
    double beta_max,
    double beta_pre,
    double beta_previous,
    bool enforce_category_bound,
    double tolerance = 1e-8) {
    if (!std::isfinite(applied) || applied < 0.0) {
        return false;
    }

    const bool known_source =
        source == "candidate" || source == "previous" ||
        source == "zero" || source == "kappa" ||
        source == "mpc_reprojected" || source == "no_cbf" ||
        source == "safe_stop" || source == "emergency_cbf";
    if (!known_source) {
        return false;
    }

    if (enforce_category_bound && applied > beta_max + tolerance) {
        return false;
    }

    if (source == "candidate") {
        return std::abs(applied - beta_pre) <= tolerance;
    }
    if (source == "previous") {
        return std::abs(applied - beta_previous) <= tolerance;
    }
    if (source == "zero" || source == "no_cbf" ||
        source == "safe_stop" || source == "emergency_cbf") {
        return std::abs(applied) <= tolerance;
    }
    return applied <= beta_pre + tolerance;
}

}  // namespace semantic_guard
