#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace semantic_guard {

struct SemanticContextWeights {
  double bias = 0.6;
  double head_on = 0.2;
  double ttc_norm = 0.15;
  double density_norm = 0.1;
};

struct SemanticPhiResult {
  bool valid = false;
  std::string reason = "invalid";
  double mu = 0.0;
  double beta_tilde = 0.0;
};

struct SemanticMarginUpdate {
  bool valid = false;
  std::string reason = "invalid";
  double mu = 0.0;
  double beta_tilde = 0.0;
  double available_margin = 0.0;
  double positive_increment_bound = 0.0;
  double beta_upper_bound = 0.0;
  double beta_pre = 0.0;
  bool category_bound_active = false;
  bool positive_increment_bound_active = false;
  bool available_margin_bound_active = false;
};

struct SemanticProjectionPolicy {
  bool enforce_category_bound = true;
  bool enforce_positive_increment_bound = true;
  bool enforce_available_margin_bound = true;
};

inline double clipUnit(double value) {
  return std::min(1.0, std::max(0.0, value));
}

inline bool validContextWeights(const SemanticContextWeights& weights) {
  return std::isfinite(weights.bias) && std::isfinite(weights.head_on) &&
         std::isfinite(weights.ttc_norm) &&
         std::isfinite(weights.density_norm) && weights.head_on >= 0.0 &&
         weights.ttc_norm >= 0.0 && weights.density_norm >= 0.0;
}

// Provisional explicit realization of the manuscript's abstract Phi:
// beta_tilde = B_bar(c) * clip(w0 + wh*f_head + wt*TTC_norm
//                              + wrho*rho_norm, 0, 1).
// Keeping Phi separate from the admissibility projection ensures that every
// semantic mode uses one auditable implementation of F05.
inline SemanticPhiResult computeProvisionalTeacherPhi(
    double beta_bar, double f_head, double ttc_norm, double density_norm,
    const SemanticContextWeights& weights) {
  SemanticPhiResult result;
  const bool finite = std::isfinite(beta_bar) && std::isfinite(f_head) &&
                      std::isfinite(ttc_norm) &&
                      std::isfinite(density_norm) &&
                      validContextWeights(weights);
  if (!finite) {
    result.reason = "non_finite_input";
    return result;
  }
  if (beta_bar < 0.0) {
    result.reason = "invalid_range";
    return result;
  }

  result.mu = clipUnit(weights.bias + weights.head_on * f_head +
                       weights.ttc_norm * ttc_norm +
                       weights.density_norm * density_norm);
  result.beta_tilde = beta_bar * result.mu;
  result.valid = true;
  result.reason = "ok";
  return result;
}

inline SemanticMarginUpdate projectTeacherSemanticMargin(
    double beta_tilde, double beta_max, double beta_previous,
    double delta_beta_positive, double h_eesm, double h_min,
    const SemanticProjectionPolicy& policy = SemanticProjectionPolicy()) {
  SemanticMarginUpdate result;
  const bool finite = std::isfinite(beta_tilde) && std::isfinite(beta_max) &&
                      std::isfinite(beta_previous) &&
                      std::isfinite(delta_beta_positive) &&
                      std::isfinite(h_eesm) && std::isfinite(h_min);
  const bool nonnegative = beta_tilde >= 0.0 && beta_max >= 0.0 &&
                           beta_previous >= 0.0 &&
                           delta_beta_positive > 0.0 && h_min >= 0.0;
  if (!finite || !nonnegative) {
    result.reason = finite ? "invalid_range" : "non_finite_input";
    return result;
  }

  result.beta_tilde = beta_tilde;
  result.available_margin = std::max(h_eesm - h_min, 0.0);
  result.positive_increment_bound = beta_previous + delta_beta_positive;
  result.beta_upper_bound = std::numeric_limits<double>::infinity();
  bool any_bound_enabled = false;
  if (policy.enforce_category_bound) {
    result.beta_upper_bound = std::min(result.beta_upper_bound, beta_max);
    any_bound_enabled = true;
  }
  if (policy.enforce_positive_increment_bound) {
    result.beta_upper_bound =
        std::min(result.beta_upper_bound, result.positive_increment_bound);
    any_bound_enabled = true;
  }
  if (policy.enforce_available_margin_bound) {
    result.beta_upper_bound =
        std::min(result.beta_upper_bound, result.available_margin);
    any_bound_enabled = true;
  }
  // Ablations that disable every F06 term still need a finite typed payload.
  // In that diagnostic-only case the candidate itself is the effective bound.
  if (!any_bound_enabled) result.beta_upper_bound = result.beta_tilde;
  result.beta_pre = std::min(result.beta_tilde, result.beta_upper_bound);

  const double scale = std::max(
      1.0, std::max(beta_max,
                    std::max(result.positive_increment_bound,
                             result.available_margin)));
  const double tolerance = 32.0 * std::numeric_limits<double>::epsilon() * scale;
  result.category_bound_active = policy.enforce_category_bound &&
      (!policy.enforce_positive_increment_bound ||
       beta_max <= result.positive_increment_bound + tolerance) &&
      (!policy.enforce_available_margin_bound ||
       beta_max <= result.available_margin + tolerance) &&
      result.beta_tilde > beta_max + tolerance;
  result.positive_increment_bound_active =
      policy.enforce_positive_increment_bound &&
      (!policy.enforce_category_bound ||
       result.positive_increment_bound <= beta_max + tolerance) &&
      (!policy.enforce_available_margin_bound ||
       result.positive_increment_bound <= result.available_margin + tolerance) &&
      result.beta_tilde > result.positive_increment_bound + tolerance;
  result.available_margin_bound_active =
      policy.enforce_available_margin_bound &&
      (!policy.enforce_category_bound ||
       result.available_margin <= beta_max + tolerance) &&
      (!policy.enforce_positive_increment_bound ||
       result.available_margin <= result.positive_increment_bound + tolerance) &&
      result.beta_tilde > result.available_margin + tolerance;
  result.valid = true;
  result.reason = "ok";
  return result;
}

inline SemanticMarginUpdate computeTeacherSemanticMargin(
    double beta_bar, double beta_max, double f_head, double ttc_norm,
    double density_norm, double beta_previous, double delta_beta_positive,
    double h_eesm, double h_min, const SemanticContextWeights& weights) {
  SemanticMarginUpdate result;
  const bool finite = std::isfinite(beta_max) &&
      std::isfinite(beta_previous) &&
      std::isfinite(delta_beta_positive) && std::isfinite(h_eesm) &&
      std::isfinite(h_min);
  const bool nonnegative = beta_max >= 0.0 && beta_previous >= 0.0 &&
                           delta_beta_positive > 0.0 && h_min >= 0.0;
  if (!finite || !nonnegative) {
    result.reason = finite ? "invalid_range" : "non_finite_input";
    return result;
  }

  const SemanticPhiResult phi = computeProvisionalTeacherPhi(
      beta_bar, f_head, ttc_norm, density_norm, weights);
  if (!phi.valid) {
    result.reason = phi.reason;
    return result;
  }
  result = projectTeacherSemanticMargin(
      phi.beta_tilde, beta_max, beta_previous, delta_beta_positive, h_eesm,
      h_min);
  result.mu = phi.mu;
  return result;
}

}  // namespace semantic_guard
