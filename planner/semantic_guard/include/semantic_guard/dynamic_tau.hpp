#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace semantic_guard {

// The mode names are part of the experiment contract.  Keep parsing strict so
// that a misspelled launch parameter cannot silently select another formula.
enum class DynamicTauMode {
  kLegacyGate = 0,
  kTeacherTca = 1,
  kTeacherKeTca = 2,
};

inline const char* dynamicTauModeName(DynamicTauMode mode) {
  switch (mode) {
    case DynamicTauMode::kLegacyGate:
      return "legacy_gate";
    case DynamicTauMode::kTeacherTca:
      return "teacher_tca";
    case DynamicTauMode::kTeacherKeTca:
      return "teacher_ke_tca";
  }
  return "invalid";
}

inline bool parseDynamicTauMode(const std::string& name,
                                DynamicTauMode* mode) {
  if (mode == nullptr) {
    return false;
  }
  if (name == "legacy_gate") {
    *mode = DynamicTauMode::kLegacyGate;
    return true;
  }
  if (name == "teacher_tca") {
    *mode = DynamicTauMode::kTeacherTca;
    return true;
  }
  if (name == "teacher_ke_tca") {
    *mode = DynamicTauMode::kTeacherKeTca;
    return true;
  }
  return false;
}

inline bool dynamicTauModeValid(DynamicTauMode mode) {
  return mode == DynamicTauMode::kLegacyGate ||
         mode == DynamicTauMode::kTeacherTca ||
         mode == DynamicTauMode::kTeacherKeTca;
}

struct DynamicTauParams {
  // Teacher-v1 is intentionally the default.  Legacy experiments must opt in
  // explicitly with mode=legacy_gate.
  DynamicTauMode mode = DynamicTauMode::kTeacherTca;
  double ke = 0.30;
  double t_max = 2.0;
  double min_speed = 1e-6;
  double min_distance = 1e-6;
  double max_tau = 2.0;
  double delta_tau = 1e-6;
};

struct DynamicTauResult {
  DynamicTauMode mode = DynamicTauMode::kTeacherTca;
  double tau = 0.0;

  // Backward-compatible diagnostic fields.  T_i retains its legacy meaning
  // in legacy_gate and aliases t_ca_clipped in either teacher mode.
  double T_i = 0.0;
  double f_r = 0.0;
  double f_v = 0.0;
  double f_T = 0.0;
  double cos_delta = 0.0;

  // Teacher-v1 audit diagnostics.
  double relative_dot = 0.0;
  double speed_squared = 0.0;
  double denominator = 0.0;
  double t_ca_raw = 0.0;
  double t_ca_clipped = 0.0;
  double tau_unclipped = 0.0;
  bool lower_clipped = false;
  bool upper_clipped = false;
  bool ke_scaled = false;

  bool closing = false;
  bool inputs_finite = false;
  bool config_valid = false;
  // computed means the selected formula was evaluated without an input,
  // configuration, or arithmetic failure. A normal clipped value tau=0 is
  // therefore computed=true but valid=false.
  bool computed = false;
  // valid retains the legacy API meaning: a strictly positive tau is active.
  bool valid = false;
  std::string reason = "invalid";
};

namespace detail {

inline bool dynamicTauFinite(double value) {
  return std::isfinite(value);
}

inline double dynamicTauPositiveSign(double value) {
  return value > 0.0 ? 1.0 : 0.0;
}

inline DynamicTauResult computeLegacyDynamicTau(
    double lx, double ly, double vx, double vy, double inflated_radius,
    const DynamicTauParams& params) {
  DynamicTauResult result;
  result.mode = DynamicTauMode::kLegacyGate;
  const double values[] = {lx, ly, vx, vy, inflated_radius, params.ke,
                           params.t_max, params.min_speed,
                           params.min_distance, params.max_tau};
  for (double value : values) {
    if (!dynamicTauFinite(value)) {
      result.reason = "non_finite_input";
      return result;
    }
  }
  result.inputs_finite = true;

  // These are the exact Legacy-v1 admissibility rules.  In particular, zero
  // ke/t_max/min thresholds remain accepted for reproducibility.
  if (inflated_radius < 0.0 || params.ke < 0.0 || params.t_max < 0.0 ||
      params.min_speed < 0.0 || params.min_distance < 0.0 ||
      params.max_tau <= 0.0) {
    result.reason = "invalid_config";
    return result;
  }
  result.config_valid = true;

  const double distance = std::hypot(lx, ly);
  const double speed = std::hypot(vx, vy);
  if (distance <= params.min_distance) {
    result.reason = "distance_degenerate";
    return result;
  }
  if (speed <= params.min_speed) {
    result.reason = "speed_degenerate";
    return result;
  }

  // Normalize before forming the angle so finite, very large coordinates do
  // not overflow the dot product used for cos_delta.
  const double nx = lx / distance;
  const double ny = ly / distance;
  const double nvx = vx / speed;
  const double nvy = vy / speed;
  result.cos_delta = nx * nvx + ny * nvy;
  if (!dynamicTauFinite(result.cos_delta)) {
    result.reason = "angle_invalid";
    return result;
  }

  // Legacy convention: l=p_obstacle-p_robot and
  // v=v_obstacle-v_robot. Closing motion therefore has cos_delta < 0.
  result.closing = result.cos_delta < 0.0;
  result.f_r = dynamicTauPositiveSign(-result.cos_delta);

  const double dot = lx * vx + ly * vy;
  const double cone_value =
      dot * dot + (inflated_radius * inflated_radius - distance * distance) *
                       speed * speed;
  if (dynamicTauFinite(cone_value)) {
    result.f_v = dynamicTauPositiveSign(cone_value);
  } else {
    // The expression above is the specified cone gate. This equivalent
    // scaled sign check keeps its result defined when finite inputs overflow.
    const double cross_unit = nx * nvy - ny * nvx;
    const double lateral_ratio = std::min(1.0, std::abs(cross_unit));
    const double lateral_distance = distance * lateral_ratio;
    result.f_v =
        dynamicTauPositiveSign(inflated_radius - lateral_distance);
  }

  const double approach_cos = std::max(0.0, -result.cos_delta);
  const double clearance = std::max(0.0, distance - inflated_radius);
  const double time_numerator = clearance * approach_cos;
  result.T_i = time_numerator / speed;
  if (!dynamicTauFinite(result.T_i)) {
    result.T_i = 0.0;
    result.reason = "tau_invalid";
    return result;
  }

  result.f_T = dynamicTauPositiveSign(params.t_max - result.T_i);
  result.computed = true;
  if (result.T_i <= 0.0 || result.f_r == 0.0) {
    result.reason = "receding_or_nonclosing";
    return result;
  }

  if (result.f_v == 0.0) {
    result.reason = "velocity_gate";
    return result;
  }
  if (result.f_T == 0.0) {
    result.reason = "time_gate";
    return result;
  }

  const double raw_tau =
      result.f_r * result.f_v * result.f_T * params.ke * result.T_i;
  if (!dynamicTauFinite(raw_tau) || raw_tau <= 0.0) {
    result.reason = "tau_invalid";
    return result;
  }

  result.tau = std::min(raw_tau, params.max_tau);
  result.valid = dynamicTauFinite(result.tau) && result.tau >= 0.0;
  result.reason = result.valid ? "active" : "tau_invalid";
  if (!result.valid) {
    result.tau = 0.0;
  }
  return result;
}

inline DynamicTauResult computeTeacherDynamicTau(
    double lx, double ly, double vx, double vy, double inflated_radius,
    const DynamicTauParams& params) {
  DynamicTauResult result;
  result.mode = params.mode;
  result.ke_scaled = params.mode == DynamicTauMode::kTeacherKeTca;

  const double common_values[] = {lx, ly, vx, vy, inflated_radius,
                                  params.max_tau, params.delta_tau};
  for (double value : common_values) {
    if (!dynamicTauFinite(value)) {
      result.reason = "non_finite_input";
      return result;
    }
  }
  if (result.ke_scaled && !dynamicTauFinite(params.ke)) {
    result.reason = "non_finite_input";
    return result;
  }
  result.inputs_finite = true;

  // Only parameters used by the selected teacher formula are validated here;
  // legacy-only t_max/min_distance cannot alter teacher_tca numerics.
  if (inflated_radius < 0.0 || params.max_tau <= 0.0 ||
      params.delta_tau <= 0.0 ||
      (result.ke_scaled && params.ke <= 0.0)) {
    result.reason = "invalid_config";
    return result;
  }
  result.config_valid = true;

  result.relative_dot = std::fma(lx, vx, ly * vy);
  result.speed_squared = std::fma(vx, vx, vy * vy);
  result.denominator = result.speed_squared + params.delta_tau;
  if (!dynamicTauFinite(result.relative_dot) ||
      !dynamicTauFinite(result.speed_squared) ||
      !dynamicTauFinite(result.denominator) || result.denominator <= 0.0) {
    result.reason = "arithmetic_invalid";
    return result;
  }

  const double distance = std::hypot(lx, ly);
  const double speed = std::hypot(vx, vy);
  if (!dynamicTauFinite(distance) || !dynamicTauFinite(speed)) {
    result.reason = "arithmetic_invalid";
    return result;
  }
  if (distance > 0.0 && speed > 0.0) {
    result.cos_delta = result.relative_dot / (distance * speed);
    if (!dynamicTauFinite(result.cos_delta)) {
      result.cos_delta = 0.0;
    }
  }
  result.closing = result.relative_dot < 0.0;
  result.f_r = dynamicTauPositiveSign(-result.relative_dot);
  // f_v and f_T are disabled gates in the teacher formula.  Unit values make
  // old CSV products auditable without reintroducing either legacy gate.
  result.f_v = 1.0;
  result.f_T = 1.0;

  result.t_ca_raw = -result.relative_dot / result.denominator;
  if (!dynamicTauFinite(result.t_ca_raw)) {
    result.t_ca_raw = 0.0;
    result.reason = "arithmetic_invalid";
    return result;
  }

  result.lower_clipped = result.t_ca_raw <= 0.0;
  result.upper_clipped = result.t_ca_raw > params.max_tau;
  result.t_ca_clipped =
      std::min(std::max(result.t_ca_raw, 0.0), params.max_tau);
  result.T_i = result.t_ca_clipped;
  result.tau_unclipped = result.ke_scaled
                             ? params.ke * result.t_ca_clipped
                             : result.t_ca_clipped;
  if (!dynamicTauFinite(result.tau_unclipped)) {
    result.tau_unclipped = 0.0;
    result.reason = "arithmetic_invalid";
    return result;
  }

  // Pure teacher_tca is already bounded by max_tau.  The extra final clip is
  // required for the diagnostic Ke-scaled variant when Ke > 1.
  result.tau = std::min(result.tau_unclipped, params.max_tau);
  result.computed = true;
  if (result.tau <= 0.0) {
    result.tau = 0.0;
    result.reason = result.relative_dot > 0.0 ? "teacher_receding"
                                              : "teacher_tangent";
    return result;
  }
  result.valid = dynamicTauFinite(result.tau) && result.tau > 0.0;
  if (!result.valid) {
    result.tau = 0.0;
    result.reason = "arithmetic_invalid";
    return result;
  }

  if (result.ke_scaled) {
    result.reason = result.tau_unclipped > params.max_tau
                        ? "teacher_ke_tca_clipped"
                        : "teacher_ke_tca_active";
  } else {
    result.reason = result.upper_clipped ? "teacher_tca_clipped"
                                         : "teacher_tca_active";
  }
  return result;
}

}  // namespace detail

inline DynamicTauResult computeDynamicTau(double lx, double ly, double vx,
                                          double vy, double inflated_radius,
                                          const DynamicTauParams& params) {
  if (!dynamicTauModeValid(params.mode)) {
    DynamicTauResult result;
    result.mode = params.mode;
    result.reason = "invalid_mode";
    return result;
  }
  if (params.mode == DynamicTauMode::kLegacyGate) {
    return detail::computeLegacyDynamicTau(lx, ly, vx, vy, inflated_radius,
                                           params);
  }
  return detail::computeTeacherDynamicTau(lx, ly, vx, vy, inflated_radius,
                                          params);
}

}  // namespace semantic_guard
