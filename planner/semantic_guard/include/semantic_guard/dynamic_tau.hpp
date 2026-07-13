#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace semantic_guard {

struct DynamicTauParams {
  double ke = 0.30;
  double t_max = 2.0;
  double min_speed = 1e-6;
  double min_distance = 1e-6;
  double max_tau = 2.0;
};

struct DynamicTauResult {
  double tau = 0.0;
  double T_i = 0.0;
  double f_r = 0.0;
  double f_v = 0.0;
  double f_T = 0.0;
  double cos_delta = 0.0;
  bool closing = false;
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

}  // namespace detail

inline DynamicTauResult computeDynamicTau(double lx, double ly,
                                          double vx, double vy,
                                          double inflated_radius,
                                          const DynamicTauParams& params) {
  DynamicTauResult result;
  const double values[] = {lx, ly, vx, vy, inflated_radius, params.ke,
                           params.t_max, params.min_speed,
                           params.min_distance, params.max_tau};
  for (double value : values) {
    if (!detail::dynamicTauFinite(value)) {
      result.reason = "non_finite_input";
      return result;
    }
  }

  if (inflated_radius < 0.0 || params.ke < 0.0 || params.t_max < 0.0 ||
      params.min_speed < 0.0 || params.min_distance < 0.0 ||
      params.max_tau <= 0.0) {
    result.reason = "invalid_config";
    return result;
  }

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
  if (!detail::dynamicTauFinite(result.cos_delta)) {
    result.reason = "angle_invalid";
    return result;
  }

  // The relative-vector convention is p_obstacle-p_robot and
  // v_obstacle-v_robot. Closing motion therefore has cos_delta < 0.
  result.closing = result.cos_delta < 0.0;
  result.f_r = detail::dynamicTauPositiveSign(-result.cos_delta);

  const double dot = lx * vx + ly * vy;
  const double cone_value =
      dot * dot + (inflated_radius * inflated_radius - distance * distance) *
                       speed * speed;
  if (detail::dynamicTauFinite(cone_value)) {
    result.f_v = detail::dynamicTauPositiveSign(cone_value);
  } else {
    // The expression above is the specified cone gate. This equivalent
    // scaled sign check keeps its result defined when finite inputs overflow.
    const double cross_unit = nx * nvy - ny * nvx;
    const double lateral_ratio =
        std::min(1.0, std::abs(cross_unit));
    const double lateral_distance = distance * lateral_ratio;
    result.f_v = detail::dynamicTauPositiveSign(
        inflated_radius - lateral_distance);
  }

  const double approach_cos = std::max(0.0, -result.cos_delta);
  const double clearance =
      std::max(0.0, distance - inflated_radius);
  const double time_numerator = clearance * approach_cos;
  result.T_i = time_numerator / speed;
  if (!detail::dynamicTauFinite(result.T_i)) {
    result.T_i = 0.0;
    result.reason = "tau_invalid";
    return result;
  }

  result.f_T = detail::dynamicTauPositiveSign(params.t_max - result.T_i);
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
  if (!detail::dynamicTauFinite(raw_tau) || raw_tau <= 0.0) {
    result.reason = "tau_invalid";
    return result;
  }

  result.tau = std::min(raw_tau, params.max_tau);
  result.valid = detail::dynamicTauFinite(result.tau) && result.tau >= 0.0;
  result.reason = result.valid ? "active" : "tau_invalid";
  if (!result.valid) {
    result.tau = 0.0;
  }
  return result;
}

}  // namespace semantic_guard
