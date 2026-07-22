#pragma once

#include <cmath>

namespace semantic_guard {

// nav_msgs/Odometry expresses twist in child_frame_id. This helper rotates
// its planar components into the world frame used by obstacle predictions.
inline bool bodyPlanarVelocityToWorld(double body_vx, double body_vy,
                                      double qx, double qy, double qz,
                                      double qw, double* world_vx,
                                      double* world_vy) {
  if (world_vx == nullptr || world_vy == nullptr) {
    return false;
  }
  const double values[] = {body_vx, body_vy, qx, qy, qz, qw};
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  const double norm_squared = qx * qx + qy * qy + qz * qz + qw * qw;
  if (!std::isfinite(norm_squared) || norm_squared <= 1e-24) {
    return false;
  }
  const double inv_norm = 1.0 / std::sqrt(norm_squared);
  qx *= inv_norm;
  qy *= inv_norm;
  qz *= inv_norm;
  qw *= inv_norm;

  // First column of the normalized quaternion rotation matrix. Its planar
  // components are cos(yaw) and sin(yaw) for this ground-robot stack.
  double cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz);
  double sin_yaw = 2.0 * (qx * qy + qw * qz);
  const double planar_norm = std::hypot(cos_yaw, sin_yaw);
  if (!std::isfinite(planar_norm) || planar_norm <= 1e-12) {
    return false;
  }
  cos_yaw /= planar_norm;
  sin_yaw /= planar_norm;
  *world_vx = cos_yaw * body_vx - sin_yaw * body_vy;
  *world_vy = sin_yaw * body_vx + cos_yaw * body_vy;
  return std::isfinite(*world_vx) && std::isfinite(*world_vy);
}

}  // namespace semantic_guard
