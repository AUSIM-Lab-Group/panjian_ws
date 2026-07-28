#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace semantic_guard {

struct EmergencyObstacle {
    double x = 0.0, y = 0.0, vx = 0.0, vy = 0.0, radius = 0.0;
};

struct EmergencyCbfParams {
    double alpha = 1.5;
    double extra_margin = 0.10;
    double v_min = -0.20;
    double v_max = 0.35;
    double omega_max = 0.80;
    double turn_gain = 1.5;
    double activation_distance = 4.0;
    double progress_v = 0.20;
};

struct EmergencyCommand {
    double v = 0.0, w = 0.0;
    bool interval_feasible = true;
    double minimum_residual = std::numeric_limits<double>::infinity();
    int active_obstacle_count = 0;
    bool progress_mode = false;
};

inline double wrapAngle(double value) {
    while (value > M_PI) value -= 2.0 * M_PI;
    while (value < -M_PI) value += 2.0 * M_PI;
    return value;
}

inline EmergencyCommand emergencyCbfCommand(
    double robot_x, double robot_y, double robot_yaw, double robot_radius,
    const std::vector<EmergencyObstacle>& obstacles,
    const EmergencyCbfParams& params,
    double preferred_yaw = std::numeric_limits<double>::quiet_NaN()) {
    EmergencyCommand out;
    double lower = params.v_min;
    double upper = params.v_max;
    double nearest_d2 = std::numeric_limits<double>::infinity();
    double away_yaw = robot_yaw;
    const double dir_x = std::cos(robot_yaw);
    const double dir_y = std::sin(robot_yaw);

    struct Constraint { double a, b; };
    std::vector<Constraint> constraints;
    for (const auto& obs : obstacles) {
        const double lx = robot_x - obs.x;
        const double ly = robot_y - obs.y;
        const double d2 = lx * lx + ly * ly;
        const double radius = robot_radius + std::max(0.0, obs.radius) +
                              params.extra_margin;
        const double h = d2 - radius * radius;
        const double a = 2.0 * (lx * dir_x + ly * dir_y);
        const double b = 2.0 * (lx * obs.vx + ly * obs.vy) - params.alpha * h;
        const bool active = std::sqrt(d2) <= params.activation_distance || b > 0.0;
        if (!active) continue;
        constraints.push_back({a, b});
        ++out.active_obstacle_count;
        if (std::abs(a) > 1e-9) {
            const double bound = b / a;
            if (a > 0.0) lower = std::max(lower, bound);
            else upper = std::min(upper, bound);
        } else if (b > 0.0) {
            lower = 1.0;
            upper = 0.0;
        }
        if (d2 < nearest_d2) {
            nearest_d2 = d2;
            away_yaw = std::atan2(ly, lx);
        }
    }

    if (constraints.empty()) {
        out.progress_mode = std::isfinite(preferred_yaw);
        out.v = out.progress_mode
                    ? std::max(params.v_min, std::min(params.progress_v, params.v_max))
                    : 0.0;
        out.w = out.progress_mode
                    ? std::max(-params.omega_max,
                               std::min(params.turn_gain * wrapAngle(
                                            preferred_yaw - robot_yaw),
                                        params.omega_max))
                    : 0.0;
        return out;
    }

    out.interval_feasible = lower <= upper;
    if (out.interval_feasible) {
        out.v = std::max(lower, std::min(0.0, upper));
    } else {
        const double candidates[] = {params.v_min, 0.0, params.v_max};
        double best = -std::numeric_limits<double>::infinity();
        for (double candidate : candidates) {
            double residual = std::numeric_limits<double>::infinity();
            for (const auto& constraint : constraints) {
                residual = std::min(residual,
                                    constraint.a * candidate - constraint.b);
            }
            if (residual > best) {
                best = residual;
                out.v = candidate;
            }
        }
    }
    out.v = std::max(params.v_min, std::min(out.v, params.v_max));
    out.w = std::max(
        -params.omega_max,
        std::min(params.turn_gain * wrapAngle(away_yaw - robot_yaw),
                 params.omega_max));
    for (const auto& constraint : constraints) {
        out.minimum_residual = std::min(
            out.minimum_residual, constraint.a * out.v - constraint.b);
    }
    return out;
}

}  // namespace semantic_guard
