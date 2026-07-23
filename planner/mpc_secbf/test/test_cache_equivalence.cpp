#include "mpc_secbf/mpc_secbf.h"

#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

struct Fixture {
    std::string name;
    bool side_enabled = false;
    Eigen::VectorXd state;
    Eigen::MatrixXd goal;
    Eigen::MatrixXd obstacles;
    std::vector<double> beta;
};

Fixture makeFixture(const std::string& name, int obstacle_count,
                    bool all_far, bool dynamic_crowd) {
    constexpr int N = 20;
    Fixture fixture;
    fixture.name = name;
    fixture.side_enabled = dynamic_crowd && obstacle_count == 1;
    fixture.state = Eigen::VectorXd::Zero(5);
    fixture.goal = Eigen::MatrixXd::Zero(3, N);
    fixture.obstacles = Eigen::MatrixXd::Zero(7, N * obstacle_count);
    fixture.beta.assign(obstacle_count, 0.3);

    for (int k = 0; k < N; ++k) {
        fixture.goal(0, k) = 0.15 * static_cast<double>(k + 1);
        fixture.goal(1, k) = 0.05 * std::sin(0.2 * static_cast<double>(k));
        fixture.goal(2, k) = 0.0;
    }

    for (int obs_idx = 0; obs_idx < obstacle_count; ++obs_idx) {
        double x = 2.5 + 0.35 * static_cast<double>(obs_idx);
        double y = 0.8 * (static_cast<double>(obs_idx) - 1.5);
        double vx = 0.0;
        double vy = 0.0;
        if (all_far) {
            x = 20.0 + static_cast<double>(obs_idx);
            y = 20.0 - 0.5 * static_cast<double>(obs_idx);
        } else if (dynamic_crowd) {
            x = 1.2 + 0.25 * static_cast<double>(obs_idx);
            y = 0.20 * (static_cast<double>(obs_idx) - 2.0);
            vx = -0.08 + 0.02 * static_cast<double>(obs_idx);
            vy = 0.03 * (static_cast<double>(obs_idx) - 2.0);
        }
        for (int k = 0; k < N; ++k) {
            const int col = obs_idx * N + k;
            fixture.obstacles(0, col) = x + 0.2 * vx * k;
            fixture.obstacles(1, col) = y + 0.2 * vy * k;
            fixture.obstacles(2, col) = 0.25;
            fixture.obstacles(3, col) = 0.25;
            fixture.obstacles(4, col) = 0.0;
            fixture.obstacles(5, col) = vx;
            fixture.obstacles(6, col) = vy;
        }
    }
    return fixture;
}

void configure(MPC_SECBF_SOLVE* solver, bool cache_enabled, bool side_enabled) {
    semantic_guard::DynamicTauParams tau;
    tau.mode = semantic_guard::DynamicTauMode::kTeacherTca;
    tau.delta_tau = 1e-6;
    tau.ke = 0.3;
    tau.t_max = 2.0;
    tau.min_speed = 1e-6;
    tau.min_distance = 1e-6;
    tau.max_tau = 2.0;
    solver->init_solver(
        0.2, 20, 1.5, 0.3, 0.8,
        std::vector<double>{1.0, 1.0, 0.05},
        std::vector<double>{0.1, 0.05},
        0.35, 0.4, 0.4, 0.01, 5000.0, 6, "seesm", true, tau,
        side_enabled, 0.05, 1e-3, 20, 1.0, 1e-3, 3.0,
        1.1, 0.02, 0.4, 8.0, cache_enabled);
}

double minH(const MPC_SECBF_SOLVE& solver) {
    double value = std::numeric_limits<double>::infinity();
    for (const auto& audit : solver.last_tau_stage_audit) {
        value = std::min(value, audit.h_eesm);
    }
    return value;
}

bool finiteOrInfEqual(double lhs, double rhs) {
    if (std::isinf(lhs) || std::isinf(rhs)) return std::isinf(lhs) && std::isinf(rhs);
    return std::isfinite(lhs) && std::isfinite(rhs);
}

struct Result {
    bool rebuild_ok = false;
    bool cache_ok = false;
    bool cache_used = false;
    bool status_equal = false;
    bool selected_equal = false;
    bool control_equal = false;
    bool slack_equal = false;
    bool barrier_equal = false;
    bool pass = false;
    double control_diff = 0.0;
    double slack_diff = 0.0;
    double barrier_diff = 0.0;
};

Result compareFixture(const Fixture& fixture) {
    MPC_SECBF_SOLVE rebuilding;
    MPC_SECBF_SOLVE cached;
    configure(&rebuilding, false, fixture.side_enabled);
    configure(&cached, true, fixture.side_enabled);
    Eigen::VectorXd state = fixture.state;
    Eigen::MatrixXd goal = fixture.goal;
    Eigen::MatrixXd obstacles = fixture.obstacles;

    Result result;
    result.rebuild_ok = rebuilding.solve(
        &state, &goal, &obstacles, fixture.beta);
    result.cache_ok = cached.solve(
        &state, &goal, &obstacles, fixture.beta);
    result.cache_used = cached.last_timing.graph_cache_enabled;
    result.status_equal = result.rebuild_ok == result.cache_ok;
    result.selected_equal = rebuilding.last_constrained_obs_count ==
                            cached.last_constrained_obs_count;

    if (result.rebuild_ok && result.cache_ok &&
        !rebuilding.predict_u.empty() && !cached.predict_u.empty()) {
        const std::size_t count = std::min(rebuilding.predict_u.size(),
                                           cached.predict_u.size());
        for (std::size_t i = 0; i < count; ++i) {
            result.control_diff = std::max(
                result.control_diff,
                std::abs(rebuilding.predict_u[i] - cached.predict_u[i]));
        }
        result.slack_diff = std::abs(rebuilding.last_slack_max - cached.last_slack_max);
        const double rebuild_h = minH(rebuilding);
        const double cache_h = minH(cached);
        if (finiteOrInfEqual(rebuild_h, cache_h) &&
            std::isfinite(rebuild_h) && std::isfinite(cache_h)) {
            result.barrier_diff = std::abs(rebuild_h - cache_h);
        } else {
            result.barrier_diff = std::numeric_limits<double>::infinity();
        }
    }
    result.control_equal = result.control_diff <= 1e-3;
    result.slack_equal = result.slack_diff <= 1e-3;
    result.barrier_equal = result.barrier_diff <= 1e-3 ||
                           (std::isinf(result.barrier_diff) &&
                            rebuilding.last_tau_stage_audit.empty() &&
                            cached.last_tau_stage_audit.empty());
    // A fallback is deliberately not counted as cache equivalence: it proves
    // fail-closed safety, but not that the parameterized graph matches the
    // rebuild graph.  The executable always prints fallback cases so they can
    // be tracked separately from true cache comparisons.
    result.pass = result.cache_used && result.status_equal &&
                  result.selected_equal && result.control_equal &&
                  result.slack_equal && result.barrier_equal;
    return result;
}

}  // namespace

int main() {
    const std::vector<Fixture> fixtures = {
        makeFixture("all_far_empty_active_set", 3, true, false),
        makeFixture("single_static_near", 1, false, false),
        makeFixture("single_dynamic_near", 1, false, true),
        makeFixture("crowd_five", 5, false, true),
    };

    std::cout << "case,rebuild_ok,cache_ok,cache_used,status_equal,selected_equal,"
                 "control_diff,slack_diff,barrier_diff,pass\n";
    bool all_pass = true;
    for (const auto& fixture : fixtures) {
        const Result result = compareFixture(fixture);
        std::cout << fixture.name << ","
                  << (result.rebuild_ok ? 1 : 0) << ","
                  << (result.cache_ok ? 1 : 0) << ","
                  << (result.cache_used ? 1 : 0) << ","
                  << (result.status_equal ? 1 : 0) << ","
                  << (result.selected_equal ? 1 : 0) << ","
                  << std::setprecision(10) << result.control_diff << ","
                  << result.slack_diff << "," << result.barrier_diff << ","
                  << (result.pass ? 1 : 0) << "\n";
        all_pass = all_pass && result.pass;
    }
    return all_pass ? 0 : 2;
}
