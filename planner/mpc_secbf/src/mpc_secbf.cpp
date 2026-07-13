#include "mpc_secbf/mpc_secbf.h"
#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <cmath>

void MPC_SECBF_SOLVE::init_solver(double Ts, int N, double v_max, double v_min, double o_max,
                                   std::vector<double> Q, std::vector<double> R,
                                   double gamma, double beta_bar_unknown, double robot_radius,
                                   double epsilon_max, double slack_weight,
                                   int max_cbf_obstacles, const std::string& cbf_metric,
                                   bool dynamic_tau_enabled,
                                   const semantic_guard::DynamicTauParams& dynamic_tau_params) {
    Ts_ = Ts;
    N_ = N;
    v_max_ = v_max;
    v_min_ = v_min;
    omega_max_ = o_max;
    Q_ = Q;
    R_ = R;
    gamma_ = gamma;
    beta_bar_unknown_ = beta_bar_unknown;
    robot_radius_ = robot_radius;
    epsilon_max_ = epsilon_max;
    slack_weight_ = slack_weight;
    max_cbf_obstacles_ = std::max(1, max_cbf_obstacles);
    cbf_metric_ = cbf_metric;
    dynamic_tau_enabled_ = dynamic_tau_enabled;
    dynamic_tau_params_ = dynamic_tau_params;

    kine_equation_ = setKinematicEquation();
    ROS_INFO("MPC-SECBF initialized: N=%d, Ts=%.2f, v_max=%.2f, gamma=%.3f, beta_unknown=%.2f, robot_radius=%.2f, epsilon_max=%.3f, max_cbf_obstacles=%d, dynamic_tau=%s",
             N_, Ts_, v_max_, gamma_, beta_bar_unknown_, robot_radius_, epsilon_max_,
             max_cbf_obstacles_, dynamic_tau_enabled_ ? "true" : "false");
}

bool MPC_SECBF_SOLVE::solve(Eigen::VectorXd* cur_state, Eigen::MatrixXd* goal_state,
                             Eigen::MatrixXd* obs_matrix, const std::vector<double>& beta_list) {
    cur_state_ptr_ = cur_state;
    goal_state_ptr_ = goal_state;
    obs_matrix_ptr_ = obs_matrix;
    last_slack_sum = 0.0;
    last_slack_mean = 0.0;
    last_slack_max = 0.0;
    last_constrained_obs_count = 0;
    last_constrained_obs_index = -1;

    int obs_num = (N_ > 0 && obs_matrix->cols() > 0) ? (obs_matrix->cols() / N_) : 0;

    // Create optimization problem
    casadi::Opti prob;
    X_k_ = prob.variable(5, N_ + 1);
    U_k_ = prob.variable(2, N_);
    casadi::MX epsilon = prob.variable(max_cbf_obstacles_, std::max(1, N_ - 1));

    casadi::MX v = U_k_(0, casadi::Slice());
    casadi::MX omega = U_k_(1, casadi::Slice());

    // Initial state constraint
    casadi::MX X_0 = prob.parameter(5);
    std::vector<double> x0_val(cur_state->data(), cur_state->data() + cur_state->size());
    prob.set_value(X_0, x0_val);
    prob.subject_to(X_k_(casadi::Slice(), 0) == X_0);

    // Reference trajectory
    casadi::MX X_ref = prob.parameter(3, N_);
    std::vector<double> xref_val(goal_state->data(), goal_state->data() + 3 * N_);
    casadi::DM xref_dm(xref_val);
    X_ref = casadi::MX::reshape(casadi::DM(xref_val), 3, N_);

    // Cost function (progressive Q)
    casadi::MX cost = 0;
    casadi::DM R_mat = casadi::DM::zeros(2, 2);
    R_mat(0, 0) = R_[0];
    R_mat(1, 1) = R_[1];
    casadi::DM Q_mat = casadi::DM::zeros(3, 3);
    Q_mat(0, 0) = Q_[0];
    Q_mat(1, 1) = Q_[1];
    Q_mat(2, 2) = Q_[2];

    for (int i = 0; i < N_; i++) {
        casadi::MX X_err = X_k_(casadi::Slice(0, 3), i) - X_ref(casadi::Slice(), i);
        casadi::MX U_i = U_k_(casadi::Slice(), i);
        cost += casadi::MX::mtimes({X_err.T(), Q_mat, X_err});
        cost += casadi::MX::mtimes({U_i.T(), R_mat, U_i});

        // 鼓励前进: 对 (v_max - v)^2 加惩罚, 使 v 接近 v_max
        // 权重 0.5: 如果 v=0 则惩罚 0.5*v_max^2, 平衡 R_[0]*v^2 的惩罚
        casadi::MX v_gap = v_max_ - U_i(0);
        cost += 0.5 * v_gap * v_gap;

        Q_mat(0, 0) += 0.05;
        Q_mat(1, 1) += 0.05;
        Q_mat(2, 2) += 0.005;
    }
    // Terminal cost
    casadi::MX X_err_e = X_k_(casadi::Slice(0, 3), N_) - X_ref(casadi::Slice(), N_ - 1);
    cost += casadi::MX::mtimes({X_err_e.T(), 1.1 * Q_mat, X_err_e});
    cost += slack_weight_ * casadi::MX::sumsqr(epsilon);

    // Control bounds
    // 允许小幅倒车 (-0.2 m/s) 用于紧急避障, 但不鼓励长距离倒车
    prob.subject_to(prob.bounded(-0.2, v, v_max_));
    prob.subject_to(prob.bounded(-omega_max_, omega, omega_max_));
    prob.subject_to(prob.bounded(0.0, epsilon, epsilon_max_));

    // Kinematic constraints
    for (int i = 0; i < N_; i++) {
        casadi::DM A = casadi::DM::zeros(5, 5);
        for (int j = 0; j < 3; j++) A(j, j) = 1.0;
        std::vector<casadi::MX> input(2);
        input[0] = X_k_(casadi::Slice(), i);
        input[1] = U_k_(casadi::Slice(), i);
        casadi::MX x_next = casadi::MX::mtimes(A, X_k_(casadi::Slice(), i)) + kine_equation_(input)[0];
        prob.subject_to(x_next == X_k_(casadi::Slice(), i + 1));
    }

    // SECBF constraints (per-obstacle with semantic β). Rank candidates first
    // so the limited CBF budget covers the nearest relevant obstacles.
    struct ObstacleCandidate {
        int original_idx;
        double distance;
    };
    std::vector<ObstacleCandidate> candidates;
    candidates.reserve(obs_num);
    for (int idx = 0; idx < obs_num; idx++) {
        Eigen::VectorXd obs_first = obs_matrix->col(idx * N_);
        Eigen::Vector2d obs_p = obs_first.head<2>();
        Eigen::Vector2d rob_p = cur_state->head<2>();
        double dist = (obs_p - rob_p).norm();
        if (dist > 8.0) continue;  // Too far, skip
        candidates.push_back({idx, dist});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const ObstacleCandidate& lhs, const ObstacleCandidate& rhs) {
                  return lhs.distance < rhs.distance;
              });

    int choose_num = 0;
    for (const auto& candidate : candidates) {
        if (choose_num >= max_cbf_obstacles_) break;
        const int original_idx = candidate.original_idx;
        // Get β for this obstacle
        double beta_i = (original_idx < (int)beta_list.size()) ? beta_list[original_idx] : beta_bar_unknown_;

        // Add CBF constraints
        for (int i = 0; i < N_ - 1; i++) {
            casadi::MX X_cur = X_k_(casadi::Slice(), i);
            casadi::MX X_nxt = X_k_(casadi::Slice(), i + 1);

            Eigen::VectorXd obs_k = obs_matrix->col(original_idx * N_ + i);
            Eigen::VectorXd obs_k1 = obs_matrix->col(original_idx * N_ + i + 1);
            if (cbf_metric_ == "distance") {
                obs_k = obs_matrix->col(original_idx * N_);
                obs_k1 = obs_matrix->col(original_idx * N_);
            }

            casadi::MX hk = h_cbf(X_cur, obs_k, beta_i);
            casadi::MX hk1 = h_cbf(X_nxt, obs_k1, beta_i);

            // Soft CBF constraint: -h_{k+1} + (1-γ)h_k ≤ ε
            casadi::MX cbf = -hk1 + (1.0 - gamma_) * hk;
            prob.subject_to(cbf <= epsilon(choose_num, i));
        }
        choose_num++;
        if (last_constrained_obs_index < 0) {
            last_constrained_obs_index = original_idx;
        }
    }
    last_constrained_obs_count = choose_num;

    prob.minimize(cost);

    // Warm start
    if (!predict_u.empty()) {
        std::vector<double> x_warm = predict_x;
        std::vector<double> u_warm = predict_u;
        std::rotate(u_warm.begin(), u_warm.begin() + 2, u_warm.end());
        u_warm[u_warm.size() - 2] = 0.0;
        u_warm[u_warm.size() - 1] = 0.0;
        std::rotate(x_warm.begin(), x_warm.begin() + 5, x_warm.end());

        casadi::DM x_guess = casadi::DM::reshape(casadi::DM(x_warm), 5, N_ + 1);
        casadi::DM u_guess = casadi::DM::reshape(casadi::DM(u_warm), 2, N_);
        prob.set_initial(X_k_, x_guess);
        prob.set_initial(U_k_, u_guess);
    }

    // Solver options
    casadi::Dict opts;
    opts["expand"] = true;
    opts["ipopt.max_iter"] = 2500;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.acceptable_tol"] = 3e-3;
    opts["ipopt.acceptable_obj_change_tol"] = 3e-3;
    prob.solver("ipopt", opts);

    try {
        solution_ = std::make_unique<casadi::OptiSol>(prob.solve());

        // Extract results
        predict_x.clear();
        predict_u.clear();
        casadi::DM state_sol = solution_->value(X_k_);
        casadi::DM ctrl_sol = solution_->value(U_k_);
        casadi::DM epsilon_sol = solution_->value(epsilon);

        for (int i = 0; i < N_ + 1; i++) {
            for (int j = 0; j < 5; j++)
                predict_x.push_back(static_cast<double>(state_sol(j, i)));
        }
        for (int i = 0; i < N_; i++) {
            predict_u.push_back(static_cast<double>(ctrl_sol(0, i)));
            predict_u.push_back(static_cast<double>(ctrl_sol(1, i)));
        }
        int slack_count = 0;
        for (int row = 0; row < static_cast<int>(epsilon_sol.size1()); row++) {
            for (int col = 0; col < static_cast<int>(epsilon_sol.size2()); col++) {
                double value = std::max(0.0, static_cast<double>(epsilon_sol(row, col)));
                last_slack_sum += value;
                last_slack_max = std::max(last_slack_max, value);
                slack_count++;
            }
        }
        if (slack_count > 0) {
            last_slack_mean = last_slack_sum / static_cast<double>(slack_count);
        }
        return true;

    } catch (const casadi::CasadiException& e) {
        std::cerr << "\033[31m[MPC-SECBF] Infeasible: \033[0m" << e.what() << std::endl;
        rotateSolution();
        return false;
    }
}

casadi::MX MPC_SECBF_SOLVE::h_cbf(casadi::MX& curpos, Eigen::VectorXd obs, double beta_i) {
    // obs layout: [x, y, radius, radius, theta, vx, vy].
    casadi::MX lx = obs(0) - curpos(0);
    casadi::MX ly = obs(1) - curpos(1);
    casadi::MX vx = obs(5) - curpos(3);
    casadi::MX vy = obs(6) - curpos(4);
    double obs_radius = obs(2);

    // Standard MPC-CBF keeps the instantaneous distance barrier and ignores beta_i.
    if (!dynamic_tau_enabled_) {
        return casadi::MX::sqrt(lx * lx + ly * ly) - obs_radius - robot_radius_;
    }

    casadi::MX tau = dynamicTauCasadi(lx, ly, vx, vy,
                                      obs_radius + robot_radius_);
    casadi::MX lookahead_x = lx + tau * vx;
    casadi::MX lookahead_y = ly + tau * vy;
    return casadi::MX::sqrt(lookahead_x * lookahead_x + lookahead_y * lookahead_y)
         - obs_radius - robot_radius_ - beta_i;
}

casadi::MX MPC_SECBF_SOLVE::dynamicTauCasadi(const casadi::MX& lx,
                                               const casadi::MX& ly,
                                               const casadi::MX& vx,
                                               const casadi::MX& vy,
                                               double inflated_radius) {
    // Keep the symbolic expression aligned with semantic_guard::computeDynamicTau.
    // The guards make every denominator finite while the validity gate preserves the
    // numeric policy for degenerate inputs and invalid configuration.
    const double min_distance = std::max(dynamic_tau_params_.min_distance, 1e-12);
    const double min_speed = std::max(dynamic_tau_params_.min_speed, 1e-12);
    const bool config_valid = std::isfinite(inflated_radius) && inflated_radius >= 0.0 &&
                              std::isfinite(dynamic_tau_params_.ke) &&
                              dynamic_tau_params_.ke >= 0.0 &&
                              std::isfinite(dynamic_tau_params_.t_max) &&
                              dynamic_tau_params_.t_max >= 0.0 &&
                              std::isfinite(dynamic_tau_params_.min_speed) &&
                              dynamic_tau_params_.min_speed >= 0.0 &&
                              std::isfinite(dynamic_tau_params_.min_distance) &&
                              dynamic_tau_params_.min_distance >= 0.0 &&
                              std::isfinite(dynamic_tau_params_.max_tau) &&
                              dynamic_tau_params_.max_tau > 0.0;

    casadi::MX distance = casadi::MX::sqrt(lx * lx + ly * ly);
    casadi::MX speed = casadi::MX::sqrt(vx * vx + vy * vy);
    casadi::MX distance_safe = casadi::MX::fmax(distance, min_distance);
    casadi::MX speed_safe = casadi::MX::fmax(speed, min_speed);
    casadi::MX distance_valid = casadi::MX::if_else(
        distance > dynamic_tau_params_.min_distance, 1.0, 0.0);
    casadi::MX speed_valid = casadi::MX::if_else(
        speed > dynamic_tau_params_.min_speed, 1.0, 0.0);
    casadi::MX valid = distance_valid * speed_valid * (config_valid ? 1.0 : 0.0);

    casadi::MX nx = lx / distance_safe;
    casadi::MX ny = ly / distance_safe;
    casadi::MX nvx = vx / speed_safe;
    casadi::MX nvy = vy / speed_safe;
    casadi::MX cos_delta = nx * nvx + ny * nvy;
    casadi::MX f_r = valid * casadi::MX::if_else(cos_delta < 0.0, 1.0, 0.0);

    casadi::MX dot = lx * vx + ly * vy;
    casadi::MX cone_value = dot * dot +
                            (inflated_radius * inflated_radius - distance * distance) *
                            speed * speed;
    casadi::MX f_v = valid * casadi::MX::if_else(cone_value > 0.0, 1.0, 0.0);

    casadi::MX approach_cos = casadi::MX::fmax(-cos_delta, 0.0);
    casadi::MX clearance = casadi::MX::fmax(distance - inflated_radius, 0.0);
    casadi::MX T_i = valid * clearance * approach_cos / speed_safe;
    casadi::MX f_T = valid * casadi::MX::if_else(
        dynamic_tau_params_.t_max - T_i > 0.0, 1.0, 0.0);
    casadi::MX raw_tau = f_r * f_v * f_T * dynamic_tau_params_.ke * T_i;
    const double max_tau = std::max(dynamic_tau_params_.max_tau, 0.0);
    return casadi::MX::if_else(raw_tau < max_tau, raw_tau, max_tau);
}

casadi::Function MPC_SECBF_SOLVE::setKinematicEquation() {
    casadi::MX x = casadi::MX::sym("x");
    casadi::MX y = casadi::MX::sym("y");
    casadi::MX theta = casadi::MX::sym("theta");
    casadi::MX vx = casadi::MX::sym("vx");
    casadi::MX vy = casadi::MX::sym("vy");
    casadi::MX state = casadi::MX::vertcat({x, y, theta, vx, vy});

    casadi::MX v = casadi::MX::sym("v");
    casadi::MX w = casadi::MX::sym("w");
    casadi::MX ctrl = casadi::MX::vertcat({v, w});

    // Differential drive kinematics
    casadi::MX rhs = casadi::MX::vertcat({
        v * casadi::MX::cos(theta) * Ts_,
        v * casadi::MX::sin(theta) * Ts_,
        w * Ts_,
        v * casadi::MX::cos(theta),
        v * casadi::MX::sin(theta)
    });

    return casadi::Function("kinematic_eq", {state, ctrl}, {rhs});
}

void MPC_SECBF_SOLVE::rotateSolution() {
    if (predict_u.empty()) return;
    std::rotate(predict_u.begin(), predict_u.begin() + 2, predict_u.end());
    predict_u[predict_u.size() - 2] = 0.0;
    predict_u[predict_u.size() - 1] = 0.0;

    if (!predict_x.empty()) {
        std::rotate(predict_x.begin(), predict_x.begin() + 5, predict_x.end());
    }
}
