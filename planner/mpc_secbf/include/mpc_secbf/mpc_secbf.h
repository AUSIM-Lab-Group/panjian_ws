#pragma once

#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <vector>
#include <string>
#include <memory>

/**
 * MPC-SECBF Solver
 * Model Predictive Control with Semantic Enhanced Control Barrier Function.
 * Uses per-obstacle β_i (from semantic_guard) instead of fixed safe_dist.
 */
class MPC_SECBF_SOLVE {
public:
    MPC_SECBF_SOLVE() = default;

    /**
     * Initialize the solver with parameters.
     */
    void init_solver(double Ts, int N, double v_max, double v_min, double o_max,
                     std::vector<double> Q, std::vector<double> R,
                     double gamma, double beta_bar_unknown, double robot_radius,
                     double epsilon_max = 0.05, double slack_weight = 1000.0,
                     int max_cbf_obstacles = 6,
                     const std::string& cbf_metric = "seesm");

    /**
     * Solve the MPC-SECBF problem.
     * @param cur_state  [x, y, theta, vx, vy] (5×1)
     * @param goal_state [x, y, theta] × N (3×N reference trajectory)
     * @param obs_matrix [x, y, R, vx_pred...] × (N*num_obs) obstacle predictions
     * @param beta_list  per-obstacle β values from semantic_guard
     * @return true if solved successfully
     */
    bool solve(Eigen::VectorXd* cur_state, Eigen::MatrixXd* goal_state,
               Eigen::MatrixXd* obs_matrix, const std::vector<double>& beta_list);

    // Results
    std::vector<double> predict_x;  // Predicted states [x,y,θ,vx,vy] × (N+1)
    std::vector<double> predict_u;  // Predicted controls [v,ω] × N
    double last_slack_sum = 0.0;
    double last_slack_mean = 0.0;
    double last_slack_max = 0.0;
    int last_constrained_obs_count = 0;

private:
    // Shared distance barrier; the caller selects predicted or frozen obstacle state.
    casadi::MX h_cbf(casadi::MX& curpos, Eigen::VectorXd obs, double beta_i);

    // Kinematic model
    casadi::Function setKinematicEquation();

    // Rotate solution for warm start
    void rotateSolution();

    // Parameters
    double Ts_ = 0.2;
    int N_ = 20;
    double v_max_ = 0.5;
    double v_min_ = 0.3;
    double omega_max_ = 0.8;
    double gamma_ = 0.35;
    double beta_bar_unknown_ = 0.4;
    double robot_radius_ = 0.4;
    double epsilon_max_ = 0.05;
    double slack_weight_ = 1000.0;
    int max_cbf_obstacles_ = 6;
    std::string cbf_metric_ = "seesm";
    std::vector<double> Q_, R_;

    // CasADi objects
    casadi::Function kine_equation_;
    casadi::MX X_k_, U_k_;
    std::unique_ptr<casadi::OptiSol> solution_;

    // State
    Eigen::VectorXd* cur_state_ptr_ = nullptr;
    Eigen::MatrixXd* goal_state_ptr_ = nullptr;
    Eigen::MatrixXd* obs_matrix_ptr_ = nullptr;
};
