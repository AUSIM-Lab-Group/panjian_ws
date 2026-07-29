#pragma once

#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <semantic_guard/dynamic_tau.hpp>
#include <vector>
#include <string>
#include <memory>
#include <map>

struct MpcTauStageAudit {
    int obstacle_index = -1;
    int stage = -1;
    double beta = 0.0;
    double lx = 0.0;
    double ly = 0.0;
    double vrel_x = 0.0;
    double vrel_y = 0.0;
    double r_base = 0.0;
    double h_eesm = 0.0;
    double h_seesm = 0.0;
    semantic_guard::DynamicTauResult tau_result;
};

struct MpcSolveTiming {
    double total_ms = 0.0;
    double graph_build_ms = 0.0;
    double ipopt_solve_ms = 0.0;
    double solution_extract_ms = 0.0;
    double tau_audit_ms = 0.0;
    double slack_audit_ms = 0.0;
    int obstacle_count = 0;
    int selected_obstacle_count = 0;
    bool success = false;
    bool graph_cache_enabled = false;
    bool graph_cache_hit = false;
    int graph_cache_slots = 0;
};

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
                     const std::string& cbf_metric = "seesm",
                     bool dynamic_tau_enabled = false,
                     const semantic_guard::DynamicTauParams& dynamic_tau_params =
                         semantic_guard::DynamicTauParams(),
                     bool side_preference_enabled = false,
                     double side_weight = 0.05,
                     double side_epsilon_n = 1e-3,
                     int side_horizon = 20,
                     double side_sign = 1.0,
                     double side_min_obstacle_speed = 1e-3,
                     double side_activation_distance = 3.0,
                     double qf_scale = 1.1,
                     double delta_u_weight = 0.02,
                     double delta_u_max = 0.4,
                     double active_set_distance_m = 8.0,
                     bool graph_cache_enabled = false,
                     double solver_max_cpu_time_ms = 0.0,
                     double reverse_v_max = 0.2);

    /**
     * Solve the MPC-SECBF problem.
     * @param cur_state  [x, y, theta, vx, vy] (5×1)
     * @param goal_state [x, y, theta] × N (3×N reference trajectory)
     * @param obs_matrix [x, y, radius, radius, theta, vx, vy] × (N*num_obs) predictions
     * @param beta_list  per-obstacle β values from semantic_guard
     * @return true if solved successfully
     */
    bool solve(Eigen::VectorXd* cur_state, Eigen::MatrixXd* goal_state,
               Eigen::MatrixXd* obs_matrix, const std::vector<double>& beta_list);

    void copyWarmStartFrom(const MPC_SECBF_SOLVE& other);

    void resetAuditMetrics();

    // Results
    std::vector<double> predict_x;  // Predicted states [x,y,θ,vx,vy] × (N+1)
    std::vector<double> predict_u;  // Predicted controls [v,ω] × N
    double last_slack_sum = 0.0;
    double last_slack_mean = 0.0;
    double last_slack_max = 0.0;
    int last_constrained_obs_count = 0;
    int last_constrained_obs_index = -1;
    double last_side_cost = 0.0;
    int last_side_dominant_obs_index = -1;
    int last_side_dominant_stage = -1;
    int last_side_candidate_count = 0;
    int last_side_dynamic_obstacle_count = 0;
    double last_side_dominant_tau = 0.0;
    double last_side_dominant_h = 0.0;
    double last_delta_u_max = 0.0;
    // Values evaluated from the optimized stage states. They are the same
    // stage-wise Teacher-v1 quantities used by the NLP, not a current-state
    // reconstruction performed by the ROS node.
    std::vector<MpcTauStageAudit> last_tau_stage_audit;
    MpcSolveTiming last_timing;
    std::string last_return_status = "not_run";
    std::string last_warm_start_source = "cold_start";

private:
    // Teacher-v1 production path: keep one parameterized CasADi graph and
    // update its measured/reference/obstacle values per cycle.  The previous
    // rebuild-per-call path remains available for legacy and diagnostic modes.
    bool solveTeacherParameterized(Eigen::VectorXd* cur_state,
                                   Eigen::MatrixXd* goal_state,
                                   Eigen::MatrixXd* obs_matrix,
                                   const std::vector<double>& beta_list);
    bool solveRebuilding(Eigen::VectorXd* cur_state,
                         Eigen::MatrixXd* goal_state,
                         Eigen::MatrixXd* obs_matrix,
                         const std::vector<double>& beta_list);
    struct TeacherGraph {
        std::unique_ptr<casadi::Opti> opti;
        casadi::MX X, U, epsilon;
        casadi::MX p_x0, p_xref, p_obs;
        casadi::MX p_beta, p_cbf_mask, p_side_mask;
        casadi::MX side_cost;
        int slots = 0;
        bool ready = false;
    };

    void initTeacherParameterizedProblem(int obstacle_slots);
    TeacherGraph& teacherGraphForSlots(int obstacle_slots);
    casadi::MX h_cbf_symbolic(const casadi::MX& curpos,
                              const casadi::MX& obs_x,
                              const casadi::MX& obs_y,
                              const casadi::MX& obs_radius,
                              const casadi::MX& obs_vx,
                              const casadi::MX& obs_vy,
                              const casadi::MX& beta_i);

    // legacy_gate receives a numeric stage tau frozen from the measured state.
    // Teacher modes ignore that value and evaluate TCA symbolically from the
    // predicted robot state at this stage.
    casadi::MX h_cbf(casadi::MX& curpos, Eigen::VectorXd obs, double beta_i,
                     double stage_tau);

    // Receding-horizon policy: recompute tau from the measured robot state at
    // every solve, then freeze it as a CasADi constant for this stage.
    double computeFrozenStageTau(const Eigen::VectorXd& obs,
                                 const Eigen::VectorXd& measured_state) const;

    // Teacher-v1 production expression. For teacher_tca/ke_tca this is part of
    // the NLP graph and depends on the predicted state X_k.
    casadi::MX dynamicTauCasadi(const casadi::MX& lx, const casadi::MX& ly,
                                const casadi::MX& vx, const casadi::MX& vy,
                                double inflated_radius);

    // Kinematic model
    casadi::Function setKinematicEquation();

    // Rotate solution for warm start
    void rotateSolution();

    // Parameters
    double Ts_ = 0.2;
    int N_ = 20;
    double v_max_ = 0.5;
    double v_min_ = 0.3;
    double reverse_v_max_ = 0.2;
    double omega_max_ = 0.8;
    double gamma_ = 0.35;
    double beta_bar_unknown_ = 0.4;
    double robot_radius_ = 0.4;
    double epsilon_max_ = 0.05;
    double slack_weight_ = 1000.0;
    int max_cbf_obstacles_ = 6;
    std::string cbf_metric_ = "seesm";
    bool dynamic_tau_enabled_ = false;
    semantic_guard::DynamicTauParams dynamic_tau_params_;
    bool side_preference_enabled_ = false;
    double side_weight_ = 0.05;
    double side_epsilon_n_ = 1e-3;
    int side_horizon_ = 20;
    double side_sign_ = 1.0;
    double side_min_obstacle_speed_ = 1e-3;
    double side_activation_distance_ = 3.0;
    double qf_scale_ = 1.1;
    double delta_u_weight_ = 0.02;
    double delta_u_max_ = 0.4;
    double active_set_distance_m_ = 8.0;
    bool graph_cache_enabled_ = false;
    double solver_max_cpu_time_sec_ = 0.0;
    std::vector<double> Q_, R_;

    // CasADi objects
    casadi::Function kine_equation_;
    casadi::MX X_k_, U_k_;
    std::unique_ptr<casadi::OptiSol> solution_;

    // Cached Teacher-v1 NLP graphs keyed by the number of active obstacle
    // slots.  The default remains disabled so Teacher-v1 behavior is frozen.
    std::map<int, std::unique_ptr<TeacherGraph>> teacher_graphs_;

    // State
    Eigen::VectorXd* cur_state_ptr_ = nullptr;
    Eigen::MatrixXd* goal_state_ptr_ = nullptr;
    Eigen::MatrixXd* obs_matrix_ptr_ = nullptr;
};
