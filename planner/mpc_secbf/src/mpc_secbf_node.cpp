#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include "mpc_secbf/mpc_secbf.h"
#include "semantic_guard/AppliedMarginArray.h"
#include "semantic_guard/PreGuardMarginArray.h"
#include "semantic_guard/PredictedObstacleArray.h"
#include "semantic_guard/dynamic_tau.hpp"
#include "semantic_guard/guard_backtracking.hpp"
#include "semantic_guard/safety_recurrence.hpp"
#include "semantic_guard/planar_velocity.hpp"
#include "semantic_guard/typed_margin_contract.hpp"

struct TypedObstacleCycle {
    ros::Time receipt_time;
    ros::Time source_time;
    std::vector<uint32_t> obstacle_ids;
    Eigen::MatrixXd obstacle_matrix;
};

struct TypedMarginCycle {
    ros::Time receipt_time;
    std::vector<uint32_t> obstacle_ids;
    std::vector<double> beta_max;
    std::vector<double> beta_previous;
    std::vector<double> beta_tilde;
    std::vector<double> available_margin;
    std::vector<double> positive_increment_bound;
    std::vector<double> beta_upper_bound;
    std::vector<double> beta_pre_guard;
    bool enforce_category_bound = true;
    bool enforce_positive_increment_bound = true;
    bool enforce_available_margin_bound = true;
};

struct PendingSafetyRecurrence {
    uint64_t obstacle_cycle_id = 0;
    double h_eesm_t = 0.0;
    double beta_t = 0.0;
    double h_eesm_pred_next = 0.0;
    double epsilon_t = 0.0;
    bool cbf_executed = false;
    bool backup_used = false;
    bool baseline_infeasible = false;
};

struct CycleTiming {
    MpcSolveTiming initial_solver;
    double guard_phase_ms = 0.0;
    double guard_solver_ms = 0.0;
    double guard_graph_build_ms = 0.0;
    double guard_ipopt_ms = 0.0;
    double guard_extract_ms = 0.0;
    double guard_tau_audit_ms = 0.0;
    double guard_slack_audit_ms = 0.0;
    int guard_attempts = 0;
};

using NodeSteadyClock = std::chrono::steady_clock;
double nodeElapsedMs(const NodeSteadyClock::time_point& start,
                     const NodeSteadyClock::time_point& end = NodeSteadyClock::now()) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

class MpcSecbfNode {
public:
    MpcSecbfNode(ros::NodeHandle& nh) : nh_(nh) {
        // Parameters
        double mpc_freq, Ts, gamma, beta_unknown, robot_radius;
        double epsilon_max, slack_weight;
        double qf_scale, delta_u_weight, delta_u_max, active_set_distance_m;
        bool graph_cache_enabled;
        double safety_delta_bar, safety_delta_beta_bar;
        int N;
        int max_cbf_obstacles;
        bool mpc_feasibility_guard_enabled;
        double guard_kappa, guard_time_budget_ms;
        int guard_max_backtracks;
        bool side_preference_enabled;
        double side_weight, side_epsilon_n, side_sign, side_min_obstacle_speed, side_activation_distance;
        int side_horizon;
        double v_max, v_min, o_max;
        std::string cbf_metric;
        nh_.param("dynamic_tau_enabled", dynamic_tau_enabled_, false);
        std::string dynamic_tau_mode;
        nh_.param<std::string>("dynamic_tau/mode", dynamic_tau_mode, "teacher_tca");
        if (!semantic_guard::parseDynamicTauMode(
                dynamic_tau_mode, &dynamic_tau_params_.mode)) {
            ROS_FATAL_STREAM("Unsupported dynamic_tau/mode: " << dynamic_tau_mode);
            throw std::runtime_error("unsupported dynamic_tau/mode");
        }
        nh_.param("dynamic_tau/delta_tau", dynamic_tau_params_.delta_tau, 1e-6);
        nh_.param("dynamic_tau/Ke", dynamic_tau_params_.ke, 0.30);
        nh_.param("dynamic_tau/Tmax", dynamic_tau_params_.t_max, 2.0);
        nh_.param("dynamic_tau/min_speed", dynamic_tau_params_.min_speed, 1e-6);
        nh_.param("dynamic_tau/min_distance", dynamic_tau_params_.min_distance, 1e-6);
        nh_.param("dynamic_tau/max_tau", dynamic_tau_params_.max_tau, 2.0);
        const bool legacy_tau_config_valid =
            std::isfinite(dynamic_tau_params_.ke) && dynamic_tau_params_.ke >= 0.0 &&
            std::isfinite(dynamic_tau_params_.t_max) && dynamic_tau_params_.t_max >= 0.0 &&
            std::isfinite(dynamic_tau_params_.min_speed) && dynamic_tau_params_.min_speed >= 0.0 &&
            std::isfinite(dynamic_tau_params_.min_distance) && dynamic_tau_params_.min_distance >= 0.0 &&
            std::isfinite(dynamic_tau_params_.max_tau) && dynamic_tau_params_.max_tau > 0.0;
        const bool teacher_tau_config_valid =
            std::isfinite(dynamic_tau_params_.delta_tau) && dynamic_tau_params_.delta_tau > 0.0 &&
            std::isfinite(dynamic_tau_params_.max_tau) && dynamic_tau_params_.max_tau > 0.0 &&
            (dynamic_tau_params_.mode != semantic_guard::DynamicTauMode::kTeacherKeTca ||
             (std::isfinite(dynamic_tau_params_.ke) && dynamic_tau_params_.ke > 0.0));
        const bool dynamic_tau_config_valid =
            dynamic_tau_params_.mode == semantic_guard::DynamicTauMode::kLegacyGate
                ? legacy_tau_config_valid
                : teacher_tau_config_valid;
        if (dynamic_tau_enabled_ && !dynamic_tau_config_valid) {
            ROS_FATAL_STREAM("Invalid Teacher-v1 dynamic tau configuration: delta_tau="
                             << dynamic_tau_params_.delta_tau
                             << " max_tau=" << dynamic_tau_params_.max_tau
                             << " Ke=" << dynamic_tau_params_.ke);
            throw std::runtime_error("invalid dynamic tau configuration");
        }
        nh_.param("mpc/mpc_frequency", mpc_freq, 10.0);
        nh_.param("mpc/step_time", Ts, 0.2);
        nh_.param("mpc/pre_step", N, 20);
        nh_.param("mpc/v_max", v_max, 0.5);
        nh_.param("mpc/v_min", v_min, 0.3);
        nh_.param("mpc/o_max", o_max, 0.8);
        nh_.param("mpc/gamma", gamma, 0.35);
        nh_.param("mpc/beta_bar_unknown", beta_unknown, 0.4);
        nh_.param("mpc/epsilon_max", epsilon_max, 0.05);
        nh_.param("mpc/slack_weight", slack_weight, 1000.0);
        nh_.param("mpc/qf_scale", qf_scale, 1.1);
        nh_.param("mpc/delta_u_weight", delta_u_weight, 0.02);
        nh_.param("mpc/delta_u_max", delta_u_max, 0.4);
        nh_.param("mpc/active_set_distance_m", active_set_distance_m, 8.0);
        nh_.param("mpc/graph_cache_enabled", graph_cache_enabled, false);
        nh_.param("mpc/safety_delta_bar", safety_delta_bar, 0.10);
        nh_.param("mpc/safety_delta_beta_bar", safety_delta_beta_bar, 0.30);
        if (!std::isfinite(qf_scale) || qf_scale <= 0.0 ||
            !std::isfinite(delta_u_weight) || delta_u_weight < 0.0 ||
            !std::isfinite(delta_u_max) || delta_u_max <= 0.0) {
            throw std::invalid_argument("invalid Teacher-v1 Qf/Delta-u parameters");
        }
        if (!std::isfinite(safety_delta_bar) || safety_delta_bar < 0.0 ||
            !std::isfinite(safety_delta_beta_bar) || safety_delta_beta_bar < 0.0) {
            throw std::invalid_argument("invalid Teacher-v1 safety recurrence bounds");
        }
        // Retain the validated runtime values for the planner and margin CSV
        // audit rows; do not let custom launch overrides appear as defaults.
        qf_scale_ = qf_scale;
        delta_u_weight_ = delta_u_weight;
        delta_u_max_ = delta_u_max;
        gamma_ = gamma;
        epsilon_max_runtime_ = epsilon_max;
        safety_delta_bar_ = safety_delta_bar;
        safety_delta_beta_bar_ = safety_delta_beta_bar;
        nh_.param("mpc/max_cbf_obstacles", max_cbf_obstacles, 6);
        nh_.param("mpc/feasibility_guard_enabled", mpc_feasibility_guard_enabled, true);
        nh_.param("mpc/guard_kappa", guard_kappa, 0.5);
        nh_.param("mpc/guard_max_backtracks", guard_max_backtracks, 6);
        nh_.param("mpc/guard_time_budget_ms", guard_time_budget_ms, 500.0);
        if (!std::isfinite(guard_kappa) || guard_kappa <= 0.0 || guard_kappa > 1.0 ||
            guard_max_backtracks < 1 || !std::isfinite(guard_time_budget_ms) ||
            guard_time_budget_ms <= 0.0) {
            throw std::invalid_argument("invalid Teacher-v1 Guard backtracking parameters");
        }
        nh_.param("mpc/typed_payload_timeout", typed_payload_timeout_sec_, 0.50);
        if (!std::isfinite(typed_payload_timeout_sec_) ||
            typed_payload_timeout_sec_ <= 0.0) {
            throw std::invalid_argument("mpc/typed_payload_timeout must be positive");
        }
        nh_.param("mpc/side_preference_enabled", side_preference_enabled, false);
        nh_.param("mpc/side_weight", side_weight, 0.05);
        nh_.param("mpc/side_epsilon_n", side_epsilon_n, 1e-3);
        nh_.param("mpc/side_horizon", side_horizon, N);
        nh_.param("mpc/side_sign", side_sign, 1.0);
        nh_.param("mpc/side_min_obstacle_speed", side_min_obstacle_speed, 1e-3);
        nh_.param("mpc/side_activation_distance", side_activation_distance, 3.0);
        nh_.param<std::string>("mpc/cbf_metric", cbf_metric, "seesm");
        if (cbf_metric != "seesm" && cbf_metric != "distance") {
            ROS_FATAL_STREAM("Unsupported mpc/cbf_metric: " << cbf_metric);
            throw std::runtime_error("unsupported mpc/cbf_metric");
        }
        if (!nh_.getParam("mpc/robot_radius", robot_radius)) {
            nh_.param("robot/radius", robot_radius, 0.4);
        }
        robot_radius_ = robot_radius;
        std::string planner_log_path;
        std::string timing_log_path;
        std::string mpc_margin_log_path;
        std::string tau_stage_log_path;
        std::string guard_attempt_log_path;
        std::string safety_recurrence_log_path;
        nh_.param<std::string>("planner_log_path", planner_log_path, "");
        nh_.param<std::string>("timing_log_path", timing_log_path, "");
        nh_.param<std::string>("mpc_margin_log_path", mpc_margin_log_path, "");
        nh_.param<std::string>("tau_stage_log_path", tau_stage_log_path, "");
        nh_.param<std::string>("guard_attempt_log_path", guard_attempt_log_path, "");
        nh_.param<std::string>("safety_recurrence_log_path", safety_recurrence_log_path, "");

        std::vector<double> Q = {1.0, 1.0, 0.05};
        std::vector<double> R = {0.1, 0.05};

        N_ = N;
        Ts_ = Ts;
        mpc_feasibility_guard_enabled_ = mpc_feasibility_guard_enabled;
        guard_kappa_ = guard_kappa;
        guard_max_backtracks_ = static_cast<std::size_t>(guard_max_backtracks);
        guard_time_budget_ms_ = guard_time_budget_ms;

        // Initialize solver
        solver_.init_solver(Ts, N, v_max, v_min, o_max, Q, R, gamma, beta_unknown, robot_radius,
                            epsilon_max, slack_weight, max_cbf_obstacles, cbf_metric,
                            dynamic_tau_enabled_, dynamic_tau_params_,
                            side_preference_enabled, side_weight, side_epsilon_n,
                            side_horizon, side_sign, side_min_obstacle_speed,
                            side_activation_distance, qf_scale, delta_u_weight,
                            delta_u_max, active_set_distance_m, graph_cache_enabled);
        side_preference_enabled_ = side_preference_enabled;
        side_weight_ = side_weight;

        openCsv(planner_csv_, planner_log_path,
                "t,obstacle_cycle_id,mpc_status,first_attempt_status,final_status,accepted_beta_source,"
                "cmd_v,cmd_w,ref_x,ref_y,tracking_error,obs_count,constrained_obs_count,beta_count,used_fallback,"
                "mpc_feasibility_guard_enabled,candidate_feasibility_checked,mpc_feasibility_guard_used,"
                "slack,slack_sum,slack_mean,slack_max,side_preference_enabled,side_weight,side_cost,"
                "side_dynamic_obstacle_count,side_candidate_count,side_dominant_obs_index,side_dominant_stage,side_dominant_tau,side_dominant_h,delta_u_max_observed,qf_scale,delta_u_weight,delta_u_bound,solve_time_ms,"
                "dynamic_tau_enabled,tau_mode,tau,tca_raw,tca_clipped,tau_scale,tau_computed,tau_active,tau_clipped_low,tau_clipped_high,"
                "T_i,f_r,f_v,f_T,tau_valid,tau_reason\n");
        openCsv(timing_csv_, timing_log_path,
                "t,obstacle_cycle_id,mpc_secbf_ms,total_loop_time_ms,initial_solver_ms,"
                "initial_graph_build_ms,initial_ipopt_solve_ms,initial_solution_extract_ms,"
                "initial_tau_audit_ms,initial_slack_audit_ms,guard_phase_ms,guard_solver_ms,"
                "guard_graph_build_ms,guard_ipopt_solve_ms,guard_solution_extract_ms,"
                "guard_tau_audit_ms,guard_slack_audit_ms,guard_attempts,obs_count,"
                "constrained_obs_count,initial_success,initial_graph_cache_hit,"
                "graph_cache_enabled,graph_cache_slots\n");
        openCsv(mpc_margin_csv_, mpc_margin_log_path,
                "t,obstacle_cycle_id,obs_id,beta_pre_guard,beta_applied,accepted_beta_source,"
                "first_attempt_status,final_status,mpc_feasibility_guard_enabled,"
                "candidate_feasibility_checked,mpc_feasibility_guard_used,delta_u_max_observed,qf_scale,delta_u_weight,delta_u_bound\n");
        openCsv(tau_stage_csv_, tau_stage_log_path,
                "t,obstacle_cycle_id,accepted_beta_source,obs_id,obs_index,stage,tau_mode,lx,ly,vrel_x,vrel_y,"
                "tca_raw,tca_clipped,tau,tau_scale,tau_computed,tau_active,tau_clipped_low,tau_clipped_high,"
                "R_base,beta,h_eesm,h_seesm,tau_valid,tau_reason\n");
        openCsv(guard_attempt_csv_, guard_attempt_log_path,
                "t,obstacle_cycle_id,attempt_index,obstacle_id,obstacle_order,q,kappa,"
                "candidate_beta,accepted_beta,candidate_beta_vector,accepted_beta_vector,"
                "solver_success,solver_status,slack_max,solve_time_ms,graph_build_ms,"
                "ipopt_solve_ms,solution_extract_ms,tau_audit_ms,slack_audit_ms,"
                "graph_cache_enabled,graph_cache_hit,graph_cache_slots,reject_reason\n");
        openCsv(safety_recurrence_csv_, safety_recurrence_log_path,
                "time,obstacle_cycle_id,obs_id,H_t,h_eesm_t,beta_t,h_eesm_pred_next,H_pred_next,"
                "h_eesm_next,beta_next,H_next,epsilon_t,epsilon_max,delta,delta_bar,"
                "delta_beta_plus,delta_beta_bar,recursion_rhs,one_step_residual,bar_w,"
                "asymptotic_bound,cbf_executed,backup_used,baseline_infeasible,theorem1_applicable,exclusion_reason\n");

        // Subscribers
        sub_odom_ = nh_.subscribe("/Odometry", 1, &MpcSecbfNode::odomCb, this);
        sub_path_ = nh_.subscribe("/global_path", 10, &MpcSecbfNode::pathCb, this);
        sub_pre_guard_ = nh_.subscribe(
            "/safety_margin/beta_pre_guard", 40,
            &MpcSecbfNode::preGuardCb, this);
        sub_obstacle_snapshot_ = nh_.subscribe(
            "/globalFsm_by_adsm/teacher_obstacle_snapshot", 40,
            &MpcSecbfNode::obstacleSnapshotCb, this);

        // Publishers
        pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        pub_local_path_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);
        pub_beta_applied_final_ =
            nh_.advertise<semantic_guard::AppliedMarginArray>("/safety_margin/beta_applied_final", 10);

        // Timers
        timer_replan_ = nh_.createTimer(ros::Duration(1.0 / mpc_freq), &MpcSecbfNode::replanCb, this);
        timer_cmd_ = nh_.createTimer(ros::Duration(0.01), &MpcSecbfNode::cmdCb, this);

        cur_state_.resize(5);
        cur_state_.setZero();
        has_odom_ = false;
        has_path_ = false;

        ROS_INFO("MPC-SECBF node started. freq=%.1f Hz, N=%d, Ts=%.2f, max_cbf_obstacles=%d",
                 mpc_freq, N, Ts, max_cbf_obstacles);
    }

    ~MpcSecbfNode() {
        if (planner_csv_.is_open()) planner_csv_.close();
        if (timing_csv_.is_open()) timing_csv_.close();
        if (mpc_margin_csv_.is_open()) mpc_margin_csv_.close();
        if (tau_stage_csv_.is_open()) tau_stage_csv_.close();
        if (guard_attempt_csv_.is_open()) guard_attempt_csv_.close();
        if (safety_recurrence_csv_.is_open()) safety_recurrence_csv_.close();
    }

private:
    void openCsv(std::ofstream& file, const std::string& path, const std::string& header) {
        if (path.empty()) return;
        file.open(path, std::ios::out);
        if (file.is_open()) {
            file << std::fixed << std::setprecision(9);
            file << header;
        } else {
            ROS_WARN("Failed to open CSV log path: %s", path.c_str());
        }
    }

    void odomCb(const nav_msgs::OdometryConstPtr& msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
        Eigen::Matrix3d R(q.normalized());
        double yaw = atan2(R.col(0)[1], R.col(0)[0]);
        double world_vx = 0.0;
        double world_vy = 0.0;
        if (!semantic_guard::bodyPlanarVelocityToWorld(
                msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w,
                &world_vx, &world_vy)) {
            has_odom_ = false;
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] invalid odometry quaternion/twist");
            return;
        }
        cur_state_ << msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      yaw, world_vx, world_vy;
        has_odom_ = true;
    }

    void pathCb(const nav_msgs::PathConstPtr& msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        int n = msg->poses.size();
        global_path_.resize(3, n);
        for (int i = 0; i < n; i++) {
            global_path_(0, i) = msg->poses[i].pose.position.x;
            global_path_(1, i) = msg->poses[i].pose.position.y;
            global_path_(2, i) = 0.0;  // yaw computed later
        }
        has_path_ = n > 0;
    }

    static bool finiteVector(const std::vector<double>& values) {
        return std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value);
        });
    }

    static bool nonnegativeFiniteVector(const std::vector<double>& values) {
        return std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value) && value >= 0.0;
        });
    }

    static bool nearlyEqual(double lhs, double rhs, double tolerance = 1e-8) {
        return std::abs(lhs - rhs) <=
            tolerance * std::max(1.0, std::max(std::abs(lhs), std::abs(rhs)));
    }

    void obstacleSnapshotCb(
        const semantic_guard::PredictedObstacleArrayConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const size_t obstacle_count = msg->obstacle_ids.size();
        const size_t expected_values =
            obstacle_count * static_cast<size_t>(7 * std::max(N_, 0));
        if (msg->cycle_id == 0 ||
            msg->cycle_id <= last_obstacle_message_cycle_id_ ||
            msg->header.stamp.isZero() || msg->header.frame_id != "world" ||
            msg->horizon_steps != static_cast<uint32_t>(N_) ||
            !std::isfinite(msg->prediction_step_sec) ||
            msg->prediction_step_sec <= 0.0 ||
            std::abs(msg->prediction_step_sec - Ts_) > 1e-6 ||
            msg->state_data.size() != expected_values ||
            !semantic_guard::hasUniqueObstacleIds(msg->obstacle_ids) ||
            !finiteVector(msg->state_data)) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[MPC-SECBF] Rejecting malformed/stale typed obstacle cycle=%lu",
                static_cast<unsigned long>(msg->cycle_id));
            return;
        }
        for (size_t obstacle_index = 0;
             obstacle_index < obstacle_count; ++obstacle_index) {
            for (int stage = 0; stage < N_; ++stage) {
                const size_t offset =
                    7 * (obstacle_index * static_cast<size_t>(N_) +
                         static_cast<size_t>(stage));
                if (msg->state_data[offset + 2] < 0.0 ||
                    msg->state_data[offset + 3] < 0.0) {
                    ROS_ERROR_THROTTLE(
                        1.0,
                        "[MPC-SECBF] Rejecting negative obstacle radius cycle=%lu",
                        static_cast<unsigned long>(msg->cycle_id));
                    return;
                }
            }
        }

        TypedObstacleCycle payload;
        payload.receipt_time = ros::Time::now();
        payload.source_time = msg->header.stamp;
        payload.obstacle_ids = msg->obstacle_ids;
        payload.obstacle_matrix.resize(
            7, static_cast<Eigen::Index>(obstacle_count *
                                         static_cast<size_t>(N_)));
        for (Eigen::Index column = 0;
             column < payload.obstacle_matrix.cols(); ++column) {
            for (Eigen::Index row = 0; row < 7; ++row) {
                payload.obstacle_matrix(row, column) =
                    msg->state_data[static_cast<size_t>(7 * column + row)];
            }
        }
        last_obstacle_message_cycle_id_ = msg->cycle_id;
        pending_obstacle_cycles_[msg->cycle_id] = std::move(payload);
        tryActivateTypedCycleLocked(msg->cycle_id);
        prunePendingCyclesLocked();
    }

    void preGuardCb(
        const semantic_guard::PreGuardMarginArrayConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const size_t count = msg->obstacle_ids.size();
        const bool matching_counts =
            msg->semantic_classes.size() == count &&
            msg->beta_bar.size() == count &&
            msg->beta_max.size() == count &&
            msg->beta_previous.size() == count &&
            msg->mu.size() == count && msg->beta_tilde.size() == count &&
            msg->available_margin.size() == count &&
            msg->positive_increment_bound.size() == count &&
            msg->beta_upper_bound.size() == count &&
            msg->beta_pre_guard.size() == count;
        const bool finite_nonnegative =
            nonnegativeFiniteVector(msg->beta_bar) &&
            nonnegativeFiniteVector(msg->beta_max) &&
            nonnegativeFiniteVector(msg->beta_previous) &&
            nonnegativeFiniteVector(msg->mu) &&
            nonnegativeFiniteVector(msg->beta_tilde) &&
            nonnegativeFiniteVector(msg->available_margin) &&
            nonnegativeFiniteVector(msg->positive_increment_bound) &&
            nonnegativeFiniteVector(msg->beta_upper_bound) &&
            nonnegativeFiniteVector(msg->beta_pre_guard);
        if (msg->obstacle_cycle_id == 0 ||
            msg->obstacle_cycle_id <= last_margin_message_cycle_id_ ||
            !matching_counts || !finite_nonnegative ||
            !semantic_guard::hasUniqueObstacleIds(msg->obstacle_ids)) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[MPC-SECBF] Rejecting malformed/stale pre-Guard cycle=%lu",
                static_cast<unsigned long>(msg->obstacle_cycle_id));
            return;
        }
        for (size_t index = 0; index < count; ++index) {
            double expected_upper = std::numeric_limits<double>::infinity();
            bool any_bound = false;
            if (msg->enforce_category_bound) {
                expected_upper = std::min(expected_upper,
                                          msg->beta_max[index]);
                any_bound = true;
            }
            if (msg->enforce_positive_increment_bound) {
                expected_upper = std::min(
                    expected_upper,
                    msg->positive_increment_bound[index]);
                any_bound = true;
            }
            if (msg->enforce_available_margin_bound) {
                expected_upper = std::min(expected_upper,
                                          msg->available_margin[index]);
                any_bound = true;
            }
            if (!any_bound) expected_upper = msg->beta_tilde[index];
            const double expected_pre =
                std::min(msg->beta_tilde[index], expected_upper);
            const double inferred_positive_increment =
                msg->positive_increment_bound[index] -
                msg->beta_previous[index];
            if (msg->semantic_classes[index].empty() ||
                msg->mu[index] > 1.0 + 1e-9 ||
                inferred_positive_increment <= 0.0 ||
                !nearlyEqual(msg->beta_upper_bound[index],
                             expected_upper) ||
                !nearlyEqual(msg->beta_pre_guard[index], expected_pre)) {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[MPC-SECBF] Rejecting F06/F07-inconsistent pre-Guard cycle=%lu",
                    static_cast<unsigned long>(msg->obstacle_cycle_id));
                return;
            }
        }

        TypedMarginCycle payload;
        payload.receipt_time = ros::Time::now();
        payload.obstacle_ids = msg->obstacle_ids;
        payload.beta_max = msg->beta_max;
        payload.beta_previous = msg->beta_previous;
        payload.beta_tilde = msg->beta_tilde;
        payload.available_margin = msg->available_margin;
        payload.positive_increment_bound = msg->positive_increment_bound;
        payload.beta_upper_bound = msg->beta_upper_bound;
        payload.beta_pre_guard = msg->beta_pre_guard;
        payload.enforce_category_bound = msg->enforce_category_bound;
        payload.enforce_positive_increment_bound =
            msg->enforce_positive_increment_bound;
        payload.enforce_available_margin_bound =
            msg->enforce_available_margin_bound;
        last_margin_message_cycle_id_ = msg->obstacle_cycle_id;
        pending_margin_cycles_[msg->obstacle_cycle_id] = std::move(payload);
        tryActivateTypedCycleLocked(msg->obstacle_cycle_id);
        prunePendingCyclesLocked();
    }

    void tryActivateTypedCycleLocked(uint64_t cycle_id) {
        if (cycle_id <= active_obstacle_cycle_id_) return;
        auto obstacle_it = pending_obstacle_cycles_.find(cycle_id);
        auto margin_it = pending_margin_cycles_.find(cycle_id);
        if (obstacle_it == pending_obstacle_cycles_.end() ||
            margin_it == pending_margin_cycles_.end()) {
            return;
        }

        std::vector<double> reordered_beta;
        std::vector<double> reordered_beta_max;
        std::vector<double> reordered_beta_previous;
        std::vector<double> reordered_beta_tilde;
        std::vector<double> reordered_available_margin;
        std::vector<double> reordered_positive_increment_bound;
        std::vector<double> reordered_beta_upper_bound;
        std::string join_reason;
        const auto reorder = [&](const std::vector<double>& values,
                                 std::vector<double>* output) {
            return semantic_guard::reorderValuesByObstacleId(
                margin_it->second.obstacle_ids, values,
                obstacle_it->second.obstacle_ids, output, &join_reason);
        };
        if (!reorder(margin_it->second.beta_pre_guard, &reordered_beta) ||
            !reorder(margin_it->second.beta_max, &reordered_beta_max) ||
            !reorder(margin_it->second.beta_previous,
                     &reordered_beta_previous) ||
            !reorder(margin_it->second.beta_tilde,
                     &reordered_beta_tilde) ||
            !reorder(margin_it->second.available_margin,
                     &reordered_available_margin) ||
            !reorder(margin_it->second.positive_increment_bound,
                     &reordered_positive_increment_bound) ||
            !reorder(margin_it->second.beta_upper_bound,
                     &reordered_beta_upper_bound)) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[MPC-SECBF] Rejecting typed cycle=%lu ID join: %s",
                static_cast<unsigned long>(cycle_id), join_reason.c_str());
            pending_obstacle_cycles_.erase(obstacle_it);
            pending_margin_cycles_.erase(margin_it);
            return;
        }

        obstacle_ids_ = obstacle_it->second.obstacle_ids;
        obs_matrix_ = obstacle_it->second.obstacle_matrix;
        beta_list_ = std::move(reordered_beta);
        beta_max_list_ = std::move(reordered_beta_max);
        beta_previous_list_ = std::move(reordered_beta_previous);
        beta_tilde_list_ = std::move(reordered_beta_tilde);
        available_margin_list_ = std::move(reordered_available_margin);
        positive_increment_bound_list_ =
            std::move(reordered_positive_increment_bound);
        beta_upper_bound_list_ = std::move(reordered_beta_upper_bound);
        active_enforce_category_bound_ =
            margin_it->second.enforce_category_bound;
        active_enforce_positive_increment_bound_ =
            margin_it->second.enforce_positive_increment_bound;
        active_enforce_available_margin_bound_ =
            margin_it->second.enforce_available_margin_bound;
        active_obstacle_cycle_id_ = cycle_id;
        active_cycle_receipt_time_ = obstacle_it->second.source_time;
        if (obstacle_it->second.receipt_time < active_cycle_receipt_time_) {
            active_cycle_receipt_time_ = obstacle_it->second.receipt_time;
        }
        if (margin_it->second.receipt_time < active_cycle_receipt_time_) {
            active_cycle_receipt_time_ = margin_it->second.receipt_time;
        }
        typed_payload_valid_ = true;
        semantic_guard::retainAcceptedMarginsForActiveIds(
            obstacle_ids_, &accepted_beta_by_id_);

        pending_obstacle_cycles_.erase(
            pending_obstacle_cycles_.begin(),
            pending_obstacle_cycles_.upper_bound(cycle_id));
        pending_margin_cycles_.erase(
            pending_margin_cycles_.begin(),
            pending_margin_cycles_.upper_bound(cycle_id));
    }

    void prunePendingCyclesLocked() {
        constexpr size_t kMaximumPendingCycles = 64;
        while (pending_obstacle_cycles_.size() > kMaximumPendingCycles) {
            pending_obstacle_cycles_.erase(pending_obstacle_cycles_.begin());
        }
        while (pending_margin_cycles_.size() > kMaximumPendingCycles) {
            pending_margin_cycles_.erase(pending_margin_cycles_.begin());
        }
    }

    void cmdCb(const ros::TimerEvent&) {
        pub_cmd_.publish(cmd_vel_);
    }

    static std::string serializeVector(const std::vector<double>& values) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(9);
        for (size_t i = 0; i < values.size(); ++i) {
            if (i != 0) stream << '|';
            stream << values[i];
        }
        return stream.str();
    }

    void writeGuardAttempt(size_t attempt_index, size_t obstacle_index,
                           size_t q, double candidate_beta,
                           const std::vector<double>& trial,
                           const std::vector<double>& accepted,
                           bool success, const std::string& status,
                           const std::string& reject_reason,
                           const MpcSolveTiming& timing) {
        if (!guard_attempt_csv_.is_open()) return;
        guard_attempt_csv_ << std::fixed << std::setprecision(9)
                           << ros::Time::now().toSec() << ","
                           << active_obstacle_cycle_id_ << ","
                           << attempt_index << ","
                           << obstacle_ids_[obstacle_index] << ","
                           << obstacle_index << "," << q << ","
                           << guard_kappa_ << "," << candidate_beta << ","
                           << trial[obstacle_index] << ","
                           << serializeVector(trial) << ","
                           << serializeVector(accepted) << ","
                           << (success ? 1 : 0) << "," << status << ","
                           << solver_.last_slack_max << "," << timing.total_ms << ","
                           << timing.graph_build_ms << "," << timing.ipopt_solve_ms << ","
                           << timing.solution_extract_ms << "," << timing.tau_audit_ms << ","
                           << timing.slack_audit_ms << ","
                           << (timing.graph_cache_enabled ? 1 : 0) << ","
                           << (timing.graph_cache_hit ? 1 : 0) << ","
                           << timing.graph_cache_slots << ","
                           << reject_reason << "\n";
        guard_attempt_csv_.flush();
    }

    bool runTeacherGuardSearch(const std::vector<double>& candidate,
                               std::vector<double>* accepted,
                               std::string* source,
                               std::string* status,
                               const ros::Time& search_start) {
        if (!accepted || !source || !status ||
            !validateBetaCountLocked(candidate, "guard_candidate")) {
            return false;
        }
        accepted->assign(candidate.size(), 0.0);
        *source = "candidate";
        *status = "guard_candidate";
        std::vector<unsigned int> ids(obstacle_ids_.begin(), obstacle_ids_.end());
        const auto order = semantic_guard::teacherRiskOrder(ids, beta_tilde_list_);
        size_t attempt_index = 0;
        if (order.empty()) {
            Eigen::MatrixXd empty_obs(7, 0);
            const bool ok = solver_.solve(&cur_state_, &goal_state_, &empty_obs, *accepted);
            *status = ok ? "guard_zero" : "baseline_infeasible";
            return ok;
        }
        for (size_t order_position = 0; order_position < order.size(); ++order_position) {
            const size_t index = order[order_position];
            bool component_accepted = false;
            for (size_t q = 0; q <= guard_max_backtracks_; ++q) {
                const double elapsed_ms = (ros::Time::now() - search_start).toSec() * 1000.0;
                if (elapsed_ms > guard_time_budget_ms_) {
                    *status = "guard_budget_exceeded";
                    return false;
                }
                std::vector<double> trial = *accepted;
                const double beta = semantic_guard::teacherBacktrackingCandidate(
                    candidate[index], guard_kappa_, q, guard_max_backtracks_);
                if (!std::isfinite(beta)) {
                    *status = "guard_invalid_candidate";
                    return false;
                }
                trial[index] = beta;
                const bool ok = solver_.solve(&cur_state_, &goal_state_, &obs_matrix_, trial);
                const MpcSolveTiming attempt_timing = solver_.last_timing;
                cycle_timing_.guard_attempts++;
                cycle_timing_.guard_solver_ms += attempt_timing.total_ms;
                cycle_timing_.guard_graph_build_ms += attempt_timing.graph_build_ms;
                cycle_timing_.guard_ipopt_ms += attempt_timing.ipopt_solve_ms;
                cycle_timing_.guard_extract_ms += attempt_timing.solution_extract_ms;
                cycle_timing_.guard_tau_audit_ms += attempt_timing.tau_audit_ms;
                cycle_timing_.guard_slack_audit_ms += attempt_timing.slack_audit_ms;
                writeGuardAttempt(attempt_index++, index, q, candidate[index], trial,
                                  *accepted, ok, ok ? "feasible" : "infeasible",
                                  ok ? "" : (q == guard_max_backtracks_ ? "zero_failed" : "retry_kappa"),
                                  attempt_timing);
                if (ok) {
                    *accepted = std::move(trial);
                    component_accepted = true;
                    if (q > 0) *source = "kappa";
                    break;
                }
            }
            if (!component_accepted) {
                *status = "baseline_infeasible";
                return false;
            }
        }
        *status = (*source == "candidate") ? "guard_candidate" : "guard_kappa";
        return true;
    }

    void replanCb(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock_o(odom_mutex_);
        std::lock_guard<std::mutex> lock_p(path_mutex_);
        std::lock_guard<std::mutex> lock_data(data_mutex_);

        if (!has_odom_ || !has_path_) return;
        const double payload_age = active_cycle_receipt_time_.isZero()
            ? std::numeric_limits<double>::infinity()
            : (ros::Time::now() - active_cycle_receipt_time_).toSec();
        if (!typed_payload_valid_ || active_obstacle_cycle_id_ == 0 ||
            !std::isfinite(payload_age) || payload_age < 0.0 ||
            payload_age > typed_payload_timeout_sec_ ||
            !cur_state_.allFinite()) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[MPC-SECBF] Rejecting absent/invalid/stale typed payload age=%.3f cycle=%lu",
                payload_age,
                static_cast<unsigned long>(active_obstacle_cycle_id_));
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            solver_.resetAuditMetrics();
            writePlannerCsv("payload_invalid", "payload_invalid", "payload_invalid", "none",
                            false, false, 0.0);
            return;
        }
        if (!validateObstacleContractLocked() ||
            !validateTypedMarginContractLocked()) {
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Rejecting cycle because obstacle payload and IDs do not match");
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            solver_.resetAuditMetrics();
            writePlannerCsv("count_mismatch", "count_mismatch", "count_mismatch", "none",
                            false, false, 0.0);
            return;
        }
        if (active_obstacle_cycle_id_ == last_processed_cycle_id_) return;
        last_processed_cycle_id_ = active_obstacle_cycle_id_;

        bool candidate_reprojected = false;
        if (active_enforce_positive_increment_bound_) {
            for (size_t index = 0; index < obstacle_ids_.size(); ++index) {
                const auto accepted_it =
                    accepted_beta_by_id_.find(obstacle_ids_[index]);
                const double accepted_previous =
                    accepted_it == accepted_beta_by_id_.end()
                        ? 0.0 : accepted_it->second;
                const double delta_positive =
                    positive_increment_bound_list_[index] -
                    beta_previous_list_[index];
                const double current_positive_bound =
                    accepted_previous + delta_positive;
                if (beta_list_[index] > current_positive_bound + 1e-9) {
                    beta_list_[index] = current_positive_bound;
                    candidate_reprojected = true;
                }
            }
        }

        // Choose goal states from global path
        chooseGoalState();
        smoothYaw(goal_state_);

        ros::Time t0 = ros::Time::now();
        cycle_timing_ = CycleTiming();

        // Solve MPC-SECBF
        std::string mpc_status = "success";
        std::string first_attempt_status = "not_run";
        std::string final_status = "not_run";
        std::string accepted_beta_source = candidate_reprojected
            ? "mpc_reprojected" : "candidate";
        bool used_fallback = false;
        bool mpc_guard_used = false;
        bool success = false;
        std::vector<double> final_beta_values;

        if (!validateBetaCountLocked(beta_list_, "candidate")) {
            first_attempt_status = "beta_count_mismatch";
        } else {
            success = solver_.solve(&cur_state_, &goal_state_, &obs_matrix_, beta_list_);
            cycle_timing_.initial_solver = solver_.last_timing;
            first_attempt_status = success ? "success" : "infeasible";
        }
        final_status = first_attempt_status;

        if (mpc_feasibility_guard_enabled_) {
            mpc_guard_used = true;
            const NodeSteadyClock::time_point guard_start = NodeSteadyClock::now();
            success = runTeacherGuardSearch(beta_list_, &final_beta_values,
                                            &accepted_beta_source, &mpc_status, t0);
            cycle_timing_.guard_phase_ms = nodeElapsedMs(guard_start);
            final_status = success ? "success" : mpc_status;
        }

        if (!success && mpc_feasibility_guard_enabled_) {
            // F09 treats failure of the explicit zero candidate as baseline
            // infeasible. Do not silently turn this theorem path into no-CBF.
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            const double cost_ms = (ros::Time::now() - t0).toSec() * 1000.0;
            writePlannerCsv("baseline_infeasible", first_attempt_status,
                            final_status, "none", false, true, cost_ms);
            return;
        }

        if (!success) {
            // Fallback: try without CBF constraints (empty beta)
            used_fallback = true;
            std::vector<double> empty_beta;
            Eigen::MatrixXd empty_obs(7, 0);
            success = solver_.solve(&cur_state_, &goal_state_, &empty_obs, empty_beta);

            if (!success) {
                // Complete failure: stop
                ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Both SECBF and fallback infeasible, STOPPING");
                cmd_vel_.linear.x = 0.0;
                cmd_vel_.angular.z = 0.0;
                double cost_ms = (ros::Time::now() - t0).toSec() * 1000.0;
                writePlannerCsv("zero", first_attempt_status, "zero", "none",
                                used_fallback, mpc_guard_used, cost_ms);
                return;
            } else {
                mpc_status = "no_cbf_fallback";
                final_status = "success";
                accepted_beta_source = "no_cbf";
                final_beta_values.assign(obstacle_ids_.size(), 0.0);
                ROS_WARN_THROTTLE(1.0, "[MPC-SECBF] Fallback (no CBF) succeeded");
            }
        } else if (accepted_beta_source == "candidate" ||
                   accepted_beta_source == "mpc_reprojected") {
            final_beta_values = beta_list_;
        }

        if (accepted_beta_source != "no_cbf" &&
            !semantic_guard::storeAcceptedMargins(
                obstacle_ids_, final_beta_values, &accepted_beta_by_id_)) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[MPC-SECBF] Refusing to store invalid accepted-margin history");
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            return;
        }

        writeMpcMarginCsv(final_beta_values, accepted_beta_source,
                          first_attempt_status, final_status, mpc_guard_used);
        publishAcceptedMargins(final_beta_values, accepted_beta_source);
        writeTauStageCsv(accepted_beta_source);
        writeSafetyRecurrenceCsv(final_beta_values, accepted_beta_source);

        // Extract first control
        if (solver_.predict_u.size() >= 2) {
            cmd_vel_.linear.x = solver_.predict_u[0];
            cmd_vel_.angular.z = solver_.predict_u[1];
        } else {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
        }

        // Publish local path
        if (!solver_.predict_x.empty()) {
            pubLocalPath(solver_.predict_x);
        }

        double cost_ms = (ros::Time::now() - t0).toSec() * 1000.0;
        writePlannerCsv(mpc_status, first_attempt_status, final_status, accepted_beta_source,
                        used_fallback, mpc_guard_used, cost_ms);
        std::cout << "\033[38;2;0;200;100m MPC-SECBF replan_time =: \033[0m" << cost_ms << "ms" << std::endl;
    }

    void writePlannerCsv(const std::string& mpc_status,
                         const std::string& first_attempt_status,
                         const std::string& final_status,
                         const std::string& accepted_beta_source,
                         bool used_fallback,
                         bool mpc_guard_used,
                         double solve_time_ms) {
        const double t = ros::Time::now().toSec();
        const double ref_x = goal_state_.cols() > 0 ? goal_state_(0, 0) : cur_state_(0);
        const double ref_y = goal_state_.cols() > 0 ? goal_state_(1, 0) : cur_state_(1);
        const double tracking_error = std::hypot(cur_state_(0) - ref_x, cur_state_(1) - ref_y);
        const int obs_count = (N_ > 0) ? static_cast<int>(obs_matrix_.cols() / N_) : 0;
        const bool obstacle_contract_valid = validateObstacleContractLocked();
        const int constrained_obs_count = obstacle_contract_valid
                                             ? solver_.last_constrained_obs_count
                                             : 0;
        const bool dynamic_tau_enabled = dynamic_tau_enabled_;
        semantic_guard::DynamicTauResult tau_result;
        tau_result.mode = dynamic_tau_params_.mode;
        if (!dynamic_tau_enabled) {
            tau_result.computed = true;
            tau_result.reason = "disabled";
        } else if (!obstacle_contract_valid || solver_.last_constrained_obs_index < 0 || N_ <= 0 ||
                   solver_.last_constrained_obs_index * N_ >= obs_matrix_.cols()) {
            tau_result.reason = "no_constrained_obstacle";
        } else if (!solver_.last_tau_stage_audit.empty()) {
            // This value was evaluated from the optimized first-stage state in
            // MPC_SECBF_SOLVE::solve(). Do not reconstruct Teacher TCA from the
            // current measurement here.
            tau_result = solver_.last_tau_stage_audit.front().tau_result;
        } else {
            tau_result.reason = "stage_audit_unavailable";
        }
        if (planner_csv_.is_open()) {
            planner_csv_ << t << ","
                         << active_obstacle_cycle_id_ << ","
                         << mpc_status << ","
                         << first_attempt_status << ","
                         << final_status << ","
                         << accepted_beta_source << ","
                         << cmd_vel_.linear.x << ","
                         << cmd_vel_.angular.z << ","
                         << ref_x << ","
                         << ref_y << ","
                         << tracking_error << ","
                         << obs_count << ","
                         << constrained_obs_count << ","
                         << beta_list_.size() << ","
                         << (used_fallback ? 1 : 0) << ","
                         << (mpc_feasibility_guard_enabled_ ? 1 : 0) << ","
                         << ((first_attempt_status == "success" || first_attempt_status == "infeasible") ? 1 : 0) << ","
                         << (mpc_guard_used ? 1 : 0) << ","
                         << solver_.last_slack_max << ","
                         << solver_.last_slack_sum << ","
                         << solver_.last_slack_mean << ","
                         << solver_.last_slack_max << ","
                         << (side_preference_enabled_ ? 1 : 0) << ","
                         << side_weight_ << ","
                         << solver_.last_side_cost << ","
                         << solver_.last_side_dynamic_obstacle_count << ","
                         << solver_.last_side_candidate_count << ","
                         << solver_.last_side_dominant_obs_index << ","
                         << solver_.last_side_dominant_stage << ","
                         << solver_.last_side_dominant_tau << ","
                         << solver_.last_side_dominant_h << ","
                         << solver_.last_delta_u_max << ","
                         << qf_scale_ << ","
                         << delta_u_weight_ << ","
                         << delta_u_max_ << ","
                         << solve_time_ms << ","
                         << dynamic_tau_enabled << ","
                         << semantic_guard::dynamicTauModeName(tau_result.mode) << ","
                         << tau_result.tau << ","
                         << tau_result.t_ca_raw << ","
                         << tau_result.t_ca_clipped << ","
                         << (tau_result.ke_scaled ? dynamic_tau_params_.ke : 1.0) << ","
                         << tau_result.computed << ","
                         << tau_result.valid << ","
                         << tau_result.lower_clipped << ","
                         << tau_result.upper_clipped << ","
                         << tau_result.T_i << ","
                         << tau_result.f_r << ","
                         << tau_result.f_v << ","
                         << tau_result.f_T << ","
                         << tau_result.computed << ","
                         << tau_result.reason << "\n";
            planner_csv_.flush();
        }
        if (timing_csv_.is_open()) {
            timing_csv_ << t << ","
                        << active_obstacle_cycle_id_ << ","
                        << solve_time_ms << ","
                        << solve_time_ms << ","
                        << cycle_timing_.initial_solver.total_ms << ","
                        << cycle_timing_.initial_solver.graph_build_ms << ","
                        << cycle_timing_.initial_solver.ipopt_solve_ms << ","
                        << cycle_timing_.initial_solver.solution_extract_ms << ","
                        << cycle_timing_.initial_solver.tau_audit_ms << ","
                        << cycle_timing_.initial_solver.slack_audit_ms << ","
                        << cycle_timing_.guard_phase_ms << ","
                        << cycle_timing_.guard_solver_ms << ","
                        << cycle_timing_.guard_graph_build_ms << ","
                        << cycle_timing_.guard_ipopt_ms << ","
                        << cycle_timing_.guard_extract_ms << ","
                        << cycle_timing_.guard_tau_audit_ms << ","
                        << cycle_timing_.guard_slack_audit_ms << ","
                        << cycle_timing_.guard_attempts << ","
                        << obstacle_ids_.size() << ","
                        << solver_.last_constrained_obs_count << ","
                        << (cycle_timing_.initial_solver.success ? 1 : 0) << ","
                        << (cycle_timing_.initial_solver.graph_cache_hit ? 1 : 0) << ","
                        << (cycle_timing_.initial_solver.graph_cache_enabled ? 1 : 0) << ","
                        << cycle_timing_.initial_solver.graph_cache_slots << "\n";
            timing_csv_.flush();
        }
    }

    void writeTauStageCsv(const std::string& accepted_beta_source) {
        if (!tau_stage_csv_.is_open()) return;
        const double t = ros::Time::now().toSec();
        for (const MpcTauStageAudit& audit : solver_.last_tau_stage_audit) {
            if (audit.obstacle_index < 0 ||
                static_cast<size_t>(audit.obstacle_index) >= obstacle_ids_.size()) {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[MPC-SECBF] Skip tau stage row because obstacle index is invalid");
                continue;
            }
            const semantic_guard::DynamicTauResult& tau = audit.tau_result;
            tau_stage_csv_ << t << ","
                           << active_obstacle_cycle_id_ << ","
                           << accepted_beta_source << ","
                           << obstacle_ids_[audit.obstacle_index] << ","
                           << audit.obstacle_index << ","
                           << audit.stage << ","
                           << semantic_guard::dynamicTauModeName(tau.mode) << ","
                           << audit.lx << ","
                           << audit.ly << ","
                           << audit.vrel_x << ","
                           << audit.vrel_y << ","
                           << tau.t_ca_raw << ","
                           << tau.t_ca_clipped << ","
                           << tau.tau << ","
                           << (tau.ke_scaled ? dynamic_tau_params_.ke : 1.0) << ","
                           << tau.computed << ","
                           << tau.valid << ","
                           << tau.lower_clipped << ","
                           << tau.upper_clipped << ","
                           << audit.r_base << ","
                           << audit.beta << ","
                           << audit.h_eesm << ","
                           << audit.h_seesm << ","
                           << tau.computed << ","
                           << tau.reason << "\n";
        }
        tau_stage_csv_.flush();
    }

    void writeSafetyRecurrenceCsv(const std::vector<double>& final_beta_values,
                                  const std::string& accepted_beta_source) {
        struct StagePair { double h0 = 0.0; double h1 = 0.0; bool has0 = false; bool has1 = false; };
        std::map<uint32_t, StagePair> current;
        for (const MpcTauStageAudit& audit : solver_.last_tau_stage_audit) {
            if (audit.obstacle_index < 0 ||
                static_cast<size_t>(audit.obstacle_index) >= obstacle_ids_.size()) continue;
            StagePair& pair = current[obstacle_ids_[audit.obstacle_index]];
            if (audit.stage == 0) { pair.h0 = audit.h_eesm; pair.has0 = true; }
            if (audit.stage == 1) { pair.h1 = audit.h_eesm; pair.has1 = true; }
        }
        if (safety_recurrence_csv_.is_open()) {
            for (const auto& previous : pending_safety_) {
                const auto now = current.find(previous.first);
                const auto id_it = std::find(obstacle_ids_.begin(), obstacle_ids_.end(), previous.first);
                if (now == current.end() || !now->second.has0 || id_it == obstacle_ids_.end()) continue;
                const size_t index = static_cast<size_t>(std::distance(obstacle_ids_.begin(), id_it));
                semantic_guard::SafetyRecurrenceInput input;
                input.gamma = gamma_;
                input.h_eesm_t = previous.second.h_eesm_t;
                input.beta_t = previous.second.beta_t;
                input.h_eesm_pred_next = previous.second.h_eesm_pred_next;
                input.h_eesm_next = now->second.h0;
                input.beta_next = index < final_beta_values.size() ? final_beta_values[index] : 0.0;
                input.epsilon_t = previous.second.epsilon_t;
                input.epsilon_max = epsilon_max_runtime_;
                input.delta_bar = safety_delta_bar_;
                input.delta_beta_bar = safety_delta_beta_bar_;
                input.cbf_executed = previous.second.cbf_executed && accepted_beta_source != "no_cbf";
                input.backup_used = previous.second.backup_used || accepted_beta_source == "no_cbf";
                input.baseline_infeasible = previous.second.baseline_infeasible;
                const auto audit = semantic_guard::auditSafetyRecurrence(input);
                std::string exclusion_reason;
                if (!audit.finite) exclusion_reason = "nonfinite_or_invalid_contract";
                else if (!input.cbf_executed) exclusion_reason = "cbf_not_executed";
                else if (input.backup_used) exclusion_reason = "backup_or_no_cbf";
                else if (input.baseline_infeasible) exclusion_reason = "baseline_infeasible";
                else if (input.epsilon_t > input.epsilon_max + 1e-12) exclusion_reason = "epsilon_bound_exceeded";
                else if (audit.delta > safety_delta_bar_ + 1e-12) exclusion_reason = "delta_bound_exceeded";
                else if (audit.delta_beta_plus > safety_delta_beta_bar_ + 1e-12) exclusion_reason = "delta_beta_bound_exceeded";
                safety_recurrence_csv_ << std::fixed << std::setprecision(9)
                    << ros::Time::now().toSec() << "," << previous.second.obstacle_cycle_id << ","
                    << previous.first << "," << audit.H_t << "," << input.h_eesm_t << "," << input.beta_t << ","
                    << input.h_eesm_pred_next << "," << audit.H_pred_next << "," << input.h_eesm_next << ","
                    << input.beta_next << "," << audit.H_next << "," << input.epsilon_t << "," << input.epsilon_max << ","
                    << audit.delta << "," << input.delta_bar << "," << audit.delta_beta_plus << ","
                    << input.delta_beta_bar << "," << audit.recursion_rhs << "," << audit.one_step_residual << ","
                    << audit.bar_w << "," << audit.asymptotic_bound << "," << input.cbf_executed << ","
                    << input.backup_used << "," << input.baseline_infeasible << ","
                    << audit.theorem1_applicable << "," << exclusion_reason << "\n";
            }
            safety_recurrence_csv_.flush();
        }
        std::map<uint32_t, PendingSafetyRecurrence> next;
        const bool executed = accepted_beta_source != "no_cbf" && accepted_beta_source != "none";
        for (const auto& item : current) {
            if (!item.second.has0 || !item.second.has1) continue;
            const auto id_it = std::find(obstacle_ids_.begin(), obstacle_ids_.end(), item.first);
            if (id_it == obstacle_ids_.end()) continue;
            const size_t index = static_cast<size_t>(std::distance(obstacle_ids_.begin(), id_it));
            PendingSafetyRecurrence pending;
            pending.obstacle_cycle_id = active_obstacle_cycle_id_;
            pending.h_eesm_t = item.second.h0;
            pending.beta_t = index < final_beta_values.size() ? final_beta_values[index] : 0.0;
            pending.h_eesm_pred_next = item.second.h1;
            pending.epsilon_t = solver_.last_slack_max;
            pending.cbf_executed = executed;
            pending.backup_used = accepted_beta_source == "no_cbf";
            pending.baseline_infeasible = accepted_beta_source == "none";
            next[item.first] = pending;
        }
        pending_safety_.swap(next);
    }

    void writeMpcMarginCsv(const std::vector<double>& final_beta_values,
                           const std::string& accepted_beta_source,
                           const std::string& first_attempt_status,
                           const std::string& final_status,
                           bool mpc_guard_used) {
        if (!mpc_margin_csv_.is_open()) return;
        if (beta_list_.size() != obstacle_ids_.size() ||
            final_beta_values.size() != obstacle_ids_.size()) {
            ROS_ERROR_THROTTLE(1.0,
                "[MPC-SECBF] Skip MPC margin audit row because beta/id counts do not match");
            return;
        }
        const double t = ros::Time::now().toSec();
        const bool candidate_checked =
            first_attempt_status == "success" || first_attempt_status == "infeasible";
        for (size_t i = 0; i < obstacle_ids_.size(); ++i) {
            mpc_margin_csv_ << t << ","
                            << active_obstacle_cycle_id_ << ","
                            << obstacle_ids_[i] << ","
                            << beta_list_[i] << ","
                            << final_beta_values[i] << ","
                            << accepted_beta_source << ","
                            << first_attempt_status << ","
                            << final_status << ","
                            << (mpc_feasibility_guard_enabled_ ? 1 : 0) << ","
                            << (candidate_checked ? 1 : 0) << ","
                            << (mpc_guard_used ? 1 : 0) << ","
                            << solver_.last_delta_u_max << ","
                            << qf_scale_ << ","
                            << delta_u_weight_ << ","
                            << delta_u_max_ << "\n";
        }
        mpc_margin_csv_.flush();
    }

    bool validateObstacleContractLocked() const {
        if (obs_matrix_.cols() == 0 && obstacle_ids_.empty()) return true;
        if (N_ <= 0 || obs_matrix_.cols() % N_ != 0) return false;
        return static_cast<size_t>(obs_matrix_.cols() / N_) == obstacle_ids_.size();
    }

    bool validateTypedMarginContractLocked() const {
        const size_t count = obstacle_ids_.size();
        return beta_list_.size() == count &&
               beta_max_list_.size() == count &&
               beta_previous_list_.size() == count &&
               beta_tilde_list_.size() == count &&
               available_margin_list_.size() == count &&
               positive_increment_bound_list_.size() == count &&
               beta_upper_bound_list_.size() == count;
    }

    bool validateBetaCountLocked(const std::vector<double>& beta,
                                 const std::string& source) const {
        if (beta.size() != obstacle_ids_.size()) {
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] %s beta/id count mismatch: beta=%zu ids=%zu",
                               source.c_str(), beta.size(), obstacle_ids_.size());
            return false;
        }
        return true;
    }

    void publishAcceptedMargins(const std::vector<double>& beta,
                                const std::string& source) {
        if (!validateObstacleContractLocked()) {
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Skip final margin publication because obstacle contract is invalid");
            return;
        }
        if (!validateBetaCountLocked(beta, source)) {
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Skip final margin publication because final beta count does not match IDs");
            return;
        }

        semantic_guard::AppliedMarginArray out;
        out.header.stamp = ros::Time::now();
        out.obstacle_cycle_id = active_obstacle_cycle_id_;
        out.obstacle_ids = obstacle_ids_;
        out.beta_applied.assign(beta.begin(), beta.end());
        out.accepted_sources.assign(beta.size(), source);
        pub_beta_applied_final_.publish(out);
    }

    void chooseGoalState() {
        int wp_num = global_path_.cols();
        if (wp_num == 0) return;

        // Find closest waypoint
        double min_dist = std::numeric_limits<double>::max();
        int closest = 0;
        for (int i = 0; i < wp_num; i++) {
            double d = (cur_state_.head<2>() - global_path_.block<2, 1>(0, i)).norm();
            if (d < min_dist) { min_dist = d; closest = i; }
        }

        goal_state_.resize(3, N_);
        double last_yaw = cur_state_(2);
        for (int i = 0; i < N_; i++) {
            int idx = std::min(closest + i, wp_num - 1);
            goal_state_.col(i) = global_path_.col(idx);
            if (i > 0) {
                Eigen::Vector2d diff = (goal_state_.col(i) - goal_state_.col(i - 1)).head<2>();
                double yaw = (diff.norm() > 0.01) ? atan2(diff.y(), diff.x()) : last_yaw;
                goal_state_(2, i - 1) = yaw;
                last_yaw = yaw;
            }
        }
        goal_state_(2, N_ - 1) = last_yaw;
    }

    void smoothYaw(Eigen::MatrixXd& ref) {
        double dyaw = ref(2, 0) - cur_state_(2);
        while (dyaw >= M_PI / 2) { ref(2, 0) -= 2 * M_PI; dyaw = ref(2, 0) - cur_state_(2); }
        while (dyaw <= -M_PI / 2) { ref(2, 0) += 2 * M_PI; dyaw = ref(2, 0) - cur_state_(2); }

        for (int i = 0; i < N_ - 1; i++) {
            dyaw = ref(2, i + 1) - ref(2, i);
            while (dyaw >= M_PI / 2) { ref(2, i + 1) -= 2 * M_PI; dyaw = ref(2, i + 1) - ref(2, i); }
            while (dyaw <= -M_PI / 2) { ref(2, i + 1) += 2 * M_PI; dyaw = ref(2, i + 1) - ref(2, i); }
        }
    }

    void pubLocalPath(const std::vector<double>& pred_x) {
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        for (size_t i = 0; i < pred_x.size() / 5; i++) {
            geometry_msgs::PoseStamped ps;
            ps.pose.position.x = pred_x[5 * i];
            ps.pose.position.y = pred_x[5 * i + 1];
            ps.pose.orientation.w = 1.0;
            path.poses.push_back(ps);
        }
        pub_local_path_.publish(path);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_, sub_path_, sub_pre_guard_, sub_obstacle_snapshot_;
    ros::Publisher pub_cmd_, pub_local_path_, pub_beta_applied_final_;
    ros::Timer timer_replan_, timer_cmd_;

    MPC_SECBF_SOLVE solver_;
    CycleTiming cycle_timing_;
    int N_;
    double Ts_;
    double robot_radius_ = 0.4;

    std::mutex odom_mutex_, path_mutex_, data_mutex_;
    Eigen::VectorXd cur_state_;
    Eigen::MatrixXd global_path_;
    Eigen::MatrixXd goal_state_;
    Eigen::MatrixXd obs_matrix_;
    std::vector<uint32_t> obstacle_ids_;
    std::vector<double> beta_list_;
    std::vector<double> beta_max_list_;
    std::vector<double> beta_previous_list_;
    std::vector<double> beta_tilde_list_;
    std::vector<double> available_margin_list_;
    std::vector<double> positive_increment_bound_list_;
    std::vector<double> beta_upper_bound_list_;
    std::map<uint32_t, double> accepted_beta_by_id_;
    std::map<uint64_t, TypedObstacleCycle> pending_obstacle_cycles_;
    std::map<uint64_t, TypedMarginCycle> pending_margin_cycles_;
    uint64_t last_obstacle_message_cycle_id_ = 0;
    uint64_t last_margin_message_cycle_id_ = 0;
    uint64_t active_obstacle_cycle_id_ = 0;
    uint64_t last_processed_cycle_id_ = 0;
    ros::Time active_cycle_receipt_time_;
    geometry_msgs::Twist cmd_vel_;
    bool has_odom_, has_path_;
    bool mpc_feasibility_guard_enabled_;
    bool side_preference_enabled_ = false;
    double side_weight_ = 0.2;
    bool typed_payload_valid_ = false;
    bool active_enforce_category_bound_ = true;
    bool active_enforce_positive_increment_bound_ = true;
    bool active_enforce_available_margin_bound_ = true;
    double typed_payload_timeout_sec_ = 0.50;
    double qf_scale_ = 1.1;
    double delta_u_weight_ = 0.02;
    double delta_u_max_ = 0.4;
    bool dynamic_tau_enabled_ = false;
    double gamma_ = 0.35;
    double epsilon_max_runtime_ = 0.05;
    double safety_delta_bar_ = 0.10;
    double safety_delta_beta_bar_ = 0.30;
    semantic_guard::DynamicTauParams dynamic_tau_params_;
    std::ofstream planner_csv_, timing_csv_, mpc_margin_csv_, tau_stage_csv_, guard_attempt_csv_, safety_recurrence_csv_;
    std::map<uint32_t, PendingSafetyRecurrence> pending_safety_;
    double guard_kappa_ = 0.5;
    std::size_t guard_max_backtracks_ = 6;
    double guard_time_budget_ms_ = 500.0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_secbf_node");
    ros::NodeHandle nh("~");
    MpcSecbfNode node(nh);
    ros::spin();
    return 0;
}
