#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <stdexcept>
#include <vector>

#include "mpc_secbf/mpc_secbf.h"
#include "semantic_guard/AppliedMarginArray.h"
#include "semantic_guard/dynamic_tau.hpp"
#include "semantic_guard/planar_velocity.hpp"
#include "semantic_fusion/SemanticObstacleArray.h"

class MpcSecbfNode {
public:
    MpcSecbfNode(ros::NodeHandle& nh) : nh_(nh) {
        // Parameters
        double mpc_freq, Ts, gamma, beta_unknown, robot_radius;
        double epsilon_max, slack_weight;
        int N;
        int max_cbf_obstacles;
        bool mpc_feasibility_guard_enabled;
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
        nh_.param("mpc/max_cbf_obstacles", max_cbf_obstacles, 6);
        nh_.param("mpc/feasibility_guard_enabled", mpc_feasibility_guard_enabled, true);
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
        nh_.param<std::string>("planner_log_path", planner_log_path, "");
        nh_.param<std::string>("timing_log_path", timing_log_path, "");
        nh_.param<std::string>("mpc_margin_log_path", mpc_margin_log_path, "");
        nh_.param<std::string>("tau_stage_log_path", tau_stage_log_path, "");

        std::vector<double> Q = {1.0, 1.0, 0.05};
        std::vector<double> R = {0.1, 0.05};

        N_ = N;
        Ts_ = Ts;
        mpc_feasibility_guard_enabled_ = mpc_feasibility_guard_enabled;

        // Initialize solver
        solver_.init_solver(Ts, N, v_max, v_min, o_max, Q, R, gamma, beta_unknown, robot_radius,
                            epsilon_max, slack_weight, max_cbf_obstacles, cbf_metric,
                            dynamic_tau_enabled_, dynamic_tau_params_,
                            side_preference_enabled, side_weight, side_epsilon_n,
                            side_horizon, side_sign, side_min_obstacle_speed,
                            side_activation_distance);
        side_preference_enabled_ = side_preference_enabled;
        side_weight_ = side_weight;

        openCsv(planner_csv_, planner_log_path,
                "t,mpc_status,first_attempt_status,final_status,accepted_beta_source,"
                "cmd_v,cmd_w,ref_x,ref_y,tracking_error,obs_count,constrained_obs_count,beta_count,used_fallback,"
                "mpc_feasibility_guard_enabled,candidate_feasibility_checked,mpc_feasibility_guard_used,"
                "slack,slack_sum,slack_mean,slack_max,side_preference_enabled,side_weight,side_cost,"
                "side_dynamic_obstacle_count,side_candidate_count,side_dominant_obs_index,side_dominant_stage,side_dominant_tau,side_dominant_h,solve_time_ms,"
                "dynamic_tau_enabled,tau_mode,tau,tca_raw,tca_clipped,tau_scale,tau_active,tau_clipped_low,tau_clipped_high,"
                "T_i,f_r,f_v,f_T,tau_valid,tau_reason\n");
        openCsv(timing_csv_, timing_log_path,
                "t,mpc_secbf_ms,total_loop_time_ms\n");
        openCsv(mpc_margin_csv_, mpc_margin_log_path,
                "t,obs_id,beta_pre_guard,beta_applied,accepted_beta_source,"
                "first_attempt_status,final_status,mpc_feasibility_guard_enabled,"
                "candidate_feasibility_checked,mpc_feasibility_guard_used\n");
        openCsv(tau_stage_csv_, tau_stage_log_path,
                "t,accepted_beta_source,obs_id,obs_index,stage,tau_mode,lx,ly,vrel_x,vrel_y,"
                "tca_raw,tca_clipped,tau,tau_scale,tau_active,tau_clipped_low,tau_clipped_high,"
                "beta,h_eesm,h_seesm,tau_valid,tau_reason\n");

        // Subscribers
        sub_odom_ = nh_.subscribe("/Odometry", 1, &MpcSecbfNode::odomCb, this);
        sub_path_ = nh_.subscribe("/global_path", 10, &MpcSecbfNode::pathCb, this);
        sub_beta_ = nh_.subscribe("/safety_margin/beta", 10, &MpcSecbfNode::betaCb, this);
        sub_obs_ = nh_.subscribe("/globalFsm_by_adsm/obs_predict_pub", 100, &MpcSecbfNode::obsCb, this);
        sub_obs_ids_ = nh_.subscribe("/globalFsm_by_adsm/obs_predict_ids", 100, &MpcSecbfNode::obsIdsCb, this);

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
        has_path_ = true;
    }

    void betaCb(const std_msgs::Float32MultiArrayConstPtr& msg) {
        std::lock_guard<std::mutex> lock(beta_mutex_);
        const bool finite_payload = std::all_of(
            msg->data.begin(), msg->data.end(), [](float value) {
                return std::isfinite(static_cast<double>(value));
            });
        if (!finite_payload) {
            beta_list_.clear();
            beta_payload_valid_ = false;
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Rejecting non-finite beta payload");
            return;
        }
        beta_list_.assign(msg->data.begin(), msg->data.end());
        beta_payload_valid_ = true;
    }

    void obsCb(const std_msgs::Float32MultiArrayConstPtr& msg) {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        const bool finite_payload = std::all_of(
            msg->data.begin(), msg->data.end(), [](float value) {
                return std::isfinite(static_cast<double>(value));
            });
        if (N_ <= 0 || msg->data.size() % (7 * N_) != 0 || !finite_payload) {
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Rejecting invalid obstacle payload size=%zu finite=%s for N=%d",
                               msg->data.size(), finite_payload ? "true" : "false", N_);
            obs_matrix_.resize(7, 0);
            obs_payload_valid_ = false;
            return;
        }
        int obs_cols = msg->data.size() / 7;
        obs_matrix_.resize(7, obs_cols);
        for (int i = 0; i < obs_cols; i++) {
            for (int j = 0; j < 7; j++)
                obs_matrix_(j, i) = msg->data[7 * i + j];
        }
        obs_payload_valid_ = true;
    }

    void obsIdsCb(const std_msgs::UInt32MultiArrayConstPtr& msg) {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        obstacle_ids_ = msg->data;
    }

    void cmdCb(const ros::TimerEvent&) {
        pub_cmd_.publish(cmd_vel_);
    }

    void replanCb(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock_o(odom_mutex_);
        std::lock_guard<std::mutex> lock_p(path_mutex_);
        std::lock_guard<std::mutex> lock_b(beta_mutex_);
        std::lock_guard<std::mutex> lock_obs(obs_mutex_);

        if (!has_odom_ || !has_path_) return;
        if (!obs_payload_valid_ || !beta_payload_valid_ || !cur_state_.allFinite()) {
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Rejecting cycle with invalid finite payload");
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            solver_.resetAuditMetrics();
            writePlannerCsv("payload_invalid", "payload_invalid", "payload_invalid", "none",
                            false, false, 0.0);
            return;
        }
        if (!validateObstacleContractLocked()) {
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Rejecting cycle because obstacle payload and IDs do not match");
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            solver_.resetAuditMetrics();
            writePlannerCsv("count_mismatch", "count_mismatch", "count_mismatch", "none",
                            false, false, 0.0);
            return;
        }

        if (accepted_beta_ids_ != obstacle_ids_) {
            accepted_beta_list_.clear();
            accepted_beta_ids_.clear();
        }

        // Choose goal states from global path
        chooseGoalState();
        smoothYaw(goal_state_);

        ros::Time t0 = ros::Time::now();

        // Solve MPC-SECBF
        std::string mpc_status = "success";
        std::string first_attempt_status = "not_run";
        std::string final_status = "not_run";
        std::string accepted_beta_source = "candidate";
        bool used_fallback = false;
        bool mpc_guard_used = false;
        bool success = false;
        std::vector<double> final_beta_values;

        if (!validateBetaCountLocked(beta_list_, "candidate")) {
            first_attempt_status = "beta_count_mismatch";
        } else {
            success = solver_.solve(&cur_state_, &goal_state_, &obs_matrix_, beta_list_);
            first_attempt_status = success ? "success" : "infeasible";
        }
        final_status = first_attempt_status;

        if (!success) {
            if (mpc_feasibility_guard_enabled_) {
                mpc_guard_used = true;
                const bool previous_beta_matches =
                    !accepted_beta_list_.empty() &&
                    accepted_beta_ids_ == obstacle_ids_ &&
                    validateBetaCountLocked(accepted_beta_list_, "previous");
                if (previous_beta_matches) {
                    success = solver_.solve(&cur_state_, &goal_state_, &obs_matrix_, accepted_beta_list_);
                    if (success) {
                        mpc_status = "guard_previous";
                        final_status = "success";
                        accepted_beta_source = "previous";
                        final_beta_values = accepted_beta_list_;
                    }
                }

                if (!success) {
                    std::vector<double> zero_beta(obstacle_ids_.size(), 0.0);
                    success = solver_.solve(&cur_state_, &goal_state_, &obs_matrix_, zero_beta);
                    if (success) {
                        mpc_status = "guard_zero";
                        final_status = "success";
                        accepted_beta_source = "zero";
                        accepted_beta_list_ = zero_beta;
                        accepted_beta_ids_ = obstacle_ids_;
                        final_beta_values = zero_beta;
                    }
                }
            }

            if (success && accepted_beta_source == "previous") {
                // Keep the previous accepted beta list unchanged.
            } else if (success && accepted_beta_source == "zero") {
                // accepted_beta_list_ was set above.
            }
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
                accepted_beta_list_.clear();
                accepted_beta_ids_.clear();
                ROS_WARN_THROTTLE(1.0, "[MPC-SECBF] Fallback (no CBF) succeeded");
            }
        } else if (accepted_beta_source == "candidate") {
            accepted_beta_list_ = beta_list_;
            accepted_beta_ids_ = obstacle_ids_;
            final_beta_values = beta_list_;
        }

        writeMpcMarginCsv(final_beta_values, accepted_beta_source,
                          first_attempt_status, final_status, mpc_guard_used);
        publishAcceptedMargins(final_beta_values, accepted_beta_source);
        writeTauStageCsv(accepted_beta_source);

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
        if (!obstacle_contract_valid || solver_.last_constrained_obs_index < 0 || N_ <= 0 ||
            solver_.last_constrained_obs_index * N_ >= obs_matrix_.cols()) {
            tau_result.reason = "no_constrained_obstacle";
        } else if (!dynamic_tau_enabled) {
            tau_result.reason = "disabled";
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
                         << solve_time_ms << ","
                         << dynamic_tau_enabled << ","
                         << semantic_guard::dynamicTauModeName(tau_result.mode) << ","
                         << tau_result.tau << ","
                         << tau_result.t_ca_raw << ","
                         << tau_result.t_ca_clipped << ","
                         << (tau_result.ke_scaled ? dynamic_tau_params_.ke : 1.0) << ","
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
                        << solve_time_ms << ","
                        << solve_time_ms << "\n";
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
                           << tau.valid << ","
                           << tau.lower_clipped << ","
                           << tau.upper_clipped << ","
                           << audit.beta << ","
                           << audit.h_eesm << ","
                           << audit.h_seesm << ","
                           << tau.computed << ","
                           << tau.reason << "\n";
        }
        tau_stage_csv_.flush();
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
                            << obstacle_ids_[i] << ","
                            << beta_list_[i] << ","
                            << final_beta_values[i] << ","
                            << accepted_beta_source << ","
                            << first_attempt_status << ","
                            << final_status << ","
                            << (mpc_feasibility_guard_enabled_ ? 1 : 0) << ","
                            << (candidate_checked ? 1 : 0) << ","
                            << (mpc_guard_used ? 1 : 0) << "\n";
        }
        mpc_margin_csv_.flush();
    }

    bool validateObstacleContractLocked() const {
        if (obs_matrix_.cols() == 0 && obstacle_ids_.empty()) return true;
        if (N_ <= 0 || obs_matrix_.cols() % N_ != 0) return false;
        return static_cast<size_t>(obs_matrix_.cols() / N_) == obstacle_ids_.size();
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
    ros::Subscriber sub_odom_, sub_path_, sub_beta_, sub_obs_, sub_obs_ids_;
    ros::Publisher pub_cmd_, pub_local_path_, pub_beta_applied_final_;
    ros::Timer timer_replan_, timer_cmd_;

    MPC_SECBF_SOLVE solver_;
    int N_;
    double Ts_;
    double robot_radius_ = 0.4;

    std::mutex odom_mutex_, path_mutex_, beta_mutex_, obs_mutex_;
    Eigen::VectorXd cur_state_;
    Eigen::MatrixXd global_path_;
    Eigen::MatrixXd goal_state_;
    Eigen::MatrixXd obs_matrix_;
    std::vector<uint32_t> obstacle_ids_;
    std::vector<double> beta_list_;
    std::vector<double> accepted_beta_list_;
    std::vector<uint32_t> accepted_beta_ids_;
    geometry_msgs::Twist cmd_vel_;
    bool has_odom_, has_path_;
    bool mpc_feasibility_guard_enabled_;
    bool side_preference_enabled_ = false;
    double side_weight_ = 0.2;
    bool obs_payload_valid_ = true;
    bool beta_payload_valid_ = true;
    bool dynamic_tau_enabled_ = false;
    semantic_guard::DynamicTauParams dynamic_tau_params_;
    std::ofstream planner_csv_, timing_csv_, mpc_margin_csv_, tau_stage_csv_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_secbf_node");
    ros::NodeHandle nh("~");
    MpcSecbfNode node(nh);
    ros::spin();
    return 0;
}
