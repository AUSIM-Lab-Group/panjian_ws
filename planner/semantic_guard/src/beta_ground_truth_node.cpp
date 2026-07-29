#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <set>
#include <unordered_set>

#include "semantic_guard/AppliedMarginArray.h"
#include "semantic_guard/GuardLog.h"
#include "semantic_guard/PreGuardMarginArray.h"
#include "semantic_guard/PredictedObstacleArray.h"
#include "semantic_guard/applied_margin_feedback.hpp"
#include "semantic_guard/dynamic_tau.hpp"
#include "semantic_guard/planar_velocity.hpp"
#include "semantic_guard/semantic_margin_update.hpp"

struct GroundTruthMarginAudit {
    uint32_t obstacle_id = 0;
    uint64_t birth_cycle = 0;
    std::string semantic_class;
    double beta_bar = 0.0;
    double beta_max = 0.0;
    double mu = 0.0;
    double beta_tilde = 0.0;
    double beta_previous = 0.0;
    double available_margin = 0.0;
    double positive_increment_bound = 0.0;
    double beta_upper_bound = 0.0;
    double beta_pre = 0.0;
    bool rate_limit_active = false;
    bool projection_active = false;
    bool category_bound_enforced = false;
    double d_i = 0.0;
    double rel_v_norm = 0.0;
    double ttc = std::numeric_limits<double>::infinity();
    double ttc_norm = 0.0;
    double inv_ttc = 0.0;
    double cos_delta = 0.0;
    double density_norm = 0.0;
    double h_phys = 0.0;
    double h_eesm = 0.0;
    double r_base = 0.0;
    semantic_guard::DynamicTauResult tau_result;
};

struct GroundTruthMarginCycle {
    std::vector<uint32_t> obstacle_ids;
    std::vector<GroundTruthMarginAudit> audits;
};

/**
 * Ground Truth β Publisher
 * For numerical simulation: reads obstacle semantic classes from params,
 * computes β using the same formula as beta_guard_node, but without
 * needing the full perception pipeline.
 *
 * Subscribes to /globalFsm_by_adsm/obs_predict_pub to know obstacle count and positions.
 */
class BetaGroundTruthNode {
public:
    BetaGroundTruthNode(ros::NodeHandle& nh) : nh_(nh) {
        // Load beta_bar
        nh_.param("beta_bar/pedestrian", beta_bar_["pedestrian"], 0.75);
        nh_.param("beta_bar/adult",      beta_bar_["adult"],      0.75);
        nh_.param("beta_bar/child",      beta_bar_["child"],      1.05);
        nh_.param("beta_bar/child_like", beta_bar_["child_like"], 1.05);
        nh_.param("beta_bar/cyclist",    beta_bar_["cyclist"],    0.90);
        nh_.param("beta_bar/vehicle",    beta_bar_["vehicle"],    0.80);
        nh_.param("beta_bar/box",        beta_bar_["box"],        0.20);
        nh_.param("beta_bar/unknown",    beta_bar_["unknown"],    0.75);

        // beta_max(c) is intentionally independent from B_bar(c). Teacher-v1
        // provisionally freezes equal numeric values until the missing table in
        // the manuscript is resolved.
        for (const auto& entry : beta_bar_) {
            nh_.param("beta_max/" + entry.first,
                      beta_max_[entry.first], entry.second);
        }

        // Load mu weights
        nh_.param("mu_weights/bias",    w_bias_,    0.6);
        nh_.param("mu_weights/heading", w_head_,    0.2);
        nh_.param("mu_weights/ttc",     w_ttc_,     0.15);
        nh_.param("mu_weights/density", w_density_, 0.1);

        // Guard params
        nh_.param("guard/enabled", guard_enabled_, true);
        nh_.param("guard/h_min", h_min_, 0.10);
        nh_.param("guard/delta_beta_positive", delta_beta_positive_, 0.30);
        nh_.param("dynamic_tau_enabled", dynamic_tau_enabled_, true);
        nh_.param<std::string>("dynamic_tau/mode", dynamic_tau_mode_name_, "teacher_tca");
        if (!semantic_guard::parseDynamicTauMode(dynamic_tau_mode_name_,
                                                 &dynamic_tau_params_.mode)) {
            ROS_FATAL_STREAM("[beta_ground_truth] unsupported dynamic_tau/mode='"
                             << dynamic_tau_mode_name_
                             << "'; expected legacy_gate, teacher_tca, or teacher_ke_tca");
            throw std::invalid_argument("unsupported dynamic_tau/mode");
        }
        nh_.param("dynamic_tau/delta_tau", dynamic_tau_params_.delta_tau, 1e-6);
        nh_.param("dynamic_tau/Ke", dynamic_tau_params_.ke, 0.30);
        nh_.param("dynamic_tau/Tmax", dynamic_tau_params_.t_max, 2.0);
        nh_.param("dynamic_tau/min_speed", dynamic_tau_params_.min_speed, 1e-6);
        nh_.param("dynamic_tau/min_distance", dynamic_tau_params_.min_distance, 1e-6);
        nh_.param("dynamic_tau/max_tau", dynamic_tau_params_.max_tau, 2.0);
        validateDynamicTauConfig();
        nh_.param<std::string>("semantic_mode", semantic_mode_, "full");
        nh_.param("guard/enable_rate_limit", enable_rate_limit_, true);
        nh_.param("guard/enable_available_projection", enable_available_projection_, true);
        nh_.param("guard/enable_guard_fallback", enable_guard_fallback_, true);
        nh_.param("fixed_beta", fixed_beta_, 0.4);
        nh_.param("robot/radius",         robot_radius_,   0.4);
        if (!std::isfinite(h_min_) || h_min_ < 0.0 ||
            !std::isfinite(delta_beta_positive_) ||
            delta_beta_positive_ <= 0.0) {
            throw std::invalid_argument("invalid Teacher semantic-margin parameters");
        }

        // Ground truth obstacle classes (from scenario config)
        // Format: list of class names, one per obstacle in order
        std::vector<std::string> default_classes = {"pedestrian", "pedestrian", "pedestrian",
                                                     "pedestrian", "pedestrian", "pedestrian", "pedestrian"};
        nh_.param("obstacle_classes", obstacle_classes_, default_classes);
        std::vector<int> configured_obstacle_ids;
        if (!nh_.getParam("obstacle_ids", configured_obstacle_ids)) {
            configured_obstacle_ids.reserve(obstacle_classes_.size());
            for (size_t index = 0; index < obstacle_classes_.size(); ++index) {
                configured_obstacle_ids.push_back(4000 + static_cast<int>(index));
            }
        }
        if (configured_obstacle_ids.size() != obstacle_classes_.size()) {
            throw std::invalid_argument(
                "obstacle_ids and obstacle_classes must have equal length");
        }
        std::set<uint32_t> unique_configured_ids;
        for (size_t index = 0; index < configured_obstacle_ids.size(); ++index) {
            if (configured_obstacle_ids[index] < 0) {
                throw std::invalid_argument("obstacle_ids must be nonnegative");
            }
            const uint32_t obstacle_id =
                static_cast<uint32_t>(configured_obstacle_ids[index]);
            if (!unique_configured_ids.insert(obstacle_id).second) {
                throw std::invalid_argument("obstacle_ids must be unique");
            }
            class_by_id_[obstacle_id] = obstacle_classes_[index];
        }

        // MPC params for obs_matrix parsing
        nh_.param("mpc/pre_step", N_, 20);
        nh_.param("mpc/step_time", prediction_step_sec_, 0.2);
        if (N_ <= 0 || !std::isfinite(prediction_step_sec_) ||
            prediction_step_sec_ <= 0.0) {
            throw std::invalid_argument("invalid typed obstacle horizon config");
        }

        // CSV logging
        std::string log_path;
        nh_.param<std::string>("log_path", log_path, "");
        if (!log_path.empty()) {
            csv_file_.open(log_path, std::ios::out);
            if (csv_file_.is_open()) {
                csv_file_ << std::fixed << std::setprecision(9);
                csv_file_ << "time,obstacle_cycle_id,obs_id,class,beta_bar,beta_max,mu,beta_requested,beta_previous,beta_pre_guard,beta_applied,"
                          << "available_margin,positive_increment_bound,guard_upper_bound,guard_passed,guard_status,accepted_source,"
                          << "semantic_mode,delta_beta,rate_limit_active,projection_active,"
                          << "d_i,rel_v_norm,ttc,ttc_norm,inv_ttc,cos_delta,rho_i,rho_norm,group_flag,"
                          << "h_ee,h_see,R_base,R_sem,tau,tau_mode,delta_tau,"
                          << "relative_dot,speed_squared,denominator,tca_raw,tca_clipped,"
                          << "tau_unclipped,lower_clipped,upper_clipped,ke_scaled,"
                          << "T_i,f_r,f_v,f_T,tau_valid,tau_reason,"
                          << "h_phys,h_eesm,h_seesm,tau_computed,tau_active\n";
            }
        }

        // Subscribers
        sub_snapshot_ = nh_.subscribe(
            "/globalFsm_by_adsm/teacher_obstacle_snapshot", 20,
            &BetaGroundTruthNode::snapshotCb, this);
        sub_odom_ = nh_.subscribe("/Odometry", 1, &BetaGroundTruthNode::odomCb, this);
        sub_applied_ = nh_.subscribe(
            "/safety_margin/beta_applied_final", 20,
            &BetaGroundTruthNode::appliedMarginCb, this);

        // Publishers
        pub_pre_guard_ = nh_.advertise<semantic_guard::PreGuardMarginArray>(
            "/safety_margin/beta_pre_guard", 20);
        pub_guard_log_ = nh_.advertise<semantic_guard::GuardLog>("/safety_margin/guard_log", 10);
        pub_vis_ = nh_.advertise<visualization_msgs::MarkerArray>("/safety_margin/vis_obstacles", 10);

        total_rollbacks_ = 0;
        has_odom_ = false;

        ROS_INFO("BetaGroundTruthNode initialized with %zu typed obstacle IDs, guard_enabled=%s, semantic_mode=%s, dynamic_tau=%s, tau_mode=%s, delta_tau=%.3e",
                 obstacle_classes_.size(), guard_enabled_ ? "true" : "false", semantic_mode_.c_str(),
                 dynamic_tau_enabled_ ? "true" : "false",
                 semantic_guard::dynamicTauModeName(dynamic_tau_params_.mode),
                 dynamic_tau_params_.delta_tau);
        for (size_t i = 0; i < obstacle_classes_.size(); i++) {
            ROS_INFO("  obs_id=%u class=%s beta_bar=%.2f beta_max=%.2f",
                     static_cast<unsigned int>(configured_obstacle_ids[i]),
                     obstacle_classes_[i].c_str(),
                     beta_bar_[obstacle_classes_[i]],
                     beta_max_[obstacle_classes_[i]]);
        }
    }

    ~BetaGroundTruthNode() {
        if (csv_file_.is_open()) csv_file_.close();
    }

private:
    void validateDynamicTauConfig() const {
        if (!dynamic_tau_enabled_) return;
        const auto mode = dynamic_tau_params_.mode;
        const bool legacy_valid = std::isfinite(dynamic_tau_params_.ke) &&
            std::isfinite(dynamic_tau_params_.t_max) &&
            std::isfinite(dynamic_tau_params_.min_speed) &&
            std::isfinite(dynamic_tau_params_.min_distance) &&
            std::isfinite(dynamic_tau_params_.max_tau) &&
            dynamic_tau_params_.ke >= 0.0 && dynamic_tau_params_.t_max >= 0.0 &&
            dynamic_tau_params_.min_speed >= 0.0 &&
            dynamic_tau_params_.min_distance >= 0.0 &&
            dynamic_tau_params_.max_tau > 0.0;
        const bool teacher_valid = std::isfinite(dynamic_tau_params_.delta_tau) &&
            std::isfinite(dynamic_tau_params_.max_tau) &&
            dynamic_tau_params_.delta_tau > 0.0 &&
            dynamic_tau_params_.max_tau > 0.0 &&
            (mode != semantic_guard::DynamicTauMode::kTeacherKeTca ||
             (std::isfinite(dynamic_tau_params_.ke) && dynamic_tau_params_.ke > 0.0));
        const bool valid = mode == semantic_guard::DynamicTauMode::kLegacyGate
            ? legacy_valid : teacher_valid;
        if (!valid) {
            ROS_FATAL("[beta_ground_truth] invalid dynamic tau config for mode=%s: Ke=%.9g Tmax=%.9g min_speed=%.9g min_distance=%.9g max_tau=%.9g delta_tau=%.9g",
                      semantic_guard::dynamicTauModeName(mode), dynamic_tau_params_.ke,
                      dynamic_tau_params_.t_max, dynamic_tau_params_.min_speed,
                      dynamic_tau_params_.min_distance, dynamic_tau_params_.max_tau,
                      dynamic_tau_params_.delta_tau);
            throw std::invalid_argument("invalid dynamic tau config");
        }
    }

    void odomCb(const nav_msgs::OdometryConstPtr& msg) {
        robot_pos_ << msg->pose.pose.position.x,
                      msg->pose.pose.position.y;
        // The twist belongs to the odometry child frame. Convert it to world
        // coordinates before subtracting world-frame predicted velocities.
        double world_vx = 0.0;
        double world_vy = 0.0;
        if (!semantic_guard::bodyPlanarVelocityToWorld(
                msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w,
                &world_vx, &world_vy)) {
            has_odom_ = false;
            ROS_ERROR_THROTTLE(
                1.0, "[beta_ground_truth] invalid odometry quaternion/twist");
            return;
        }
        robot_vel_ << world_vx, world_vy;
        has_odom_ = true;
    }

    bool hasUniqueIds(const std::vector<uint32_t>& ids) const {
        std::unordered_set<uint32_t> seen;
        seen.reserve(ids.size());
        for (uint32_t id : ids) {
            if (!seen.insert(id).second) return false;
        }
        return true;
    }

    void updateActiveIds(const std::vector<uint32_t>& ids,
                         uint64_t current_cycle) {
        std::unordered_set<uint32_t> next(ids.begin(), ids.end());
        for (auto it = beta_prev_.begin(); it != beta_prev_.end();) {
            if (next.count(it->first) == 0) {
                it = beta_prev_.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = active_birth_cycle_.begin();
             it != active_birth_cycle_.end();) {
            if (next.count(it->first) == 0) {
                it = active_birth_cycle_.erase(it);
            } else {
                ++it;
            }
        }
        for (uint32_t id : ids) {
            if (active_ids_.count(id) == 0) {
                active_birth_cycle_[id] = current_cycle;
            }
        }
        active_ids_.swap(next);
    }

    void snapshotCb(
        const semantic_guard::PredictedObstacleArrayConstPtr& msg) {
        if (msg->cycle_id == 0 || msg->cycle_id <= last_snapshot_cycle_id_) {
            ROS_ERROR_THROTTLE(
                1.0, "[beta_ground_truth] reject stale/zero obstacle cycle=%lu",
                static_cast<unsigned long>(msg->cycle_id));
            return;
        }
        if (N_ <= 0 || msg->horizon_steps != static_cast<uint32_t>(N_) ||
            msg->header.stamp.isZero() || msg->header.frame_id != "world" ||
            !std::isfinite(msg->prediction_step_sec) ||
            msg->prediction_step_sec <= 0.0 ||
            std::abs(msg->prediction_step_sec - prediction_step_sec_) > 1e-6 ||
            msg->state_data.size() !=
                msg->obstacle_ids.size() * static_cast<size_t>(7 * N_) ||
            !hasUniqueIds(msg->obstacle_ids) ||
            !std::all_of(msg->state_data.begin(), msg->state_data.end(),
                         [](double value) { return std::isfinite(value); })) {
            ROS_ERROR_THROTTLE(
                1.0, "[beta_ground_truth] reject malformed typed obstacle cycle=%lu",
                static_cast<unsigned long>(msg->cycle_id));
            return;
        }
        for (uint32_t obstacle_id : msg->obstacle_ids) {
            if (class_by_id_.count(obstacle_id) == 0) {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[beta_ground_truth] reject unconfigured obstacle ID=%u",
                    obstacle_id);
                return;
            }
        }
        for (size_t obstacle_index = 0;
             obstacle_index < msg->obstacle_ids.size(); ++obstacle_index) {
            for (int stage = 0; stage < N_; ++stage) {
                const size_t offset =
                    7 * (obstacle_index * static_cast<size_t>(N_) +
                         static_cast<size_t>(stage));
                if (msg->state_data[offset + 2] < 0.0 ||
                    msg->state_data[offset + 3] < 0.0) {
                    ROS_ERROR_THROTTLE(
                        1.0,
                        "[beta_ground_truth] reject negative radius in typed cycle=%lu",
                        static_cast<unsigned long>(msg->cycle_id));
                    return;
                }
            }
        }

        last_snapshot_cycle_id_ = msg->cycle_id;
        updateActiveIds(msg->obstacle_ids, msg->cycle_id);
        if (!has_odom_) return;

        semantic_guard::PreGuardMarginArray beta_msg;
        beta_msg.header = msg->header;
        beta_msg.header.stamp = ros::Time::now();
        beta_msg.obstacle_cycle_id = msg->cycle_id;
        beta_msg.enforce_category_bound = guard_enabled_;
        beta_msg.enforce_positive_increment_bound =
            guard_enabled_ && enable_rate_limit_;
        beta_msg.enforce_available_margin_bound =
            guard_enabled_ && enable_available_projection_;
        GroundTruthMarginCycle cycle;
        cycle.obstacle_ids = msg->obstacle_ids;
        cycle.audits.reserve(msg->obstacle_ids.size());

        const int obs_num = static_cast<int>(msg->obstacle_ids.size());
        for (int idx = 0; idx < obs_num; idx++) {
            const uint32_t obstacle_id = msg->obstacle_ids[idx];
            // Get obstacle position (first timestep)
            const size_t offset = static_cast<size_t>(7 * N_ * idx);
            double obs_x = msg->state_data[offset + 0];
            double obs_y = msg->state_data[offset + 1];
            double obs_r = msg->state_data[offset + 2];
            double obs_vx = msg->state_data[offset + 5];
            double obs_vy = msg->state_data[offset + 6];
            if (obs_r < 0.0) {
                ROS_ERROR_THROTTLE(
                    1.0, "[beta_ground_truth] reject negative obstacle radius");
                return;
            }

            Eigen::Vector2d obs_pos(obs_x, obs_y);
            Eigen::Vector2d obs_vel(obs_vx, obs_vy);

            // Get semantic class
            const std::string cls = class_by_id_.at(obstacle_id);
            double beta_bar_val = beta_bar_.count(cls) ? beta_bar_[cls] : beta_bar_["unknown"];
            double beta_max_val = beta_max_.count(cls) ? beta_max_[cls] : beta_max_["unknown"];

            // Compute context features
            // Teacher convention: l=p_robot-p_obstacle and
            // v_rel=v_robot-v_obstacle.
            Eigen::Vector2d p_rel = robot_pos_ - obs_pos;
            Eigen::Vector2d v_rel = robot_vel_ - obs_vel;

            double f_head = 0.0;
            double d_i = p_rel.norm();
            double rel_v_norm = v_rel.norm();
            double cos_delta = 0.0;
            if (p_rel.norm() > 0.01 && v_rel.norm() > 0.01) {
                cos_delta = p_rel.normalized().dot(v_rel.normalized());
                f_head = std::max(0.0, -cos_delta);
            }

            double closing_speed = 0.0;
            if (p_rel.norm() > 0.01) {
                closing_speed = std::max(-p_rel.normalized().dot(v_rel), 0.0);
            }
            double ttc = (closing_speed > 1e-6) ? (d_i / closing_speed)
                                                : std::numeric_limits<double>::infinity();
            double inv_ttc = std::isfinite(ttc) && ttc > 1e-6 ? 1.0 / ttc : 0.0;
            double ttc_norm = 0.0;
            if (closing_speed > 0.01) {
                ttc_norm = std::max(0.0, std::min(1.0, 1.0 - ttc / 5.0));
            }

            double density_norm = std::min(1.0, (obs_num - 1) / 5.0);

            semantic_guard::SemanticContextWeights context_weights;
            context_weights.bias = w_bias_;
            context_weights.head_on = w_head_;
            context_weights.ttc_norm = w_ttc_;
            context_weights.density_norm = w_density_;
            const auto phi = semantic_guard::computeProvisionalTeacherPhi(
                beta_bar_val, f_head, ttc_norm, density_norm,
                context_weights);
            if (!phi.valid) {
                ROS_ERROR_THROTTLE(
                    1.0, "[beta_ground_truth] provisional Phi is invalid: %s",
                    phi.reason.c_str());
                return;
            }
            const double mu = phi.mu;
            const double beta_hat = computeRequestedBeta(
                beta_bar_val, mu, phi.beta_tilde);

            // Guard check: β̂ ≤ h_EE - η
            // h_EE = ||p_rel + τ v_rel|| - R_obs - R_robot
            const double inflated_radius = obs_r + robot_radius_;
            semantic_guard::DynamicTauResult tau_result;
            if (dynamic_tau_enabled_) {
                tau_result = semantic_guard::computeDynamicTau(
                    p_rel.x(), p_rel.y(), v_rel.x(), v_rel.y(),
                    inflated_radius, dynamic_tau_params_);
            } else {
                tau_result.mode = dynamic_tau_params_.mode;
                tau_result.tau = 0.0;
                tau_result.computed = true;
                tau_result.valid = false;
                tau_result.reason = "disabled";
            }
            Eigen::Vector2d lookahead_rel_pos = p_rel + tau_result.tau * v_rel;
            double h_phys = p_rel.norm() - obs_r - robot_radius_;
            double h_ee = lookahead_rel_pos.norm() - obs_r - robot_radius_;
            double beta_prev = getPrevBeta(obstacle_id);
            semantic_guard::SemanticProjectionPolicy policy;
            policy.enforce_category_bound = guard_enabled_;
            policy.enforce_positive_increment_bound =
                guard_enabled_ && enable_rate_limit_;
            policy.enforce_available_margin_bound =
                guard_enabled_ && enable_available_projection_;
            const auto update = semantic_guard::projectTeacherSemanticMargin(
                beta_hat, beta_max_val, beta_prev, delta_beta_positive_, h_ee,
                h_min_, policy);
            if (!update.valid) {
                ROS_ERROR_THROTTLE(
                    1.0, "[beta_ground_truth] semantic update is invalid: %s",
                    update.reason.c_str());
                return;
            }

            beta_msg.obstacle_ids.push_back(obstacle_id);
            beta_msg.semantic_classes.push_back(cls);
            beta_msg.beta_bar.push_back(beta_bar_val);
            beta_msg.beta_max.push_back(beta_max_val);
            beta_msg.beta_previous.push_back(beta_prev);
            beta_msg.mu.push_back(mu);
            beta_msg.beta_tilde.push_back(beta_hat);
            beta_msg.available_margin.push_back(update.available_margin);
            beta_msg.positive_increment_bound.push_back(
                update.positive_increment_bound);
            beta_msg.beta_upper_bound.push_back(update.beta_upper_bound);
            beta_msg.beta_pre_guard.push_back(update.beta_pre);

            GroundTruthMarginAudit audit;
            audit.obstacle_id = obstacle_id;
            audit.birth_cycle = active_birth_cycle_.at(obstacle_id);
            audit.semantic_class = cls;
            audit.beta_bar = beta_bar_val;
            audit.beta_max = beta_max_val;
            audit.mu = mu;
            audit.beta_tilde = beta_hat;
            audit.beta_previous = beta_prev;
            audit.available_margin = update.available_margin;
            audit.positive_increment_bound = update.positive_increment_bound;
            audit.beta_upper_bound = update.beta_upper_bound;
            audit.beta_pre = update.beta_pre;
            audit.rate_limit_active = update.positive_increment_bound_active;
            audit.projection_active =
                std::abs(update.beta_pre - beta_hat) > 1e-9;
            audit.category_bound_enforced = policy.enforce_category_bound;
            audit.d_i = d_i;
            audit.rel_v_norm = rel_v_norm;
            audit.ttc = ttc;
            audit.ttc_norm = ttc_norm;
            audit.inv_ttc = inv_ttc;
            audit.cos_delta = cos_delta;
            audit.density_norm = density_norm;
            audit.h_phys = h_phys;
            audit.h_eesm = h_ee;
            audit.r_base = obs_r + robot_radius_;
            audit.tau_result = tau_result;
            cycle.audits.push_back(audit);
        }

        pending_cycles_[msg->cycle_id] = cycle;
        while (pending_cycles_.size() > 64) {
            pending_cycles_.erase(pending_cycles_.begin());
        }
        pub_pre_guard_.publish(beta_msg);

        // Publish visualization markers (obstacle spheres + β safety circles)
        publishVisualization(msg);
    }

    void appliedMarginCb(
        const semantic_guard::AppliedMarginArrayConstPtr& msg) {
        if (msg->obstacle_cycle_id == 0 ||
            msg->obstacle_cycle_id <= last_feedback_cycle_id_) {
            ROS_ERROR_THROTTLE(
                1.0, "[beta_ground_truth] reject stale/zero MPC feedback cycle=%lu",
                static_cast<unsigned long>(msg->obstacle_cycle_id));
            return;
        }
        auto cycle_it = pending_cycles_.find(msg->obstacle_cycle_id);
        if (cycle_it == pending_cycles_.end()) {
            ROS_ERROR_THROTTLE(
                1.0, "[beta_ground_truth] reject feedback for unknown cycle=%lu",
                static_cast<unsigned long>(msg->obstacle_cycle_id));
            return;
        }
        const GroundTruthMarginCycle& cycle = cycle_it->second;
        const size_t count = cycle.obstacle_ids.size();
        if (msg->obstacle_ids != cycle.obstacle_ids ||
            cycle.audits.size() != count ||
            msg->beta_applied.size() != count ||
            msg->accepted_sources.size() != count ||
            !hasUniqueIds(msg->obstacle_ids)) {
            ROS_ERROR_THROTTLE(
                1.0, "[beta_ground_truth] reject feedback ID/count mismatch");
            return;
        }
        for (size_t index = 0; index < count; ++index) {
            const GroundTruthMarginAudit& audit = cycle.audits[index];
            const std::string& source = msg->accepted_sources[index];
            const double applied = msg->beta_applied[index];
            if (!semantic_guard::validateAppliedMarginFeedback(
                    source, applied, audit.beta_max, audit.beta_pre,
                    audit.beta_previous, audit.category_bound_enforced)) {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[beta_ground_truth] reject invalid feedback payload/source");
                return;
            }
        }

        semantic_guard::GuardLog log_msg;
        log_msg.header = msg->header;
        log_msg.obstacle_cycle_id = msg->obstacle_cycle_id;
        for (size_t index = 0; index < count; ++index) {
            const GroundTruthMarginAudit& audit = cycle.audits[index];
            const double beta_applied = msg->beta_applied[index];
            const std::string& accepted_source = msg->accepted_sources[index];
            const bool candidate_accepted =
                accepted_source == "candidate" &&
                std::abs(beta_applied - audit.beta_pre) <= 1e-8;
            const auto birth_it = active_birth_cycle_.find(audit.obstacle_id);
            const bool same_lifecycle =
                active_ids_.count(audit.obstacle_id) != 0 &&
                birth_it != active_birth_cycle_.end() &&
                birth_it->second == audit.birth_cycle;
            if (accepted_source != "no_cbf" &&
                accepted_source != "safe_stop" &&
                accepted_source != "emergency_cbf" && same_lifecycle) {
                beta_prev_[audit.obstacle_id] = beta_applied;
            }
            if (!candidate_accepted) ++total_rollbacks_;

            log_msg.obstacle_ids.push_back(audit.obstacle_id);
            log_msg.beta_requested.push_back(audit.beta_tilde);
            log_msg.beta_pre_guard.push_back(audit.beta_pre);
            log_msg.beta_applied.push_back(beta_applied);
            log_msg.h_ee_values.push_back(audit.h_eesm);
            log_msg.guard_passed.push_back(candidate_accepted);
            log_msg.accepted_sources.push_back(accepted_source);

            const double delta_beta =
                std::max(0.0, beta_applied - audit.beta_previous);
            const double h_seesm = audit.h_eesm - beta_applied;
            const double r_sem = audit.r_base + beta_applied;
            std::string guard_status;
            if (!candidate_accepted) {
                guard_status = "mpc_" + accepted_source;
            } else if (audit.projection_active) {
                guard_status = "pre_guard_clip";
            } else {
                guard_status = "accept";
            }
            const bool tau_computed = !dynamic_tau_enabled_ ||
                                      audit.tau_result.computed;
            const bool tau_active =
                std::isfinite(audit.tau_result.tau) &&
                audit.tau_result.tau > 0.0;
            if (csv_file_.is_open()) {
                csv_file_ << ros::Time::now().toSec() << ","
                          << msg->obstacle_cycle_id << ","
                          << audit.obstacle_id << ","
                          << audit.semantic_class << ","
                          << audit.beta_bar << "," << audit.beta_max << ","
                          << audit.mu << "," << audit.beta_tilde << ","
                          << audit.beta_previous << "," << audit.beta_pre << ","
                          << beta_applied << "," << audit.available_margin << ","
                          << audit.positive_increment_bound << ","
                          << audit.beta_upper_bound << ","
                          << (candidate_accepted ? 1 : 0) << ","
                          << guard_status << ","
                          << sanitizeCsvField(accepted_source) << ","
                          << semantic_mode_ << "," << delta_beta << ","
                          << (audit.rate_limit_active ? 1 : 0) << ","
                          << (audit.projection_active ? 1 : 0) << ","
                          << audit.d_i << "," << audit.rel_v_norm << ","
                          << (std::isfinite(audit.ttc) ? audit.ttc : -1.0) << ","
                          << audit.ttc_norm << "," << audit.inv_ttc << ","
                          << audit.cos_delta << "," << audit.density_norm << ","
                          << audit.density_norm << ",0,"
                          << audit.h_eesm << "," << h_seesm << ","
                          << audit.r_base << "," << r_sem << ","
                          << audit.tau_result.tau << ","
                          << semantic_guard::dynamicTauModeName(
                                 dynamic_tau_params_.mode) << ","
                          << dynamic_tau_params_.delta_tau << ","
                          << audit.tau_result.relative_dot << ","
                          << audit.tau_result.speed_squared << ","
                          << audit.tau_result.denominator << ","
                          << audit.tau_result.t_ca_raw << ","
                          << audit.tau_result.t_ca_clipped << ","
                          << audit.tau_result.tau_unclipped << ","
                          << audit.tau_result.lower_clipped << ","
                          << audit.tau_result.upper_clipped << ","
                          << audit.tau_result.ke_scaled << ","
                          << audit.tau_result.T_i << ","
                          << audit.tau_result.f_r << ","
                          << audit.tau_result.f_v << ","
                          << audit.tau_result.f_T << "," << tau_computed << ","
                          << sanitizeCsvField(audit.tau_result.reason) << ","
                          << audit.h_phys << "," << audit.h_eesm << ","
                          << h_seesm << "," << tau_computed << ","
                          << tau_active << "\n";
            }
        }
        if (csv_file_.is_open()) csv_file_.flush();
        log_msg.total_rollbacks = total_rollbacks_;
        pub_guard_log_.publish(log_msg);
        last_feedback_cycle_id_ = msg->obstacle_cycle_id;
        pending_cycles_.erase(pending_cycles_.begin(),
                              pending_cycles_.upper_bound(
                                  msg->obstacle_cycle_id));
    }

    void publishVisualization(
        const semantic_guard::PredictedObstacleArrayConstPtr& msg) {
        visualization_msgs::MarkerArray markers;
        const int obs_num = static_cast<int>(msg->obstacle_ids.size());

        // Color map per class: R, G, B
        auto getColor = [](const std::string& cls) -> std::tuple<float,float,float> {
            if (cls == "pedestrian") return {0.0, 1.0, 0.0};       // 绿色
            if (cls == "adult")      return {0.0, 1.0, 0.0};       // 绿色
            if (cls == "child")      return {1.0, 0.5, 0.0};       // 橙色
            if (cls == "child_like") return {1.0, 0.5, 0.0};       // 橙色
            if (cls == "cyclist")    return {1.0, 1.0, 0.0};       // 黄色
            if (cls == "vehicle")    return {1.0, 0.0, 0.0};       // 红色
            if (cls == "box")        return {0.5, 0.5, 0.5};       // 灰色
            return {0.8, 0.8, 0.8};                                 // unknown 浅灰
        };

        for (int idx = 0; idx < obs_num; idx++) {
            const size_t offset = static_cast<size_t>(7 * N_ * idx);
            double obs_x = msg->state_data[offset + 0];
            double obs_y = msg->state_data[offset + 1];
            double obs_r = msg->state_data[offset + 2];
            uint32_t obstacle_id = msg->obstacle_ids[idx];
            const std::string cls = class_by_id_.count(obstacle_id)
                ? class_by_id_.at(obstacle_id) : "unknown";
            double beta_i = getPrevBeta(obstacle_id);

            auto color = getColor(cls);
            float r = std::get<0>(color);
            float g = std::get<1>(color);
            float b = std::get<2>(color);

            // Marker 1: 障碍物实体 (圆柱)
            visualization_msgs::Marker obs_marker;
            obs_marker.header.frame_id = "world";
            obs_marker.header.stamp = ros::Time::now();
            obs_marker.ns = "obstacles";
            obs_marker.id = idx * 3;
            obs_marker.type = visualization_msgs::Marker::CYLINDER;
            obs_marker.action = visualization_msgs::Marker::ADD;
            obs_marker.pose.position.x = obs_x;
            obs_marker.pose.position.y = obs_y;
            obs_marker.pose.position.z = 0.5;
            obs_marker.pose.orientation.w = 1.0;
            obs_marker.scale.x = obs_r * 2.0;
            obs_marker.scale.y = obs_r * 2.0;
            obs_marker.scale.z = 1.0;
            obs_marker.color.r = r;
            obs_marker.color.g = g;
            obs_marker.color.b = b;
            obs_marker.color.a = 0.8;
            obs_marker.lifetime = ros::Duration(0.2);
            markers.markers.push_back(obs_marker);

            // Marker 2: β 安全圈 (半透明圆环)
            visualization_msgs::Marker beta_circle;
            beta_circle.header.frame_id = "world";
            beta_circle.header.stamp = ros::Time::now();
            beta_circle.ns = "beta_circles";
            beta_circle.id = idx * 3 + 1;
            beta_circle.type = visualization_msgs::Marker::CYLINDER;
            beta_circle.action = visualization_msgs::Marker::ADD;
            beta_circle.pose.position.x = obs_x;
            beta_circle.pose.position.y = obs_y;
            beta_circle.pose.position.z = 0.02;
            beta_circle.pose.orientation.w = 1.0;
            double total_safe = obs_r + robot_radius_ + beta_i;
            beta_circle.scale.x = total_safe * 2.0;
            beta_circle.scale.y = total_safe * 2.0;
            beta_circle.scale.z = 0.02;
            beta_circle.color.r = r;
            beta_circle.color.g = g;
            beta_circle.color.b = b;
            beta_circle.color.a = 0.2;
            beta_circle.lifetime = ros::Duration(0.2);
            markers.markers.push_back(beta_circle);

            // Marker 3: 类别文字标签
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = "world";
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "labels";
            text_marker.id = idx * 3 + 2;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position.x = obs_x;
            text_marker.pose.position.y = obs_y;
            text_marker.pose.position.z = 1.3;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.3;
            char buf[64];
            snprintf(buf, sizeof(buf), "%s\nb=%.2f", cls.c_str(), beta_i);
            text_marker.text = buf;
            text_marker.color.r = r;
            text_marker.color.g = g;
            text_marker.color.b = b;
            text_marker.color.a = 1.0;
            text_marker.lifetime = ros::Duration(0.2);
            markers.markers.push_back(text_marker);
        }

        pub_vis_.publish(markers);
    }

    double getPrevBeta(uint32_t id) {
        if (beta_prev_.count(id)) return beta_prev_[id];
        return 0.0;
    }

    double computeRequestedBeta(double beta_bar_val, double mu,
                                double full_candidate) const {
        if (semantic_mode_ == "none") return 0.0;
        if (semantic_mode_ == "fixed") return fixed_beta_;
        if (semantic_mode_ == "category_only") return beta_bar_val;
        if (semantic_mode_ == "context_only") {
            auto it = beta_bar_.find("unknown");
            double context_beta_bar = (it != beta_bar_.end()) ? it->second : 0.75;
            return context_beta_bar * mu;
        }
        return full_candidate;
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_snapshot_, sub_odom_, sub_applied_;
    ros::Publisher pub_pre_guard_, pub_guard_log_, pub_vis_;

    std::map<std::string, double> beta_bar_;
    std::map<std::string, double> beta_max_;
    double w_bias_, w_head_, w_ttc_, w_density_;
    double h_min_, delta_beta_positive_, robot_radius_, fixed_beta_;
    double prediction_step_sec_ = 0.2;
    bool guard_enabled_, enable_rate_limit_, enable_available_projection_, enable_guard_fallback_;
    bool dynamic_tau_enabled_ = false;
    semantic_guard::DynamicTauParams dynamic_tau_params_;
    std::string dynamic_tau_mode_name_ = "teacher_tca";
    std::string semantic_mode_;
    int N_;

    std::vector<std::string> obstacle_classes_;
    std::map<uint32_t, std::string> class_by_id_;
    std::unordered_set<uint32_t> active_ids_;
    std::map<uint32_t, uint64_t> active_birth_cycle_;
    Eigen::Vector2d robot_pos_, robot_vel_;
    bool has_odom_;
    std::map<uint32_t, double> beta_prev_;
    std::map<uint64_t, GroundTruthMarginCycle> pending_cycles_;
    uint64_t last_snapshot_cycle_id_ = 0;
    uint64_t last_feedback_cycle_id_ = 0;
    uint32_t total_rollbacks_;
    std::ofstream csv_file_;

    static std::string sanitizeCsvField(const std::string& field) {
        std::string sanitized = field;
        std::replace(sanitized.begin(), sanitized.end(), ',', ';');
        std::replace(sanitized.begin(), sanitized.end(), '\n', ' ');
        std::replace(sanitized.begin(), sanitized.end(), '\r', ' ');
        return sanitized;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "beta_ground_truth_node");
    ros::NodeHandle nh("~");
    BetaGroundTruthNode node(nh);
    ros::spin();
    return 0;
}
