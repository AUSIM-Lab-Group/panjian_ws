#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
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

#include "semantic_guard/GuardLog.h"
#include "semantic_guard/dynamic_tau.hpp"

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
        nh_.param("beta_bar/pedestrian", beta_bar_["pedestrian"], 0.4);
        nh_.param("beta_bar/adult",      beta_bar_["adult"],      0.4);
        nh_.param("beta_bar/child",      beta_bar_["child"],      0.7);
        nh_.param("beta_bar/child_like", beta_bar_["child_like"], 0.7);
        nh_.param("beta_bar/cyclist",    beta_bar_["cyclist"],    0.6);
        nh_.param("beta_bar/vehicle",    beta_bar_["vehicle"],    0.5);
        nh_.param("beta_bar/box",        beta_bar_["box"],        0.1);
        nh_.param("beta_bar/unknown",    beta_bar_["unknown"],    0.4);

        // Load mu weights
        nh_.param("mu_weights/bias",    w_bias_,    0.6);
        nh_.param("mu_weights/heading", w_head_,    0.2);
        nh_.param("mu_weights/ttc",     w_ttc_,     0.15);
        nh_.param("mu_weights/density", w_density_, 0.1);

        // Guard params
        nh_.param("guard/enabled",        guard_enabled_,   true);
        nh_.param("guard/eta",            eta_,            0.1);
        nh_.param("guard/max_delta_beta", max_delta_beta_, 0.3);
        nh_.param("dynamic_tau_enabled", dynamic_tau_enabled_, false);
        nh_.param("dynamic_tau/Ke", dynamic_tau_params_.ke, 0.30);
        nh_.param("dynamic_tau/Tmax", dynamic_tau_params_.t_max, 2.0);
        nh_.param("dynamic_tau/min_speed", dynamic_tau_params_.min_speed, 1e-6);
        nh_.param("dynamic_tau/min_distance", dynamic_tau_params_.min_distance, 1e-6);
        nh_.param("dynamic_tau/max_tau", dynamic_tau_params_.max_tau, 2.0);
        nh_.param("guard/tau",            tau_,            0.20);
        nh_.param<std::string>("semantic_mode", semantic_mode_, "full");
        nh_.param("guard/enable_rate_limit", enable_rate_limit_, true);
        nh_.param("guard/enable_available_projection", enable_available_projection_, true);
        nh_.param("guard/enable_guard_fallback", enable_guard_fallback_, true);
        nh_.param("fixed_beta", fixed_beta_, 0.4);
        nh_.param("robot/radius",         robot_radius_,   0.4);

        // Ground truth obstacle classes (from scenario config)
        // Format: list of class names, one per obstacle in order
        std::vector<std::string> default_classes = {"pedestrian", "pedestrian", "pedestrian",
                                                     "pedestrian", "pedestrian", "pedestrian", "pedestrian"};
        nh_.param("obstacle_classes", obstacle_classes_, default_classes);

        // MPC params for obs_matrix parsing
        nh_.param("mpc/pre_step", N_, 20);

        // CSV logging
        std::string log_path;
        nh_.param<std::string>("log_path", log_path, "");
        if (!log_path.empty()) {
            csv_file_.open(log_path, std::ios::out);
            if (csv_file_.is_open()) {
                csv_file_ << std::fixed << std::setprecision(9);
                csv_file_ << "time,obs_id,class,beta_bar,mu,beta_requested,beta_applied,"
                          << "guard_upper_bound,guard_passed,guard_status,"
                          << "semantic_mode,delta_beta,rate_limit_active,projection_active,"
                          << "d_i,rel_v_norm,ttc,ttc_norm,inv_ttc,cos_delta,rho_i,rho_norm,group_flag,"
                          << "h_ee,h_see,R_base,R_sem,tau,T_i,f_r,f_v,f_T,tau_valid,tau_reason\n";
            }
        }

        // Subscribers
        sub_obs_ = nh_.subscribe("/globalFsm_by_adsm/obs_predict_pub", 10, &BetaGroundTruthNode::obsCb, this);
        sub_obs_ids_ = nh_.subscribe("/globalFsm_by_adsm/obs_predict_ids", 10, &BetaGroundTruthNode::obsIdsCb, this);
        sub_odom_ = nh_.subscribe("/Odometry", 1, &BetaGroundTruthNode::odomCb, this);

        // Publishers
        pub_beta_ = nh_.advertise<std_msgs::Float32MultiArray>("/safety_margin/beta", 10);
        pub_guard_log_ = nh_.advertise<semantic_guard::GuardLog>("/safety_margin/guard_log", 10);
        pub_vis_ = nh_.advertise<visualization_msgs::MarkerArray>("/safety_margin/vis_obstacles", 10);

        total_rollbacks_ = 0;
        has_odom_ = false;

        ROS_INFO("BetaGroundTruthNode initialized with %zu obstacle classes, guard_enabled=%s, semantic_mode=%s",
                 obstacle_classes_.size(), guard_enabled_ ? "true" : "false", semantic_mode_.c_str());
        for (size_t i = 0; i < obstacle_classes_.size(); i++) {
            ROS_INFO("  obs[%zu] = %s, beta_bar = %.2f", i, obstacle_classes_[i].c_str(),
                     beta_bar_[obstacle_classes_[i]]);
        }
    }

    ~BetaGroundTruthNode() {
        if (csv_file_.is_open()) csv_file_.close();
    }

private:
    void obsIdsCb(const std_msgs::UInt32MultiArrayConstPtr& msg) {
        obstacle_ids_ = msg->data;
    }

    void odomCb(const nav_msgs::OdometryConstPtr& msg) {
        robot_pos_ << msg->pose.pose.position.x,
                      msg->pose.pose.position.y;
        robot_vel_ << msg->twist.twist.linear.x,
                      msg->twist.twist.linear.y;
        has_odom_ = true;
    }

    void obsCb(const std_msgs::Float32MultiArrayConstPtr& msg) {
        if (!has_odom_) return;

        // Parse obstacle count from message
        int total_floats = msg->data.size();
        if (N_ <= 0 || total_floats % (7 * N_) != 0) {
            ROS_ERROR_THROTTLE(1.0, "[beta_ground_truth] Invalid obstacle payload size=%d for N=%d",
                               total_floats, N_);
            return;
        }
        int obs_num = total_floats / (7 * N_);
        if (obs_num == 0) return;
        if (obstacle_ids_.size() != static_cast<size_t>(obs_num)) {
            ROS_ERROR_THROTTLE(1.0, "[beta_ground_truth] obs/id count mismatch: obs=%d ids=%zu",
                               obs_num, obstacle_ids_.size());
            return;
        }

        std_msgs::Float32MultiArray beta_msg;
        semantic_guard::GuardLog log_msg;
        log_msg.header.stamp = ros::Time::now();

        for (int idx = 0; idx < obs_num; idx++) {
            const uint32_t obstacle_id = obstacle_ids_[idx];
            // Get obstacle position (first timestep)
            double obs_x = msg->data[7 * N_ * idx + 0];
            double obs_y = msg->data[7 * N_ * idx + 1];
            double obs_r = msg->data[7 * N_ * idx + 2];
            double obs_vx = msg->data[7 * N_ * idx + 5];
            double obs_vy = msg->data[7 * N_ * idx + 6];

            Eigen::Vector2d obs_pos(obs_x, obs_y);
            Eigen::Vector2d obs_vel(obs_vx, obs_vy);

            // Get semantic class
            std::string cls = (idx < (int)obstacle_classes_.size()) ?
                              obstacle_classes_[idx] : "unknown";
            double beta_bar_val = beta_bar_.count(cls) ? beta_bar_[cls] : beta_bar_["unknown"];

            // Compute context features
            Eigen::Vector2d p_rel = obs_pos - robot_pos_;
            Eigen::Vector2d v_rel = obs_vel - robot_vel_;

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

            // Compute μ and β̂
            double mu = std::max(0.0, std::min(1.0,
                w_bias_ + w_head_ * f_head + w_ttc_ * ttc_norm + w_density_ * density_norm));
            double beta_hat = computeRequestedBeta(beta_bar_val, mu);

            // Guard check: β̂ ≤ h_EE - η
            // h_EE = ||p_rel + τ v_rel|| - R_obs - R_robot
            const double inflated_radius = obs_r + robot_radius_;
            semantic_guard::DynamicTauResult tau_result;
            if (dynamic_tau_enabled_) {
                tau_result = semantic_guard::computeDynamicTau(
                    p_rel.x(), p_rel.y(), v_rel.x(), v_rel.y(),
                    inflated_radius, dynamic_tau_params_);
            } else {
                tau_result.tau = tau_;
                tau_result.valid = true;
                tau_result.reason = "fixed_config";
            }
            Eigen::Vector2d lookahead_rel_pos = p_rel + tau_result.tau * v_rel;
            double h_ee = lookahead_rel_pos.norm() - obs_r - robot_radius_;
            double guard_upper_bound = std::min(beta_bar_val, std::max(0.0, h_ee - eta_));
            bool guard_pass = guard_enabled_
                ? (!enable_available_projection_ || !enable_guard_fallback_ || beta_hat <= guard_upper_bound)
                : true;

            // Apply rate limiting, projection, and last-value fallback.
            double beta_prev = getPrevBeta(obstacle_id);
            double beta_before_projection = beta_hat;
            double beta_final;
            std::string guard_status;
            bool rate_limit_active = false;
            bool projection_active = false;
            if (!guard_enabled_) {
                beta_final = beta_hat;
                guard_status = "disabled";
            } else if (enable_available_projection_ && enable_guard_fallback_ && guard_upper_bound <= 1e-9) {
                beta_final = 0.0;
                total_rollbacks_++;
                guard_status = "zero";
            } else if (guard_pass) {
                double raw_delta = beta_hat - beta_prev;
                double delta = enable_rate_limit_
                    ? std::max(-max_delta_beta_, std::min(max_delta_beta_, raw_delta))
                    : raw_delta;
                rate_limit_active = std::abs(delta - raw_delta) > 1e-9;
                double beta_limited = beta_prev + delta;
                beta_before_projection = clampSemanticBeta(beta_limited, beta_bar_val);
                beta_final = projectAvailable(beta_before_projection, guard_upper_bound);
                projection_active = std::abs(beta_final - beta_before_projection) > 1e-9;
                guard_status = (std::abs(beta_final - beta_hat) > 1e-9 ||
                                std::abs(delta - raw_delta) > 1e-9) ? "project" : "accept";
            } else {
                beta_before_projection = clampSemanticBeta(beta_prev, beta_bar_val);
                beta_final = projectAvailable(beta_before_projection, guard_upper_bound);
                projection_active = std::abs(beta_final - beta_before_projection) > 1e-9;
                total_rollbacks_++;
                guard_status = enable_guard_fallback_ && beta_final > 1e-9 ? "fallback" : "zero";
            }
            beta_final = std::max(0.0, beta_final);
            double delta_beta = std::max(0.0, beta_final - beta_prev);
            double h_see = h_ee - beta_final;
            double r_base = obs_r + robot_radius_;
            double r_sem = r_base + beta_final;
            beta_prev_[obstacle_id] = beta_final;

            beta_msg.data.push_back(static_cast<float>(beta_final));

            // Log
            log_msg.obstacle_ids.push_back(obstacle_id);
            log_msg.beta_requested.push_back(beta_hat);
            log_msg.beta_applied.push_back(beta_final);
            log_msg.h_ee_values.push_back(h_ee);
            log_msg.guard_passed.push_back(guard_pass);

            if (csv_file_.is_open()) {
                csv_file_ << ros::Time::now().toSec() << ","
                          << obstacle_id << "," << cls << ","
                          << beta_bar_val << "," << mu << ","
                          << beta_hat << "," << beta_final << ","
                          << guard_upper_bound << "," << (guard_pass ? 1 : 0) << ","
                          << guard_status << ","
                          << semantic_mode_ << "," << delta_beta << ","
                          << (rate_limit_active ? 1 : 0) << ","
                          << (projection_active ? 1 : 0) << ","
                          << d_i << "," << rel_v_norm << ","
                          << (std::isfinite(ttc) ? ttc : -1.0) << "," << ttc_norm << "," << inv_ttc << ","
                          << cos_delta << "," << density_norm << "," << density_norm << ",0,"
                          << h_ee << "," << h_see << ","
                          << r_base << "," << r_sem << ","
                          << tau_result.tau << "," << tau_result.T_i << ","
                          << tau_result.f_r << "," << tau_result.f_v << ","
                          << tau_result.f_T << "," << tau_result.valid << ","
                          << sanitizeCsvField(tau_result.reason) << "\n";
            }
        }

        log_msg.total_rollbacks = total_rollbacks_;
        pub_beta_.publish(beta_msg);
        pub_guard_log_.publish(log_msg);

        // Publish visualization markers (obstacle spheres + β safety circles)
        publishVisualization(msg);
    }

    void publishVisualization(const std_msgs::Float32MultiArrayConstPtr& msg) {
        visualization_msgs::MarkerArray markers;
        int total_floats = msg->data.size();
        int obs_num = (N_ > 0) ? (total_floats / (7 * N_)) : 0;
        if (obstacle_ids_.size() != static_cast<size_t>(obs_num)) return;

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
            double obs_x = msg->data[7 * N_ * idx + 0];
            double obs_y = msg->data[7 * N_ * idx + 1];
            double obs_r = msg->data[7 * N_ * idx + 2];
            uint32_t obstacle_id = obstacle_ids_[idx];
            std::string cls = (idx < (int)obstacle_classes_.size()) ?
                              obstacle_classes_[idx] : "unknown";
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

    double computeRequestedBeta(double beta_bar_val, double mu) const {
        if (semantic_mode_ == "none") return 0.0;
        if (semantic_mode_ == "fixed") return fixed_beta_;
        if (semantic_mode_ == "category_only") return beta_bar_val;
        if (semantic_mode_ == "context_only") {
            auto it = beta_bar_.find("unknown");
            double context_beta_bar = (it != beta_bar_.end()) ? it->second : 0.4;
            return context_beta_bar * mu;
        }
        return beta_bar_val * mu;
    }

    double clampSemanticBeta(double beta, double beta_bar_val) const {
        double upper = semantic_mode_ == "fixed" ? std::max(fixed_beta_, beta_bar_val) : beta_bar_val;
        if (semantic_mode_ == "none") upper = 0.0;
        return std::min(std::max(0.0, beta), upper);
    }

    double projectAvailable(double beta, double guard_upper_bound) const {
        if (!enable_available_projection_) return beta;
        return std::min(std::max(0.0, beta), guard_upper_bound);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_obs_, sub_obs_ids_, sub_odom_;
    ros::Publisher pub_beta_, pub_guard_log_, pub_vis_;

    std::map<std::string, double> beta_bar_;
    double w_bias_, w_head_, w_ttc_, w_density_;
    double eta_, max_delta_beta_, tau_, robot_radius_, fixed_beta_;
    bool guard_enabled_, enable_rate_limit_, enable_available_projection_, enable_guard_fallback_;
    bool dynamic_tau_enabled_ = false;
    semantic_guard::DynamicTauParams dynamic_tau_params_;
    std::string semantic_mode_;
    int N_;

    std::vector<std::string> obstacle_classes_;
    std::vector<uint32_t> obstacle_ids_;
    Eigen::Vector2d robot_pos_, robot_vel_;
    bool has_odom_;
    std::map<uint32_t, double> beta_prev_;
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
