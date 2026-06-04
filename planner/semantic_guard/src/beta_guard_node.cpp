#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <map>
#include <string>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>

#include "semantic_fusion/SemanticObstacle.h"
#include "semantic_fusion/SemanticObstacleArray.h"
#include "semantic_guard/GuardLog.h"

class BetaGuardNode {
public:
    BetaGuardNode(ros::NodeHandle& nh) : nh_(nh) {
        // Load beta_bar from params
        nh_.param("beta_bar/pedestrian", beta_bar_["pedestrian"], 0.4);
        nh_.param("beta_bar/child",      beta_bar_["child"],      0.7);
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
        nh_.param("guard/tau",            tau_,            0.2);

        // Robot params
        nh_.param("robot/radius", robot_radius_, 0.4);

        // CSV logging
        std::string log_path;
        nh_.param<std::string>("log_path", log_path, "");
        if (!log_path.empty()) {
            csv_file_.open(log_path, std::ios::out);
            if (csv_file_.is_open()) {
                csv_file_ << std::fixed << std::setprecision(9);
                csv_file_ << "time,obs_id,class,beta_bar,mu,beta_requested,beta_applied,"
                          << "guard_upper_bound,guard_passed,guard_status,"
                          << "d_i,rel_v_norm,ttc,inv_ttc,cos_delta,rho_i,group_flag,"
                          << "h_ee,h_see,R_base,R_sem\n";
                ROS_INFO("Guard log writing to: %s", log_path.c_str());
            }
        }

        // Subscribers
        sub_semantic_ = nh_.subscribe("/semantic_obstacles", 10, &BetaGuardNode::semanticCb, this);
        sub_odom_ = nh_.subscribe("/Odometry", 1, &BetaGuardNode::odomCb, this);

        // Publishers
        pub_beta_ = nh_.advertise<std_msgs::Float32MultiArray>("/safety_margin/beta", 10);
        pub_guard_log_ = nh_.advertise<semantic_guard::GuardLog>("/safety_margin/guard_log", 10);

        total_rollbacks_ = 0;
        has_odom_ = false;

        ROS_INFO("BetaGuardNode initialized. guard_enabled=%s, eta=%.2f, max_delta_beta=%.2f, tau=%.2f",
                 guard_enabled_ ? "true" : "false", eta_, max_delta_beta_, tau_);
    }

    ~BetaGuardNode() {
        if (csv_file_.is_open()) csv_file_.close();
    }

private:
    void odomCb(const nav_msgs::OdometryConstPtr& msg) {
        robot_pos_ << msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z;
        robot_vel_ << msg->twist.twist.linear.x,
                      msg->twist.twist.linear.y;
        has_odom_ = true;
    }

    void semanticCb(const semantic_fusion::SemanticObstacleArrayConstPtr& msg) {
        if (!has_odom_) return;

        std_msgs::Float32MultiArray beta_msg;
        semantic_guard::GuardLog log_msg;
        log_msg.header.stamp = ros::Time::now();

        for (const auto& obs : msg->obstacles) {
            // Step 1: Compute β̂
            std::string cls = obs.semantic_class;
            double beta_bar_val = beta_bar_.count(cls) ? beta_bar_[cls] : beta_bar_["unknown"];

            double mu = std::max(0.0, std::min(1.0,
                w_bias_ + w_head_ * obs.heading_factor
                        + w_ttc_ * obs.ttc_norm
                        + w_density_ * obs.density_norm));

            double beta_hat = beta_bar_val * mu;

            // Step 2: Compute h_EE = ||p_rel + τ v_rel|| - R_obs - R_robot
            Eigen::Vector2d obs_pos(obs.position.x, obs.position.y);
            Eigen::Vector2d obs_vel(obs.velocity.x, obs.velocity.y);
            Eigen::Vector2d p_rel = obs_pos - robot_pos_.head<2>();
            Eigen::Vector2d v_rel = obs_vel - robot_vel_;
            double d_i = p_rel.norm();
            double rel_v_norm = v_rel.norm();
            double cos_delta = 0.0;
            if (d_i > 1e-6 && rel_v_norm > 1e-6) {
                cos_delta = p_rel.normalized().dot(v_rel.normalized());
            }
            double closing_speed = 0.0;
            if (d_i > 1e-6) {
                closing_speed = std::max(-p_rel.normalized().dot(v_rel), 0.0);
            }
            double ttc = (closing_speed > 1e-6) ? (d_i / closing_speed)
                                                : std::numeric_limits<double>::infinity();
            double inv_ttc = std::isfinite(ttc) && ttc > 1e-6 ? 1.0 / ttc : 0.0;
            Eigen::Vector2d lookahead_rel_pos = p_rel + tau_ * v_rel;
            double h_ee = lookahead_rel_pos.norm() - obs.radius - robot_radius_;
            double guard_upper_bound = h_ee - eta_;

            // Step 3: Guard check: β̂ ≤ h_EE - η
            // Ensures h_SEE = h_EE - β ≥ -η (feasibility guarantee)
            bool guard_pass = guard_enabled_ ? (beta_hat <= guard_upper_bound) : true;

            // Step 4: Apply with rate limiting
            double beta_final;
            double beta_prev = getPrevBeta(obs.id);
            std::string guard_status;

            if (!guard_enabled_) {
                beta_final = beta_hat;
                guard_status = "disabled";
            } else if (guard_pass) {
                double delta = beta_hat - beta_prev;
                delta = std::max(-max_delta_beta_, std::min(max_delta_beta_, delta));
                beta_final = beta_prev + delta;
                guard_status = (std::abs(delta - (beta_hat - beta_prev)) > 1e-9) ? "rate_limited" : "accept";
            } else {
                beta_final = beta_prev;  // Rollback
                total_rollbacks_++;
                guard_status = "rollback";
            }

            // Ensure non-negative
            beta_final = std::max(0.0, beta_final);
            double h_see = h_ee - beta_final;
            double r_base = obs.radius + robot_radius_;
            double r_sem = r_base + beta_final;

            // Update state
            beta_prev_[obs.id] = beta_final;
            beta_msg.data.push_back(static_cast<float>(beta_final));

            // Log
            log_msg.obstacle_ids.push_back(obs.id);
            log_msg.beta_requested.push_back(beta_hat);
            log_msg.beta_applied.push_back(beta_final);
            log_msg.h_ee_values.push_back(h_ee);
            log_msg.guard_passed.push_back(guard_pass);

            // CSV log
            if (csv_file_.is_open()) {
                csv_file_ << ros::Time::now().toSec() << ","
                          << obs.id << "," << cls << ","
                          << beta_bar_val << "," << mu << ","
                          << beta_hat << "," << beta_final << ","
                          << guard_upper_bound << "," << (guard_pass ? 1 : 0) << ","
                          << guard_status << ","
                          << d_i << "," << rel_v_norm << ","
                          << (std::isfinite(ttc) ? ttc : -1.0) << "," << inv_ttc << ","
                          << cos_delta << "," << obs.density_norm << ",0,"
                          << h_ee << "," << h_see << ","
                          << r_base << "," << r_sem << "\n";
            }
        }

        log_msg.total_rollbacks = total_rollbacks_;

        pub_beta_.publish(beta_msg);
        pub_guard_log_.publish(log_msg);
    }

    double getPrevBeta(uint32_t id) {
        if (beta_prev_.count(id)) return beta_prev_[id];
        return beta_bar_["unknown"];  // First time: use unknown default
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_semantic_, sub_odom_;
    ros::Publisher pub_beta_, pub_guard_log_;

    // Config
    std::map<std::string, double> beta_bar_;
    double w_bias_, w_head_, w_ttc_, w_density_;
    double eta_, max_delta_beta_, tau_;
    double robot_radius_;
    bool guard_enabled_;

    // State
    Eigen::Vector3d robot_pos_;
    Eigen::Vector2d robot_vel_;
    bool has_odom_;
    std::map<uint32_t, double> beta_prev_;
    uint32_t total_rollbacks_;

    // Logging
    std::ofstream csv_file_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "beta_guard_node");
    ros::NodeHandle nh("~");
    BetaGuardNode node(nh);
    ros::spin();
    return 0;
}
