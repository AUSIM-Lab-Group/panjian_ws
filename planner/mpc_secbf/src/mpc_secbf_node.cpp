#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <vector>

#include "mpc_secbf/mpc_secbf.h"
#include "semantic_guard/AppliedMarginArray.h"
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
        double v_max, v_min, o_max;
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
        if (!nh_.getParam("mpc/robot_radius", robot_radius)) {
            nh_.param("robot/radius", robot_radius, 0.4);
        }
        std::string planner_log_path;
        std::string timing_log_path;
        nh_.param<std::string>("planner_log_path", planner_log_path, "");
        nh_.param<std::string>("timing_log_path", timing_log_path, "");

        std::vector<double> Q = {1.0, 1.0, 0.05};
        std::vector<double> R = {0.1, 0.05};

        N_ = N;
        Ts_ = Ts;
        mpc_feasibility_guard_enabled_ = mpc_feasibility_guard_enabled;

        // Initialize solver
        solver_.init_solver(Ts, N, v_max, v_min, o_max, Q, R, gamma, beta_unknown, robot_radius,
                            epsilon_max, slack_weight, max_cbf_obstacles);

        openCsv(planner_csv_, planner_log_path,
                "t,mpc_status,first_attempt_status,final_status,accepted_beta_source,"
                "cmd_v,cmd_w,obs_count,constrained_obs_count,beta_count,used_fallback,mpc_feasibility_guard_used,"
                "slack,slack_sum,slack_mean,slack_max,solve_time_ms\n");
        openCsv(timing_csv_, timing_log_path,
                "t,mpc_secbf_ms,total_loop_time_ms\n");

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
        Eigen::Matrix3d R(q);
        double yaw = atan2(R.col(0)[1], R.col(0)[0]);
        double v = msg->twist.twist.linear.x;
        cur_state_ << msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      yaw, v * cos(yaw), v * sin(yaw);
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
        beta_list_.assign(msg->data.begin(), msg->data.end());
    }

    void obsCb(const std_msgs::Float32MultiArrayConstPtr& msg) {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        if (N_ <= 0 || msg->data.size() % (7 * N_) != 0) {
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Invalid obstacle payload size=%zu for N=%d",
                               msg->data.size(), N_);
            obs_matrix_.resize(7, 0);
            return;
        }
        int obs_cols = msg->data.size() / 7;
        obs_matrix_.resize(7, obs_cols);
        for (int i = 0; i < obs_cols; i++) {
            for (int j = 0; j < 7; j++)
                obs_matrix_(j, i) = msg->data[7 * i + j];
        }
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
        if (!validateObstacleContractLocked()) {
            ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Rejecting cycle because obstacle payload and IDs do not match");
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            writePlannerCsv("count_mismatch", "count_mismatch", "count_mismatch", "none",
                            false, false, 0.0);
            return;
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
                if (!accepted_beta_list_.empty() && validateBetaCountLocked(accepted_beta_list_, "previous")) {
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
                ROS_WARN_THROTTLE(1.0, "[MPC-SECBF] Fallback (no CBF) succeeded");
            }
        } else if (accepted_beta_source == "candidate") {
            accepted_beta_list_ = beta_list_;
            final_beta_values = beta_list_;
        }

        publishAcceptedMargins(final_beta_values, accepted_beta_source);

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
        const int obs_count = (N_ > 0) ? static_cast<int>(obs_matrix_.cols() / N_) : 0;
        const int constrained_obs_count = solver_.last_constrained_obs_count;
        if (planner_csv_.is_open()) {
            planner_csv_ << t << ","
                         << mpc_status << ","
                         << first_attempt_status << ","
                         << final_status << ","
                         << accepted_beta_source << ","
                         << cmd_vel_.linear.x << ","
                         << cmd_vel_.angular.z << ","
                         << obs_count << ","
                         << constrained_obs_count << ","
                         << beta_list_.size() << ","
                         << (used_fallback ? 1 : 0) << ","
                         << (mpc_guard_used ? 1 : 0) << ","
                         << solver_.last_slack_max << ","
                         << solver_.last_slack_sum << ","
                         << solver_.last_slack_mean << ","
                         << solver_.last_slack_max << ","
                         << solve_time_ms << "\n";
            planner_csv_.flush();
        }
        if (timing_csv_.is_open()) {
            timing_csv_ << t << ","
                        << solve_time_ms << ","
                        << solve_time_ms << "\n";
            timing_csv_.flush();
        }
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

    std::mutex odom_mutex_, path_mutex_, beta_mutex_, obs_mutex_;
    Eigen::VectorXd cur_state_;
    Eigen::MatrixXd global_path_;
    Eigen::MatrixXd goal_state_;
    Eigen::MatrixXd obs_matrix_;
    std::vector<uint32_t> obstacle_ids_;
    std::vector<double> beta_list_;
    std::vector<double> accepted_beta_list_;
    geometry_msgs::Twist cmd_vel_;
    bool has_odom_, has_path_;
    bool mpc_feasibility_guard_enabled_;
    std::ofstream planner_csv_, timing_csv_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_secbf_node");
    ros::NodeHandle nh("~");
    MpcSecbfNode node(nh);
    ros::spin();
    return 0;
}
