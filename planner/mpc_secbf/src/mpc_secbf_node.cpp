#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <mutex>
#include <vector>

#include "mpc_secbf/mpc_secbf.h"
#include "semantic_fusion/SemanticObstacleArray.h"

class MpcSecbfNode {
public:
    MpcSecbfNode(ros::NodeHandle& nh) : nh_(nh) {
        // Parameters
        double mpc_freq, Ts, gamma, beta_unknown;
        int N;
        double v_max, v_min, o_max;
        nh_.param("mpc/mpc_frequency", mpc_freq, 10.0);
        nh_.param("mpc/step_time", Ts, 0.2);
        nh_.param("mpc/pre_step", N, 20);
        nh_.param("mpc/v_max", v_max, 0.5);
        nh_.param("mpc/v_min", v_min, 0.3);
        nh_.param("mpc/o_max", o_max, 0.8);
        nh_.param("mpc/gamma", gamma, 0.35);
        nh_.param("mpc/beta_bar_unknown", beta_unknown, 0.4);

        std::vector<double> Q = {1.0, 1.0, 0.05};
        std::vector<double> R = {0.1, 0.05};

        N_ = N;
        Ts_ = Ts;

        // Initialize solver
        solver_.init_solver(Ts, N, v_max, v_min, o_max, Q, R, gamma, beta_unknown);

        // Subscribers
        sub_odom_ = nh_.subscribe("/Odometry", 1, &MpcSecbfNode::odomCb, this);
        sub_path_ = nh_.subscribe("/global_path", 10, &MpcSecbfNode::pathCb, this);
        sub_beta_ = nh_.subscribe("/safety_margin/beta", 10, &MpcSecbfNode::betaCb, this);
        sub_obs_ = nh_.subscribe("/globalFsm_by_adsm/obs_predict_pub", 100, &MpcSecbfNode::obsCb, this);

        // Publishers
        pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        pub_local_path_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);

        // Timers
        timer_replan_ = nh_.createTimer(ros::Duration(1.0 / mpc_freq), &MpcSecbfNode::replanCb, this);
        timer_cmd_ = nh_.createTimer(ros::Duration(0.01), &MpcSecbfNode::cmdCb, this);

        cur_state_.resize(5);
        cur_state_.setZero();
        has_odom_ = false;
        has_path_ = false;

        ROS_INFO("MPC-SECBF node started. freq=%.1f Hz, N=%d, Ts=%.2f", mpc_freq, N, Ts);
    }

private:
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
        int obs_num = msg->data.size() / 7;
        obs_matrix_.resize(7, obs_num);
        for (int i = 0; i < obs_num; i++) {
            for (int j = 0; j < 7; j++)
                obs_matrix_(j, i) = msg->data[7 * i + j];
        }
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

        // Choose goal states from global path
        chooseGoalState();
        smoothYaw(goal_state_);

        ros::Time t0 = ros::Time::now();

        // Solve MPC-SECBF
        bool success = solver_.solve(&cur_state_, &goal_state_, &obs_matrix_, beta_list_);

        if (!success) {
            // Fallback: try without CBF constraints (empty beta)
            std::vector<double> empty_beta;
            Eigen::MatrixXd empty_obs(7, 0);
            success = solver_.solve(&cur_state_, &goal_state_, &empty_obs, empty_beta);

            if (!success) {
                // Complete failure: stop
                ROS_ERROR_THROTTLE(1.0, "[MPC-SECBF] Both SECBF and fallback infeasible, STOPPING");
                cmd_vel_.linear.x = 0.0;
                cmd_vel_.angular.z = 0.0;
                return;
            } else {
                ROS_WARN_THROTTLE(1.0, "[MPC-SECBF] Fallback (no CBF) succeeded");
            }
        }

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
        std::cout << "\033[38;2;0;200;100m MPC-SECBF replan_time =: \033[0m" << cost_ms << "ms" << std::endl;
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
    ros::Subscriber sub_odom_, sub_path_, sub_beta_, sub_obs_;
    ros::Publisher pub_cmd_, pub_local_path_;
    ros::Timer timer_replan_, timer_cmd_;

    MPC_SECBF_SOLVE solver_;
    int N_;
    double Ts_;

    std::mutex odom_mutex_, path_mutex_, beta_mutex_, obs_mutex_;
    Eigen::VectorXd cur_state_;
    Eigen::MatrixXd global_path_;
    Eigen::MatrixXd goal_state_;
    Eigen::MatrixXd obs_matrix_;
    std::vector<double> beta_list_;
    geometry_msgs::Twist cmd_vel_;
    bool has_odom_, has_path_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_secbf_node");
    ros::NodeHandle nh("~");
    MpcSecbfNode node(nh);
    ros::spin();
    return 0;
}
