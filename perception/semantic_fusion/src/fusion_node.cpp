#include "semantic_fusion/fusion_node.h"
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace semantic_fusion {

FusionNode::FusionNode(ros::NodeHandle& nh)
    : nh_(nh), tf_listener_(tf_buffer_)
{
    // Parameters
    nh_.param("use_ground_truth", use_ground_truth_, false);
    nh_.param("iou_threshold", iou_threshold_, 0.2);
    nh_.param("camera_frame", camera_frame_, std::string("camera_color_optical_frame"));
    nh_.param("lidar_frame", lidar_frame_, std::string("body"));

    // Subscribers
    sub_odom_ = nh_.subscribe("/Odometry", 1, &FusionNode::odomCb, this);
    sub_clusters_ = nh_.subscribe("/l_shape_fitting/jsk_bbox_array", 1, &FusionNode::clusterCb, this);
    sub_detections_ = nh_.subscribe("/yolo/detections", 1, &FusionNode::detectionCb, this);
    sub_camera_info_ = nh_.subscribe("/camera/color/camera_info", 1, &FusionNode::cameraInfoCb, this);

    // Publisher
    pub_semantic_obs_ = nh_.advertise<SemanticObstacleArray>("/semantic_obstacles", 10);

    // Timer: 10Hz fusion
    fusion_timer_ = nh_.createTimer(ros::Duration(0.1), &FusionNode::fusionTimerCb, this);

    robot_pos_.setZero();
    robot_vel_.setZero();
    K_.setIdentity();

    ROS_INFO("SemanticFusionNode initialized. ground_truth=%s", use_ground_truth_ ? "true" : "false");
}

void FusionNode::odomCb(const nav_msgs::OdometryConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    robot_pos_ << msg->pose.pose.position.x,
                  msg->pose.pose.position.y,
                  msg->pose.pose.position.z;
    robot_vel_ << msg->twist.twist.linear.x,
                  msg->twist.twist.linear.y,
                  msg->twist.twist.linear.z;
    has_odom_ = true;
}

void FusionNode::clusterCb(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cluster_mutex_);
    clusters_.clear();
    for (size_t i = 0; i < msg->boxes.size(); i++) {
        LidarCluster c;
        c.id = i;
        c.position << msg->boxes[i].pose.position.x,
                      msg->boxes[i].pose.position.y,
                      msg->boxes[i].pose.position.z;
        c.dimensions << msg->boxes[i].dimensions.x,
                        msg->boxes[i].dimensions.y,
                        msg->boxes[i].dimensions.z;
        c.radius = std::max(c.dimensions.x(), c.dimensions.y()) / 2.0;
        clusters_.push_back(c);
    }
}

void FusionNode::detectionCb(const vision_msgs::Detection2DArrayConstPtr& msg) {
    std::lock_guard<std::mutex> lock(det_mutex_);
    detections_.clear();
    for (const auto& det : msg->detections) {
        VisualDetection vd;
        vd.bbox << det.bbox.center.x, det.bbox.center.y,
                   det.bbox.size_x, det.bbox.size_y;
        vd.semantic_class = det.source_img.encoding;  // semantic class passed via encoding field
        vd.confidence = det.results.empty() ? 0.0 : det.results[0].score;
        if (vd.semantic_class.empty()) vd.semantic_class = "unknown";
        detections_.push_back(vd);
    }
}

void FusionNode::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (has_camera_info_) return;
    K_ << msg->K[0], msg->K[1], msg->K[2],
           msg->K[3], msg->K[4], msg->K[5],
           msg->K[6], msg->K[7], msg->K[8];
    has_camera_info_ = true;
    ROS_INFO("Camera intrinsics received.");
}

void FusionNode::fusionTimerCb(const ros::TimerEvent& e) {
    if (!has_odom_) return;

    std::lock_guard<std::mutex> lock_c(cluster_mutex_);
    std::lock_guard<std::mutex> lock_d(det_mutex_);
    std::lock_guard<std::mutex> lock_o(odom_mutex_);

    SemanticObstacleArray out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "world";

    if (clusters_.empty()) {
        pub_semantic_obs_.publish(out_msg);
        return;
    }

    // Assign semantic classes
    std::vector<std::string> assigned_classes(clusters_.size(), "unknown");

    if (!detections_.empty() && has_camera_info_ && !use_ground_truth_) {
        // Get transform: lidar_frame → camera_frame
        Eigen::Isometry3d T_cam_lidar = Eigen::Isometry3d::Identity();
        try {
            geometry_msgs::TransformStamped tf_msg =
                tf_buffer_.lookupTransform(camera_frame_, lidar_frame_, ros::Time(0), ros::Duration(0.1));
            T_cam_lidar = tf2::transformToEigen(tf_msg);
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "TF lookup failed (%s → %s): %s",
                              lidar_frame_.c_str(), camera_frame_.c_str(), ex.what());
            // Fallback: all unknown
        }

        // Project each cluster center to image and compute IoU with YOLO detections
        std::vector<std::vector<double>> cost_matrix(clusters_.size(),
                                                      std::vector<double>(detections_.size(), 1.0));

        for (size_t i = 0; i < clusters_.size(); i++) {
            Eigen::Vector3d pt_cam = T_cam_lidar * clusters_[i].position;
            Eigen::Vector2d pixel = projectToImage(pt_cam, K_);
            if (pixel.x() < 0) continue;  // Behind camera

            // Create a pseudo bbox around projected point (use cluster radius for size estimate)
            double proj_size = (K_(0, 0) * clusters_[i].radius * 2.0) / pt_cam.z();
            Eigen::Vector4d cluster_bbox;
            cluster_bbox << pixel.x(), pixel.y(), proj_size, proj_size * 1.5;

            for (size_t j = 0; j < detections_.size(); j++) {
                double iou = computeIoU(cluster_bbox, detections_[j].bbox);
                cost_matrix[i][j] = 1.0 - iou;
            }
        }

        // Hungarian matching
        std::vector<int> assignment = hungarianMatch(cost_matrix);
        for (size_t i = 0; i < assignment.size(); i++) {
            if (assignment[i] >= 0 && cost_matrix[i][assignment[i]] < (1.0 - iou_threshold_)) {
                assigned_classes[i] = detections_[assignment[i]].semantic_class;
            }
        }
    }

    // Build output messages
    for (size_t i = 0; i < clusters_.size(); i++) {
        SemanticObstacle obs;
        obs.id = i;
        obs.semantic_class = assigned_classes[i];
        obs.position.x = clusters_[i].position.x();
        obs.position.y = clusters_[i].position.y();
        obs.position.z = clusters_[i].position.z();

        // Velocity estimation (finite difference)
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        if (prev_positions_.count(i) && prev_times_.count(i)) {
            double dt = (ros::Time::now() - prev_times_[i]).toSec();
            if (dt > 0.01 && dt < 1.0) {
                vel = (clusters_[i].position - prev_positions_[i]) / dt;
            }
        }
        prev_positions_[i] = clusters_[i].position;
        prev_times_[i] = ros::Time::now();
        velocities_[i] = vel;

        obs.velocity.x = vel.x();
        obs.velocity.y = vel.y();
        obs.velocity.z = vel.z();
        obs.radius = clusters_[i].radius;

        // Context features
        obs.heading_factor = computeHeadingFactor(clusters_[i].position, vel);
        obs.ttc_norm = computeTTCNorm(clusters_[i].position, vel);
        obs.density_norm = computeDensityNorm(clusters_[i].position);

        obs.beta_hat = 0.0;       // Computed by beta_guard_node
        obs.guard_passed = false;  // Set by beta_guard_node

        out_msg.obstacles.push_back(obs);
    }

    pub_semantic_obs_.publish(out_msg);
}

double FusionNode::computeHeadingFactor(const Eigen::Vector3d& obs_pos, const Eigen::Vector3d& obs_vel) {
    Eigen::Vector2d p_rel = (obs_pos - robot_pos_).head<2>();
    Eigen::Vector2d v_rel = (obs_vel - robot_vel_).head<2>();

    double p_norm = p_rel.norm();
    double v_norm = v_rel.norm();
    if (p_norm < 0.01 || v_norm < 0.01) return 0.0;

    double cos_angle = p_rel.normalized().dot(v_rel.normalized());
    return std::max(0.0, -cos_angle);  // Negative cos = approaching head-on
}

double FusionNode::computeTTCNorm(const Eigen::Vector3d& obs_pos, const Eigen::Vector3d& obs_vel) {
    Eigen::Vector2d p_rel = (obs_pos - robot_pos_).head<2>();
    Eigen::Vector2d v_rel = (obs_vel - robot_vel_).head<2>();

    double closing_speed = std::max(-p_rel.normalized().dot(v_rel), 0.0);
    if (closing_speed < 0.01) return 0.0;  // Not approaching

    double ttc_raw = p_rel.norm() / closing_speed;
    return std::max(0.0, std::min(1.0, 1.0 - ttc_raw / 5.0));  // Normalize: 5s → 0, 0s → 1
}

double FusionNode::computeDensityNorm(const Eigen::Vector3d& obs_pos) {
    int count = 0;
    for (const auto& c : clusters_) {
        if ((c.position - obs_pos).norm() < 3.0 && (c.position - obs_pos).norm() > 0.1) {
            count++;
        }
    }
    return std::min(1.0, count / 5.0);  // 5 neighbors = max density
}

std::vector<int> FusionNode::hungarianMatch(const std::vector<std::vector<double>>& cost_matrix) {
    // Simple greedy matching (for production, replace with proper Hungarian)
    int rows = cost_matrix.size();
    int cols = cost_matrix.empty() ? 0 : cost_matrix[0].size();
    std::vector<int> assignment(rows, -1);
    std::vector<bool> col_used(cols, false);

    for (int i = 0; i < rows; i++) {
        double min_cost = std::numeric_limits<double>::max();
        int best_j = -1;
        for (int j = 0; j < cols; j++) {
            if (!col_used[j] && cost_matrix[i][j] < min_cost) {
                min_cost = cost_matrix[i][j];
                best_j = j;
            }
        }
        if (best_j >= 0) {
            assignment[i] = best_j;
            col_used[best_j] = true;
        }
    }
    return assignment;
}

void FusionNode::spin() {
    ros::spin();
}

}  // namespace semantic_fusion

int main(int argc, char** argv) {
    ros::init(argc, argv, "semantic_fusion_node");
    ros::NodeHandle nh("~");
    semantic_fusion::FusionNode node(nh);
    node.spin();
    return 0;
}
