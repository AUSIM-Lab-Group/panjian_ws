#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <string>

#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <vision_msgs/Detection2DArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "semantic_fusion/SemanticObstacle.h"
#include "semantic_fusion/SemanticObstacleArray.h"
#include "semantic_fusion/projection.h"

namespace semantic_fusion {

struct LidarCluster {
    uint32_t id;
    Eigen::Vector3d position;
    Eigen::Vector3d dimensions;
    double radius;
};

struct VisualDetection {
    Eigen::Vector4d bbox;  // cx, cy, w, h (pixels)
    std::string semantic_class;
    double confidence;
};

class FusionNode {
public:
    FusionNode(ros::NodeHandle& nh);
    void spin();

private:
    void odomCb(const nav_msgs::OdometryConstPtr& msg);
    void clusterCb(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg);
    void detectionCb(const vision_msgs::Detection2DArrayConstPtr& msg);
    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg);
    void fusionTimerCb(const ros::TimerEvent& e);

    // Context feature computation
    double computeHeadingFactor(const Eigen::Vector3d& obs_pos, const Eigen::Vector3d& obs_vel);
    double computeTTCNorm(const Eigen::Vector3d& obs_pos, const Eigen::Vector3d& obs_vel);
    double computeDensityNorm(const Eigen::Vector3d& obs_pos);

    // Hungarian matching
    std::vector<int> hungarianMatch(const std::vector<std::vector<double>>& cost_matrix);

    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_, sub_clusters_, sub_detections_, sub_camera_info_;
    ros::Publisher pub_semantic_obs_;
    ros::Timer fusion_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // State
    std::mutex odom_mutex_, cluster_mutex_, det_mutex_;
    Eigen::Vector3d robot_pos_, robot_vel_;
    std::vector<LidarCluster> clusters_;
    std::vector<VisualDetection> detections_;
    Eigen::Matrix3d K_;  // Camera intrinsics
    bool has_camera_info_ = false;
    bool has_odom_ = false;

    // Parameters
    bool use_ground_truth_ = false;
    double iou_threshold_ = 0.2;
    std::string camera_frame_ = "camera_color_optical_frame";
    std::string lidar_frame_ = "body";

    // Track velocity estimation (simple finite difference)
    std::map<uint32_t, Eigen::Vector3d> prev_positions_;
    std::map<uint32_t, ros::Time> prev_times_;
    std::map<uint32_t, Eigen::Vector3d> velocities_;
};

}  // namespace semantic_fusion
