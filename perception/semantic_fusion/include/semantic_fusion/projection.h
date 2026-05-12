#pragma once

#include <Eigen/Dense>
#include <sensor_msgs/CameraInfo.h>

namespace semantic_fusion {

/**
 * Project a 3D point (in camera frame) to 2D image pixel coordinates.
 */
Eigen::Vector2d projectToImage(const Eigen::Vector3d& point_cam,
                               const Eigen::Matrix3d& K);

/**
 * Compute IoU between two 2D bounding boxes.
 * bbox format: [cx, cy, w, h]
 */
double computeIoU(const Eigen::Vector4d& bbox_a, const Eigen::Vector4d& bbox_b);

}  // namespace semantic_fusion
