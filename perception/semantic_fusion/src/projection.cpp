#include "semantic_fusion/projection.h"
#include <algorithm>

namespace semantic_fusion {

Eigen::Vector2d projectToImage(const Eigen::Vector3d& point_cam,
                               const Eigen::Matrix3d& K) {
    if (point_cam.z() <= 0.0) {
        return Eigen::Vector2d(-1, -1);  // Behind camera
    }
    Eigen::Vector3d pixel = K * point_cam / point_cam.z();
    return pixel.head<2>();
}

double computeIoU(const Eigen::Vector4d& bbox_a, const Eigen::Vector4d& bbox_b) {
    // Convert [cx, cy, w, h] to [x1, y1, x2, y2]
    double a_x1 = bbox_a(0) - bbox_a(2) / 2.0;
    double a_y1 = bbox_a(1) - bbox_a(3) / 2.0;
    double a_x2 = bbox_a(0) + bbox_a(2) / 2.0;
    double a_y2 = bbox_a(1) + bbox_a(3) / 2.0;

    double b_x1 = bbox_b(0) - bbox_b(2) / 2.0;
    double b_y1 = bbox_b(1) - bbox_b(3) / 2.0;
    double b_x2 = bbox_b(0) + bbox_b(2) / 2.0;
    double b_y2 = bbox_b(1) + bbox_b(3) / 2.0;

    double inter_x1 = std::max(a_x1, b_x1);
    double inter_y1 = std::max(a_y1, b_y1);
    double inter_x2 = std::min(a_x2, b_x2);
    double inter_y2 = std::min(a_y2, b_y2);

    double inter_w = std::max(0.0, inter_x2 - inter_x1);
    double inter_h = std::max(0.0, inter_y2 - inter_y1);
    double inter_area = inter_w * inter_h;

    double area_a = bbox_a(2) * bbox_a(3);
    double area_b = bbox_b(2) * bbox_b(3);
    double union_area = area_a + area_b - inter_area;

    if (union_area <= 0.0) return 0.0;
    return inter_area / union_area;
}

}  // namespace semantic_fusion
