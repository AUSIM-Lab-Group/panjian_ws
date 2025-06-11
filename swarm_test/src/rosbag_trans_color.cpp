#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher pub2, pub3, pub4, pub5, pub6, pub7;
ros::Subscriber sub2, sub3, sub4, sub5, sub6, sub7;

void odomTrajCallback2(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray new_markers = *msg;

    // 修改颜色
    for (auto& marker : new_markers.markers) {
        marker.color.r = 0.0;  // 红色
        marker.color.g = 1.0;  // 绿色
        marker.color.b = 0.0;  // 蓝色
        marker.color.a = 1.0;  // 不透明度
    }

    // 发布修改后的 MarkerArray
    pub2.publish(new_markers);
}

void odomTrajCallback3(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray new_markers = *msg;

    // 修改颜色
    for (auto& marker : new_markers.markers) {
        marker.color.r = 0.0;  // 红色
        marker.color.g = 0.0;  // 绿色
        marker.color.b = 1.0;  // 蓝色
        marker.color.a = 1.0;  // 不透明度
    }

    // 发布修改后的 MarkerArray
    pub3.publish(new_markers);
}

void odomTrajCallback4(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray new_markers = *msg;

    // 修改颜色
    for (auto& marker : new_markers.markers) {
        marker.color.r = 0.5;  // 红色
        marker.color.g = 0.5;  // 绿色
        marker.color.b = 0.0;  // 蓝色
        marker.color.a = 1.0;  // 不透明度
    }

    // 发布修改后的 MarkerArray
    pub4.publish(new_markers);
}

void odomTrajCallback5(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray new_markers = *msg;

    // 修改颜色
    for (auto& marker : new_markers.markers) {
        marker.color.r = 0.5;  // 红色
        marker.color.g = 0.0;  // 绿色
        marker.color.b = 0.5;  // 蓝色
        marker.color.a = 1.0;  // 不透明度
    }

    // 发布修改后的 MarkerArray
    pub5.publish(new_markers);
}

void odomTrajCallback6(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray new_markers = *msg;

    // 修改颜色
    for (auto& marker : new_markers.markers) {
        marker.color.r = 0.0;  // 红色
        marker.color.g = 0.5;  // 绿色
        marker.color.b = 0.5;  // 蓝色
        marker.color.a = 1.0;  // 不透明度
    }

    // 发布修改后的 MarkerArray
    pub6.publish(new_markers);
}

void odomTrajCallback7(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray new_markers = *msg;
    // 修改颜色
    for (auto& marker : new_markers.markers) {
        marker.color.r = 0.4;  // 红色
        marker.color.g = 0.0;  // 绿色
        marker.color.b = 0.1;  // 蓝色
        marker.color.a = 1.0;  // 不透明度
    }
    // 发布修改后的 MarkerArray
    pub7.publish(new_markers);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_trans_color_node");
    ros::NodeHandle nh;

    // 订阅 /odom_traj 话题
    sub2 = nh.subscribe("/odom_traj2", 10, odomTrajCallback2);
    sub3 = nh.subscribe("/odom_traj3", 10, odomTrajCallback3);
    sub4 = nh.subscribe("/odom_traj4", 10, odomTrajCallback4);
    sub5 = nh.subscribe("/odom_traj5", 10, odomTrajCallback5);
    sub6 = nh.subscribe("/odom_traj6", 10, odomTrajCallback6);
    sub7 = nh.subscribe("/odom_traj7", 10, odomTrajCallback7);

    // 发布 /odom_traj_use 话题
    pub2 = nh.advertise<visualization_msgs::MarkerArray>("/odom_traj2_use", 10);
    pub3 = nh.advertise<visualization_msgs::MarkerArray>("/odom_traj3_use", 10);
    pub4 = nh.advertise<visualization_msgs::MarkerArray>("/odom_traj4_use", 10);
    pub5 = nh.advertise<visualization_msgs::MarkerArray>("/odom_traj5_use", 10);
    pub6 = nh.advertise<visualization_msgs::MarkerArray>("/odom_traj6_use", 10);
    pub7 = nh.advertise<visualization_msgs::MarkerArray>("/odom_traj7_use", 10);

    ros::spin();
    return 0;
}
