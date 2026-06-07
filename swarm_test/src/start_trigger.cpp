/* 发布目标点开始测试 */

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher goal_pub;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "start_trigger_node");

  ros::NodeHandle node;
  ros::NodeHandle pnh("~");

  double goal_x = 21.0;
  double goal_y = 0.0;
  double goal_z = 0.0;
  pnh.param("goal_x", goal_x, goal_x);
  pnh.param("goal_y", goal_y, goal_y);
  pnh.param("goal_z", goal_z, goal_z);
  Eigen::Vector3d goal_point(goal_x, goal_y, goal_z);

  goal_pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  // 延迟时间发布目标点
  ros::Duration(3.0).sleep();

  geometry_msgs::PoseStamped goal_2D;
  goal_2D.header.stamp =ros::Time::now();
  goal_2D.header.frame_id = "world";
  goal_2D.pose.orientation.w = 1;
  goal_2D.pose.position.x = goal_point[0];
  goal_2D.pose.position.y = goal_point[1];
  goal_2D.pose.position.z = goal_point[2];
  goal_pub.publish(goal_2D);

  ROS_WARN("[start_trigger_node]: goal has publisher, test is starting!!!!!!");

  ros::spin();

  return 0;
}
