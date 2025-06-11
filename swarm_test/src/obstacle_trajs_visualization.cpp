#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen> 
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <fstream>

#include "obs_manager_for_data_process/obs_manager.hpp"

ros::Subscriber obs_sub;
ros::Subscriber odom_sub;
ros::Subscriber cmd_sub;
ros::Publisher obs_trajs_pub;
ros::Timer check_date;

double step_time_use;
int traj_steps_use;


std::vector<Eigen::MatrixXd> obs_traj_rc; // 原始轨迹: step_nums X 2xobs_num
std::vector<Eigen::MatrixXd> obs_traj_Use;
std::vector<double> radius_obs;

bool has_obs_traj = false;
bool has_odom = false;
bool has_cmd = false;

Eigen::MatrixXd obs_pos;
Eigen::Vector2d ro_cmd;
ros::Time last_odom_time;
int odom_time_stamp = 0;
Eigen::Vector2d last_position(0, 0);
Eigen::Vector2d last_vel_rate(0, 0);
Eigen::Vector2d last_vel_acc(0, 0);
double last_yaw = 0;
double last_yaw_rate = 0;
double last_yaw_acc = 0;

std::vector<Eigen::MatrixXd> obs_ls;
std::vector<Eigen::Vector2d> cmd_ls;
std::vector<int> odom_time_ls;
std::vector<Eigen::Vector2d> pos_ls;
std::vector<Eigen::Vector2d> vel_ls;
std::vector<Eigen::Vector2d> acc_ls;
std::vector<double> yaw_ls;
std::vector<double> yaw_rate_ls;
std::vector<double> yaw_acc_ls;

int max_obs_num = 0;

void obs_traj_callback(const visualization_msgs::MarkerArray& msg){
    // msg: 0.2x20
    if (msg.markers.size() == 0){
        has_obs_traj = false;
        obs_pos.resize(2, 3);
        obs_pos.setZero();
        return;
    }
    has_obs_traj = true;
    int step_num_ = msg.markers.size();
    int obs_num_ = msg.markers[0].points.size();
    obs_pos.resize(2, 3); // 障碍物的最大数目为3
    obs_pos.setZero();
    for(int i = 0; i < obs_num_; i++){
        obs_pos(0, i) = msg.markers[step_num_-1].points[i].x;
        obs_pos(1, i) = msg.markers[step_num_-1].points[i].y;
    }
    // ROS_INFO("obs_traj.size is:%d", step_num_); // 轨迹步数
    // ROS_INFO("obs_num is:%d", obs_num_); // 每个时刻的障碍物个数
    // ROS_INFO("obs_traj_array.size is:%d", msg.markers[step_num_-1].points.size()); // 每个时刻的障碍物个数
    // if(obs_num_ > max_obs_num) max_obs_num = obs_num_;
    // ROS_INFO("max_obs_num is:%d", max_obs_num); // 每个时刻的障碍物个数
    // 记录轨迹
    obs_traj_rc.clear();
    obs_traj_rc = std::vector<Eigen::MatrixXd>(step_num_, Eigen::MatrixXd(2, obs_num_));
    for(int i = 0; i < step_num_; i++){
        for(int j = 0; j < obs_num_; j++){
            obs_traj_rc[i](0, j) = msg.markers[step_num_-1].points[i*obs_num_+j].x;
            obs_traj_rc[i](1, j) = msg.markers[step_num_-1].points[i*obs_num_+j].y;
        }
    }

    for(int i = 0; i < obs_num_; i++){
        radius_obs.push_back(msg.markers[step_num_-1].scale.x);
    }

    // 重组轨迹
    obs_traj_Use.clear();
    obs_traj_Use = std::vector<Eigen::MatrixXd>(traj_steps_use, Eigen::MatrixXd(2, obs_num_));
    for(int i = 0; i < traj_steps_use; i++){
        double time_index_ = i*step_time_use/0.2;
        int index_ = int(time_index_);
        // std::cout<< "index is:"<< index_<< std::endl;
        // std::cout<< "err is:"<< err_<< std::endl;
        double err_ = time_index_ - index_;
        
        obs_traj_Use[i] = obs_traj_rc[index_]*(1-err_) + obs_traj_rc[index_+1]*err_;
    }

    // 显示轨迹
    visualization_msgs::MarkerArray obs_trajs_Array;
    visualization_msgs::Marker obs_traj_One;
    ros::Time time_now = ros::Time::now();

    obs_traj_One.header.frame_id = "world";
    obs_traj_One.header.stamp = time_now;
    obs_traj_One.type = visualization_msgs::Marker::SPHERE_LIST; // SPHERE_LIST
    obs_traj_One.action = visualization_msgs::Marker::ADD;
    obs_traj_One.id = 0;
    obs_traj_One.lifetime = ros::Duration(0.20);
    obs_traj_One.color.a = 0.80f;
    obs_traj_One.color.r = 0.00f;
    obs_traj_One.color.g = 0.50f;
    obs_traj_One.color.b = 0.80f;
    obs_traj_One.pose.orientation.w = 1.0;

    for(int i=0; i<obs_num_; i++){
        obs_traj_One.id = i;
        for(int j=0; j<traj_steps_use; j++){
            geometry_msgs::Point p;
            p.x = obs_traj_Use[j](0, i);
            p.y = obs_traj_Use[j](1, i);
            p.z = 0.25;
            obs_traj_One.points.push_back(p);
        }
        obs_traj_One.scale.x = 0.8;
        obs_traj_One.scale.y = 0.8;
        obs_traj_One.scale.z = 0.8;
        obs_trajs_Array.markers.push_back(obs_traj_One);
    }
    obs_trajs_pub.publish(obs_trajs_Array);
}

void odom_callback(const nav_msgs::Odometry& msg){
    if (!has_odom){
        has_odom = true;
        last_odom_time = msg.header.stamp;
        last_position = Eigen::Vector2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
        tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, last_yaw);
        return;
    }
    // 计算时间间隔
    double dt = (msg.header.stamp - last_odom_time).toSec();
    // 计算当前速度
    Eigen::Vector2d curr_position(msg.pose.pose.position.x, msg.pose.pose.position.y);
    Eigen::Vector2d curr_vel_rate = (curr_position - last_position) / dt;
    // 计算当前角速度   
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, curr_yaw;
    m.getRPY(roll, pitch, curr_yaw);
    double curr_yaw_rate = (curr_yaw - last_yaw) / dt;
    // 计算加速度
    Eigen::Vector2d curr_vel_acc = (curr_vel_rate - last_vel_rate) / dt;
    double curr_yaw_acc = (curr_yaw_rate - last_yaw_rate) / dt;

    // 更新数据
    last_odom_time = msg.header.stamp;
    last_position = curr_position;
    last_vel_rate = curr_vel_rate;
    last_vel_acc = curr_vel_acc;
    last_yaw = curr_yaw;
    last_yaw_rate = curr_yaw_rate;
    last_yaw_acc = curr_yaw_acc;
    
    // std::cout<< "curr_position is:"<< curr_position(0)<< "\t"<< curr_position(1)<< std::endl;
    // std::cout<< "curr_vel_rate is:"<< curr_vel_rate(0)<< "\t"<< curr_vel_rate(1)<< std::endl;
    // std::cout<< "curr_vel_acc is:"<< curr_vel_acc(0)<< "\t"<< curr_vel_acc(1)<< std::endl;
    // std::cout<< "curr_yaw is:"<< curr_yaw<< std::endl;
    // std::cout<< "curr_yaw_rate is:"<< curr_yaw_rate<< std::endl;
    // std::cout<< "curr_yaw_acc is:"<< curr_yaw_acc<< std::endl;
    // std::cout<< "==================================================\n";
}

void cmd_callback(const geometry_msgs::Twist& msg){
    if (!has_cmd) has_cmd = true;
    ro_cmd = Eigen::Vector2d(msg.linear.x, msg.angular.z);
}

void checkDataCb(const ros::TimerEvent& event){
    if(!has_odom || !has_cmd) return;
    // 保存odom数据
    odom_time_ls.push_back(odom_time_stamp++);
    pos_ls.push_back(last_position); vel_ls.push_back(last_vel_rate); acc_ls.push_back(last_vel_acc);
    yaw_ls.push_back(last_yaw); yaw_rate_ls.push_back(last_yaw_rate); yaw_acc_ls.push_back(last_yaw_acc);
    
    // 保存cmd数据
    cmd_ls.push_back(ro_cmd);

    // 保存obs数据
    obs_ls.push_back(obs_pos);
    return;
}

void save_data(){
    std::ofstream csvFile("/home/jane/Ws_Jane_github/dynamic_avoidance/src/swarm_test/docs/data0302/data1.csv");
    
    if (!csvFile.is_open()) {
        std::cerr << "无法打开文件进行写入！" << std::endl;
        return;
    }
    // 默认清空文件内容，再写入数据，以逗号分隔
    csvFile<< "Timestamp,Position X,Position Y,Velocity X,Velocity Y,Acceleration X,Acceleration Y,Yaw,Yaw Rate,Yaw Acceleration,Command v,Command w, ObsX1, ObsY1, ObsX2, ObsY2, ObsX3, ObsY3\n";
    
    for(int i=0; i<odom_time_ls.size(); ++i){
        csvFile<< odom_time_ls[i]<< ","
               << pos_ls[i](0)<< ","<< pos_ls[i](1)<< ","
               << vel_ls[i](0)<< ","<< vel_ls[i](1)<< ","
               << acc_ls[i](0)<< ","<< acc_ls[i](1)<< ","
               << yaw_ls[i]<< ","
               << yaw_rate_ls[i]<< ","
               << yaw_acc_ls[i]<< ","
               << cmd_ls[i](0)<< ","<< cmd_ls[i](1)<< ",";
        
        // 写入obs_ls数据
        for(int j=0; j<3; ++j){
            if(abs(obs_ls[i](0,j))<1E-2 && abs(obs_ls[i](1,j))<1E-2){
                csvFile<< ",";
                if(j<2) csvFile<< ",";
            }
            else{
                csvFile<< obs_ls[i](0,j)<< ","<< obs_ls[i](1,j);
                if(j<2) csvFile<< ",";
            }
        }
        csvFile<< "\n";
    }
    csvFile.close();
    std::cout<< "save_data finish!"<< std::endl;
    return;
}


int main (int argc, char** argv) {
    ros::init(argc, argv, "obstacle_trajs_visualization");
    ros::NodeHandle nh;

    nh.param("obs_vis_node/step_time_use", step_time_use, 0.2);
    nh.param("obs_vis_node/traj_steps_use", traj_steps_use, 20);
    obs_sub = nh.subscribe("/trajs_predicted", 100, obs_traj_callback);
    odom_sub = nh.subscribe("/Odometry", 100, odom_callback);
    cmd_sub = nh.subscribe("/cmd_vel1", 100, cmd_callback);
    obs_trajs_pub = nh.advertise<visualization_msgs::MarkerArray>("/obs_trajs_Use", 100);
    check_date = nh.createTimer(ros::Duration(0.1), checkDataCb, false);
    ROS_WARN("Obstacle Trajectory Visualization Node Started");

    while (ros::ok()) ros::spin();
    std::cout<< "======================= end ======================="<< std::endl;
    std::cout<< "odom_size = "<< odom_time_ls.size()<< std::endl;
    std::cout<< "cmd_size = "<< cmd_ls.size()<< std::endl;
    std::cout<< "obs_size = "<< obs_ls.size()<< std::endl;
    save_data();
}