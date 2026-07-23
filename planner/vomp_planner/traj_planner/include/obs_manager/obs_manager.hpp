#pragma once
#ifndef _OBS_MANAGER_H
#define _OBS_MANAGER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <unordered_map>
#include <unordered_set>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

// #include <costmap_converter/ObstacleArrayMsg.h>   // TEB预测轨迹消息类型
#include <std_msgs/Float32MultiArray.h>           // 清华DCBF预测轨迹消息类型
#include <std_msgs/UInt32MultiArray.h>
#include "dynamic_simulator/DynTraj.h"            // 动态障碍物预测轨迹消息类型
#include "semantic_guard/AppliedMarginArray.h"
#include "semantic_guard/PredictedObstacleArray.h"
#include "semantic_guard/dynamic_tau.hpp"

// 障碍物轨迹类型结构体
struct obstacle_traj      
{
  int Id_;                  // 障碍物id号
  ros::Time start_time;     // 轨迹开始时间
  Eigen::Vector2d bbox_;    // 包围盒
  double circle_R_;         // 障碍物圆半径

  /* 轨迹真值参数: 三叶草轨迹 */
  Eigen::Vector2d scale_;   // 轨迹参数
  Eigen::Vector2d pos_xy_;  // 轨迹参数
  double slower_;           // 轨迹参数
  double offset_;           // 轨迹参数
  double start_delay_;      // 轨迹启动延迟，用于同步机器人和障碍物启动

  /* 预测轨迹参数: 二次多项式 */
  double Time0;              // 预测轨迹开始时间
  double Time1;              // 预测轨迹结束时间
  Eigen::Vector3d coeff_x;   // 二次多项式系数x
  Eigen::Vector3d coeff_y;   // 二次多项式系数y
};

struct applied_margin_cache_entry
{
  double beta_applied = 0.0;
  std::string accepted_source;
  ros::Time message_stamp;
  ros::Time receipt_time;
};

class Obs_Manager
{
public:
  Obs_Manager(){};    // 空的构造和析构
  ~Obs_Manager(){};

  void init(ros::NodeHandle &nh)   // Obs_Manager初始化
  {
    nh.param("obs_manager/use_GroundTruth", is_use_GroundTruth, true);
    nh.param("obs_manager/pre_step", pre_step, 25);
    nh.param("obs_manager/step_time", step_time, 0.1);
    nh.param("obs_manager/teacher_snapshot_period", teacher_snapshot_period_, 0.05);
    if (!std::isfinite(teacher_snapshot_period_) || teacher_snapshot_period_ <= 0.0) {
      throw std::invalid_argument("obs_manager/teacher_snapshot_period must be positive");
    }
    nh.param("search/global_seesm_enable", global_seesm_enable_, false);
    nh.param("search/global_seesm_tau", tau_global_, 0.20);
    nh.param("search/global_seesm_margin_timeout", global_seesm_margin_timeout_, 0.50);
    nh.param<std::string>("search/global_seesm_log_path", global_seesm_log_path_, std::string(""));
    nh.param("search/dynamic_tau_enabled", dynamic_tau_enabled_, true);
    nh.param<std::string>("search/dynamic_tau/mode", dynamic_tau_mode_name_, "teacher_tca");
    if (!semantic_guard::parseDynamicTauMode(dynamic_tau_mode_name_,
                                             &dynamic_tau_params_.mode)) {
      ROS_FATAL_STREAM("[global_seesm] unsupported search/dynamic_tau/mode='"
                       << dynamic_tau_mode_name_
                       << "'; expected legacy_gate, teacher_tca, or teacher_ke_tca");
      throw std::invalid_argument("unsupported search/dynamic_tau/mode");
    }
    nh.param("search/dynamic_tau/delta_tau", dynamic_tau_params_.delta_tau, 1e-6);
    nh.param("search/dynamic_tau/Ke", dynamic_tau_params_.ke, 0.30);
    nh.param("search/dynamic_tau/Tmax", dynamic_tau_params_.t_max, 2.0);
    nh.param("search/dynamic_tau/min_speed", dynamic_tau_params_.min_speed, 1e-6);
    nh.param("search/dynamic_tau/min_distance", dynamic_tau_params_.min_distance, 1e-6);
    nh.param("search/dynamic_tau/max_tau", dynamic_tau_params_.max_tau, 2.0);
    validateDynamicTauConfig();

    ROS_WARN("obs_manager pre_step is: %d", pre_step);
    ROS_WARN("obs_manager step_time is: %f", step_time);
    // 使用障碍物轨迹真值或障碍物预测轨迹
    if (is_use_GroundTruth) {
      obsTraj_sub = nh.subscribe("/trajs", 100, &Obs_Manager::obsTrajCallback, this);
    } else {
      ROS_WARN("use_dynamic_perception_data!");
      predicted_Traj_sub = nh.subscribe("/trajs_predicted", 100, &Obs_Manager::predict_Traj_Callback, this);
    }

    obsTraj_pub = nh.advertise<visualization_msgs::MarkerArray>("obs_traj_vis", 1, true);

    dcbfTraj_pub = nh.advertise<std_msgs::Float32MultiArray>("obs_predict_pub", 100, true);
    obsId_pub = nh.advertise<std_msgs::UInt32MultiArray>("obs_predict_ids", 100, true);
    teacher_snapshot_pub_ =
        nh.advertise<semantic_guard::PredictedObstacleArray>(
            "teacher_obstacle_snapshot", 20, true);
    teacher_snapshot_timer_ = nh.createTimer(
        ros::Duration(teacher_snapshot_period_),
        &Obs_Manager::publishTeacherObstacleSnapshot, this);

    // The experiment contract requires every declared CSV artifact to exist.
    // Keep an auditable header-only log when global SEESM is disabled; data rows
    // remain mandatory only for enabled runs.
    prepareGlobalSeesmLog();
    if (global_seesm_enable_) {
      applied_margin_sub_ = nh.subscribe("/safety_margin/beta_applied_final", 10,
                                         &Obs_Manager::appliedMarginCallback, this);
      ROS_WARN("global SEESM check enabled, tau=%f, margin_timeout=%f, dynamic_tau=%s, tau_mode=%s, delta_tau=%.3e",
               tau_global_, global_seesm_margin_timeout_,
               dynamic_tau_enabled_ ? "true" : "false",
               semantic_guard::dynamicTauModeName(dynamic_tau_params_.mode),
               dynamic_tau_params_.delta_tau);
    }

    // tebTraj_pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("move_base/TebLocalPlannerROS/obstacles", 100, true);

    // show_timer = nh.createTimer(ros::Duration(0.10), &Obs_Manager::showObs_callback, this);

    std::cout << "ObsManager init done !!!" << std::endl;
  }

  // 作为障碍物往返直线运动的函数
  void sfunc(double A, double T, double tt, double& res_pos, double& res_vel, double init_pos) const
  {
    double k = 2*A/T;
    if(tt<T) {
      res_pos = -k * tt + A + init_pos;
      res_vel = -k;
    }
    else {
      res_pos = k * tt - 3*A + init_pos;
      res_vel = k;
    }
    
  }

  // 根据时间返回障碍物的状态: posVel_list: P_x, P_y, V_x, V_y;   radius_lists: 对应的障碍物半径
  void get_obs_state(ros::Time cur_time, std::vector<Eigen::Vector4d>& posVel_list, std::vector<double>& radius_lists)
  {
    const auto active_obstacles = collectActiveObstacles(cur_time);
    for (const obstacle_traj* obstacle : active_obstacles) {
      Eigen::Vector4d cur_posVel;
      double radius = 0.0;
      computeObstacleStateAt(*obstacle, cur_time, cur_posVel, radius);
      posVel_list.push_back(cur_posVel);
      radius_lists.push_back(radius);
    }
  }

  int get_obs_size() {

    ros::Time cur_time = ros::Time::now();

    return static_cast<int>(collectActiveObstacles(cur_time).size());
  }

  bool is_collide(Eigen::Vector2d pos, double robot_R, ros::Time cur_time) // 根据当前位置和时间，返回碰撞状态
  {
    std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
    std::vector<double> radius_list;            // 获取当前时间的障碍物半径
    get_obs_state(cur_time, posVel_list, radius_list);

    for (int i = 0; i < posVel_list.size(); i++) {  // 遍历障碍物状态容器，查询是否碰撞
      double distance = (pos - posVel_list[i].head(2)).norm();

      if(distance <= radius_list[i] + robot_R - 0.02 + 0.3) {  // (当前位置与障碍物中心距离) 小于 (障碍物半径 + 机器人半径)
        return true;                              // 返回碰撞!
      }
    }
    return false;
  }

  bool is_VO_unsafe(Eigen::Vector4d posVel, double robot_R, ros::Time cur_time) // 根据当前位置和时间，返回VO安全状态
  {
    std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
    std::vector<double> radius_list;            // 获取当前时间的障碍物半径
    get_obs_state(cur_time, posVel_list, radius_list);

    for (int i = 0; i < posVel_list.size(); i++) {  // 遍历障碍物状态容器，查询VO状态
      Eigen::Vector2d rel_dis, rel_vel;
      double radius_vo = radius_list[i] + robot_R;
      rel_dis = posVel.head(2) - posVel_list[i].head(2);
      rel_vel = posVel.tail(2) - posVel_list[i].tail(2);

      Eigen::MatrixXd rjt_rj = rel_dis * rel_dis.transpose();       //计算VO代价的中间变量
      Eigen::MatrixXd rjt_vj = rel_dis * rel_vel.transpose();       //计算VO代价的中间变量
      Eigen::MatrixXd vjt_vj = rel_vel * rel_vel.transpose();       //计算VO代价的中间变量

      double dis_x_vel = rel_dis.transpose() * rel_vel; // dis_x_vel的正负就表示了角度是锐角还是钝角
      double vo = pow(dis_x_vel, 2) / rel_vel.squaredNorm() - rel_dis.squaredNorm() + pow(radius_vo, 2);

      double constant_time = 2.0;   // 允许的最小碰撞时3
      // double t_collide = (rjt_rj.norm() - rel_dis.norm() * robot_R) / rjt_vj.norm();   // 碰撞时间： (距离-半径)/速度投影
      double t_collide = (rel_dis.norm()-radius_vo)*rel_dis.norm()/std::abs(dis_x_vel);

      if (rel_dis.norm() <= radius_vo) {   // 是否碰撞
        return true;
      }
      // if (vo > 0.0 && dis_x_vel < 0.0 && t_collide < constant_time) {  // 是否违反VO
      if (dis_x_vel < 0.0){
        if (vo>0.0 && t_collide < constant_time){
          return true;
        }
      }
    }
    return false;
  }

  bool is_Adsm_unsafe(Eigen::Vector4d posVel, double robot_R, ros::Time cur_time) // 根据当前位置和时间，返回VO安全状态
  {
    std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
    std::vector<double> radius_list;            // 获取当前时间的障碍物半径
    get_obs_state(cur_time, posVel_list, radius_list);  
    double safety_distance = 0.3;
    double collision_time = 2.0;
    double adsm_time;
    double scale = 0.95;

    for (int i = 0; i < posVel_list.size(); i++) {  // 遍历障碍物状态容器，查询VO状态
      Eigen::Vector2d rel_dis, rel_vel, pre_dis;
      double radius_vo = radius_list[i] + robot_R;
      rel_dis = posVel_list[i].head(2)-posVel.head(2);
      rel_vel = posVel_list[i].tail(2)-posVel.tail(2);

      double dis_x_vel = rel_dis.transpose() * rel_vel; // dis_x_vel的正负就表示了角度是锐角还是钝角


      if(dis_x_vel < 0.0){
        // 相互靠近
        double t_collide = (rel_dis.norm()-radius_vo)*rel_dis.norm()/std::abs(dis_x_vel);
        adsm_time = scale*t_collide;
        pre_dis = rel_dis + rel_vel*adsm_time;
        // if(pre_dis.norm() <= radius_vo + safety_distance && t_collide <= collision_time ){
        if(pre_dis.norm() < radius_vo + safety_distance ){
          return true;
        }
      }

      // 判断是否碰撞
      if (rel_dis.norm() <= radius_vo+ safety_distance) {  
        return true;
      }
    }
    return false;
  }

  void beginGlobalSeesmReplan()
  {
    if (!global_seesm_enable_) {
      return;
    }
    current_global_replan_id_ += 1;
    current_global_replan_wall_start_ = ros::WallTime::now();
  }

  bool is_SEESM_unsafe(const Eigen::Vector4d& robot_state,
                       double robot_R,
                       const ros::Time& prediction_time,
                       bool shot_check)
  {
    if (!global_seesm_enable_) {
      return false;
    }

    const auto active_obstacles = collectActiveObstacles(prediction_time);
    bool rejected = false;
    for (const obstacle_traj* obstacle : active_obstacles) {
      Eigen::Vector4d obstacle_state;
      double radius = 0.0;
      computeObstacleStateAt(*obstacle, prediction_time, obstacle_state, radius);

      double beta_applied = 0.0;
      double margin_age_ms = -1.0;
      std::string accepted_source = "missing";
      std::string reason = "accepted";

      applied_margin_cache_entry margin_entry;
      bool has_margin_entry = false;
      {
        std::lock_guard<std::mutex> lock(applied_margin_mutex_);
        auto cache_it = applied_margin_cache_.find(obstacle->Id_);
        if (cache_it != applied_margin_cache_.end()) {
          margin_entry = cache_it->second;
          has_margin_entry = true;
        }
      }

      const ros::Time receipt_time = ros::Time::now();
      if (!has_margin_entry) {
        reason = "missing";
      } else {
        accepted_source = margin_entry.accepted_source.empty() ? "unknown" : margin_entry.accepted_source;
        const ros::Time margin_time =
            margin_entry.message_stamp.isZero() ? margin_entry.receipt_time : margin_entry.message_stamp;
        if (!margin_time.isZero()) {
          margin_age_ms = std::max(0.0, (receipt_time - margin_time).toSec() * 1000.0);
        }

        const bool margin_is_stale =
            !margin_time.isZero() && (receipt_time - margin_time).toSec() > global_seesm_margin_timeout_;
        if (accepted_source == "no_cbf") {
          reason = "no_cbf";
        } else if (margin_is_stale) {
          reason = "stale";
        } else {
          beta_applied = margin_entry.beta_applied;
        }
      }

      Eigen::Vector2d p_rel = robot_state.head<2>() - obstacle_state.head<2>();
      Eigen::Vector2d v_rel = robot_state.tail<2>() - obstacle_state.tail<2>();
      semantic_guard::DynamicTauResult tau_result;
      if (dynamic_tau_enabled_) {
        tau_result = semantic_guard::computeDynamicTau(
            p_rel.x(), p_rel.y(), v_rel.x(), v_rel.y(), radius + robot_R,
            dynamic_tau_params_);
      } else {
        tau_result.mode = dynamic_tau_params_.mode;
        tau_result.tau = 0.0;
        tau_result.computed = true;
        tau_result.valid = false;
        tau_result.reason = "disabled";
      }

      const double h_phys = p_rel.norm() - radius - robot_R;
      const double h_eesm =
          (p_rel + tau_result.tau * v_rel).norm() - radius - robot_R;
      const double h_seesm = h_eesm - beta_applied;

      const bool physical_rejected = h_phys <= 0.0;
      const bool seesm_rejected = h_seesm <= 0.0;
      const bool obstacle_rejected = physical_rejected || seesm_rejected;
      if (physical_rejected) {
        appendReason(reason, "h_phys");
      } else if (seesm_rejected) {
        appendReason(reason, "h_seesm");
      }

      writeGlobalSeesmLog(prediction_time.toSec(), obstacle->Id_, beta_applied, accepted_source,
                          margin_age_ms, h_phys, h_eesm, h_seesm, tau_result,
                          obstacle_rejected && !shot_check,
                          obstacle_rejected && shot_check, reason);
      rejected = rejected || obstacle_rejected;
    }

    return rejected;
  }

private:

  void validateDynamicTauConfig() const
  {
    if (!dynamic_tau_enabled_) return;
    const auto mode = dynamic_tau_params_.mode;
    const bool legacy_valid = std::isfinite(dynamic_tau_params_.ke) &&
        std::isfinite(dynamic_tau_params_.t_max) &&
        std::isfinite(dynamic_tau_params_.min_speed) &&
        std::isfinite(dynamic_tau_params_.min_distance) &&
        std::isfinite(dynamic_tau_params_.max_tau) &&
        dynamic_tau_params_.ke >= 0.0 && dynamic_tau_params_.t_max >= 0.0 &&
        dynamic_tau_params_.min_speed >= 0.0 &&
        dynamic_tau_params_.min_distance >= 0.0 &&
        dynamic_tau_params_.max_tau > 0.0;
    const bool teacher_valid = std::isfinite(dynamic_tau_params_.delta_tau) &&
        std::isfinite(dynamic_tau_params_.max_tau) &&
        dynamic_tau_params_.delta_tau > 0.0 &&
        dynamic_tau_params_.max_tau > 0.0 &&
        (mode != semantic_guard::DynamicTauMode::kTeacherKeTca ||
         (std::isfinite(dynamic_tau_params_.ke) && dynamic_tau_params_.ke > 0.0));
    const bool valid = mode == semantic_guard::DynamicTauMode::kLegacyGate
        ? legacy_valid : teacher_valid;
    if (!valid) {
      ROS_FATAL("[global_seesm] invalid dynamic tau config for mode=%s: Ke=%.9g Tmax=%.9g min_speed=%.9g min_distance=%.9g max_tau=%.9g delta_tau=%.9g",
                semantic_guard::dynamicTauModeName(mode), dynamic_tau_params_.ke,
                dynamic_tau_params_.t_max, dynamic_tau_params_.min_speed,
                dynamic_tau_params_.min_distance, dynamic_tau_params_.max_tau,
                dynamic_tau_params_.delta_tau);
      throw std::invalid_argument("invalid search/dynamic_tau config");
    }
  }

  bool is_use_GroundTruth;
  bool is_play_bag;
  bool global_seesm_enable_ = false;
  bool dynamic_tau_enabled_ = false;

  ros::Subscriber obsTraj_sub, predicted_Traj_sub, applied_margin_sub_;

  ros::Publisher obsTraj_pub, dcbfTraj_pub, tebTraj_pub, obsId_pub;
  ros::Publisher teacher_snapshot_pub_;

  ros::Timer show_timer, teacher_snapshot_timer_;

  int pre_step;
  double step_time;
  double teacher_snapshot_period_ = 0.05;
  uint64_t teacher_snapshot_cycle_id_ = 0;
  uint64_t last_applied_margin_cycle_id_ = 0;
  double tau_global_ = 0.20;
  semantic_guard::DynamicTauParams dynamic_tau_params_;
  std::string dynamic_tau_mode_name_ = "teacher_tca";
  double global_seesm_margin_timeout_ = 0.50;
  std::string global_seesm_log_path_;
  std::ofstream global_seesm_log_stream_;
  ros::WallTime current_global_replan_wall_start_;
  uint64_t current_global_replan_id_ = 0;
  std::unordered_map<int, applied_margin_cache_entry> applied_margin_cache_;
  std::mutex applied_margin_mutex_;
  std::mutex global_seesm_log_mutex_;

  std::unordered_map<int, obstacle_traj> obstalce_trajs_;         // 所有障碍物轨迹，哈希表形式存储

  void appliedMarginCallback(const semantic_guard::AppliedMarginArray::ConstPtr& msg)
  {
    const size_t obstacle_count = msg->obstacle_ids.size();
    if (msg->beta_applied.size() != obstacle_count ||
        msg->accepted_sources.size() != obstacle_count) {
      ROS_WARN_THROTTLE(1.0,
                        "[global_seesm] ignore applied margin message: ids=%zu beta=%zu sources=%zu",
                        obstacle_count, msg->beta_applied.size(), msg->accepted_sources.size());
      return;
    }
    if (msg->obstacle_cycle_id == 0) {
      ROS_WARN_THROTTLE(
          1.0,
          "[global_seesm] ignore applied margin message: zero obstacle cycle");
      return;
    }

    std::lock_guard<std::mutex> lock(applied_margin_mutex_);
    if (msg->obstacle_cycle_id <= last_applied_margin_cycle_id_) {
      ROS_WARN_THROTTLE(
          1.0,
          "[global_seesm] ignore stale applied margin cycle=%lu last=%lu",
          static_cast<unsigned long>(msg->obstacle_cycle_id),
          static_cast<unsigned long>(last_applied_margin_cycle_id_));
      return;
    }

    const ros::Time receipt_time = ros::Time::now();
    const ros::Time message_stamp = msg->header.stamp.isZero() ? receipt_time : msg->header.stamp;

    std::unordered_set<uint32_t> seen_ids;
    seen_ids.reserve(obstacle_count);
    std::unordered_map<int, applied_margin_cache_entry> validated_cache;
    validated_cache.reserve(obstacle_count);
    for (size_t i = 0; i < obstacle_count; ++i) {
      const uint32_t obstacle_id = msg->obstacle_ids[i];
      const double beta = msg->beta_applied[i];
      if (!seen_ids.insert(obstacle_id).second) {
        ROS_WARN_THROTTLE(1.0, "[global_seesm] ignore applied margin message: duplicate obstacle id=%u",
                          obstacle_id);
        return;
      }
      if (!std::isfinite(beta) || beta < 0.0) {
        ROS_WARN_THROTTLE(1.0, "[global_seesm] ignore applied margin message: invalid beta for obstacle id=%u",
                          obstacle_id);
        return;
      }
      if (msg->accepted_sources[i].empty()) {
        ROS_WARN_THROTTLE(
            1.0,
            "[global_seesm] ignore applied margin message: empty source for obstacle id=%u",
            obstacle_id);
        return;
      }

      applied_margin_cache_entry entry;
      entry.beta_applied = beta;
      entry.accepted_source = msg->accepted_sources[i];
      entry.message_stamp = message_stamp;
      entry.receipt_time = receipt_time;
      validated_cache[static_cast<int>(obstacle_id)] = entry;
    }

    last_applied_margin_cycle_id_ = msg->obstacle_cycle_id;
    applied_margin_cache_.swap(validated_cache);
  }

  void prepareGlobalSeesmLog()
  {
    if (global_seesm_log_path_.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(global_seesm_log_mutex_);
    if (global_seesm_log_stream_.is_open()) {
      return;
    }

    global_seesm_log_stream_.open(global_seesm_log_path_.c_str(), std::ios::out | std::ios::trunc);
    if (!global_seesm_log_stream_.is_open()) {
      ROS_WARN_STREAM("[global_seesm] failed to open log path: " << global_seesm_log_path_);
      return;
    }

    global_seesm_log_stream_
        << "t,replan_id,global_seesm_enable,obs_id,beta_applied,accepted_source,"
        << "margin_age_ms,h_phys,h_eesm,h_seesm,primitive_rejected,shot_rejected,reason,global_replan_ms,"
        << "tau,tau_mode,delta_tau,relative_dot,speed_squared,denominator,tca_raw,tca_clipped,"
        << "tau_unclipped,lower_clipped,upper_clipped,ke_scaled,"
        << "T_i,f_r,f_v,f_T,tau_computed,tau_active,tau_valid,tau_reason\n";
    global_seesm_log_stream_.flush();
  }

  static void appendReason(std::string& base, const std::string& suffix)
  {
    if (base.empty() || base == "accepted") {
      base = suffix;
      return;
    }
    base += "|" + suffix;
  }

  static std::string sanitizeCsvField(const std::string& field)
  {
    std::string sanitized = field;
    std::replace(sanitized.begin(), sanitized.end(), ',', ';');
    std::replace(sanitized.begin(), sanitized.end(), '\n', ' ');
    std::replace(sanitized.begin(), sanitized.end(), '\r', ' ');
    return sanitized;
  }

  void writeGlobalSeesmLog(double t,
                           int obs_id,
                           double beta_applied,
                           const std::string& accepted_source,
                           double margin_age_ms,
                           double h_phys,
                           double h_eesm,
                           double h_seesm,
                           const semantic_guard::DynamicTauResult& tau_result,
                           bool primitive_rejected,
                           bool shot_rejected,
                           const std::string& reason)
  {
    if (!global_seesm_log_stream_.is_open()) {
      return;
    }

    double global_replan_ms = 0.0;
    if (!current_global_replan_wall_start_.isZero()) {
      global_replan_ms =
          (ros::WallTime::now() - current_global_replan_wall_start_).toSec() * 1000.0;
    }

    std::lock_guard<std::mutex> lock(global_seesm_log_mutex_);
    global_seesm_log_stream_ << t << ","
                             << current_global_replan_id_ << ","
                             << (global_seesm_enable_ ? 1 : 0) << ","
                             << obs_id << ","
                             << beta_applied << ","
                             << sanitizeCsvField(accepted_source) << ","
                             << margin_age_ms << ","
                             << h_phys << ","
                             << h_eesm << ","
                             << h_seesm << ","
                             << (primitive_rejected ? 1 : 0) << ","
                             << (shot_rejected ? 1 : 0) << ","
                             << sanitizeCsvField(reason) << ","
                             << global_replan_ms << ","
                             << tau_result.tau << ","
                             << semantic_guard::dynamicTauModeName(dynamic_tau_params_.mode) << ","
                             << dynamic_tau_params_.delta_tau << ","
                             << tau_result.relative_dot << ","
                             << tau_result.speed_squared << ","
                             << tau_result.denominator << ","
                             << tau_result.t_ca_raw << ","
                             << tau_result.t_ca_clipped << ","
                             << tau_result.tau_unclipped << ","
                             << tau_result.lower_clipped << ","
                             << tau_result.upper_clipped << ","
                             << tau_result.ke_scaled << ","
                             << tau_result.T_i << ","
                             << tau_result.f_r << ","
                             << tau_result.f_v << ","
                             << tau_result.f_T << ","
                             << tau_result.computed << ","
                             << tau_result.valid << ","
                             << tau_result.computed << ","
                             << sanitizeCsvField(tau_result.reason) << "\n";
    global_seesm_log_stream_.flush();
  }

  bool isObstacleActiveAt(const obstacle_traj& obs, const ros::Time& query_time) const
  {
    if (is_use_GroundTruth) {
      return (query_time - obs.start_time).toSec() < 5.0 - 1e-3;
    }
    return query_time.toSec() <= obs.Time1 - 0.05;
  }

  void computeObstacleStateAt(const obstacle_traj& obs,
                              const ros::Time& query_time,
                              Eigen::Vector4d& cur_posVel,
                              double& radius) const
  {
    radius = obs.circle_R_;
    if (is_use_GroundTruth) {
      const double rel_time = query_time.toSec() - obs.Time0;
      const double delayed_rel_time = rel_time - obs.start_delay_;
      if (delayed_rel_time < 0.0) {
        sfunc(obs.scale_.x(), obs.slower_, 0.0, cur_posVel[0], cur_posVel[2], obs.pos_xy_.x());
        sfunc(obs.scale_.y(), obs.slower_, 0.0, cur_posVel[1], cur_posVel[3], obs.pos_xy_.y());
        cur_posVel[2] = 0.0;
        cur_posVel[3] = 0.0;
      } else {
        const double tmpc_X = fmod((delayed_rel_time + obs.offset_), 2 * obs.slower_);
        sfunc(obs.scale_.x(), obs.slower_, tmpc_X, cur_posVel[0], cur_posVel[2], obs.pos_xy_.x());
        sfunc(obs.scale_.y(), obs.slower_, tmpc_X, cur_posVel[1], cur_posVel[3], obs.pos_xy_.y());
      }
      return;
    }

    const double time_now = query_time.toSec() - obs.Time0;
    cur_posVel[0] = obs.coeff_x[0] * pow(time_now / 5.0, 2) +
                    obs.coeff_x[1] * time_now / 5.0 + obs.coeff_x[2];
    cur_posVel[1] = obs.coeff_y[0] * pow(time_now / 5.0, 2) +
                    obs.coeff_y[1] * time_now / 5.0 + obs.coeff_y[2];
    cur_posVel[2] = 2.0 * obs.coeff_x[0] * time_now / 25.0 + obs.coeff_x[1] / 5;
    cur_posVel[3] = 2.0 * obs.coeff_y[0] * time_now / 25.0 + obs.coeff_y[1] / 5;
  }

  std::vector<const obstacle_traj*> collectActiveObstacles(const ros::Time& query_time) const
  {
    std::vector<const obstacle_traj*> active_obstacles;
    active_obstacles.reserve(obstalce_trajs_.size());
    for (const auto& traj_entry : obstalce_trajs_) {
      if (isObstacleActiveAt(traj_entry.second, query_time)) {
        active_obstacles.push_back(&traj_entry.second);
      }
    }
    std::sort(active_obstacles.begin(), active_obstacles.end(),
              [](const obstacle_traj* lhs, const obstacle_traj* rhs) {
                return lhs->Id_ < rhs->Id_;
              });
    return active_obstacles;
  }

  void showObs_callback(const ros::TimerEvent& e)         // 定时器回调，定时更新障碍物显示
  {
    show_Obs_traj();
  }

  void show_Obs_traj() {
    if(obstalce_trajs_.size() == 0) return; // 没有障碍物轨迹，返回

    visualization_msgs::MarkerArray obs_balls_msg;
    visualization_msgs::Marker obs_ball;

    ros::Time time_now = ros::Time::now();

    obs_ball.header.frame_id = "world";
    obs_ball.header.stamp = time_now;
    obs_ball.type = visualization_msgs::Marker::SPHERE_LIST;  // CUBE:立方体, Sphere:球体, Cylinder:圆柱体
    obs_ball.action = visualization_msgs::Marker::ADD;
    obs_ball.id = 0;
    obs_ball.lifetime = ros::Duration(0.20);
    obs_ball.color.a = 0.50f;
    obs_ball.color.r = 0.00f;
    obs_ball.color.g = 0.50f;
    obs_ball.color.b = 0.80f;
    obs_ball.pose.orientation.w = 1.0;

    // 展示障碍物的预测轨迹 
    for (double add_time = 0.0; add_time < pre_step*step_time*0.8; add_time += step_time) {
      ros::Time time_index = time_now + ros::Duration(add_time);

      std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
      std::vector<double> radius_list;            // 获取当前时间的障碍物半径
      obs_ball.id = obs_ball.id + 1;
      get_obs_state(time_index, posVel_list, radius_list);
      for (int i = 0; i < posVel_list.size(); i++) {
        geometry_msgs::Point p;
        p.x = posVel_list[i].x();
        p.y = posVel_list[i].y();
        p.z = 0.25;
        obs_ball.points.push_back(p);
        obs_ball.scale.x = radius_list[i]*2;
        obs_ball.scale.y = radius_list[i]*2;
        obs_ball.scale.z = radius_list[i]*2;
      }
      obs_balls_msg.markers.push_back(obs_ball);
    }
    obsTraj_pub.publish(obs_balls_msg);
  }

  void pub_DCBF_traj() {
    if(obstalce_trajs_.size() == 0) {
      return; // 没有障碍物轨迹，返回
    }
    std_msgs::Float32MultiArray dcbf_msgs;
    std_msgs::Float32MultiArray acbf_msgs;
    std_msgs::UInt32MultiArray obs_ids_msg;
    ros::Time time_now = ros::Time::now();
    const auto active_obstacles = collectActiveObstacles(time_now);
    if (active_obstacles.empty()) {
      return;
    }

    int N_ = pre_step;                        // mpc预测步长，需与清华mpc-dcbf参数保持一致
    double delta_t_ = step_time;             // mpc离散时间，需与清华mpc-dcbf参数保持一致
    // std::cout<<"pre_step is:"<< N_<< std::endl;
    // std::cout<<"delta_time is:"<< delta_t_<< std::endl;

    dcbf_msgs.data.resize(5 * N_ * active_obstacles.size()); // 给障碍物矩阵分配内存
    acbf_msgs.data.resize(7 * N_ * active_obstacles.size());
    obs_ids_msg.data.resize(active_obstacles.size());
    for (int j = 0; j < active_obstacles.size(); ++j) {
      obs_ids_msg.data[j] = static_cast<uint32_t>(active_obstacles[j]->Id_);
    }

    for (int i = 0; i < N_; i++) {                // 预测第i步
      double add_time = (double)(i * delta_t_);
      ros::Time time_index = time_now + ros::Duration(add_time);
      for (int j = 0; j < active_obstacles.size(); j++) {  // 对于第j个障碍物
        Eigen::Vector4d pos_vel;
        double radius = 0.0;
        computeObstacleStateAt(*active_obstacles[j], time_index, pos_vel, radius);
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 0 ] = pos_vel.x();  // x坐标
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 1 ] = pos_vel.y();  // y坐标
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 2 ] = radius;       // 椭圆半长轴a
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 3 ] = radius;       // 椭圆半短轴b
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 4 ] = 0.0;                 // 椭圆方位角theta
        // for janedipan_test
        acbf_msgs.data[ 7 * N_ * j + 7 * i + 0 ] = pos_vel.x();  // x坐标
        acbf_msgs.data[ 7 * N_ * j + 7 * i + 1 ] = pos_vel.y();  // y坐标
        acbf_msgs.data[ 7 * N_ * j + 7 * i + 2 ] = radius;       // 椭圆半长轴a
        acbf_msgs.data[ 7 * N_ * j + 7 * i + 3 ] = radius;       // 椭圆半短轴b
        acbf_msgs.data[ 7 * N_ * j + 7 * i + 4 ] = 0.0;                 // 椭圆方位角theta
        acbf_msgs.data[ 7 * N_ * j + 7 * i + 5 ] = pos_vel.z();
        acbf_msgs.data[ 7 * N_ * j + 7 * i + 6 ] = pos_vel.w();
      }
    }
    // 可以添加一个符号标志
    obsId_pub.publish(obs_ids_msg);
    dcbfTraj_pub.publish(acbf_msgs);
  }

  void publishTeacherObstacleSnapshot(const ros::TimerEvent&)
  {
    semantic_guard::PredictedObstacleArray snapshot;
    snapshot.header.stamp = ros::Time::now();
    snapshot.header.frame_id = "world";
    snapshot.cycle_id = ++teacher_snapshot_cycle_id_;
    snapshot.horizon_steps = static_cast<uint32_t>(std::max(pre_step, 0));
    snapshot.prediction_step_sec = step_time;

    const auto active_obstacles = collectActiveObstacles(snapshot.header.stamp);
    snapshot.obstacle_ids.reserve(active_obstacles.size());
    for (const obstacle_traj* obstacle : active_obstacles) {
      if (obstacle->Id_ < 0) {
        ROS_ERROR_THROTTLE(
            1.0, "[teacher_snapshot] refusing negative obstacle ID=%d",
            obstacle->Id_);
        return;
      }
      snapshot.obstacle_ids.push_back(static_cast<uint32_t>(obstacle->Id_));
    }

    const size_t horizon = static_cast<size_t>(snapshot.horizon_steps);
    snapshot.state_data.resize(7 * horizon * active_obstacles.size());
    for (size_t stage = 0; stage < horizon; ++stage) {
      const ros::Time prediction_time =
          snapshot.header.stamp + ros::Duration(stage * step_time);
      for (size_t obstacle_index = 0;
           obstacle_index < active_obstacles.size(); ++obstacle_index) {
        Eigen::Vector4d position_velocity;
        double radius = 0.0;
        computeObstacleStateAt(*active_obstacles[obstacle_index], prediction_time,
                               position_velocity, radius);
        const size_t offset =
            7 * horizon * obstacle_index + 7 * stage;
        snapshot.state_data[offset + 0] = position_velocity.x();
        snapshot.state_data[offset + 1] = position_velocity.y();
        snapshot.state_data[offset + 2] = radius;
        snapshot.state_data[offset + 3] = radius;
        snapshot.state_data[offset + 4] = 0.0;
        snapshot.state_data[offset + 5] = position_velocity.z();
        snapshot.state_data[offset + 6] = position_velocity.w();
      }
    }
    teacher_snapshot_pub_.publish(snapshot);
  }

  // void pub_TEB_traj() {
  //   if(obstalce_trajs_.size() == 0) return; // 没有障碍物轨迹，返回

  //   ros::Time time_index = ros::Time::now();
  //   std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
  //   std::vector<double> radius_list;            // 获取当前时间的障碍物半径

  //   get_obs_state(time_index, posVel_list, radius_list);

  //   costmap_converter::ObstacleArrayMsg obs_Array;
  //   obs_Array.header.frame_id = "world";
  //   obs_Array.header.stamp = time_index;

  //   for (int i = 0; i < posVel_list.size(); i++) {
  //     costmap_converter::ObstacleMsg obs_tmp;
  //     obs_tmp.header.frame_id = "world";
  //     obs_tmp.header.stamp = time_index;
  //     obs_tmp.id = i;
  //     obs_tmp.polygon.points.resize(1);
  //     obs_tmp.polygon.points[0].x = posVel_list[i].x();
  //     obs_tmp.polygon.points[0].y = posVel_list[i].y();
  //     obs_tmp.polygon.points[0].z = 0.0;
  //     obs_tmp.radius = radius_list[i];

  //     double yaw = atan2(posVel_list[i][3], posVel_list[i][2]);
  //     tf::Quaternion quat;
  //     quat.setRPY(0.0, 0.0, yaw);
  //     tf::quaternionTFToMsg(quat, obs_tmp.orientation);

  //     obs_tmp.velocities.twist.linear.x = posVel_list[i][2];
  //     obs_tmp.velocities.twist.linear.y = posVel_list[i][3];
  //     obs_tmp.velocities.twist.linear.z = 0.0;
  //     obs_tmp.velocities.twist.angular.x = 0.0;
  //     obs_tmp.velocities.twist.angular.y = 0.0;
  //     obs_tmp.velocities.twist.angular.z = 0.0;

  //     obs_Array.obstacles.push_back(obs_tmp);
  //   }
  //   tebTraj_pub.publish(obs_Array);
  // }

  // --------------------使用障碍物轨迹真值
  void obsTrajCallback(const dynamic_simulator::DynTraj& msg)     // 接收障碍物真值轨迹的回调函数
  {
    obstacle_traj tmp_obs;    // 将障碍物消息转为vo_obstacle格式
    tmp_obs.Id_ = msg.id;
    tmp_obs.start_time = msg.header.stamp;
    tmp_obs.Time0 = msg.start_time.stamp.toSec();

    tmp_obs.scale_   << msg.s_num[0], msg.s_num[1];
    tmp_obs.pos_xy_  << msg.s_num[3], msg.s_num[4];
    tmp_obs.slower_ = msg.s_num[6];
    tmp_obs.offset_ = msg.s_num[7];
    tmp_obs.start_delay_ = (msg.s_num.size() > 8) ? msg.s_num[8] : 0.0;
    tmp_obs.bbox_ << msg.bbox[0], msg.bbox[1];

    Eigen::Vector2d bbox_half = tmp_obs.bbox_ / 2.0;  // 取bbox外接圆半径
    tmp_obs.circle_R_ = bbox_half.norm();

    // 更新障碍物轨迹容器
    auto find_ptr = obstalce_trajs_.find(tmp_obs.Id_);
    if (find_ptr != obstalce_trajs_.end()) {  // 如果障碍物之前已存在，替换哈希表中对应值
      find_ptr->second = tmp_obs;
    }
    else {                                    // 如果障碍物之前不存在，加入到哈希表中
      obstalce_trajs_.insert(std::make_pair(tmp_obs.Id_, tmp_obs));
    }

    // 删除超时的障碍物轨迹
    ros::Time time_now = ros::Time::now(); //当前时刻
    std::vector<std::unordered_map<int, obstacle_traj>::iterator> elements_to_remove;

    for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {

      double time_out = (time_now - traj_iter->second.start_time).toSec();

      if(time_out >= 3.0 - 1e-3 && traj_iter != obstalce_trajs_.end()) {  // 删除超时的障碍物轨迹
        // ROS_INFO("traj_ID is:%d",traj_iter->first);
        // ROS_INFO("over time_now is:%f", time_now.toSec());
        // ROS_INFO("over traj_time_start is:%f", traj_iter->second.start_time.toSec());
        ROS_INFO("ove delay time is: %f", time_out);
        elements_to_remove.push_back(traj_iter);
      }
    }

    // 删除临时容器中的元素
    for (const auto& elem : elements_to_remove) {
      obstalce_trajs_.erase(elem);
    }

    // std::cout << "obstalce_trajs_.size() := " << obstalce_trajs_.size() << std::endl;

    // 预测轨迹可视化
    show_Obs_traj();

    // 发布mpc-dcbf的预测轨迹消息
    pub_DCBF_traj();

    // 发布TEB的预测轨迹消息
    // pub_TEB_traj();
  }

  // --------------------动态感知得出障碍物预测轨迹
  void predict_Traj_Callback(const dynamic_simulator::DynTraj& msg)     // 接收障碍物预测轨迹的回调函数
  {
    obstacle_traj tmp_obs;    // 将障碍物消息转为vo_obstacle格式
    tmp_obs.Id_ = msg.id;
    tmp_obs.start_time = msg.header.stamp;
    tmp_obs.coeff_x <<  msg.pwp_mean.all_coeff_x[0].data[0],
                        msg.pwp_mean.all_coeff_x[0].data[1],
                        msg.pwp_mean.all_coeff_x[0].data[2];
    tmp_obs.coeff_y <<  msg.pwp_mean.all_coeff_y[0].data[0],
                        msg.pwp_mean.all_coeff_y[0].data[1],
                        msg.pwp_mean.all_coeff_y[0].data[2];
    tmp_obs.Time0 = msg.pwp_mean.times[0];
    tmp_obs.Time1 = msg.pwp_mean.times[1];
    tmp_obs.bbox_ << msg.bbox[0], msg.bbox[1];

    Eigen::Vector2d bbox_half = tmp_obs.bbox_ / 2.0;  // 取bbox外接圆半径
    tmp_obs.circle_R_ = bbox_half.norm();

    // 更新障碍物轨迹容器
    auto find_ptr = obstalce_trajs_.find(tmp_obs.Id_);
    if (find_ptr != obstalce_trajs_.end()) {  // 如果障碍物之前已存在，替换哈希表中对应值
      find_ptr->second = tmp_obs;
    }
    else {                                    // 如果障碍物之前不存在，加入到哈希表中
      obstalce_trajs_.insert(std::make_pair(tmp_obs.Id_, tmp_obs));
    }

    // 删除操作1: 删除超时的障碍物轨迹
    ros::Time time_now = ros::Time::now(); //当前时刻
    std::vector<std::unordered_map<int, obstacle_traj>::iterator> elements_to_remove;
    for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {
      double time_out = time_now.toSec() - traj_iter->second.Time0;
      if(time_out >= 1.0 - 1e-3 && traj_iter != obstalce_trajs_.end()) {  // 删除超时的障碍物轨迹
        ROS_INFO("over time is:%f", time_out);
        elements_to_remove.push_back(traj_iter);
      }
    }
    // 删除临时容器中的元素
    for (const auto& elem : elements_to_remove) {
      obstalce_trajs_.erase(elem);
    }

    // 删除操作2: 加速度过大的无效障碍物轨迹
    elements_to_remove.clear();
    for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {

      double time_rel = time_now.toSec() - traj_iter->second.Time0;

      double acc_x = 2.0 * traj_iter->second.coeff_x[0] / 25.0;
      double acc_y = 2.0 * traj_iter->second.coeff_y[0] / 25.0;

      // std::cout << "acc_x := " << acc_x << ", acc_y :=" << acc_y << std::endl;

      if (abs(acc_x) > 2.0 || abs(acc_y) > 2.0 ) {
        elements_to_remove.push_back(traj_iter);
      }
    }
    // 删除临时容器中的元素
    for (const auto& elem : elements_to_remove) {
      obstalce_trajs_.erase(elem);
    }

    // std::cout << "obstalce_trajs_.size() := " << obstalce_trajs_.size() << std::endl;

    // 预测轨迹可视化
    show_Obs_traj();

    // 发布mpc-dcbf的预测轨迹消息
    pub_DCBF_traj();

    // 发布TEB的预测轨迹消息
    // pub_TEB_traj();
  }

};

#endif
