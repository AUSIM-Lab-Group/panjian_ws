#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import json
import os
import yaml
from datetime import datetime

from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float64
from geometry_msgs.msg import PoseStamped


class GlobalPathDataProcessor:
    def __init__(self):
        rospy.init_node('global_path_data_processor', anonymous=True)
        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        # 参数配置
        self.robot_radius = rospy.get_param('~robot_radius', 0.4)
        self.max_replans = rospy.get_param('~max_replans', 5)
        self.num_obstacles = rospy.get_param('~num_obstacles', 10)
        self.time_step = rospy.get_param('~time_step', 0.2)
        self.pre_step = rospy.get_param('~pre_step', 20)
        self.output_dir = rospy.get_param('~output_dir', os.path.join(package_root, 'output'))
        
        # 数据存储
        self.replan_count = 0
        self.path_data = []
        self.current_path = None
        self.current_obstacles = []
        self.initial_path_received = False
        self.latest_search_time = 0.0
        self.waiting_for_replan_path = False
        
        # 新增：成功/失败统计
        self.successful_replans = 0
        self.failed_replans = 0
        
        # 添加时间戳缓存
        self.obstacle_data_buffer = {}  # 时间戳 -> 障碍物数据
        self.max_buffer_age = 1.0  # 缓存1秒内的数据
        
        # 订阅器
        self.path_sub = rospy.Subscriber('/global_path', Path, self.path_callback)
        self.path_pub = rospy.Publisher('/global_path_pub', Path, queue_size=10)
        self.obs_sub = rospy.Subscriber('/globalFsm_by_adsm/obs_predict_pub', Float32MultiArray, self.obstacle_callback)
        self.search_time_sub = rospy.Subscriber('/global_planning/search_time', Float64, self.search_time_callback)
        
        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)
        
        rospy.loginfo("全局路径数据处理器已启动")
        rospy.loginfo(f"机器人半径: {self.robot_radius}m")
        rospy.loginfo(f"最大重规划次数: {self.max_replans}")
        rospy.loginfo(f"输出目录: {self.output_dir}")
        rospy.loginfo("注意: 将跳过初始规划，只记录重规划数据")

    def search_time_callback(self, msg):
        """搜索时间回调函数"""
        search_time = msg.data
        
        if search_time < 0:
            # 负值表示初始规划，跳过
            # rospy.loginfo(f"检测到初始规划搜索时间: {abs(search_time):.4f}s (跳过记录)")
            self.initial_path_received = True
            return
        else:
            # 正值表示重规划
            self.latest_search_time = search_time
            self.waiting_for_replan_path = True
            # rospy.loginfo(f"接收到重规划搜索时间: {search_time:.4f}s，等待路径数据...")
        

    def path_callback(self, msg):
        """路径回调函数 - 增加成功/失败判断"""
        current_time = rospy.Time.now()
        path_stamp = msg.header.stamp
        
        if self.current_obstacles:
            first_obs = self.current_obstacles[0]
            # rospy.loginfo(f"第一个障碍物: 位置({first_obs['x']:.2f},{first_obs['y']:.2f}) 半径{first_obs['radius']:.2f}")
        
        # 如果还没有接收到初始规划标记，跳过
        if not self.initial_path_received:
            rospy.loginfo("等待初始规划完成...")
            return
        
        # 如果没有等待重规划路径，说明这是初始路径或者其他非重规划路径，跳过
        if not self.waiting_for_replan_path:
            return
            
        # 检查是否已达到最大重规划次数
        if self.replan_count >= self.max_replans:
            return
        
        # 使用真实的搜索时间
        search_time = self.latest_search_time
        self.waiting_for_replan_path = False  # 重置等待标志

        path_msg = Path()
        path_msg.header.stamp = path_stamp
        path_msg.header.frame_id = "world"

        # 提取路径点
        path_points = []
        for pose in msg.poses:
            if len(path_msg.poses) < 20:
                path_msg.poses.append(pose)

            path_points.append([
                pose.pose.position.x,
                pose.pose.position.y,
                0.0,  # vx (后续计算)
                0.0,  # vy (后续计算)  
                0.0   # 时间戳相对值
            ])

        # 发布路径
        self.path_pub.publish(path_msg)

        # 查找最接近路径时间戳的障碍物数据
        matching_obstacles = self.find_matching_obstacles(path_stamp)
        
        if matching_obstacles is None:
            rospy.logwarn(f"未找到匹配时间戳的障碍物数据，路径时间: {path_stamp.to_sec()}")
            matching_obstacles = self.current_obstacles
        
        # 计算运动学参数
        velocities, angular_velocities = self.calculate_path_kinematics(path_points)
        velocity_variance = float(np.var(velocities)) if len(velocities) > 1 else 0.0
        angular_velocity_variance = float(np.var(angular_velocities)) if len(angular_velocities) > 1 else 0.0
        
        # 计算安全指标 - 现在返回四个值，包括有效障碍物数量
        min_distance, isad_score, avg_distance, valid_obstacles_count = self.calculate_safety_metrics(path_points, matching_obstacles)
        
        # 检查重规划是否成功（无负距离） - 确保返回Python原生bool类型
        is_successful = bool(min_distance >= 0.0)
        
        if is_successful:
            self.successful_replans += 1
            status = "成功"
        else:
            self.failed_replans += 1
            status = "失败"
        
        # 记录时间差信息
        time_diff = abs((rospy.Time.now() - path_stamp).to_sec())
        rospy.loginfo(f"路径-障碍物时间差: {time_diff:.3f}s")
        
        # 存储重规划数据（包括失败的）
        path_info = {
            'replan_id': self.replan_count + 1,
            'timestamp': path_stamp,
            'search_time': float(search_time),
            'path_length': len(path_points),
            'velocity_variance': velocity_variance,
            'angular_velocity_variance': angular_velocity_variance,
            'min_obstacle_distance': float(min_distance),
            'global_average_distance': float(avg_distance),
            'isad_score': float(isad_score),
            'is_successful': is_successful,  # 新增字段
            'valid_obstacles_count': valid_obstacles_count,  # 新增字段
            'path_points': path_points,
            'obstacles_snapshot': matching_obstacles.copy() if matching_obstacles else []
        }
        
        self.path_data.append(path_info)
        self.current_path = path_points
        self.replan_count += 1
        
        # 添加数据验证
        rospy.loginfo(f"✅ 记录第 {self.replan_count} 次重规划 [{status}]:")
        rospy.loginfo(f"  真实搜索时间: {search_time:.4f}s")
        rospy.loginfo(f"  路径点数: {len(path_points)}")
        rospy.loginfo(f"  有效障碍物数量: {valid_obstacles_count}/{len(matching_obstacles)}")
        rospy.loginfo(f"  速度方差: {velocity_variance:.6f}")
        rospy.loginfo(f"  角速度方差: {angular_velocity_variance:.6f}")
        rospy.loginfo(f"  最小障碍物距离: {min_distance:.4f}m")
        rospy.loginfo(f"  全局平均距离: {avg_distance:.4f}m")
        rospy.loginfo(f"  ISAD评分: {isad_score:.4f}")
        
        # 如果距离为负，给出警告
        if min_distance < 0:
            rospy.logwarn(f"⚠️  检测到碰撞风险！最小安全距离为负值: {min_distance:.4f}m - 标记为失败")
        
        # 达到最大次数时保存结果
        if self.replan_count >= self.max_replans:
            self.save_results()
            success_rate = self.successful_replans / self.max_replans * 100
            rospy.loginfo(f"🎉 已完成 {self.max_replans} 次重规划记录")
            rospy.loginfo(f"📊 成功率: {self.successful_replans}/{self.max_replans} = {success_rate:.1f}%")

    def obstacle_callback(self, msg):
        """障碍物信息回调函数 - 带时间戳缓存"""
        current_time = rospy.Time.now()
        obstacles = self.parse_obstacles(msg)  # 现有的解析逻辑
        
        # 缓存障碍物数据
        self.obstacle_data_buffer[current_time] = obstacles
        
        # 清理过期数据
        self.cleanup_old_obstacle_data(current_time)
        
        # 更新当前障碍物（用于实时显示）
        self.current_obstacles = obstacles

    def parse_obstacles(self, msg):
        """解析障碍物信息"""
        obstacles = []
        data_size = len(msg.data)
        
        # 详细日志输出，帮助调试
        if self.replan_count == 0:  # 只在第一次记录时输出详细信息
            rospy.loginfo(f"障碍物预测数据总长度: {data_size}")
        
        if data_size % 7 == 0:
            total_data_points = data_size // 7
            
            # 根据数据结构解析：10个障碍物 × 20个时间步 = 200个数据点
            num_obstacles = self.num_obstacles  # 固定10个障碍物
            time_steps = self.pre_step     # 每个障碍物20个时间步预测
            
            if total_data_points == num_obstacles * time_steps:
                rospy.loginfo_throttle(5.0, f"解析预测轨迹：{num_obstacles}个障碍物，每个{time_steps}个时间步")
                
                # 提取每个障碍物的完整预测轨迹
                for i in range(num_obstacles):
                    obstacle_trajectory = []
                    
                    # 提取这个障碍物所有时间步的数据
                    for t in range(time_steps):
                        time_idx = i * time_steps * 7 + t * 7
                        
                        if time_idx + 6 < len(msg.data):
                            trajectory_point = {
                                'x': msg.data[time_idx],
                                'y': msg.data[time_idx + 1],
                                'radius_a': msg.data[time_idx + 2],
                                'radius_b': msg.data[time_idx + 3],
                                'theta': msg.data[time_idx + 4],
                                'vx': msg.data[time_idx + 5],
                                'vy': msg.data[time_idx + 6],
                                'radius': max(msg.data[time_idx + 2], msg.data[time_idx + 3]),
                                'time_step': t
                            }
                            obstacle_trajectory.append(trajectory_point)
                    
                    # 将完整轨迹作为一个障碍物对象
                    if obstacle_trajectory:
                        obstacle = {
                            'id': i,
                            'trajectory': obstacle_trajectory,
                            # 当前状态（第0步）用于兼容性
                            'x': obstacle_trajectory[0]['x'],
                            'y': obstacle_trajectory[0]['y'],
                            'radius': obstacle_trajectory[0]['radius'],
                            'vx': obstacle_trajectory[0]['vx'],
                            'vy': obstacle_trajectory[0]['vy']
                        }
                        obstacles.append(obstacle)
            else:
                # 如果数据点数量不符合预期，使用备用解析方法
                rospy.logwarn(f"数据点数量异常: {total_data_points}，期望: {num_obstacles * time_steps}")
                obstacles = self.parse_fallback_method(msg.data, total_data_points)
        else:
            rospy.logwarn_throttle(5.0, f"障碍物数据长度不是7的倍数: {data_size}")
        
        # 输出最终解析的障碍物数量
        if len(obstacles) != len(self.current_obstacles):
            rospy.loginfo(f"障碍物数量更新: {len(self.current_obstacles)} -> {len(obstacles)}")
            
            # 输出前几个障碍物的位置，用于验证
            if len(obstacles) > 0:
                for i in range(min(3, len(obstacles))):
                    obs = obstacles[i]
                    traj_len = len(obs.get('trajectory', []))
                    rospy.loginfo(f"  障碍物{i+1}: 当前位置=({obs['x']:.2f}, {obs['y']:.2f}), 轨迹点数={traj_len}")
        
        return obstacles

    def parse_fallback_method(self, data, total_points):
        """备用解析方法：当数据格式不符合预期时使用"""
        obstacles = []
        
        # 尝试智能识别障碍物数量
        if total_points >= 100:  # 如果数据点很多，可能是多时间步预测
            # 假设时间步数，尝试识别障碍物数量
            possible_time_steps = [10, 15, 20, 25, 30]
            
            for time_step in possible_time_steps:
                if total_points % time_step == 0:
                    num_obs = total_points // time_step
                    if 5 <= num_obs <= 15:  # 合理的障碍物数量范围
                        rospy.loginfo(f"备用方法：识别到{num_obs}个障碍物，{time_step}个时间步")
                        
                        for i in range(num_obs):
                            obstacle_trajectory = []
                            base_idx = i * time_step * 7
                            for t in range(time_step):
                                idx = base_idx + t * 7
                                if idx + 6 < len(data):
                                    trajectory_point = {
                                        'x': data[idx],
                                        'y': data[idx + 1],
                                        'radius_a': data[idx + 2],
                                        'radius_b': data[idx + 3],
                                        'theta': data[idx + 4],
                                        'vx': data[idx + 5],
                                        'vy': data[idx + 6],
                                        'radius': max(data[idx + 2], data[idx + 3]),
                                        'time_step': t
                                    }
                                    obstacle_trajectory.append(trajectory_point)

                            if obstacle_trajectory:
                                obstacle = {
                                    'id': i,
                                    'trajectory': obstacle_trajectory,
                                    'x': obstacle_trajectory[0]['x'],
                                    'y': obstacle_trajectory[0]['y'],
                                    'radius_a': obstacle_trajectory[0]['radius_a'],
                                    'radius_b': obstacle_trajectory[0]['radius_b'],
                                    'theta': obstacle_trajectory[0]['theta'],
                                    'vx': obstacle_trajectory[0]['vx'],
                                    'vy': obstacle_trajectory[0]['vy'],
                                    'radius': obstacle_trajectory[0]['radius']
                                }
                                obstacles.append(obstacle)
                    break
        
        # 如果备用方法也失败，取前N个数据点作为障碍物当前状态
        if len(obstacles) == 0:
            max_obs = min(20, total_points)  # 最多取20个
            rospy.logwarn(f"所有解析方法失败，取前{max_obs}个数据点")
            
            for i in range(max_obs):
                idx = i * 7
                if idx + 6 < len(data):
                    trajectory_point = {
                        'x': data[idx],
                        'y': data[idx + 1],
                        'radius_a': data[idx + 2],
                        'radius_b': data[idx + 3],
                        'theta': data[idx + 4],
                        'vx': data[idx + 5],
                        'vy': data[idx + 6],
                        'radius': max(data[idx + 2], data[idx + 3]),
                        'time_step': 0
                    }
                    obstacle = {
                        'id': i,
                        'trajectory': [trajectory_point],
                        'x': trajectory_point['x'],
                        'y': trajectory_point['y'],
                        'radius_a': trajectory_point['radius_a'],
                        'radius_b': trajectory_point['radius_b'],
                        'theta': trajectory_point['theta'],
                        'vx': trajectory_point['vx'],
                        'vy': trajectory_point['vy'],
                        'radius': trajectory_point['radius']
                    }
                    obstacles.append(obstacle)
        
        return obstacles

    def calculate_path_kinematics(self, path_points):
        """计算路径运动学参数"""
        if len(path_points) < 2:
            return [], []
        
        velocities = []
        angular_velocities = []
        dt = self.time_step
        
        for i in range(len(path_points)):
            path_points[i][4] = i * dt
            
            if i < len(path_points) - 1:
                dx = path_points[i + 1][0] - path_points[i][0]
                dy = path_points[i + 1][1] - path_points[i][1]
                v = np.sqrt(dx**2 + dy**2) / dt
                velocities.append(v)
                
                if v > 1e-6:
                    path_points[i][2] = dx / dt
                    path_points[i][3] = dy / dt
                
                if i < len(path_points) - 2:
                    theta1 = np.arctan2(dy, dx)
                    dx2 = path_points[i + 2][0] - path_points[i + 1][0] 
                    dy2 = path_points[i + 2][1] - path_points[i + 1][1]
                    theta2 = np.arctan2(dy2, dx2)
                    dtheta = theta2 - theta1
                    if dtheta > np.pi:
                        dtheta -= 2 * np.pi
                    elif dtheta < -np.pi:
                        dtheta += 2 * np.pi
                    omega = dtheta / dt
                    angular_velocities.append(omega)
        
        return velocities, angular_velocities

    def calculate_safety_metrics(self, trajectory, obstacles):
        """计算安全距离指标 - 增加障碍物过滤机制"""
        if not trajectory or not obstacles:
            return float('inf'), 1.0, float('inf'), 0
        
        # 障碍物预测时间步数
        obstacle_prediction_steps = self.pre_step
        dt = self.time_step  # 时间步长
        max_prediction_time = obstacle_prediction_steps * dt
        
        # 只考虑有效预测时间范围内的轨迹点
        valid_trajectory = []
        for i, point in enumerate(trajectory):
            point_time = i * dt
            if point_time <= max_prediction_time:
                valid_trajectory.append(point)
            else:
                break
        
        if not valid_trajectory:
            return float('inf'), 1.0, float('inf'), 0
        
        # 障碍物过滤：计算每个障碍物在20步内的平均距离
        valid_obstacles = []
        for obs in obstacles:
            distances_to_obs = []
            
            # 使用障碍物预测轨迹进行过滤
            for i, point in enumerate(valid_trajectory):
                pos = np.array(point[:2])
                
                # 获取对应时间步的障碍物位置
                if 'trajectory' in obs and i < len(obs['trajectory']):
                    obs_point = obs['trajectory'][i]
                    obs_pos = np.array([obs_point['x'], obs_point['y']])
                else:
                    # 回退到当前位置
                    obs_pos = np.array([obs['x'], obs['y']])
                
                center_distance = np.linalg.norm(pos - obs_pos)
                distances_to_obs.append(center_distance)
            
            if distances_to_obs:
                avg_distance_to_obs = np.mean(distances_to_obs)
                if avg_distance_to_obs <= 6.0:  # 平均距离不超过6m的障碍物才计入统计
                    valid_obstacles.append(obs)
                else:
                    rospy.loginfo_throttle(10.0, f"过滤远距离障碍物：平均距离{avg_distance_to_obs:.2f}m > 6.0m")
        
        rospy.loginfo_throttle(10.0, f"安全计算：轨迹总点数={len(trajectory)}, 有效点数={len(valid_trajectory)}")
        rospy.loginfo_throttle(5.0, f"障碍物过滤：原始{len(obstacles)}个 -> 有效{len(valid_obstacles)}个")
        
        if not valid_obstacles:
            return float('inf'), 1.0, float('inf'), 0
        
        min_global_distance = float('inf')
        collision_count = 0
        
        # 全局平均距离计算
        total_distances = []
        
        for i, point in enumerate(valid_trajectory):
            pos = np.array(point[:2])
            point_min_distance = float('inf')
            
            for j, obs in enumerate(valid_obstacles):
                # 使用对应时间步的障碍物位置
                if 'trajectory' in obs and i < len(obs['trajectory']):
                    obs_point = obs['trajectory'][i]
                    obs_pos = np.array([obs_point['x'], obs_point['y']])
                    obs_radius = obs_point['radius']
                else:
                    # 回退到当前位置
                    obs_pos = np.array([obs['x'], obs['y']])
                    obs_radius = obs['radius']
                
                center_distance = np.linalg.norm(pos - obs_pos)
                
                if i < 5 and j < 3:  # 输出前几个点的调试信息
                    rospy.loginfo(f"轨迹点{i+1}(t={i*dt:.1f}s)与障碍物{j+1}(t={i*dt:.1f}s)距离: {center_distance:.3f}m")
                
                # 有效安全距离（考虑物体尺寸）
                effective_distance = center_distance - self.robot_radius - obs_radius
                
                if effective_distance < 0:
                    collision_count += 1
                
                min_global_distance = min(min_global_distance, effective_distance)
                point_min_distance = min(point_min_distance, effective_distance)
            
            if point_min_distance != float('inf'):
                total_distances.append(point_min_distance)
        
        # 计算全局平均距离
        if total_distances:
            global_average_distance = float(np.mean(total_distances))
        else:
            global_average_distance = float('inf')
        
        if collision_count > 0:
            rospy.logwarn(f"检测到 {collision_count} 次潜在碰撞")
        
        # ISAD计算使用有效障碍物
        isad_score, raw_isad = self.calculate_isad(valid_trajectory, valid_obstacles, self.robot_radius)
        
        return min_global_distance, isad_score, global_average_distance, len(valid_obstacles)

    def calculate_isad(self, trajectory, obstacles, robot_radius=0.5):
        """改进的安全平均距离计算 - 只计算有效预测范围内的点"""
        if not trajectory or not obstacles:
            return 1.0, float('inf')
        
        total_weighted_dist = 0.0
        total_weights = 0.0
        
        # 障碍物预测时间步数
        obstacle_prediction_steps = 20
        dt = self.time_step
        max_prediction_time = obstacle_prediction_steps * dt
        
        for i in range(len(trajectory)):
            point_time = i * dt
            if point_time > max_prediction_time:
                break  # 超出有效预测范围，停止计算
            
            pos = trajectory[i][:2]
            vel = trajectory[i][2:4]
            time = trajectory[i][4]
            
            min_dist_at_t = float('inf')
            
            for obs in obstacles:
                # 只在有效预测时间内计算
                if time <= max_prediction_time:
                    obs_pos = self.predict_obstacle_position(obs, time)
                    obs_vel = [obs['vx'], obs['vy']]
                    obs_size = obs['radius']
                    
                    center_distance = np.linalg.norm(np.array(pos) - np.array(obs_pos))
                    effective_distance = center_distance - robot_radius - obs_size
                    
                    # 相对速度权重计算
                    rel_vel = np.array(vel) - np.array(obs_vel)
                    dir_to_obs = (np.array(obs_pos) - np.array(pos))
                    if np.linalg.norm(dir_to_obs) > 1e-6:
                        dir_to_obs /= np.linalg.norm(dir_to_obs)
                        rel_speed_along_dir = np.dot(rel_vel, dir_to_obs)
                        vel_weight = 1.0 / (1.0 + np.exp(-1.0 * rel_speed_along_dir))
                    else:
                        vel_weight = 0.5
                    
                    # 时间权重
                    time_weight = 1.0 / (1.0 + 0.05 * i)
                    
                    # 加权距离
                    safe_distance = max(effective_distance, -1.0)
                    weighted_dist = safe_distance * vel_weight * time_weight
                    
                    if effective_distance < min_dist_at_t:
                        min_dist_at_t = effective_distance
            
            # 累积加权距离
            total_weighted_dist += max(min_dist_at_t, 0.0)
            total_weights += 1
        
        # 计算原始ISAD
        raw_isad = total_weighted_dist / total_weights if total_weights > 0 else 0.0
        
        # 归一化
        if raw_isad >= 2.0:
            normalized_isad = 0.95
        elif raw_isad >= 1.0:
            normalized_isad = 0.8 + 0.15 * (raw_isad - 1.0)
        elif raw_isad >= 0.5:
            normalized_isad = 0.5 + 0.3 * (raw_isad - 0.5) / 0.5
        elif raw_isad >= 0.0:
            normalized_isad = 0.2 + 0.3 * raw_isad / 0.5
        else:
            normalized_isad = 0.1
        
        return normalized_isad, raw_isad

    def predict_obstacle_position(self, obs, target_time):
        """预测障碍物位置"""
        current_time = 0
        time_delta = target_time - current_time
        return [
            obs['x'] + obs['vx'] * time_delta,
            obs['y'] + obs['vy'] * time_delta
        ]

    def calculate_summary_statistics(self):
        """计算汇总统计 - 基于重规划数据，增加成功率统计"""
        if not self.path_data:
            return {}
        
        search_times = [float(p['search_time']) for p in self.path_data]
        velocity_variances = [float(p['velocity_variance']) for p in self.path_data]
        angular_velocity_variances = [float(p['angular_velocity_variance']) for p in self.path_data]
        min_distances = [float(p['min_obstacle_distance']) for p in self.path_data]
        avg_distances = [float(p['global_average_distance']) for p in self.path_data]
        isad_scores = [float(p['isad_score']) for p in self.path_data]
        
        # 只统计成功的重规划数据
        successful_data = [p for p in self.path_data if p['is_successful']]
        
        if successful_data:
            successful_search_times = [float(p['search_time']) for p in successful_data]
            successful_min_distances = [float(p['min_obstacle_distance']) for p in successful_data]
            successful_avg_distances = [float(p['global_average_distance']) for p in successful_data]
            successful_isad_scores = [float(p['isad_score']) for p in successful_data]
        else:
            successful_search_times = [0]
            successful_min_distances = [0]
            successful_avg_distances = [0]
            successful_isad_scores = [0]
        
        return {
            # 总体统计
            'total_replans': len(self.path_data),
            'successful_replans': self.successful_replans,
            'failed_replans': self.failed_replans,
            'success_rate': float(self.successful_replans / max(self.replan_count, 1)),
            
            # 所有重规划的时间统计
            'total_search_time': float(sum(search_times)),
            'average_search_time': float(np.mean(search_times)),
            'max_search_time': float(max(search_times)),
            'min_search_time': float(min(search_times)),
            
            # 只统计成功重规划的性能指标
            'successful_average_search_time': float(np.mean(successful_search_times)),
            'successful_average_min_obstacle_distance': float(np.mean(successful_min_distances)),
            'successful_average_global_average_distance': float(np.mean(successful_avg_distances)),
            'successful_average_isad_score': float(np.mean(successful_isad_scores)),
            
            # 原有的全体统计（包含失败的）
            'average_velocity_variance': float(np.mean(velocity_variances)),
            'max_velocity_variance': float(max(velocity_variances)),
            'min_velocity_variance': float(min(velocity_variances)),
            
            'average_angular_velocity_variance': float(np.mean(angular_velocity_variances)),
            'max_angular_velocity_variance': float(max(angular_velocity_variances)),
            'min_angular_velocity_variance': float(min(angular_velocity_variances)),
            
            'global_min_obstacle_distance': float(min(min_distances)),
            'average_min_obstacle_distance': float(np.mean(min_distances)),
            'max_min_obstacle_distance': float(max(min_distances)),
            
            'average_global_average_distance': float(np.mean(avg_distances)),
            'max_global_average_distance': float(max(avg_distances)),
            'min_global_average_distance': float(min(avg_distances)),
            
            'average_isad_score': float(np.mean(isad_scores)),
            'max_isad_score': float(max(isad_scores)),
            'min_isad_score': float(min(isad_scores))
        }

    def generate_yaml_summary(self, results_data):
        """生成简洁的YAML总结 - 增加成功率统计"""
        metadata = results_data['metadata']
        summary = results_data['summary']
        detailed_data = results_data['detailed_data']
        
        yaml_data = {
            "experiment_info": {
                "timestamp": metadata['timestamp'],
                "robot_radius": metadata['robot_radius'],
                "total_replans_recorded": metadata['total_replans_recorded'],
                "note": "只记录重规划数据，使用真实搜索时间",
                "safety_calculation_note": "安全指标只计算前20个时间步（2秒）内的轨迹点",
                "smoothness_calculation_note": "平滑度指标计算全部轨迹点",
                "obstacle_filtering_note": "过滤平均距离超过6m的障碍物，负距离标记为失败"
            },
            
            "replan_performance_summary": {
                "total_replans": summary['total_replans'],
                "successful_replans": summary['successful_replans'],
                "failed_replans": summary['failed_replans'],
                "success_rate_percentage": round(summary['success_rate'] * 100, 1),
                
                "search_time_metrics": {
                    "all_replans": {
                        "average_seconds": round(summary['average_search_time'], 4),
                        "fastest_seconds": round(summary['min_search_time'], 4),
                        "slowest_seconds": round(summary['max_search_time'], 4),
                        "total_seconds": round(summary['total_search_time'], 4)
                    },
                    "successful_replans_only": {
                        "average_seconds": round(summary['successful_average_search_time'], 4)
                    }
                },
                
                "smoothness_metrics": {
                    "average_velocity_variance": round(summary['average_velocity_variance'], 6),
                    "average_angular_velocity_variance": round(summary['average_angular_velocity_variance'], 6)
                },
                
                "safety_metrics": {
                    "all_replans": {
                        "global_minimum_distance_meters": round(summary['global_min_obstacle_distance'], 4),
                        "average_minimum_distance_meters": round(summary['average_min_obstacle_distance'], 4),
                        "average_global_average_distance_meters": round(summary['average_global_average_distance'], 4),
                        "average_isad_score": round(summary['average_isad_score'], 4)
                    },
                    "successful_replans_only": {
                        "average_minimum_distance_meters": round(summary['successful_average_min_obstacle_distance'], 4),
                        "average_global_average_distance_meters": round(summary['successful_average_global_average_distance'], 4),
                        "average_isad_score": round(summary['successful_average_isad_score'], 4)
                    }
                }
            },
            
            "detailed_replan_data": {}
        }
        
        # 每次重规划的详细数据
        for i, plan_data in enumerate(detailed_data):
            replan_key = f"replan_{i+1}"
            status = "成功" if plan_data['is_successful'] else "失败"
            yaml_data["detailed_replan_data"][replan_key] = {
                "description": f"第{i+1}次重规划 - {status}",
                "status": status,
                "search_time_seconds": round(plan_data['search_time'], 4),
                "velocity_variance": round(plan_data['velocity_variance'], 6),
                "angular_velocity_variance": round(plan_data['angular_velocity_variance'], 6),
                "min_obstacle_distance_meters": round(plan_data['min_obstacle_distance'], 4),
                "global_average_distance_meters": round(plan_data['global_average_distance'], 4),
                "isad_score": round(plan_data['isad_score'], 4),
                "path_length_points": plan_data['path_length'],
                "valid_obstacles_count": plan_data['valid_obstacles_count'],
                "total_obstacles_detected": len(plan_data['obstacles_snapshot'])
            }
        
        return yaml_data

    def save_results(self):
        """保存结果 - 增加成功率记录"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        results = {
            'metadata': {
                'timestamp': timestamp,
                'robot_radius': self.robot_radius,
                'max_replans': self.max_replans,
                'total_replans_recorded': len(self.path_data),
                'successful_replans': self.successful_replans,
                'failed_replans': self.failed_replans,
                'success_rate': self.successful_replans / max(self.replan_count, 1),
                'note': 'Only replan data recorded with real search time from C++ planner'
            },
            'summary': self.calculate_summary_statistics(),
            'detailed_data': []
        }
        
        # 转换数据为可序列化格式
        for path_info in self.path_data:
            serializable_info = path_info.copy()
            serializable_info['timestamp'] = path_info['timestamp'].to_sec()
            
            # 修复：确保所有numpy类型转换为Python原生类型
            serializable_info['is_successful'] = bool(path_info['is_successful'])
            serializable_info['search_time'] = float(path_info['search_time'])
            serializable_info['velocity_variance'] = float(path_info['velocity_variance'])
            serializable_info['angular_velocity_variance'] = float(path_info['angular_velocity_variance'])
            serializable_info['min_obstacle_distance'] = float(path_info['min_obstacle_distance'])
            serializable_info['global_average_distance'] = float(path_info['global_average_distance'])
            serializable_info['isad_score'] = float(path_info['isad_score'])
            serializable_info['valid_obstacles_count'] = int(path_info['valid_obstacles_count'])
            serializable_info['path_length'] = int(path_info['path_length'])
            serializable_info['replan_id'] = int(path_info['replan_id'])
            
            results['detailed_data'].append(serializable_info)
        
        # 保存JSON文件  
        json_filename = f"global_path_analysis_{timestamp}.json"
        json_filepath = os.path.join(self.output_dir, json_filename)
        
        try:
            with open(json_filepath, 'w', encoding='utf-8') as f:
                json.dump(results, f, indent=2, ensure_ascii=False)
            rospy.loginfo(f"📄 JSON数据已保存: {json_filepath}")
        except Exception as e:
            rospy.logerr(f"保存JSON失败: {e}")
            return
        
        # 保存YAML总结文件
        yaml_filename = f"global_path_summary_{timestamp}.yaml"
        yaml_filepath = os.path.join(self.output_dir, yaml_filename)
        
        try:
            yaml_summary = self.generate_yaml_summary(results)
            
            with open(yaml_filepath, 'w', encoding='utf-8') as f:
                # 写入注释
                f.write("# ============================================================================\n")
                f.write("# 全局路径重规划性能分析报告\n") 
                f.write("# ============================================================================\n")
                f.write("#\n")
                f.write("# 指标说明:\n")
                f.write("#   search_time_seconds: 重规划真实搜索时间(秒) - 来自C++规划器\n")
                f.write("#   velocity_variance: 路径速度方差 (m/s)²\n")
                f.write("#   angular_velocity_variance: 路径角速度方差 (rad/s)²\n") 
                f.write("#   min_obstacle_distance_meters: 与障碍物最小安全距离(米)\n")
                f.write("#   global_average_distance_meters: 前20个轨迹点与障碍物平均距离(米)\n")
                f.write("#   isad_score: 改进安全平均距离评分 [0,1]\n")
                f.write("#   success_rate_percentage: 重规划成功率(%) = 成功次数/总次数\n")
                f.write("#\n")
                f.write("# 过滤规则:\n") 
                f.write("#   - 平均距离超过6m的障碍物不计入统计\n")
                f.write("#   - 最小距离为负值时标记为失败重规划\n")
                f.write("#   - 成功率 = 无碰撞重规划次数 / 总重规划次数\n")
                f.write("# ============================================================================\n\n")
                
                yaml.dump(yaml_summary, f, default_flow_style=False, allow_unicode=True, indent=2)
            
            rospy.loginfo(f"📊 YAML总结已保存: {yaml_filepath}")
            
        except Exception as e:
            rospy.logerr(f"保存YAML失败: {e}")

    def run(self):
        """运行节点"""
        self.verify_parameters()
        rospy.spin()

    def find_matching_obstacles(self, target_time):
        """查找最接近目标时间的障碍物数据"""
        if not self.obstacle_data_buffer:
            return None
        
        # 找最接近的时间戳
        closest_time = min(self.obstacle_data_buffer.keys(), 
                         key=lambda t: abs((t - target_time).to_sec()))
        
        time_diff = abs((closest_time - target_time).to_sec())
        
        # 如果时间差太大，返回None
        if time_diff > 0.5:  # 500ms阈值
            return None
        
        return self.obstacle_data_buffer[closest_time]
    
    def cleanup_old_obstacle_data(self, current_time):
        """清理过期的障碍物数据"""
        cutoff_time = current_time - rospy.Duration(self.max_buffer_age)
        
        keys_to_remove = [t for t in self.obstacle_data_buffer.keys() if t < cutoff_time]
        for key in keys_to_remove:
            del self.obstacle_data_buffer[key]

    def verify_parameters(self):
        """验证参数匹配"""
        # 检查机器人半径
        global_planner_radius = rospy.get_param('/obs_manager/robot_radius', 0.4)
        if abs(global_planner_radius - self.robot_radius) > 0.01:
            rospy.logwarn(f"机器人半径不匹配: 全局规划器={global_planner_radius}, 数据处理器={self.robot_radius}")
        
        # 检查安全距离
        safe_dist = rospy.get_param('/local_planner/safe_dist', 0.9)  
        rospy.loginfo(f"全局规划器安全距离: {safe_dist}m")
        
        # 检查时间步长
        step_time = rospy.get_param('/obs_manager/step_time', 0.1)
        rospy.loginfo(f"障碍物预测时间步长: {step_time}s")


if __name__ == '__main__':
    try:
        processor = GlobalPathDataProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass 
