#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试全局路径数据处理器的功能
发布一些模拟的全局路径和障碍物数据
"""

import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

def create_test_path(start_x, start_y, end_x, end_y, num_points=10):
    """创建测试路径"""
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()
    
    # 生成路径点
    for i in range(num_points):
        t = i / (num_points - 1)
        
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        
        # 简单的直线插值，加一些曲线
        x = start_x + t * (end_x - start_x) + 0.5 * np.sin(t * np.pi * 2) * (1 - t)
        y = start_y + t * (end_y - start_y) + 0.3 * np.cos(t * np.pi * 3) * (1 - t)
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        
        path.poses.append(pose)
    
    return path

def create_test_obstacles():
    """创建测试障碍物数据"""
    obstacles = Float32MultiArray()
    
    # 创建3个障碍物，每个障碍物7个参数 [x, y, radius_a, radius_b, theta, vx, vy]
    obs_data = [
        # 障碍物1
        5.0,  # x
        2.0,  # y
        0.5,  # radius_a
        0.5,  # radius_b
        0.0,  # theta
        -0.5, # vx
        0.2,  # vy
        
        # 障碍物2
        8.0,  # x
        -1.0, # y
        0.4,  # radius_a
        0.4,  # radius_b
        0.0,  # theta
        0.3,  # vx
        0.1,  # vy
        
        # 障碍物3
        12.0, # x
        1.5,  # y
        0.6,  # radius_a
        0.6,  # radius_b
        0.0,  # theta
        -0.2, # vx
        -0.3, # vy
    ]
    
    obstacles.data = obs_data
    return obstacles

def main():
    rospy.init_node('test_global_path_processor', anonymous=True)
    
    # 发布器
    path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
    obs_pub = rospy.Publisher('/globalFsm_by_adsm/obs_predict_pub', Float32MultiArray, queue_size=1)
    
    rospy.loginfo("测试全局路径数据处理器...")
    rospy.loginfo("将发布3次不同的路径和障碍物数据")
    
    rate = rospy.Rate(0.5)  # 0.5 Hz，每2秒发布一次
    
    # 测试路径配置
    test_paths = [
        (0.0, 0.0, 10.0, 2.0),   # 路径1: 从(0,0)到(10,2)
        (0.0, 0.0, 15.0, -1.0),  # 路径2: 从(0,0)到(15,-1) 
        (0.0, 0.0, 12.0, 3.0),   # 路径3: 从(0,0)到(12,3)
    ]
    
    # 等待订阅者连接
    rospy.sleep(1.0)
    
    for i, (start_x, start_y, end_x, end_y) in enumerate(test_paths):
        if rospy.is_shutdown():
            break
            
        rospy.loginfo(f"发布第 {i+1} 条测试路径...")
        
        # 先发布障碍物数据
        obstacles = create_test_obstacles()
        obs_pub.publish(obstacles)
        
        rospy.sleep(0.1)  # 短暂延迟
        
        # 再发布路径数据
        path = create_test_path(start_x, start_y, end_x, end_y, 15)
        path_pub.publish(path)
        
        rospy.loginfo(f"路径 {i+1}: ({start_x}, {start_y}) -> ({end_x}, {end_y})")
        rospy.loginfo(f"路径点数: {len(path.poses)}")
        
        # 等待下次发布
        if i < len(test_paths) - 1:  # 不是最后一次
            rate.sleep()
    
    rospy.loginfo("测试完成！等待数据处理器保存结果...")
    rospy.sleep(3.0)  # 等待处理器完成保存
    
    rospy.loginfo("测试结束")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 