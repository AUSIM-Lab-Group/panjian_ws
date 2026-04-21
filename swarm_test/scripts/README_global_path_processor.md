# 全局路径数据处理器使用说明

## 功能描述

`global_path_data_processor.py` 是一个ROS节点，用于记录和分析全局路径重规划的性能指标，提供多维度的路径质量评估。

## 主要功能

1. **监听全局路径**: 订阅 `/global_path` 话题
2. **监听障碍物信息**: 订阅 `/globalFsm_by_adsm/obs_predict_pub` 话题  
3. **监听搜索时间**: 订阅 `/global_planning/search_time` 话题
4. **自动过滤**: 跳过初始规划，只记录重规划数据
5. **智能过滤**: 过滤远距离障碍物，提高分析准确性
6. **成功率统计**: 基于碰撞检测的重规划成功率分析

## 配置参数

- `robot_radius`: 机器人半径(默认: 0.4m)
- `max_replans`: 最大记录的重规划次数(默认: 100次)
- `num_obstacles`: 障碍物数量(默认: 4个)
- `output_dir`: 输出文件目录，默认写入 `swarm_test/output/`

## 核心指标详解

### 1. 搜索时间指标 (Search Time Metrics)

**描述**: 来自C++全局规划器的真实计算时间
- 负值: 初始规划时间(跳过记录)
- 正值: 重规划时间(记录分析)

**统计量**:
- 平均搜索时间: \( \bar{t} = \frac{1}{n}\sum_{i=1}^{n} t_i \)
- 总搜索时间: \( T_{total} = \sum_{i=1}^{n} t_i \)

### 2. 路径平滑度指标 (Smoothness Metrics)

#### 速度方差 (Velocity Variance)
**公式**: 
\[ \sigma_v^2 = \frac{1}{m-1}\sum_{i=1}^{m} (v_i - \bar{v})^2 \]

其中：
- \( v_i = \frac{\sqrt{(x_{i+1}-x_i)^2 + (y_{i+1}-y_i)^2}}{\Delta t} \)
- \( \Delta t = 0.1s \) (时间步长)
- \( m \) 为路径点数量

#### 角速度方差 (Angular Velocity Variance)
**公式**:
\[ \sigma_\omega^2 = \frac{1}{k-1}\sum_{i=1}^{k} (\omega_i - \bar{\omega})^2 \]

其中：
- \( \omega_i = \frac{\theta_{i+1} - \theta_i}{\Delta t} \)
- \( \theta_i = \arctan2(y_{i+1}-y_i, x_{i+1}-x_i) \)
- 角度差异经过 \([-\pi, \pi]\) 归一化处理

### 3. 安全距离指标 (Safety Metrics)

#### 最小障碍物距离 (Minimum Obstacle Distance)
**定义**: 轨迹点与障碍物间的最小有效安全距离

**公式**:
\[ d_{min} = \min_{i,j} (d_{center}(p_i, obs_j) - r_{robot} - r_{obs_j}) \]

其中：
- \( d_{center}(p_i, obs_j) = \|p_i - obs_j\|_2 \) (中心距离)
- \( r_{robot} \): 机器人半径
- \( r_{obs_j} \): 障碍物j的半径
- 负值表示潜在碰撞

#### 全局平均距离 (Global Average Distance)
**描述**: 前20个轨迹点与有效障碍物的平均最小距离

**公式**:
\[ d_{avg} = \frac{1}{n_{valid}} \sum_{i=1}^{n_{valid}} \min_j (d_{center}(p_i, obs_j) - r_{robot} - r_{obs_j}) \]

其中 \( n_{valid} = \min(20, n_{total}) \) (有效轨迹点数)

### 4. ISAD评分 (Improved Safety Average Distance)

**描述**: 综合考虑距离、相对速度和时间衰减的安全评分

**核心公式**:
\[ ISAD = \frac{\sum_{i=1}^{n_{valid}} w_{time}(i) \cdot \max(d_{min}(i), 0)}{\sum_{i=1}^{n_{valid}} w_{time}(i)} \]

#### 速度权重计算
\[ w_{vel}(i,j) = \frac{1}{1 + e^{-\vec{v}_{rel} \cdot \hat{d}_{ij}}} \]

其中：
- \( \vec{v}_{rel} = \vec{v}_{obs_j} - \vec{v}_{robot_i} \) (相对速度向量)
- \( \hat{d}_{ij} = \frac{\vec{obs_j} - \vec{p_i}}{\|\vec{obs_j} - \vec{p_i}\|} \) (方向单位向量)

#### 时间权重计算
\[ w_{time}(i) = \frac{1}{1 + 0.05 \cdot i} \]

#### 归一化映射

## 输出格式

数据保存为JSON格式，包含：

### 元数据 (metadata)
- 时间戳
- 机器人半径
- 最大重规划次数
- 实际记录的重规划次数

### 汇总统计 (summary)
- 总搜索时间、平均搜索时间、最大/最小搜索时间
- 平均/最大/最小速度方差
- 平均/最大/最小角速度方差
- 全局最小障碍物距离、平均最小障碍物距离
- 平均/最大/最小ISAD评分

### 详细数据 (detailed_data)
每次重规划的详细记录：
- 重规划ID
- 时间戳
- 搜索时间
- 路径长度
- 速度方差
- 角速度方差
- 最小障碍物距离
- ISAD评分
- 路径点坐标
- 障碍物快照

## 使用方法

1. 在launch文件中添加节点(已在start_test.launch中添加)
2. 启动系统:
   ```bash
   roslaunch swarm_test start_test.launch
   ```
3. 当完成 n 次路径重规划后，数据会自动保存到 `swarm_test/output/` 目录，或者保存到 launch 中显式指定的 `output_dir`

## ISAD指标说明

改进的安全平均距离(Improved Safety Average Distance)考虑：
- 机器人与障碍物的实际距离
- 相对速度权重
- 时间衰减权重
- 归一化到0-1之间的安全评分

越接近1表示越安全，越接近0表示越危险。

## 输出文件示例

```json
{
  "metadata": {
    "timestamp": "20231201_143022",
    "robot_radius": 0.3,
    "max_replans": 3,
    "total_replans_recorded": 3
  },
  "summary": {
    "total_search_time": 0.456,
    "average_search_time": 0.228,
    "average_velocity_variance": 0.025,
    "global_min_obstacle_distance": 0.234,
    "average_isad_score": 0.842
  },
  "detailed_data": [...]
}
``` 
