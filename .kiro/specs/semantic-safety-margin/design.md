# Semantic Safety Margin (β) — Technical Design

## 1. System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ROS Node Graph                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │ D435 Camera  │    │  RSLidar     │    │  FAST-LIO            │  │
│  │ /camera/     │    │ /rslidar_    │    │ /Odometry            │  │
│  │  color/image │    │  points      │    │ /fastLIO/non_ground  │  │
│  └──────┬───────┘    └──────┬───────┘    └──────────┬───────────┘  │
│         │                   │                       │               │
│         ▼                   ▼                       │               │
│  ┌──────────────┐    ┌──────────────┐              │               │
│  │ yolo_node    │    │ ri_dbscan +  │              │               │
│  │ (Python/GPU) │    │ l_shape +    │              │               │
│  │              │    │ prediction   │              │               │
│  │ /yolo/       │    │ /clustering/ │              │               │
│  │  detections  │    │  cluster_arr │              │               │
│  └──────┬───────┘    └──────┬───────┘              │               │
│         │                   │                       │               │
│         ▼                   ▼                       │               │
│  ┌─────────────────────────────────┐               │               │
│  │   semantic_fusion_node (C++)    │               │               │
│  │   IoU + Hungarian matching      │               │               │
│  │   → /semantic_obstacles         │               │               │
│  └──────────────┬──────────────────┘               │               │
│                 │                                   │               │
│                 ▼                                   │               │
│  ┌─────────────────────────────────┐               │               │
│  │   beta_guard_node (C++)         │◄──────────────┘               │
│  │   β̂ = β̄(c) × μ(φ)             │                               │
│  │   Guard: β̂ ≤ h_EE - η          │                               │
│  │   → /safety_margin/beta         │                               │
│  └──────────────┬──────────────────┘                               │
│                 │                                                   │
│                 ▼                                                   │
│  ┌─────────────────────────────────┐                               │
│  │   mpc_secbf_node (C++/CasADi)  │◄── /Odometry                  │
│  │   independent MPC-SECBF        │◄── /global_path               │
│  │   per-obstacle β_i in CBF       │                               │
│  │   → /cmd_vel                    │                               │
│  └─────────────────────────────────┘                               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## 2. Component Design

### 2.1 yolo_node (新增 Python 节点)

**包**: `perception/semantic_detection/`

**职责**: RGB 图像 → YOLOv8n → 2D 检测结果

```python
# 伪代码
class YoloNode:
    def __init__(self):
        self.model = YOLO("yolov8n.pt")  # ultralytics
        self.sub_image = rospy.Subscriber("/camera/color/image_raw", Image, self.cb)
        self.pub_det = rospy.Publisher("/yolo/detections", Detection2DArray)

    def cb(self, msg):
        img = bridge.imgmsg_to_cv2(msg)
        results = self.model(img, conf=0.5, classes=[0,1,2,5,7])
        # 0=person, 1=bicycle, 2=car, 5=bus, 7=truck
        # 映射: person→pedestrian, small_person→child, box→box, car/bus/truck→vehicle
        det_msg = self.results_to_ros(results)
        self.pub_det.publish(det_msg)
```

**输入**: `/camera/color/image_raw` (sensor_msgs/Image, 640×480, 10Hz)

**输出**: `/yolo/detections` (vision_msgs/Detection2DArray)

**延迟目标**: ≤ 30ms (GPU)

---

### 2.2 semantic_fusion_node (新增 C++ 节点)

**包**: `perception/semantic_fusion/`

**职责**: 视觉 2D bbox + LiDAR 3D 聚类 → 语义障碍物

**算法**:
1. 将 LiDAR 聚类中心投影到图像平面（使用 camera-lidar 外参 + 内参）
2. 计算投影点与 YOLO bbox 的 IoU
3. 匈牙利匹配（代价 = 1 - IoU）
4. 匹配成功：赋予语义类别；失败：标记 unknown
5. 计算上下文特征 φ

```cpp
struct SemanticObstacle {
    uint32_t id;
    std::string semantic_class;  // pedestrian/child/cyclist/vehicle/box/unknown
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double radius;
    // 上下文特征
    double heading_factor;  // cos(angle between relative_vel and relative_pos)
    double ttc_norm;        // ||p_rel|| / max(||v_rel · p_hat||, eps)
    double density_norm;    // 半径 3m 内障碍物数量 / max_density
};
```

**上下文特征计算**:
```cpp
// 迎面指标 f_head ∈ [0, 1]
// 1 = 正面迎来, 0 = 远离
Eigen::Vector2d p_rel = obs.position.head<2>() - robot_pos.head<2>();
Eigen::Vector2d v_rel = obs.velocity.head<2>() - robot_vel.head<2>();
double cos_angle = p_rel.normalized().dot(v_rel.normalized());
double f_head = std::max(0.0, -cos_angle);  // 负号：v_rel 指向机器人时为正

// TTC (归一化到 [0, 1])
double closing_speed = std::max(-p_rel.normalized().dot(v_rel), 0.0);
double ttc_raw = (closing_speed > 0.01) ? p_rel.norm() / closing_speed : 10.0;
double ttc_norm = std::max(0.0, 1.0 - ttc_raw / 5.0);  // 5s 内归一化

// 局部密度 (归一化到 [0, 1])
int count_nearby = count_obstacles_within_radius(3.0);
double density = std::min(1.0, count_nearby / 5.0);  // 5 个为满密度
```

**输入**:
- `/yolo/detections` (Detection2DArray)
- `/clustering/cluster_array` (jsk_recognition_msgs/BoundingBoxArray)
- `/Odometry` (nav_msgs/Odometry)
- `/camera_info` (sensor_msgs/CameraInfo)
- TF: camera_init → camera_color_optical_frame

**输出**: `/semantic_obstacles` (自定义 SemanticObstacleArray)

**延迟目标**: ≤ 5ms

---

### 2.3 beta_guard_node (新增 C++ 节点)

**包**: `planner/semantic_guard/` (独立包，与 mpc_dcbf 解耦)

**职责**: β 计算 + Guard 安全审查 + 发布到 MPC

**算法**:
```cpp
class BetaGuardNode {
    // 配置
    std::map<std::string, double> beta_bar_;  // 各类别 β̄
    double w_head_, w_ttc_, w_density_, w_bias_;  // μ 权重
    double eta_;  // Guard 余量
    double max_delta_beta_;  // 最大单步变化

    // 状态
    std::map<uint32_t, double> beta_prev_;  // 上一时刻各障碍物 β (按 ID 索引)

    void semanticObsCb(const SemanticObstacleArray& msg) {
        std::vector<double> beta_out;

        for (auto& obs : msg.obstacles) {
            // Step 1: 计算 β̂
            double beta_bar = beta_bar_[obs.semantic_class];
            double mu = std::clamp(
                w_bias_ + w_head_ * obs.heading_factor
                        + w_ttc_ * obs.ttc_norm
                        + w_density_ * obs.density_norm,
                0.0, 1.0);
            double beta_hat = beta_bar * mu;

            // Step 2: Guard 检查
            // h_EE = ||p_rel + tau * v_rel|| - R_obs - R_robot
            Eigen::Vector2d lookahead_rel_pos = p_rel + tau_ * v_rel;
            double h_ee = lookahead_rel_pos.norm()
                          - obs.radius - robot_radius_;
            bool guard_pass = (beta_hat <= h_ee - eta_);

            // Step 3: 回退逻辑
            double beta_final;
            if (guard_pass) {
                // 限制单步变化
                double beta_prev_i = get_prev_beta(obs.id);
                double delta = beta_hat - beta_prev_i;
                delta = std::clamp(delta, -max_delta_beta_, max_delta_beta_);
                beta_final = beta_prev_i + delta;
            } else {
                beta_final = get_prev_beta(obs.id);  // 回退
            }

            beta_out.push_back(beta_final);
            update_prev_beta(obs.id, beta_final);
        }

        publish_beta(beta_out);
        publish_guard_log(...);
    }
};
```

**输入**:
- `/semantic_obstacles` (SemanticObstacleArray)
- `/Odometry` (nav_msgs/Odometry)

**输出**:
- `/safety_margin/beta` (std_msgs/Float32MultiArray)
- `/safety_margin/guard_log` (自定义 GuardLog)

**延迟目标**: ≤ 2ms

---

### 2.4 MPC-SECBF (新建独立包 planner/mpc_secbf/)

**包**: `planner/mpc_secbf/` — 与 `mpc_dcbf` 并列，完全独立实现

**设计思路**: 从 `mpc_dcbf` fork 核心求解框架（CasADi Opti + 运动学模型），但 CBF 约束部分重写为 per-obstacle β 版本。

**核心改动（相对于 mpc_dcbf）**:

```cpp
// mpc_secbf.h — 新的求解器类
class MPC_SECBF_SOLVE {
    // 从 /safety_margin/beta 接收 per-obstacle β
    std::vector<double> beta_list_;

    // 统一安全函数定义:
    //   h_EE  = ||p_obs_pred - p_robot|| - R_obs - R_robot
    //   h_SEE = h_EE - β_i                         ← 语义增强安全函数
    //
    // Guard: β̂ ≤ h_EE - η                         ← Guard 审的是 h_EE
    // MPC:   h_SEE_{k+1} ≥ (1-γ) × h_SEE_k        ← MPC 约束用 h_SEE

    // 语义增强 CBF 函数
    casadi::MX h_secbf(casadi::MX& _curpos, Eigen::VectorXd _obs, double beta_i);

    // 安全约束设置
    void set_secbf_constraint(casadi::Opti& opt, int obs_index);

    // 求解
    bool solve(Eigen::VectorXd* cur_state, Eigen::MatrixXd* goal_state,
               Eigen::MatrixXd* obs_matrix);
};

// h_secbf: 统一安全函数
// h_EE  = ||p_obs_pred - p_robot|| - R_obs - R_robot
// h_SEE = h_EE - β_i
casadi::MX MPC_SECBF_SOLVE::h_secbf(casadi::MX& _curpos, Eigen::VectorXd _obs, double beta_i) {
    casadi::MX dx = _obs(0) - _curpos(0);
    casadi::MX dy = _obs(1) - _curpos(1);
    // h_EE = ||p_obs_pred - p_robot|| - R_obs - R_robot
    casadi::MX h_EE = casadi::MX::sqrt(dx*dx + dy*dy) - _obs(2) - robot_radius_;
    // h_SEE = h_EE - β_i  (β 是在 EESM 安全裕度上再扣的语义余量)
    casadi::MX h_SEE = h_EE - beta_i;
    return h_SEE;
}

// SECBF 约束
void MPC_SECBF_SOLVE::set_secbf_constraint(casadi::Opti& opt, int obs_index) {
    double beta_i = (obs_index < beta_list_.size()) ? beta_list_[obs_index] : beta_bar_unknown_;  // beta_bar_unknown_ 默认 0.4，从 YAML 加载
    for(int i = 0; i < N_s-1; i++) {
        casadi::MX X_   = X_k(casadi::Slice(), i);
        casadi::MX X_1  = X_k(casadi::Slice(), i+1);
        casadi::MX hk  = h_secbf(X_,  obs_matrix_->col(obs_index*N_s+i),  beta_i);
        casadi::MX hk1 = h_secbf(X_1, obs_matrix_->col(obs_index*N_s+i+1), beta_i);
        casadi::MX cbf = -hk1 + (1-gamma_)*hk;
        opt.subject_to(cbf <= 0);
    }
}
```

**ROS 节点 (mpc_secbf_node.cpp)**:
- 订阅: `/Odometry`, `/global_path`, `/safety_margin/beta`, `/globalFsm_by_adsm/obs_predict_pub`
- 发布: `/cmd_vel`, `/local_path`
- 障碍物预测位置来自 `/globalFsm_by_adsm/obs_predict_pub`，β 来自 `/safety_margin/beta`

**与 mpc_dcbf 的关系**:
- `mpc_dcbf` 保持原样，作为对比基线 B1
- `mpc_secbf` 是新方法，独立编译、独立运行
- 对比实验通过 launch 文件切换启动哪个节点

---

## 3. Data Flow

```
时间线 (每 100ms 一个周期):
────────────────────────────────────────────────────────────────────

t=0ms   D435 image arrives
        RSLidar scan arrives

t=0-30ms  [GPU] yolo_node: image → detections (30ms)
t=0-10ms  [CPU] ri_dbscan + l_shape: pointcloud → clusters (10ms)

t=30ms  [CPU] semantic_fusion_node: detections + clusters → semantic_obs (5ms)

t=35ms  [CPU] beta_guard_node: semantic_obs + odom → β values (2ms)

t=37ms  [CPU] mpc_node_c: β + obs_predict + odom + global_path → cmd_vel (40-60ms)

t=97ms  cmd_vel published → 底盘执行
────────────────────────────────────────────────────────────────────
总延迟: 30 + 5 + 2 + 60 = 97ms (worst case)
实际: YOLO 和 DBSCAN 并行 → 30 + 5 + 2 + 40 = 77ms (typical)
```

**关键优化**: YOLO (GPU) 和 DBSCAN (CPU) 并行执行，不串行等待。fusion 节点等两者都到齐后再融合。

---

## 4. Package Structure (新增)

```
panjian_ws/
├── perception/
│   ├── semantic_detection/          # 新增: YOLO 节点
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── scripts/
│   │   │   └── yolo_node.py
│   │   ├── config/
│   │   │   └── yolo_config.yaml
│   │   ├── launch/
│   │   │   └── yolo.launch
│   │   └── models/
│   │       └── yolov8n.pt
│   │
│   └── semantic_fusion/             # 新增: 融合节点
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/semantic_fusion/
│       │   ├── fusion_node.h
│       │   └── projection.h
│       ├── src/
│       │   ├── fusion_node.cpp
│       │   └── projection.cpp
│       ├── msg/
│       │   ├── SemanticObstacle.msg
│       │   └── SemanticObstacleArray.msg
│       └── launch/
│           └── fusion.launch
│
├── planner/
│   ├── semantic_guard/              # 新增: β 计算 + Guard 审查
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/semantic_guard/
│   │   │   └── beta_guard.h
│   │   ├── src/
│   │   │   └── beta_guard_node.cpp
│   │   ├── msg/
│   │   │   └── GuardLog.msg
│   │   ├── config/
│   │   │   └── semantic_safety_margin.yaml
│   │   └── launch/
│   │       └── beta_guard.launch
│   │
│   ├── mpc_secbf/                   # 新增: MPC-SECBF 控制器 (与 mpc_dcbf 并列)
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/mpc_secbf/
│   │   │   ├── mpc_secbf.h         # MPC-SECBF 求解器
│   │   │   └── mpc_secbf_planner.h # 规划器主类
│   │   ├── src/
│   │   │   ├── mpc_secbf.cpp       # CasADi 求解 + h_secbf + per-obstacle β
│   │   │   └── mpc_secbf_node.cpp  # ROS 节点入口
│   │   └── launch/
│   │       └── mpc_secbf.launch
│   │
│   └── mpc_dcbf/                    # 原有，不改动 (作为对比基线 B1)
│       ├── ...（保持原样）
│
└── swarm_test/
    ├── launch/
    │   └── secbf_planner.launch    # 新增: 数值仿真用
    ├── launch_exp/
    │   └── exp_secbf_planner.launch # 新增: 实车用
    └── config/
        └── secbf_scenarios.yaml    # 新增: 4 个场景配置
```

---

## 5. Message Definitions

### SemanticObstacle.msg
```
uint32 id
string semantic_class
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 radius
float64 beta_hat
float64 heading_factor
float64 ttc_norm
float64 density_norm
bool guard_passed
```

### SemanticObstacleArray.msg
```
std_msgs/Header header
SemanticObstacle[] obstacles
```

### GuardLog.msg
```
std_msgs/Header header
uint32[] obstacle_ids
float64[] beta_requested
float64[] beta_applied
float64[] h_ee_values
bool[] guard_passed
uint32 total_rollbacks
```

---

## 6. Camera-LiDAR Calibration

**外参** (D435 相对于 LiDAR 的位姿):
- 通过 TF 树获取: `camera_color_optical_frame` → `velodyne_link` (或 `body`)
- 实车需要标定一次，写入 `static_transform_publisher`

**内参** (D435):
- 从 `/camera/color/camera_info` 获取 K 矩阵
- 投影公式: `[u, v, 1]^T = K × [R|t] × [X, Y, Z, 1]^T`

---

## 7. Simulation Support

### Gazebo 中的语义障碍物

在 `dynamic_simulator` 的 `obstacles_param_gazebo.yaml` 中新增 `semantic_class` 字段:

```yaml
obstacles:
  - {x: 5.0, y: 0.0, z: 0.75, ..., semantic_class: "pedestrian"}
  - {x: 8.0, y: 2.0, z: 0.5,  ..., semantic_class: "child"}
  - {x: 3.0, y: -1.0, z: 0.3, ..., semantic_class: "box"}
```

仿真模式下，`semantic_fusion_node` 可以直接从 ground truth 获取类别（通过 `_perception_GroundTruth` 参数），跳过 YOLO 推理。

---

## 8. Theoretical Verification Design

### Safety Lower Bound 验证

**定理**: 在 Guard 机制下，闭环系统满足:
```
inf_t h(X_t, obs_i) ≥ -(ε_max + Δ̄β) / γ
```

> **注**: 此为工程验证用的近似常数下界。论文中使用 v7 H 节的严格形式（涉及多障碍物累积影响、ε_k 时变分布和 Guard 回退可行性引理的联合证明）。

其中:
- ε_max: MPC 离散化误差上界
- Δ̄β: 单步最大 β 变化量 (配置参数 `max_delta_beta`)
- γ: CBF 衰减率

**验证方式**:
1. 运行实验，记录所有 h_EE 值到 CSV
2. 离线脚本计算 `min(h_EE)` 和理论下界 `-(ε_max + Δ̄β) / γ`
3. 验证 `min(h_EE) > 理论下界`

```python
# scripts/verify_safety_bound.py
import pandas as pd
import numpy as np

df = pd.read_csv("guard_log.csv")
h_min = df["h_ee"].min()
gamma = 0.35
eps_max = 0.05  # 离散化误差 (从 MPC 步长估计)
delta_bar_beta = 0.3  # 配置值
theoretical_bound = -(eps_max + delta_bar_beta) / gamma

print(f"min(h_EE) = {h_min:.4f}")
print(f"Theoretical bound = {theoretical_bound:.4f}")
print(f"Safety guaranteed: {h_min > theoretical_bound}")
```

---

## 9. Backward Compatibility

| 场景 | 行为 |
|------|------|
| `mpc_dcbf` legacy modes 0-4 | 完全不变，不订阅 `/safety_margin/beta`，使用原有固定安全余量逻辑 |
| `mpc_secbf_node` 但 β 话题无数据 | 回退到 β = β̄(unknown) = 0.4，总安全边界为 R_obs + R_robot + 0.4 |
| 无 D435 相机 | `semantic_fusion_node` 回退到纯 LiDAR 模式，所有障碍物 class=unknown |
| `yolo_node` 崩溃 | fusion 节点 3 秒无 YOLO 输入后自动回退 unknown |
