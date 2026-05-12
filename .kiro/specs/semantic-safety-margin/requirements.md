# Semantic Safety Margin (β) — Requirements

## 1. Overview

**Feature**: 在 EESM-MPC-ECBF 框架上新增语义安全余量 β 模块，使机器人对不同类别障碍物（行人、儿童、纸箱、车辆）保持差异化安全距离，并保证闭环安全。

**研究目标**: 证明在线 β 参数更新不破坏 practical safety 保证，同时实现感知→语义→控制的端到端管道（CPU+GPU 总延迟 < 70ms）。

---

## 2. Functional Requirements

### 2.1 第一层：语义感知与融合

| ID | EARS 格式需求 |
|----|--------------|
| FR-1.1 | **When** RGB-D 图像到达 **the system shall** 使用 YOLOv8n 检测障碍物并输出语义类别 c ∈ {pedestrian, child, cyclist, vehicle, box, unknown} 和 2D bounding box |
| FR-1.2 | **When** LiDAR 点云到达 **the system shall** 使用现有 DBSCAN+L-shape 管道输出障碍物位置 p、速度 v、半径 R |
| FR-1.3 | **When** 视觉检测和 LiDAR 聚类同时可用 **the system shall** 通过 IoU + 匈牙利匹配将语义类别关联到 LiDAR 障碍物 |
| FR-1.4 | **When** 匹配成功 **the system shall** 输出完整语义障碍物消息：{id, class c, position p, velocity v, radius R, context φ} |
| FR-1.5 | **When** 视觉检测不可用（相机故障/遮挡）**the system shall** 回退到纯 LiDAR 模式，所有障碍物标记为 class=unknown |
| FR-1.6 | **The system shall** 计算上下文特征 φ = {迎面指标 f_head, TTC, 局部密度 ρ} |

### 2.2 第二层：语义→安全参数映射

| ID | EARS 格式需求 |
|----|--------------|
| FR-2.1 | **When** 语义障碍物消息到达 **the system shall** 计算 β̂ = β̄(c) × μ(φ) |
| FR-2.2 | **The system shall** 使用解析公式 μ = clip(0.6 + 0.2×f_head + 0.15×TTC_norm + 0.1×ρ_norm, 0, 1)，不依赖任何学习模型 |
| FR-2.3 | **The system shall** 从 YAML 配置文件加载各类别基础安全余量 β̄(c)，默认值（v7 标准）：pedestrian=0.4m, child=0.7m, cyclist=0.6m, vehicle=0.5m, box=0.1m, unknown=0.4m |
| FR-2.4 | **When** β̂ 计算完成 **the system shall** 发布 per-obstacle 的 β 值到 MPC 求解器 |

### 2.3 第三层：Guard 审查 + MPC-SECBF

| ID | EARS 格式需求 |
|----|--------------|
| FR-3.1 | **When** β̂ 更新到达 **the system shall** 执行 Guard 检查：β̂_i ≤ h_EE(X_t, obs_i) - η |
| FR-3.2 | **If** Guard 检查失败 **the system shall** 回退到 β_{t-1}（上一时刻的安全余量），不使用新 β̂ |
| FR-3.3 | **When** Guard 通过 **the system shall** 将 β_i 替换 MPC 中原来的固定 safe_dist，作为 per-obstacle CBF 约束参数 |
| FR-3.4 | **The system shall** 支持 controller_type=5 (MPC-SECBF) 作为新的控制器模式，与现有 0-4 模式并列 |
| FR-3.5 | **When** controller_type ≠ 5 **the system shall** 保持原有 DCBF/ACBF 行为不变（向后兼容） |
| FR-3.6 | **When** MPC-SECBF 求解 infeasible **the system shall** 回退到 DCBF fallback（复用现有机制），并发零速停车 |

### 2.4 理论验证

| ID | EARS 格式需求 |
|----|--------------|
| FR-4.1 | **The system shall** 在每个 MPC 周期记录 h_EE(X_t, obs_i)、β_i、Guard 结果到日志文件 |
| FR-4.2 | **The system shall** 提供离线脚本验证 inf_t h ≥ -(ε_max + Δ̄β) / γ 是否被违反（注：工程验证用此近似下界；论文中使用 v7 H 节的严格形式，涉及多障碍物累积影响和 ε_k 分布） |
| FR-4.3 | **The system shall** 记录 Guard 回退次数、回退原因、回退前后 β 差值 |

### 2.5 实验场景

| ID | EARS 格式需求 |
|----|--------------|
| FR-5.1 | **The system shall** 支持 4 个社交导航场景：(S1) 行人对向穿越, (S2) 儿童突然出现, (S3) Feasibility-Critical（1 个 child 迎面 1.5m/s，初始 5m，验证 Guard 在 h_EE≈0.4 时触发回退）, (S4) 混合场景（行人+车辆+纸箱） |
| FR-5.2 | **The system shall** 在每个场景中记录：到达时间、最小障碍物距离、平均速度、CBF 值时序、β 值时序、Guard 回退次数 |
| FR-5.3 | **The system shall** 支持 3 种对比基线：(B1) 原 ACBF 固定 safe_dist, (B2) MPC-SECBF 无 Guard, (B3) MPC-SECBF 有 Guard（本方法） |

---

## 3. Non-Functional Requirements

| ID | 需求 |
|----|------|
| NFR-1 | YOLOv8n 推理延迟 ≤ 30ms（GPU，640×480 输入） |
| NFR-2 | 融合模块延迟 ≤ 5ms |
| NFR-3 | β 计算 + Guard 检查延迟 ≤ 2ms |
| NFR-4 | MPC-SECBF 求解延迟 ≤ 60ms（含 per-obstacle β 约束） |
| NFR-5 | 端到端管道（感知→β→MPC）总延迟 < 70ms |
| NFR-6 | 系统在 10Hz 频率下稳定运行，无内存泄漏 |
| NFR-7 | 仿真（Gazebo）和实车（Scout+D435+RSLidar）使用同一套代码，通过 launch 参数切换 |
| NFR-8 | 新增模块不破坏现有 controller_type 0-4 的功能 |
| NFR-9 | β̄(c) 配置文件可热加载（rosparam / dynamic_reconfigure） |

---

## 4. Interface Requirements

### 4.1 新增 ROS Topic

| Topic | 类型 | 发布者 | 订阅者 | 频率 |
|-------|------|--------|--------|------|
| `/semantic_obstacles` | 自定义 msg (SemanticObstacleArray) | 融合节点 | β 计算节点, MPC | 10Hz |
| `/safety_margin/beta` | std_msgs/Float32MultiArray | β 计算节点 | MPC-SECBF | 10Hz |
| `/safety_margin/guard_log` | 自定义 msg (GuardLog) | Guard 节点 | 数据记录 | 10Hz |
| `/yolo/detections` | vision_msgs/Detection2DArray | YOLO 节点 | 融合节点 | 10Hz |

### 4.2 新增自定义消息

```
# SemanticObstacle.msg
uint32 id
string semantic_class        # pedestrian/child/cyclist/vehicle/box/unknown
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 radius
float64 beta                 # 语义安全余量
float64 ttc                  # time-to-collision
float64 heading_factor       # 迎面指标 [0,1]
float64 density_norm         # 局部密度 (归一化)
bool guard_passed            # Guard 是否通过
```

### 4.3 与现有系统的接口改造

| 现有接口 | 改造方式 |
|----------|----------|
| `mpc_cbf.cpp` 中 `safe_dist = 0.3 + 0.4` | 当 controller_type=5 时，从 `/safety_margin/beta` 读取 per-obstacle β_i 替换 safe_dist |
| `obs_matrix_` (7×N 矩阵) | 扩展为 8×N，第 8 行存 β_i |
| `set_safety_st()` 中 `h1/h2/h3` 函数 | 新增 `h_secbf()` 使用 β_i 替代固定 safe_dist |
| `exp_acbf_planner_use.launch` | 新增 controller=5 选项 + β 相关参数 |

---

## 5. Configuration Requirements

```yaml
# config/semantic_safety_margin.yaml
semantic_safety:
  enabled: true
  controller_type: 5  # MPC-SECBF

  # 各类别基础安全余量 β̄(c) [meters] — v7 标准值
  beta_bar:
    pedestrian: 0.4
    child: 0.7
    cyclist: 0.6
    vehicle: 0.5
    box: 0.1
    unknown: 0.4

  # 上下文调制系数
  mu_weights:
    heading: 0.2
    ttc: 0.15
    density: 0.1
    bias: 0.6

  # Guard 参数
  guard:
    eta: 0.1          # Guard 余量 η
    max_delta_beta: 0.3  # 单步最大 β 变化量 Δ̄β

  # CBF 参数
  cbf:
    gamma: 0.35
    tau_scale: 0.30

  # YOLO 配置
  yolo:
    model_path: "yolov8n.pt"
    confidence_threshold: 0.5
    input_size: [640, 480]
    device: "cuda:0"
```

---

## 6. Acceptance Criteria

| AC | 验收条件 |
|----|----------|
| AC-1 | 数值仿真中，4 个场景的 MPC-SECBF 到达率 ≥ 95%，且 min(h) > -(ε_max + Δ̄β)/γ |
| AC-2 | Gazebo 仿真中，行人场景的最小距离 > β̄(pedestrian)×0.8，纸箱场景的最小距离 > β̄(box)×0.8 |
| AC-3 | 端到端延迟实测 < 70ms（在实车 GPU 上） |
| AC-4 | Guard 回退机制在 β 突变时正确触发（可通过人为制造突变测试） |
| AC-5 | controller_type=0-4 的行为与改动前完全一致（回归测试） |
| AC-6 | 无 Guard 版本 (B2) 在 S3 场景中出现 h < 0（证明 Guard 的必要性） |
| AC-7 | 本方法 (B3) 在所有场景中 h 始终 > -(ε_max + Δ̄β)/γ（证明理论下界） |

---

## 7. Out of Scope

- 深度学习训练 β 映射（本方法用解析公式）
- 多机器人协同避障
- 3D 障碍物检测（只用 2D bbox + LiDAR 3D 位置融合）
- 语义地图构建
- 非差速底盘（阿克曼转向等）

---

## 8. Dependencies

| 依赖 | 版本 | 用途 |
|------|------|------|
| ultralytics (YOLOv8) | ≥ 8.0 | 语义检测 |
| torch + torchvision | ≥ 1.10 | GPU 推理 |
| cv_bridge | ROS Noetic | ROS↔OpenCV 图像转换 |
| vision_msgs | ROS Noetic | 检测结果消息类型 |
| Intel RealSense D435 | — | RGB-D 输入（实车） |
| Gazebo RGB camera plugin | — | RGB 输入（仿真） |

---

## 9. Risks & Mitigations

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| YOLOv8 推理超时 | 整体延迟 > 70ms | 降分辨率 320×240 或用 TensorRT 加速 |
| 视觉-LiDAR 融合误匹配 | β 分配给错误障碍物 | 加 temporal consistency 检查（连续 3 帧一致才更新类别） |
| Guard 频繁回退 | β 无法更新，退化为固定 safe_dist | 调大 η 或加 hysteresis |
| CasADi per-obstacle β 增加求解时间 | MPC 超时 | 限制最多 3 个 active 障碍物（现有代码已有 `choose_num<2` 限制） |
| obstacle_prediction_node heap corruption | 感知链路崩溃 | 修复 Hungarian 算法内存 bug（已知问题） |
