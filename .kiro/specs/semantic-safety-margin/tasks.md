# Semantic Safety Margin (β) — Implementation Tasks

## Phase 1: 基础设施 + 消息定义

### Task 1.1: 创建 semantic_fusion 包骨架 + 自定义消息
- [ ] 创建 `perception/semantic_fusion/` 包（CMakeLists.txt, package.xml）
- [ ] 定义 `SemanticObstacle.msg` 和 `SemanticObstacleArray.msg`
- [ ] 定义 `GuardLog.msg`（放在 `planner/mpc_dcbf/msg/`）
- [ ] 编译验证消息生成成功
- **验收**: `rosmsg show semantic_fusion/SemanticObstacleArray` 输出正确字段

### Task 1.2: 创建 semantic_detection 包骨架 (YOLO)
- [ ] 创建 `perception/semantic_detection/` 包
- [ ] 添加 `scripts/yolo_node.py` 空框架（订阅图像、发布 Detection2DArray）
- [ ] 添加 `config/yolo_config.yaml` 和 `launch/yolo.launch`
- [ ] 安装依赖: `pip3 install ultralytics`，确认 `import ultralytics` 成功
- **验收**: `roslaunch semantic_detection yolo.launch` 能启动（即使无图像输入）

### Task 1.3: 创建语义安全余量配置文件
- [ ] 创建 `planner/mpc_dcbf/config/semantic_safety_margin.yaml`
- [ ] 包含: beta_bar（各类别）、mu_weights、guard 参数、cbf 参数
- [ ] 创建 `swarm_test/config/secbf_scenarios.yaml`（4 个场景定义）
- **验收**: `rosparam load` 成功，`rosparam get /semantic_safety` 返回正确值

---

## Phase 2: 第一层 — 语义感知

### Task 2.1: 实现 yolo_node.py (GPU 推理)
- [ ] 加载 YOLOv8n 模型，订阅 `/camera/color/image_raw`
- [ ] 推理后发布 `vision_msgs/Detection2DArray`
- [ ] COCO 类别映射: person→pedestrian, 小尺寸 person→child, suitcase/backpack→box, car/bus/truck→vehicle
- [ ] 添加 `confidence_threshold` 和 `device` 参数
- [ ] 测量推理延迟，确认 ≤ 30ms
- **验收**: 用 D435 实时图像，`rostopic echo /yolo/detections` 能看到检测结果

### Task 2.2: 实现 semantic_fusion_node (C++)
- [ ] 订阅 `/yolo/detections` + `/clustering/cluster_array` + `/Odometry` + `/camera/color/camera_info`
- [ ] 实现 LiDAR 聚类中心 → 图像平面投影（camera-lidar 外参 + 内参）
- [ ] 实现 IoU 计算 + 匈牙利匹配
- [ ] 计算上下文特征 φ: heading_factor, ttc_norm, density_norm
- [ ] 发布 `/semantic_obstacles` (SemanticObstacleArray)
- [ ] 无 YOLO 输入时回退: 所有障碍物 class=unknown
- **验收**: Gazebo 中有动态障碍物时，`rostopic echo /semantic_obstacles` 输出正确类别和特征

### Task 2.3: 仿真 Ground Truth 模式
- [ ] 当 `_perception_GroundTruth=true` 时，fusion 节点直接从 `dynamic_simulator/DynTraj` 获取类别
- [ ] 跳过 YOLO 推理和 IoU 匹配
- [ ] 从 `secbf_scenarios.yaml` 读取各障碍物的 semantic_class
- **验收**: 数值仿真中不需要 GPU/相机也能输出语义障碍物

---

## Phase 3: 第二层 — β 计算 + Guard

### Task 3.1: 实现 beta_guard_node (C++)
- [ ] 订阅 `/semantic_obstacles` + `/Odometry`
- [ ] 从 YAML 加载 beta_bar、mu_weights、guard 参数
- [ ] 实现 β̂ = β̄(c) × μ(φ) 计算
- [ ] 实现 Guard 检查: β̂ ≤ h_EE(X_t) - η
- [ ] 实现回退逻辑: Guard 失败 → 使用 β_{t-1}
- [ ] 实现单步变化限制: |Δβ| ≤ max_delta_beta
- [ ] 发布 `/safety_margin/beta` (Float32MultiArray)
- [ ] 发布 `/safety_margin/guard_log` (GuardLog)
- **验收**: 静止时 β 输出稳定；人走近时 β 增大；Guard 在 β 突变时触发回退

### Task 3.2: Guard 日志记录
- [ ] 每个 MPC 周期记录: obstacle_id, beta_requested, beta_applied, h_ee, guard_passed
- [ ] 输出到 CSV 文件（路径可配置）
- [ ] 记录 Guard 回退总次数
- **验收**: 运行 30 秒后 CSV 文件有完整数据，可用 Python 脚本分析

---

## Phase 4: 第三层 — MPC-SECBF 控制器

### Task 4.1: mpc_cbf.cpp 新增 SECBF 控制器
- [ ] controller_ls 新增 "SECBF" (index=5)
- [ ] 新增 `sub_beta_` 订阅 `/safety_margin/beta`
- [ ] 新增 `rcvBetaCallBack` 回调，存入 `beta_list_`
- [ ] 新增 `h_secbf()` 函数: 使用 per-obstacle β_i 替代固定 safe_dist
- [ ] `set_safety_st()` 新增 `SECBF` 分支
- [ ] 当 β 话题无数据时回退到固定 safe_dist=0.7
- **验收**: `controller_type=5` 时 MPC 正常求解，β 值影响避障距离

### Task 4.2: Infeasible 处理 + 向后兼容
- [ ] SECBF infeasible → 复用现有 DCBF fallback 机制
- [ ] DCBF 也 infeasible → 零速停车（已有逻辑）
- [ ] 确认 controller_type=0-4 行为完全不变（回归测试）
- **验收**: 数值仿真中 controller=4 的结果与改动前一致

### Task 4.3: 创建 MPC-SECBF launch 文件
- [ ] `planner/mpc_dcbf/launch/mpc_secbf.launch` (独立 MPC 节点)
- [ ] `swarm_test/launch/secbf_planner.launch` (数值仿真完整链路)
- [ ] `swarm_test/launch_exp/exp_secbf_planner.launch` (实车完整链路)
- [ ] 参数: controller=5, gamma, tau_scale, beta 配置路径
- **验收**: `roslaunch swarm_test secbf_planner.launch` 能启动完整链路

---

## Phase 5: 集成测试 + 数值仿真

### Task 5.1: 数值仿真 4 场景验证
- [ ] 配置 S1: 行人对向穿越（1 个 pedestrian 障碍物，v=1.0m/s 迎面）
- [ ] 配置 S2: 儿童突然出现（1 个 child 障碍物，从侧面突入）
- [ ] 配置 S3: 纸箱静态障碍（3 个 box 障碍物，静止）
- [ ] 配置 S4: 混合场景（pedestrian + vehicle + box）
- [ ] 每个场景跑 3 种基线: B1(ACBF固定), B2(SECBF无Guard), B3(SECBF有Guard)
- [ ] 记录: 到达时间、min_distance、avg_speed、h_min、β 时序、Guard 回退次数
- **验收**: 12 组实验数据完整，B3 在所有场景中 h > 理论下界

### Task 5.2: 理论安全下界验证
- [ ] 实现 `scripts/verify_safety_bound.py`
- [ ] 从 guard_log.csv 计算 min(h_EE)
- [ ] 计算理论下界 -(ε_max + Δ̄β) / γ
- [ ] 验证 B3 满足下界，B2 在 S2 场景违反下界
- **验收**: 脚本输出 "Safety guaranteed: True" (B3) 和 "Safety violated" (B2-S2)

### Task 5.3: 延迟性能测试
- [ ] 测量各模块实际延迟: YOLO, fusion, beta_guard, MPC
- [ ] 确认端到端 < 70ms (typical) / < 100ms (worst case)
- [ ] 生成延迟分布直方图
- **验收**: 95th percentile 延迟 < 80ms

---

## Phase 6: Gazebo 仿真 + 实车准备

### Task 6.1: Gazebo 语义障碍物场景
- [ ] 修改 `obstacles_param_gazebo.yaml` 添加 semantic_class 字段
- [ ] 修改 `generate_agents_fixed_poses_gazebo.py` 发布带类别的障碍物
- [ ] 在 Gazebo 中用不同颜色/模型区分类别
- **验收**: Gazebo 中能看到不同类型障碍物，RViz 中 `/semantic_obstacles` 有正确类别

### Task 6.2: Camera-LiDAR 标定 + 实车集成
- [ ] 标定 D435 与 RSLidar 的外参（或使用已有 TF）
- [ ] 确认 `camera_color_optical_frame` → `body` 的 TF 链完整
- [ ] 实车上测试 YOLO + fusion 链路
- [ ] 确认实车端到端延迟 < 70ms
- **验收**: 实车上 `rostopic hz /semantic_obstacles` = 10Hz，类别正确

### Task 6.3: 实车 4 场景实验
- [ ] 在实际环境中复现 S1-S4 场景
- [ ] 录制 rosbag 用于离线分析
- [ ] 生成论文用的数据表格和图表
- **验收**: 4 个场景的 rosbag + 数据分析结果完整

---

## Phase 7: 文档 + 收尾

### Task 7.1: 更新 README + Record.md
- [ ] README 新增 MPC-SECBF 章节（启动方式、参数说明）
- [ ] Record.md 新增时间线条目
- [ ] 更新包结构图

### Task 7.2: 论文数据整理
- [ ] 生成对比表格: B1 vs B2 vs B3 × S1-S4
- [ ] 生成 h(t) 时序图、β(t) 时序图
- [ ] 生成 Guard 回退统计
- [ ] 生成延迟分布图

### Task 7.3: Git 提交 + 推送
- [ ] 提交所有改动到 `feat/semantic-safety-margin`
- [ ] 推送到远程
- [ ] 创建 PR 到 `lxr_replay_claude_code`
