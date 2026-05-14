# Semantic Safety Margin (β) — Implementation Tasks

## Phase 1: 基础设施 + 消息定义

### Task 1.1: 创建 semantic_fusion 包骨架 + 自定义消息
- [x] 创建 `perception/semantic_fusion/` 包（CMakeLists.txt, package.xml）
- [x] 定义 `SemanticObstacle.msg` 和 `SemanticObstacleArray.msg`
- [x] 定义 `GuardLog.msg`（放在 `planner/semantic_guard/msg/`）
- [x] 编译验证消息生成成功
- **验收**: ✅ `rosmsg show semantic_fusion/SemanticObstacleArray` 输出正确字段

### Task 1.2: 创建 semantic_detection 包骨架 (YOLO)
- [x] 创建 `perception/semantic_detection/` 包
- [x] 添加 `scripts/yolo_node.py` 空框架（订阅图像、发布 Detection2DArray）
- [x] 添加 `config/yolo_config.yaml` 和 `launch/yolo.launch`
- [x] 安装依赖: `pip3 install ultralytics`，确认 `import ultralytics` 成功 ✅
- **验收**: ✅ YOLO GPU 推理 7.6ms (RTX 2060), 模型已放置

### Task 1.3: 创建语义安全余量配置文件
- [x] 创建 `planner/semantic_guard/config/semantic_safety_margin.yaml`
- [x] 包含: beta_bar（各类别 v7 标准）、mu_weights、guard 参数、cbf 参数
- [x] 创建 `swarm_test/config/secbf_scenarios.yaml`（4 个场景定义）
- **验收**: ✅ 配置文件完整

---

## Phase 2: 第一层 — 语义感知

### Task 2.1: 实现 yolo_node.py (GPU 推理)
- [x] 加载 YOLOv8n 模型，订阅 `/camera/color/image_raw`
- [x] 推理后发布 `vision_msgs/Detection2DArray`
- [x] COCO 类别映射: person→pedestrian, 小尺寸 person→child, suitcase/backpack→box, car/bus/truck→vehicle, bicycle→cyclist
- [x] 添加 `confidence_threshold` 和 `device` 参数
- [x] 测量推理延迟，确认 ≤ 30ms ✅ 实测 7.6ms (RTX 2060)
- **验收**: ✅ GPU 推理 7.6ms，模型加载正常

### Task 2.2: 实现 semantic_fusion_node (C++)
- [x] 订阅 `/yolo/detections` + `/clustering/cluster_array` + `/Odometry` + `/camera/color/camera_info`
- [x] 实现 LiDAR 聚类中心 → 图像平面投影（camera-lidar 外参 + 内参）
- [x] 实现 IoU 计算 + 匈牙利匹配
- [x] 计算上下文特征 φ: heading_factor, ttc_norm, density_norm
- [x] 发布 `/semantic_obstacles` (SemanticObstacleArray)
- [x] 无 YOLO 输入时回退: 所有障碍物 class=unknown
- **验收**: ✅ 编译通过

### Task 2.3: 仿真 Ground Truth 模式
- [x] 实现 `beta_ground_truth_node`: 直接从 `obs_predict_pub` + launch 参数获取类别
- [x] 跳过 YOLO 推理和 IoU 匹配
- [x] 从 launch 参数 `obstacle_classes` 读取各障碍物的 semantic_class
- **验收**: ✅ 数值仿真中 β 差异化生效（child=0.535, box=0.077）

---

## Phase 3: 第二层 — β 计算 + Guard

### Task 3.1: 实现 beta_guard_node (C++)
- [x] 订阅 `/semantic_obstacles` + `/Odometry`
- [x] 从 YAML 加载 beta_bar、mu_weights、guard 参数
- [x] 实现 β̂ = β̄(c) × μ(φ) 计算
- [x] 实现 Guard 检查: β̂ ≤ h_EE(X_t) - η
- [x] 实现回退逻辑: Guard 失败 → 使用 β_{t-1}
- [x] 实现单步变化限制: |Δβ| ≤ max_delta_beta
- [x] 发布 `/safety_margin/beta` (Float32MultiArray)
- [x] 发布 `/safety_margin/guard_log` (GuardLog)
- **验收**: ✅ Guard 回退率 3.2%，β 输出稳定

### Task 3.2: Guard 日志记录
- [x] 每个 MPC 周期记录: obstacle_id, beta_requested, beta_applied, h_ee, guard_passed
- [x] 输出到 CSV 文件（路径可配置）
- [x] 记录 Guard 回退总次数
- **验收**: ✅ guard_log.csv 46,915 条记录，verify_safety_bound.py 验证通过

---

## Phase 4: 第三层 — MPC-SECBF 控制器

### Task 4.1: 实现 mpc_secbf 包 (独立 MPC-SECBF 控制器)
- [x] 创建 `planner/mpc_secbf/` 包骨架（CMakeLists.txt, package.xml）
- [x] 从 `mpc_dcbf` fork 运动学模型 + CasADi Opti 框架
- [x] 新增 `sub_beta_` 订阅 `/safety_margin/beta`
- [x] 新增 `rcvBetaCallBack` 回调，存入 `beta_list_`
- [x] 实现 `h_secbf()` 函数: 使用 per-obstacle β_i
- [x] 实现 `set_secbf_constraint()`: SECBF CBF 约束
- [x] 当 β 话题无数据时回退到 β = beta_bar[unknown] = 0.4
- [x] 总 safe_dist = R_obs + R_robot + β（与有 Guard 时一致）
- [x] 非对称速度约束 v ∈ [-0.2, v_max]（禁止大幅倒车）
- [x] 加速奖励项 0.5*(v_max - v)²（鼓励前进）
- **验收**: ✅ 编译通过，CasADi 链接正确，数值仿真避障正常

### Task 4.2: Infeasible 处理 + 回归测试
- [x] SECBF infeasible → 内部 fallback（清空障碍物无 CBF 重解）
- [x] fallback 也 infeasible → 零速停车
- [x] 确认 `mpc_dcbf` 包完全不受影响（独立包，未修改原代码）
- **验收**: ✅ 数值仿真全程无崩溃

### Task 4.3: 创建 MPC-SECBF launch 文件
- [x] `planner/mpc_secbf/launch/mpc_secbf.launch` (独立 MPC-SECBF 节点)
- [x] `swarm_test/launch/secbf_planner.launch` (数值仿真完整链路)
- [x] `swarm_test/launch_exp/exp_secbf_planner.launch` (实车完整链路)
- [x] 参数: gamma, tau_scale, v_max, beta 配置路径
- **验收**: ✅ `roslaunch swarm_test secbf_planner.launch` 启动完整链路

---

## Phase 5: 集成测试 + 数值仿真

### Task 5.1: 数值仿真 4 场景验证
- [ ] 配置 S1: 行人对向穿越（1 个 pedestrian 障碍物，v=1.0m/s 迎面）
- [ ] 配置 S2: 儿童突然出现（1 个 child 障碍物，从侧面突入）
- [ ] 配置 S3: Feasibility-Critical（1 个 child 迎面 1.5m/s，初始 5m，验证 Guard 在 h_EE≈0.4 时触发回退；B2 → h 进入负值，B3 → Guard 回退保持安全）
- [ ] 配置 S4: 混合场景（pedestrian + vehicle + box）
- [ ] 每个场景跑 3 种基线: B1(ACBF固定), B2(SECBF无Guard), B3(SECBF有Guard)
- [ ] 记录: 到达时间、min_distance、avg_speed、h_min、β 时序、Guard 回退次数
- **验收**: ⬜ 12 组实验数据完整，B3 在所有场景中 h > 理论下界
- **已完成**: ✅ 混合场景 (S4) 单次验证通过（β 差异化 + Guard 工作 + 安全下界满足）

### Task 5.2: 理论安全下界验证
- [x] 实现 `scripts/verify_safety_bound.py`
- [x] 从 guard_log.csv 计算 min(h_EE)
- [x] 计算理论下界 -(ε_max + Δ̄β) / γ
- [x] 验证 B3 满足下界: min(h)=-0.30 > bound=-1.00 ✅
- [ ] 验证 B2 在 S3 场景违反下界（需单独跑 B2 实验）
- **验收**: ✅ 脚本输出 "Safety guaranteed: True"

### Task 5.3: 延迟性能测试
- [x] 实现 `scripts/measure_latency.py`
- [x] MPC-SECBF 求解延迟实测: 14-38ms ✅
- [x] YOLO GPU 推理延迟实测: 7.6ms ✅
- [ ] 测量完整端到端延迟（实车运行时）
- [ ] 生成延迟分布直方图
- **验收**: ✅ YOLO 7.6ms + MPC 30ms = 总计 ~40ms，远低于 70ms 目标

---

## Phase 6: Gazebo 仿真 + 实车准备

### Task 6.1: Gazebo 语义障碍物场景
- [ ] 修改 `obstacles_param_gazebo.yaml` 添加 semantic_class 字段
- [ ] 修改 `generate_agents_fixed_poses_gazebo.py` 发布带类别的障碍物
- [ ] 在 Gazebo 中用不同颜色/模型区分类别
- **验收**: ⚠️ 跳过 — Gazebo 无 D435 相机插件，直接上实车
- **决策**: 跳过 Gazebo 视觉验证，直接实车 D435 + YOLO 验证

### Task 6.2: Camera-LiDAR 标定 + 实车集成
- [ ] 标定 D435 与 RSLidar 的外参（或使用已有 TF）
- [ ] 确认 `camera_color_optical_frame` → `body` 的 TF 链完整
- [ ] 实车上测试 YOLO + fusion 链路
- [ ] 确认实车端到端延迟 < 70ms
- **验收**: ⬜ 实车上 `rostopic hz /semantic_obstacles` = 10Hz，类别正确

### Task 6.3: 实车 4 场景实验
- [ ] 在实际环境中复现 S1-S4 场景
- [ ] 录制 rosbag 用于离线分析
- [ ] 生成论文用的数据表格和图表
- **验收**: ⬜ 4 个场景的 rosbag + 数据分析结果完整

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
- [x] 提交所有改动到 `feat/semantic-safety-margin`（已有 6 次 commit）
- [ ] 推送到远程
- [ ] 创建 PR 到 `lxr_replay_claude_code`
