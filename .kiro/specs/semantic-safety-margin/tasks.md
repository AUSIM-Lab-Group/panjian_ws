# Semantic Safety Margin (β) — Implementation Tasks

> 当前执行证据（2026-06-07）：实现层面的语义安全余量、Guard、MPC-SECBF 和基础仿真脚手架已有构建/smoke 记录。当前后续工作重点已经从“大矩阵 12 组仿真”收缩为论文投稿最小实验链：日志系统、类别感知、上下文调制、Guard 临界、主对比、实时性和小规模实车展示。实验公式口径以代码和 `论文公式.md` 为准：`h_SEE = ||p_rel + tau v_rel|| - R_obs - R_robot - beta_i`，不再额外叠加固定 `R_safe`。

## Phase 1: 基础设施 + 消息定义

### Task 1.1: 创建 semantic_fusion 包骨架 + 自定义消息
- [x] 创建 `perception/semantic_fusion/` 包（CMakeLists.txt, package.xml）
- [x] 定义 `SemanticObstacle.msg` 和 `SemanticObstacleArray.msg`
- [x] 定义 `GuardLog.msg`（放在 `planner/semantic_guard/msg/`）
- [x] 编译验证消息生成成功
- **验收**: ✅ 消息包参与 2026-05-23 白名单构建；`rosmsg show semantic_fusion/SemanticObstacleArray` 和 `rosmsg show semantic_guard/GuardLog` 字段已复核。

### Task 1.2: 创建 semantic_detection 包骨架 (YOLO)
- [x] 创建 `perception/semantic_detection/` 包
- [x] 添加 `scripts/yolo_node.py` 空框架（订阅图像、发布 Detection2DArray）
- [x] 添加 `config/yolo_config.yaml` 和 `launch/yolo.launch`
- [x] 安装依赖: `pip3 install ultralytics`，确认 `import ultralytics` 成功 ✅
- **验收**: ✅ `semantic_detection` 参与 2026-05-23 白名单构建；YOLO GPU 7.6ms 为既有记录，实车 dry-run 时需重新记录。

### Task 1.3: 创建语义安全余量配置文件
- [x] 创建 `planner/semantic_guard/config/semantic_safety_margin.yaml`
- [x] 包含: beta_bar（各类别 v7 标准）、mu_weights、guard 参数、cbf 参数
- [x] 创建 `swarm_test/config/secbf_scenarios.yaml`（4 个场景定义）
- **验收**: ✅ 配置文件存在并参与 2026-05-23 白名单构建。

---

## Phase 2: 第一层 — 语义感知

### Task 2.1: 实现 yolo_node.py (GPU 推理)
- [x] 加载 YOLOv8n 模型，订阅 `/camera/color/image_raw`
- [x] 推理后发布 `vision_msgs/Detection2DArray`
- [x] COCO 类别映射: person→pedestrian, 小尺寸 person→child, suitcase/backpack→box, car/bus/truck→vehicle, bicycle→cyclist
- [x] 添加 `confidence_threshold` 和 `device` 参数
- [x] 测量推理延迟，确认 ≤ 30ms ✅ 实测 7.6ms (RTX 2060)
- **验收**: ✅ 节点脚本参与 2026-05-23 白名单构建；GPU 推理延迟在 Phase 5.3/6.2 重新记录。

### Task 2.2: 实现 semantic_fusion_node (C++)
- [x] 订阅 `/yolo/detections` + `/clustering/cluster_array` + `/Odometry` + `/camera/color/camera_info`
- [x] 实现 LiDAR 聚类中心 → 图像平面投影（camera-lidar 外参 + 内参）
- [x] 实现 IoU 计算 + 匈牙利匹配
- [x] 计算上下文特征 φ: heading_factor, ttc_norm, density_norm
- [x] 发布 `/semantic_obstacles` (SemanticObstacleArray)
- [x] 无 YOLO 输入时回退: 所有障碍物 class=unknown
- **验收**: ✅ 2026-05-23 `semantic_fusion_node` 编译通过。

### Task 2.3: 仿真 Ground Truth 模式
- [x] 实现 `beta_ground_truth_node`: 直接从 `obs_predict_pub` + launch 参数获取类别
- [x] 跳过 YOLO 推理和 IoU 匹配
- [x] 从 launch 参数 `obstacle_classes` 读取各障碍物的 semantic_class
- **验收**: ✅ 2026-05-23 `beta_ground_truth_node` 编译通过；β 差异化运行证据将在 Phase 5 重新采集。

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
- **验收**: ✅ 2026-05-23 `beta_guard_node` 编译通过；Guard 回退率和 β 输出稳定性将在 Phase 5/7 重新采集。

### Task 3.2: Guard 日志记录
- [x] 每个 MPC 周期记录: obstacle_id, beta_requested, beta_applied, h_ee, guard_passed
- [x] 输出到 CSV 文件（路径可配置）
- [x] 记录 Guard 回退总次数
- **验收**: ✅ 日志字段和脚本已存在；`guard_log.csv` 与 `verify_safety_bound.py` 结果将在 Phase 5 每次 run 后记录。

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
- [x] 总安全边界 = R_obs + R_robot + β（与有 Guard 时一致）
- [x] 非对称速度约束 v ∈ [-0.2, v_max]（禁止大幅倒车）
- [x] 加速奖励项 0.5*(v_max - v)²（鼓励前进）
- **验收**: ✅ 2026-05-23 `mpc_secbf_node` 编译通过，CasADi 链接可用；数值仿真避障将在 Phase 5 验证。

### Task 4.2: Infeasible 处理 + 回归测试
- [x] SECBF infeasible → 内部 fallback（清空障碍物无 CBF 重解）
- [x] fallback 也 infeasible → 零速停车
- [x] 确认 `mpc_dcbf` 包完全不受影响（独立包，未修改原代码）
- **验收**: ✅ 2026-05-23 `mpc_dcbf` 回归构建通过；SECBF infeasible 运行行为将在 Phase 5 验证。

### Task 4.3: 创建 MPC-SECBF launch 文件
- [x] `planner/mpc_secbf/launch/mpc_secbf.launch` (独立 MPC-SECBF 节点)
- [x] `swarm_test/launch/secbf_planner.launch` (数值仿真完整链路)
- [x] `swarm_test/launch_exp/exp_secbf_planner.launch` (实车完整链路)
- [x] 参数: gamma, tau_scale, v_max, beta 配置路径
- **验收**: ✅ launch 文件存在且相关 package 编译通过；`roslaunch swarm_test secbf_planner.launch` smoke run 将在 Phase 5 执行。

---

## Phase 5: 实验数据链打通

### Task 5.1: 统一实验公式和日志契约
- [x] 确认运行代码输出的安全函数口径为 `h_SEE = ||p_rel + tau v_rel|| - R_obs - R_robot - beta_i`
- [x] 确认实验配置不再使用额外固定 `R_safe` 字段；固定安全距离 baseline 用 `beta_i ≡ d_safe`
- [x] 确认 Guard 上界记录为 `b_i^eta(X_t)=min(beta_bar_i,max(0,h_EE(X_t)-eta))`
- [x] 确认 `guard_status` 至少区分 `accept/project/fallback/zero`
- **验收**: ✅ `meta.yaml`、Guard log 和论文公式中的安全函数完全一致，没有 `R_safe` 作为额外半径项；2026-06-07 已通过 `check_phase5_contract.py` 和相关包构建。

### Task 5.2: 建立实验目录和 meta.yaml 模板
- [x] 建立 `experiments/Exp0_log_check/`
- [x] 建立 `experiments/Exp1_main_comparison/`
- [x] 建立 `experiments/Exp2_category_aware/`
- [x] 建立 `experiments/Exp3_context_modulation/`
- [x] 建立 `experiments/Exp4_guard_critical/`
- [x] 建立 `experiments/Exp7_runtime/`
- [x] 建立 `experiments/Exp8_real_world_demo/`
- [x] 每个实验目录准备 `meta.yaml` 模板，字段至少包含 `experiment_id, scenario, method, map_name, start, goal, robot_radius, obstacle_radius, beta_table, guard_eta, guard_enable, mpc_horizon, dt, random_seed`
- **验收**: ✅ 任意一次 run 可从 `meta.yaml` 复现实验配置；2026-06-07 已完成 YAML 解析和必填字段检查。

### Task 5.3: 完成日志系统字段检查
- [x] 跑一组最短场景，生成 `robot_log.csv`
- [x] 生成 `obstacle_log.csv`
- [x] 生成 `margin_guard_log.csv`
- [x] 生成 `planner_log.csv`
- [x] 生成 `timing_log.csv`
- [x] 生成 `event_log.csv`
- [x] 写或运行字段检查脚本，确认所有 csv 包含论文图表所需字段
- **验收**: ✅ `t, id, class, d_i, rel_v, TTC, r_i/mu, beta_bar, beta_hat, beta, guard_upper_bound, h_EE, h_SEE, slack, mpc_status` 均可从 csv 读取；2026-06-07 S3/B3 6 秒短仿真 run `20260607_124616_S3_feasibility_critical_B3_SECBF_with_guard` 通过字段检查和安全界验证。

---

## Phase 6: 组件证据实验

### Task 6.1: Exp.2 类别感知实验
- [x] 固定地图为 `12m x 6m`，起点 `(0,0)`，终点 `(12,0)`
- [x] 使用单个障碍物，固定几何半径和运动方式，只改变类别 `box/adult/child-like/cyclist`
- [ ] 每类先跑 5 次 sanity check，曲线正常后补到至少 20 次随机种子
- [x] 对比 `Fixed-margin`、`Category-only`、`SEESM Ours`
- [ ] 统计 `D_min, beta_mean, beta_max, path_length, travel_time, success_rate` 的 `mean ± std`
- [x] 生成类别轨迹对比图
- [x] 生成 `D_min/beta` 柱状图或表格
- **当前证据**: 2026-06-07 已完成 12 秒静态单障碍物 sanity matrix（4 类 × 3 方法 × 1 次），run root 为 `experiments/Exp2_category_aware/runs_static/20260607_130759_*`；12 个 run 均通过 CSV 字段检查和 safety bound 验证，已生成 `analysis/exp2_run_metrics.csv`、`analysis/exp2_summary_stats.csv`、`analysis/trajectory_by_class.png`、`analysis/dmin_by_class.png`、`analysis/beta_by_class.png`。
- **验收**: ⬜ 仍需补足每类至少 5 次 sanity 和至少 20 次随机种子；当前单次 sanity 中 `Category_only` 已满足 `D_min(child_like) > D_min(adult) > D_min(box)`，`SEESM_Ours` 的 `beta` 满足 `child_like > cyclist > adult > box`，但 `SEESM_Ours` 的 `D_min` 仍需多 seed 均值确认。

### Task 6.2: Exp.3 上下文调制实验
- [x] 固定类别为 `adult`，固定障碍物几何尺寸
- [x] 配置 `static` 状态
- [x] 配置 `same-direction` 状态
- [x] 配置 `crossing` 状态
- [x] 配置 `frontal-approaching` 状态
- [ ] 每个状态先跑 5 次 sanity check，曲线正常后补到至少 20 次随机种子
- [x] 记录 `f_head/cos_delta, TTC, TTC_norm, rho_norm, r_i或mu, beta_hat, beta, h_EE, h_SEE`
- [x] 生成三联图：`mu(t)` 或 `r_i(t)`、`beta_hat(t)`、`beta(t)`
- **当前证据**: 2026-06-07 已完成 4 个上下文 × `SEESM_Ours` × 1 次的 12 秒 sanity，run root 为 `experiments/Exp3_context_modulation/runs_sanity_v5/20260607_174908_*`；4 个 run 均通过 CSV 字段检查，已生成 `analysis/exp3_run_metrics.csv`、`analysis/exp3_summary_stats.csv`、`analysis/context_mu_beta_curves.png`、`analysis/context_beta_bar.png`。单次 sanity 的 `beta_max` 满足 `frontal_approaching(0.359888) > crossing(0.358382) > same_direction(0.322340) > static(0.312430)`；`beta_mean` 中 `crossing(0.272733)` 与 `same_direction(0.275147)` 仍接近，需多 seed 或更长时间窗确认。
- **验收**: ⬜ 最终多 seed 统计需满足 `beta_frontal > beta_crossing > beta_same_direction/static`，证明 `beta_i` 不是静态查表

### Task 6.3: Exp.4 Safety Guard 临界实验
- [ ] 设置单机器人 + 单行人临界场景，起点 `(0,0)`，终点 `(8,0)`，行人靠近路径，例如 `(4,0.5)`
- [ ] 通过类别切换 `adult -> child-like` 或快速迎面接近，让 `beta_hat > guard_upper_bound`
- [ ] 对比 `Context w/o Guard` 与 `SEESM Ours`
- [ ] 记录 `beta_hat, guard_upper_bound, beta, guard_status, h_EE, h_SEE, slack_mean, slack_max, mpc_status, cmd_v, cmd_w`
- [ ] 生成 `beta_hat / guard_upper_bound / beta` 三曲线图
- [ ] 统计 `MPC feasible rate, infeasible_count, slack_max, min_h_SEE, guard_project_count, guard_fallback_count`
- **验收**: ⬜ 无 Guard 更容易出现大 slack 或 infeasible；有 Guard 时 `beta` 被压到可行上界内

---

## Phase 7: 主结果实验

### Task 7.1: Exp.1 主对比实验
- [ ] 地图设为 `20m x 8m`，起点 `(0,0)`，终点 `(20,0)`
- [ ] 放置 4--6 个动态语义障碍物，包含 `box/adult/child-like/cyclist`
- [ ] 覆盖横穿、斜向、往复、正面靠近四类运动模式
- [ ] 对比 `Fixed-margin`、`Context w/o Guard`、`SEESM Ours`
- [ ] 时间允许时加入 `Category-only`
- [ ] 每种方法至少 20 次随机种子
- [ ] 统计 `success_rate, collision_count, infeasible_count, D_min, PSV, path_length, travel_time, velocity_smoothness, angular_smoothness, total_loop_time`
- [ ] 生成代表性轨迹图
- [ ] 生成 robot-obstacle distance over time 曲线
- **验收**: ⬜ 结果能说明 SEESM Ours 在安全/社会距离提升的同时，不显著牺牲效率和实时性

### Task 7.2: Exp.7 实时性实验
- [ ] 在主对比场景中打开 timing log
- [ ] 统计 `semantic perception` 耗时
- [ ] 统计 `tracking/prediction` 耗时
- [ ] 统计 `association` 耗时
- [ ] 统计 `margin generation` 耗时
- [ ] 统计 `Safety Guard` 耗时
- [ ] 统计 `global planning` 耗时
- [ ] 统计 `MPC-SECBF` 耗时
- [ ] 统计 `total loop` 耗时
- [ ] 输出 `mean / max / std / 95th percentile`
- **验收**: ⬜ 若 `total_loop_time_mean < 100ms`，可写 10 Hz 平均实时；否则按分频策略说明 YOLO、tracking、Guard、MPC 和 global replan 频率

### Task 7.3: Exp.8 小规模实车展示实验
- [ ] 仅使用安全场景，不找真实儿童参与危险实验
- [ ] 场景 1：adult 横穿机器人路径，对比 `Fixed-margin` 与 `SEESM Ours`
- [ ] 场景 2：box 与 adult 或 child-like dummy，展示类别导致的距离差异
- [ ] 每个场景每种方法至少 3 次，最好 5--10 次
- [ ] 保存正面视频
- [ ] 保存俯视视频
- [ ] 保存 RViz 录屏
- [ ] 保存 rosbag
- [ ] 保存 csv
- [ ] 保存参数文件和实验照片
- [ ] 实车表格只报告 `D_min, path_length, travel_time, success, total_loop_time`
- **验收**: ⬜ 实车结果作为可运行展示，不写过强统计显著性结论

---

## Phase 8: 增强实验和论文收尾

### Task 8.1: Global/Local 一致性增强实验
- [ ] 确认 `beta_i` 已显式接入全局搜索或语义风险代价
- [ ] 对比 `Local-only SEESM` 与 `Full SEESM`
- [ ] 记录 global path、local trajectory、path deviation、slack、D_min、PSV
- [ ] 统计 `D_ref-local`
- **验收**: ⬜ 只有完成 global 接入后才把该实验写入主论文结果；否则放未来工作或扩展实验

### Task 8.2: Group-aware 增强实验
- [ ] 确认 group detection 或规则 group flag 稳定
- [ ] 配置两人距离 `3.0m` 的非 group 场景
- [ ] 配置两人距离 `1.0--1.5m` 且同向/相近的 group 场景
- [ ] 记录 group_id、group_flag、beta_i、robot trajectory、GSV、D_min、path_length
- **验收**: ⬜ 机器人不穿越 group 内部空间，同时绕行代价可解释

### Task 8.3: 论文图表和数据整理
- [ ] 生成参数表，来源为各实验 `meta.yaml`
- [ ] 生成类别感知轨迹图和 `D_min/beta` 表
- [ ] 生成上下文调制三联图
- [ ] 生成 Guard 曲线和 slack/MPC 可行率表
- [ ] 生成主对比指标表
- [ ] 生成实时性表
- [ ] 生成实车展示图和简表
- [ ] 每张图都标注对应 csv 来源
- **验收**: ⬜ 所有论文图表都能由实验目录中的 csv、rosbag 或视频材料追溯

### Task 8.4: 记录、提交和清理
- [ ] 每次代码或实验文档改动后追加 `.kiro/specs/semantic-safety-margin/memory.md`
- [ ] 需要逐条执行证据时同步追加 `.kiro/specs/semantic-safety-margin/change-record.md`
- [ ] 清理或归档 `swarm_test/output/secbf_runs/` 中不需要提交的实验输出
- [ ] 确认 `论文公式.md` 是否加入版本控制
- [ ] 确认 `实验设置.md`、`tasks.md`、`memory.md` 是否一起提交
- [ ] 推送到远程并按需要创建 PR
- **验收**: ⬜ 工作区只保留有意提交的源码、配置、论文文档和必要实验摘要
