# OpenCode 实验理解对话记录

**日期**：2026-06-07  
**主题**：SEESM 论文实验流程与 Exp2/Exp3 详解

---

## 一、项目概述

### 1.1 工作区位置
```
/home/lxr20/lxr/panjian_ws
```

### 1.2 项目类型
这是一个 **ROS 1 (Noetic) catkin 工作区**，用于**移动机器人语义感知动态避障**的研究项目。

### 1.3 核心方法
**MPC-SECBF (Model Predictive Control - Semantic Extended Control Barrier Function)**，等价于 **SEESM (Semantic Extended Euclidean Safety Metric)**。

### 1.4 核心思想
机器人通过视觉语义识别（YOLOv8）识别障碍物类别（行人、儿童、骑车人、车辆、箱子），然后生成**类别特定和上下文感知的安全裕度**，并由 Safety Guard 保护 MPC 可行性。

### 1.5 核心安全函数
```
h_SEE = ||p_rel + τ v_rel|| - R_obs - R_robot - β_i
```

---

## 二、目录结构

```
panjian_ws/
├── hardware/              # 实车硬件驱动（LiDAR、IMU、相机、底盘）
├── state_estimation/      # 状态估计/SLAM（FAST-LIO）
├── perception/            # 感知模块
│   ├── semantic_detection/ # YOLOv8 语义检测
│   ├── semantic_fusion/    # 2D-3D 语义融合
│   ├── dynamic_perception/ # DBSCAN 聚类、轨迹预测
│   └── plan_env/           # SDF 地图
├── planner/               # 规划控制算法
│   ├── mpc_secbf/         # 本文核心：MPC-SECBF 控制器
│   ├── semantic_guard/    # 语义裕度生成 + Safety Guard
│   ├── mpc_dcbf/          # 基线方法
│   ├── dwa_planner/       # DWA 局部规划器
│   └── vomp_planner/      # 全局路径搜索
├── simulation_tools/      # 仿真环境工具
├── simulation_models/     # Gazebo 机器人模型
├── experiments/           # 实验计划模板
└── swarm_test/            # 测试编排包（启动文件、脚本、配置）
```

---

## 三、技术栈

- **语言**：C++14（核心算法）、Python 3（脚本、YOLO）
- **框架**：ROS 1 Noetic、CasADi（MPC 优化）、Eigen、PCL、OpenCV、Gazebo
- **构建**：Catkin，需本地编译 CasADi 3.7.0

---

## 四、实验总体安排

### 4.1 实验要证明的三件事

1. **语义类别会改变安全裕度**：例如 `child_like` 应比 `adult`、`box` 保持更大的安全距离。
2. **上下文会调制安全裕度**：同一类别在静止、同向、横穿、迎面接近等状态下应有不同 `beta_i`。
3. **Safety Guard 能保证语义余量不破坏 MPC-SECBF 可行性**：当 `beta_hat` 过大时，应被投影、回退或置零。

### 4.2 执行顺序

```
Phase 5 → Phase 6.1 (Exp2) → Phase 6.2 (Exp3) → Phase 6.3 (Exp4) → Phase 7
```

---

## 五、通用实验步骤

### Step 1: 构建工作区
```bash
source /opt/ros/noetic/setup.bash
cd /home/lxr20/lxr/panjian_ws
catkin_make --source . \
  -DCATKIN_WHITELIST_PACKAGES="dynamic_simulator;robot_simulator;map_generator;laser_simulator;plan_env;traj_planner;semantic_fusion;semantic_guard;mpc_secbf;swarm_test" \
  -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"
source devel/setup.bash
```

### Step 2: 运行静态契约检查
```bash
python3 swarm_test/scripts/check_phase5_contract.py
python3 -m py_compile \
  swarm_test/scripts/run_secbf_sim_experiments.py \
  swarm_test/scripts/analyze_exp2_category.py \
  swarm_test/scripts/check_experiment_csv_fields.py \
  swarm_test/scripts/phase5_csv_logger.py
```

### Step 3: 跑仿真
仿真由 runner 同时启动：
1. `roscore`
2. `swarm_test/launch/secbf_planner.launch`
3. `swarm_test/launch/start_test.launch`

### Step 4: 检查每个 run 的 CSV 字段
```bash
python3 swarm_test/scripts/check_experiment_csv_fields.py <run_dir>
```

每个 run 至少应包含：
```
meta.yaml
robot_log.csv
obstacle_log.csv
margin_guard_log.csv
planner_log.csv
timing_log.csv
event_log.csv
summary.csv
summary.md
verify_safety_bound.txt
```

### Step 5: 分析和出图
```bash
python3 swarm_test/scripts/analyze_exp2_category.py \
  experiments/Exp2_category_aware/runs_static \
  --output-dir experiments/Exp2_category_aware/analysis
```

---

## 六、Task 6.1: Exp2 类别感知实验详解

### 6.1 实验目的

**证明语义类别会改变安全裕度**：不同类别的障碍物应该有不同的 `beta_i`，而不是使用固定安全距离。

### 6.2 期望趋势

```
beta(child_like) > beta(cyclist) > beta(adult) > beta(box)
D_min(child_like) > D_min(adult) > D_min(box)
```

### 6.3 控制变量（不变因素）

| 因素 | 固定值 | 说明 |
|------|--------|------|
| **地图大小** | 12m × 6m | 简单走廊场景 |
| **起点** | (0, 0) | 固定起点 |
| **终点** | (12, 0) | 固定终点 |
| **障碍物位置** | (6, 0) | 路径正中央 |
| **障碍物运动** | 静态 | `slower: 999.0`, `scale: 0,0,0` |
| **障碍物半径** | 0.4m | 统一尺寸 |
| **机器人半径** | 0.4m | 统一尺寸 |
| **MPC 参数** | horizon=20, dt=0.1, gamma=0.35 | 固定控制参数 |
| **Guard 参数** | eta=0.1, max_delta_beta=0.3, tau=0.2 | 固定保护参数 |

### 6.4 自变量（变化因素）

**唯一变化：`semantic_class`**

| 场景名称 | 类别 | beta_bar | 预期风险等级 |
|----------|------|----------|--------------|
| `Exp2_category_box` | box | 0.1m | 低风险 |
| `Exp2_category_adult` | adult | 0.4m | 中风险 |
| `Exp2_category_child_like` | child_like | 0.7m | 高风险 |
| `Exp2_category_cyclist` | cyclist | 0.6m | 中高风险 |

### 6.5 对比方法（3 种基线）

| 方法 | beta_bar | mu 权重 | Guard | 目的 |
|------|----------|---------|-------|------|
| **Fixed_margin** | 所有类别=0.4 | bias=1.0, 其余=0.0 | 启用 | 固定裕度基线 |
| **Category_only** | 各类别不同 | bias=1.0, 其余=0.0 | 启用 | 仅类别感知 |
| **SEESM_Ours** | 各类别不同 | bias=0.6, heading=0.2, ttc=0.15, density=0.1 | 启用 | 完整方法 |

**关键对比**：
- `Fixed_margin` vs `Category_only`：证明**类别有效**
- `Category_only` vs `SEESM_Ours`：证明**上下文调制有效**

### 6.6 场景配置（YAML）

```yaml
Exp2_category_box:
  description: "Exp2 category-aware sanity: single static box on path"
  map: {x: 12.0, y: 6.0, z: 3.0}
  start: {x: 0.0, y: 0.0}
  obstacles:
    - {x: 6.0, y: 0.0, z: 0.5, offset: 0.0, slower: 999.0,
       scale_x: 0.0, scale_y: 0.0, scale_z: 0.0,
       semantic_class: "box"}
  goal: {x: 12.0, y: 0.0}
```

### 6.7 测量指标

| 指标 | 计算方法 | 物理含义 |
|------|----------|----------|
| `D_min` | `min(d_i - R_obs - R_robot)` | 机器人表面到障碍物表面的最小间距 |
| `beta_mean` | `mean(beta_applied)` | 语义安全裕度的平均应用值 |
| `beta_max` | `max(beta_applied)` | 语义安全裕度的最大值 |
| `path_length` | 轨迹点累加距离 | 路径总长度 |
| `travel_time` | 终点时间 - 起点时间 | 行驶总时长 |
| `success` | 距离终点 ≤ 0.8m | 是否到达目标 |

### 6.8 当前状态

已跑单次 sanity run，结果符合预期：

```
Category_only D_min:
  box        0.328
  adult      0.368
  child_like 0.468
  cyclist    0.413

SEESM_Ours beta_mean:
  box        0.072
  adult      0.283
  cyclist    0.418
  child_like 0.488
```

### 6.9 下一步计划

1. 每类每方法补到 5 次 sanity
2. 曲线正常后补到至少 20 次随机种子
3. 用 `exp2_summary_stats.csv` 报告 `mean ± std`
4. 最终验收重点看：
   - `D_min(child_like) > D_min(adult) > D_min(box)`
   - `path_length` 没有异常增大
   - `beta_mean` 按类别单调变化

---

## 七、Task 6.2: Exp3 上下文调制实验详解

### 7.1 实验目的

**证明上下文会调制安全裕度**：同一类别在不同运动状态下应该产生不同的 `beta_i` 值，而不是静态查表。

### 7.2 期望趋势

```
beta(frontal) > beta(crossing) > beta(same_direction) > beta(static)
```

### 7.3 控制变量（不变因素）

| 因素 | 固定值 | 说明 |
|------|--------|------|
| **语义类别** | `adult` | 固定为成人 |
| **beta_bar** | 0.4m | 成人类别的基础安全裕度 |
| **地图大小** | 12m × 6m | 简单走廊场景 |
| **起点** | (0, 0) | 固定起点 |
| **终点** | (12, 0) | 固定终点 |
| **障碍物初始位置** | (6, 0) | 路径正中央 |
| **障碍物半径** | 0.4m | 统一尺寸 |
| **机器人半径** | 0.4m | 统一尺寸 |

### 7.4 自变量（变化因素）

**唯一变化：运动状态（motion pattern）**

| 场景名称 | 运动状态 | 速度 | 方向 | 预期风险 |
|----------|----------|------|------|----------|
| `Exp3_context_static` | 静止 | 0 m/s | - | 低 |
| `Exp3_context_same_dir` | 同向运动 | 0.5 m/s | +x | 中 |
| `Exp3_context_crossing` | 横穿 | 0.8 m/s | +y | 中高 |
| `Exp3_context_frontal` | 迎面接近 | 1.0 m/s | -x | 高 |

### 7.5 对比方法（3 种基线）

| 方法 | beta_bar | mu 权重 | Guard | 目的 |
|------|----------|---------|-------|------|
| **Fixed_margin** | 0.4 (所有状态) | bias=1.0, 其余=0.0 | 启用 | 固定裕度基线 |
| **Context_only** | 0.4 | bias=0.6, heading=0.2, ttc=0.15, density=0.1 | 启用 | 仅上下文调制 |
| **SEESM_Ours** | 0.4 | bias=0.6, heading=0.2, ttc=0.15, density=0.1 | 启用 | 完整方法 |

### 7.6 上下文特征计算

```cpp
// 航向因子
cos_delta = p_rel.normalized().dot(v_rel.normalized())
f_head = max(0, -cos_delta)

// 接近速度
closing_speed = max(-p_rel.normalized().dot(v_rel), 0)

// 碰撞时间
ttc = (closing_speed > 1e-6) ? (d_i / closing_speed) : infinity
ttc_norm = max(0, min(1, 1 - ttc/5))

// 密度因子
density_norm = min(1, (obs_num - 1) / 5)

// 上下文调制系数
mu = clip(w_bias + w_head * f_head + w_ttc * ttc_norm + w_density * density_norm, 0, 1)
```

### 7.7 预期 mu 和 beta 值

**预期 mu 值**：
```
mu(static) = 0.6 + 0 + 0 + 0 = 0.6
mu(same_dir) = 0.6 + 0 + 0.03 + 0 = 0.63
mu(crossing) = 0.6 + 0.14 + 0.075 + 0 = 0.815
mu(frontal) = 0.6 + 0.2 + 0.12 + 0 = 0.92
```

**预期 beta 值**（beta_bar=0.4）：
```
beta(static) = 0.4 × 0.6 = 0.24
beta(same_dir) = 0.4 × 0.63 = 0.252
beta(crossing) = 0.4 × 0.815 = 0.326
beta(frontal) = 0.4 × 0.92 = 0.368
```

### 7.8 验收标准

| 指标 | 预期趋势 | 统计要求 |
|------|----------|----------|
| `beta_mean` | frontal > crossing > same_dir > static | p < 0.05 |
| `D_min` | frontal > crossing > same_dir > static | p < 0.05 |
| `path_length` | 无显著差异 | p > 0.05 |
| `success` | 100% | - |

---

## 八、核心算法详解

### 8.1 语义安全裕度计算

**文件**：`planner/semantic_guard/src/beta_ground_truth_node.cpp`

```cpp
// 上下文调制系数
mu = clip(w_bias + w_head * f_head + w_ttc * ttc_norm + w_density * density_norm, 0, 1)

// 语义安全裕度
beta_hat = beta_bar_val * mu

// Guard 检查: β̂ ≤ h_EE - η
h_EE = ||p_rel + τ * v_rel|| - R_obs - R_robot
guard_upper_bound = min(β_bar, max(0, h_EE - η))
```

### 8.2 Guard 状态机

```cpp
if (!guard_enabled) {
    beta_final = beta_hat;           // 禁用 Guard
    status = "disabled";
} else if (guard_upper_bound <= 1e-9) {
    beta_final = 0.0;                // 强制置零
    status = "zero";
} else if (guard_pass) {
    // 限速 + 投影
    delta = clip(beta_hat - beta_prev, -max_delta, +max_delta)
    beta_limited = beta_prev + delta
    beta_final = clip(beta_limited, 0, guard_upper_bound)
    status = "project" 或 "accept";
} else {
    // 回退到上一值
    beta_final = clip(beta_prev, 0, guard_upper_bound)
    status = "fallback" 或 "zero";
}
```

### 8.3 MPC-SECBF 约束

**文件**：`planner/mpc_secbf/src/mpc_secbf.cpp`

```cpp
// 安全函数
h_EE = ||l|| - R_obs - R_robot
h_SEE = h_EE - β_i

// CBF 约束
-h_{k+1} + (1-γ)h_k ≤ 0
```

---

## 九、运行实验的终端指令

### 9.1 Exp2 实验

```bash
# 1. 构建工作区
source /opt/ros/noetic/setup.bash
cd /home/lxr20/lxr/panjian_ws
catkin_make --source . \
  -DCATKIN_WHITELIST_PACKAGES="dynamic_simulator;robot_simulator;map_generator;laser_simulator;plan_env;traj_planner;semantic_fusion;semantic_guard;mpc_secbf;swarm_test" \
  -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"
source devel/setup.bash

# 2. 运行 Exp2 实验（5 次重复）
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario Exp2_category_box,Exp2_category_adult,Exp2_category_child_like,Exp2_category_cyclist \
  --baseline Fixed_margin,Category_only,SEESM_Ours \
  --duration-sec 12 \
  --repeat 5 \
  --output-root /home/lxr20/lxr/panjian_ws/experiments/Exp2_category_aware/runs_static \
  --roscore auto

# 3. 分析结果
python3 swarm_test/scripts/analyze_exp2_category.py \
  experiments/Exp2_category_aware/runs_static \
  --output-dir experiments/Exp2_category_aware/analysis
```

### 9.2 Exp3 实验

```bash
# 运行 Exp3 实验（5 次重复）
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario Exp3_context_static,Exp3_context_same_dir,Exp3_context_crossing,Exp3_context_frontal \
  --baseline Fixed_margin,Context_only,SEESM_Ours \
  --duration-sec 15 \
  --repeat 5 \
  --output-root /home/lxr20/lxr/panjian_ws/experiments/Exp3_context_modulation/runs \
  --roscore auto

# 分析结果
python3 swarm_test/scripts/analyze_exp3_context.py \
  experiments/Exp3_context_modulation/runs \
  --output-dir experiments/Exp3_context_modulation/analysis
```

### 9.3 实验矩阵大小

| 实验 | 场景数 | 基线数 | 组合数 | 重复次数 | 总运行次数 |
|------|--------|--------|--------|----------|------------|
| Exp2 | 4 | 3 | 12 | 5 (sanity) | 60 |
| Exp2 | 4 | 3 | 12 | 20 (最终) | 240 |
| Exp3 | 4 | 3 | 12 | 5 (sanity) | 60 |
| Exp3 | 4 | 3 | 12 | 20 (最终) | 240 |

每次运行启动 3 个进程（roscore + planner + start_test）。

---

## 十、关键文件索引

### 10.1 实验配置

| 文件 | 用途 |
|------|------|
| `swarm_test/config/secbf_scenarios.yaml` | 场景定义 |
| `planner/semantic_guard/config/semantic_safety_margin.yaml` | 安全裕度参数 |

### 10.2 实验运行

| 文件 | 用途 |
|------|------|
| `swarm_test/scripts/run_secbf_sim_experiments.py` | 自动化实验运行器 |
| `swarm_test/launch/secbf_planner.launch` | 规划器启动文件 |
| `swarm_test/launch/start_test.launch` | 测试环境启动文件 |

### 10.3 数据分析

| 文件 | 用途 |
|------|------|
| `swarm_test/scripts/analyze_exp2_category.py` | Exp2 分析脚本 |
| `swarm_test/scripts/check_experiment_csv_fields.py` | CSV 字段检查 |
| `swarm_test/scripts/check_phase5_contract.py` | 公式/日志契约检查 |

### 10.4 核心算法

| 文件 | 用途 |
|------|------|
| `planner/semantic_guard/src/beta_ground_truth_node.cpp` | 语义 beta + Guard |
| `planner/mpc_secbf/src/mpc_secbf.cpp` | MPC-SECBF 控制器 |

---

## 十一、Exp2 与 Exp3 的关系

| 实验 | 变化因素 | 证明内容 |
|------|----------|----------|
| **Exp2** | 语义类别 | 类别影响安全裕度 |
| **Exp3** | 运动状态 | 上下文调制安全裕度 |

**两者结合**：证明 `beta_i = beta_bar(c) × mu(φ)` 公式中，**类别**和**上下文**都对安全裕度有显著影响。

---

## 十二、论文价值

**Exp2 和 Exp3 为论文提供了核心证据**：
1. 语义安全裕度不是固定常数，而是随障碍物类别动态变化的
2. 上下文调制使安全裕度更加精细和合理
3. 语义感知的安全裕度比固定安全距离更合理

实验结果将直接用于论文中的**类别感知对比实验**和**上下文调制对比实验**章节。

---

**记录时间**：2026-06-07  
**记录工具**：OpenCode (mimo-v2.5-pro)
