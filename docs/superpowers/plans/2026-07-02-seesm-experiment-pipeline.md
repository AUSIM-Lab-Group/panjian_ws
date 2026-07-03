# SEESM 实验流水线实现计划

> **给 agentic workers：** 必需子技能：使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`，按任务逐项实现本计划。各步骤使用 checkbox（`- [ ]`）语法，便于跟踪进度。

**目标：** 让现有 SEESM/MPC-SECBF 仿真流水线能够产出论文实验所需的消融、可行性、slack 以及闭环安全下界证据。

**架构：** 保持现有 ROS launch 和 runner 流程不变。在 `semantic_guard` 中加入细粒度 semantic-margin 消融参数，在 `mpc_secbf` 中加入软 SECBF slack 和求解尝试指标，然后扩展 Python runner 与验证脚本，使每次运行都能记录可直接用于论文整理的 CSV 字段。

**技术栈：** ROS Noetic、C++14、CasADi/Ipopt、Python 3 CSV/YAML 脚本。

**当前执行状态（2026-07-02 16:08）：** 代码实现已完成到可开始实验的 smoke-test 水准。已通过静态契约检查、Python 编译检查、`semantic_guard;mpc_secbf;swarm_test` 白名单构建、runner dry-run、短时仿真、CSV 字段检查和 `h_see` 安全下界验证；根目录 aggregate `summary.csv` 也已补齐。

---

### 任务 1：静态契约与安全下界测试

**文件：**
- 修改：`swarm_test/scripts/check_phase5_contract.py`
- 修改：`swarm_test/scripts/check_experiment_csv_fields.py`
- 修改：`swarm_test/scripts/verify_safety_bound.py`

- [ ] **步骤 1：在正式代码修改前扩展静态检查**

要求检查 semantic ablation 参数、`h_see` 安全验证，以及新的 planner CSV 字段。

- [ ] **步骤 2：运行检查并确认其失败**

运行：
```bash
python3 swarm_test/scripts/check_phase5_contract.py
python3 -m py_compile swarm_test/scripts/verify_safety_bound.py swarm_test/scripts/check_experiment_csv_fields.py
```

预期结果：在 C++ 代码更新前，contract 检查应失败。

- [ ] **步骤 3：将验证器更新为使用 `h_see` 和正向 beta 增量**

`verify_safety_bound.py` 应验证 `h_see >= -(eps_max + observed_delta_beta_max or configured delta_bar_beta) / gamma`，并输出每个障碍物的摘要。

### 任务 2：Semantic Guard 消融开关

**文件：**
- 修改：`planner/semantic_guard/src/beta_ground_truth_node.cpp`
- 修改：`planner/semantic_guard/src/beta_guard_node.cpp`
- 修改：`planner/semantic_guard/launch/beta_ground_truth.launch`
- 修改：`swarm_test/launch/secbf_planner.launch`

- [ ] **步骤 1：新增参数**

新增 `semantic_mode`、`enable_rate_limit`、`enable_available_projection`、`enable_guard_fallback` 和 `fixed_beta`。

- [ ] **步骤 2：实现 beta 选择逻辑**

支持 `full`、`category_only`、`context_only`、`fixed` 和 `none`。保持代码标准安全函数 `h_SEE = h_EE - beta` 不变。

- [ ] **步骤 3：记录消融状态**

在 `margin_guard_log.csv` 中记录 `semantic_mode`、`rate_limited`、`projection_active` 和 `delta_beta`。

### 任务 3：MPC Slack 与可行性指标

**文件：**
- 修改：`planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h`
- 修改：`planner/mpc_secbf/src/mpc_secbf.cpp`
- 修改：`planner/mpc_secbf/src/mpc_secbf_node.cpp`
- 修改：`planner/mpc_secbf/launch/mpc_secbf.launch`

- [ ] **步骤 1：加入软 SECBF slack**

为每个活跃障碍物和预测时域约束创建有界 slack 变量。将 slack 加入目标函数惩罚，并保存 `last_slack_sum`、`last_slack_mean` 和 `last_slack_max`。

- [ ] **步骤 2：加入可行性尝试遥测**

记录 `first_attempt_status`、`final_status`、`used_fallback`、`accepted_beta_source`、`solve_time_ms`，以及障碍物数量和 beta 数量。

- [ ] **步骤 3：保留 no-CBF fallback 作为最终应急行为**

先尝试 candidate beta，再尝试上一次接受的 beta，第三步尝试 zero semantic beta，最后再使用 no-CBF fallback。

### 任务 4：Runner Baseline 与结果汇总

**文件：**
- 修改：`swarm_test/scripts/run_secbf_sim_experiments.py`

- [ ] **步骤 1：新增 baseline 矩阵**

新增 `Context_only`、`No_rate_limit`、`No_projection`、`No_mpc_guard`、`No_semantic` 和 `Unguarded_SEESM`。

- [ ] **步骤 2：传递新的 launch 参数**

将 semantic 与 MPC guard 参数转发到 `secbf_planner.launch`。

- [ ] **步骤 3：汇总论文指标**

向 `summary.csv` 中加入 infeasibility/fallback rates、slack 统计、最大正向 delta beta、最小 `h_see` 和计算时间。

### 任务 5：验证

**文件：**
- 现有脚本与 ROS package。

- [ ] **步骤 1：运行静态脚本检查**

运行：
```bash
python3 swarm_test/scripts/check_phase5_contract.py
python3 -m py_compile swarm_test/scripts/run_secbf_sim_experiments.py swarm_test/scripts/verify_safety_bound.py swarm_test/scripts/analyze_exp2_category.py swarm_test/scripts/analyze_exp3_context.py swarm_test/scripts/check_experiment_csv_fields.py
```

- [ ] **步骤 2：构建本次涉及的 ROS package**

运行：
```bash
source /opt/ros/noetic/setup.bash
catkin_make --source . -DCATKIN_WHITELIST_PACKAGES="semantic_guard;mpc_secbf;swarm_test" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"
```

- [ ] **步骤 3：dry-run runner 命令**

运行：
```bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario Exp2_category_box --baseline SEESM_Ours,No_rate_limit,No_projection,No_mpc_guard --dry-run
```
