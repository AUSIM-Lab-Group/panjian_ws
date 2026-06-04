# 语义安全余量仿真到实车实施计划

> **给后续智能执行者：** 必须使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans` 按任务逐步执行本计划。步骤使用复选框（`- [ ]`）追踪状态。

**目标：** 在不影响现有控制器功能的前提下，先跑通可复现的数值仿真，再完成 Gazebo 或硬件邻近 smoke 检查，最后完成实车代码 dry-run。

**架构：** 保持 `planner/mpc_secbf` 与 `planner/semantic_guard` 独立于现有 `planner/mpc_dcbf` 控制器族。数值仿真阶段使用 ground-truth 语义 beta，实车代码阶段使用 YOLO + semantic_fusion + Guard，物理运动前通过 dry-run remap 保护 `/cmd_vel`。每一次实施改动都必须记录到 `.kiro/specs/semantic-safety-margin/change-record.md`。

**技术栈：** ROS Noetic、catkin、C++14、Python 3、CasADi 3.7、Ipopt、Eigen、YAML、YOLOv8、semantic_fusion 消息、Scout/D435/LiDAR 实车链路。

---

## 审批门槛

本文档只是执行计划。用户审阅并明确批准前，不实施代码、不运行长时间实验、不向机器人发送运动命令。

执行必须从 `superpowers:using-git-worktrees` 或等效隔离检查开始。当前工作区已有未提交改动，包括语义安全代码改动、`README.md`、`state_estimation/FAST_LIO/PCD/` 和 `论文思路.md`；必须保留所有用户改动，不能回退无关文件。

## 公式契约

所有代码、测试和文档必须与下列公式保持一致：

```text
h_EE(i,k)  = ||p_rel(i,k) + tau * v_rel(i,k)|| - R_obs,i - R_robot
h_SEE(i,k) = h_EE(i,k) - beta_i,k

Guard: beta_hat_i,k <= h_EE(i,k) - eta
MPC:   h_SEE(i,k+1) >= (1 - gamma) * h_SEE(i,k)
```

baseline 等价关系：

```text
Zhang2026 fixed d_safe = 0.4
当所有障碍物的 beta_i 恒定为 0.4 时，我们的方法退化为该 baseline。
```

代码命名规则：

```text
robot_radius 表示机器人本体半径。
beta 表示语义安全余量。
不得在 MPC-SECBF 内重新引入额外固定语义距离。
```

## 文件职责

### 计划与记录

- 创建或更新：`.kiro/specs/semantic-safety-margin/execution-plan.md`
  - 保存本文档这份经审阅的执行计划。
- 创建或更新：`.kiro/specs/semantic-safety-margin/change-record.md`
  - 每一次实现或验证改动追加一条记录。
- 仿真通过后修改：`.kiro/specs/semantic-safety-margin/requirements.md`
  - 让需求文档与独立 `mpc_secbf` 实现和当前公式契约对齐。
- 仿真通过后修改：`.kiro/specs/semantic-safety-margin/design.md`
  - 用当前 Guard + MPC-SECBF 架构替换过时设计片段。
- 仿真通过后修改：`.kiro/specs/semantic-safety-margin/tasks.md`
  - 只把有验证证据的项目标记为完成，并记录证据。
- 端到端验证后修改：`README.md`
  - 添加面向使用者的启动和验证说明。
- 端到端验证后修改：`Record.md`
  - 添加已验证仿真和实车代码结果的简要时间线。

### 核心运行时代码

- 仅当验证发现不一致时修改：`planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h`
  - 保持 solver 接口明确包含 `robot_radius` 和语义 `beta`。
- 仅当验证发现不一致时修改：`planner/mpc_secbf/src/mpc_secbf.cpp`
  - 保持 `h_SEE = h_EE - beta_i`，并使用全局规划器给出的预测障碍物位置。
- 仅当验证发现不一致时修改：`planner/mpc_secbf/src/mpc_secbf_node.cpp`
  - 保持 topic 订阅和 solver 参数加载稳定。
- 仅当验证发现不一致时修改：`planner/mpc_secbf/launch/mpc_secbf.launch`
  - 暴露运行参数，同时不改变现有默认行为。
- 仅当验证发现不一致时修改：`planner/semantic_guard/src/beta_guard_node.cpp`
  - 实车链路 Guard 必须显式使用 `p_rel + tau * v_rel` 前瞻。
- 仅当验证发现不一致时修改：`planner/semantic_guard/src/beta_ground_truth_node.cpp`
  - 仿真 Guard 必须显式使用 `p_rel + tau * v_rel` 前瞻。
- 仅当验证发现不一致时修改：`planner/semantic_guard/config/semantic_safety_margin.yaml`
  - 保存 `beta_bar`、`mu_weights`、`guard`、`cbf` 和 `yolo` 参数。

### 仿真与实验脚手架

- 修改：`swarm_test/config/secbf_scenarios.yaml`
  - 作为 S1-S4 场景定义和 B1-B3 baseline 标签的单一来源。
- 修改：`swarm_test/launch/secbf_planner.launch`
  - 数值仿真规划链路，使用 ground-truth 语义 beta。
- 修改：`swarm_test/launch/start_test.launch`
  - 数值仿真的动态障碍物生成和数据处理器。
- 创建：`swarm_test/scripts/run_secbf_sim_experiments.py`
  - 运行 S1-S4 × B1-B3，采集日志并写入汇总表。
- 修改：`swarm_test/scripts/verify_safety_bound.py`
  - 兼容按单次实验目录组织的输出。
- 修改：`swarm_test/scripts/measure_latency.py`
  - 记录仿真和实车 dry-run 的端到端 topic 延迟。

### Gazebo 与实车代码集成

- 数值仿真通过后再修改：`swarm_test/launch/start_gazebo_env.launch`
  - Gazebo smoke 入口。
- 数值仿真通过后再修改：`simulation_tools/dynamic_simulator/config/obstacles_param_gazebo.yaml`
  - 为 Gazebo 障碍物 smoke 测试添加语义类别。
- 数值仿真通过后再修改：`swarm_test/launch_exp/exp_secbf_planner.launch`
  - 实车规划链路，包含语义融合、Guard、全局规划器和 MPC-SECBF。
- dry-run 审查后才修改：`swarm_test/launch_exp/exp_hardware.launch`
  - 除非 topic 名称阻塞实车代码 dry-run，否则硬件 bringup 保持不变。
- dry-run 审查后才修改：`swarm_test/launch_exp/start_perception.launch`
  - 除非 YOLO/fusion topic 接线错误，否则感知 bringup 保持不变。
- dry-run 审查后才修改：`perception/semantic_detection/launch/yolo.launch`
  - YOLO 输入 topic 和模型路径。
- dry-run 审查后才修改：`perception/semantic_fusion/launch/fusion.launch`
  - fusion topic 和坐标系参数。

## 验证命令

除非执行环境缺少依赖，否则使用以下命令。若缺少依赖，必须在改动记录中写明缺少的依赖和替代命令。

### 构建命令

```bash
source /opt/ros/noetic/setup.bash
mkdir -p /tmp/panjian_verify_ws/src
ln -sfn /home/lxr20/lxr/panjian_ws /tmp/panjian_verify_ws/src/panjian_ws
cd /tmp/panjian_verify_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="semantic_fusion;semantic_detection;semantic_guard;mpc_secbf;swarm_test;dynamic_simulator;map_generator;robot_simulator;laser_simulator;traj_planner;plan_env" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"
```

期望证据：

```text
[100%] Built target semantic_fusion_node
[100%] Built target beta_guard_node
[100%] Built target beta_ground_truth_node
[100%] Built target mpc_secbf_node
```

### 公式静态检查

```bash
cd /home/lxr20/lxr/panjian_ws
rg -n "robot_radius|h_EE|h_SEE|tau_|lookahead_rel_pos|beta_i" planner/mpc_secbf planner/semantic_guard
```

期望证据：

```text
planner/mpc_secbf/src/mpc_secbf.cpp contains h_EE = distance - obstacle radius - robot_radius
planner/mpc_secbf/src/mpc_secbf.cpp contains h_SEE = h_EE - beta_i
planner/semantic_guard/src/beta_guard_node.cpp contains p_rel + tau_ * v_rel
planner/semantic_guard/src/beta_ground_truth_node.cpp contains p_rel + tau_ * v_rel
```

### 现有控制器回归构建

```bash
source /opt/ros/noetic/setup.bash
cd /tmp/panjian_verify_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="mpc_dcbf;swarm_test;dynamic_simulator;map_generator;robot_simulator;laser_simulator;traj_planner;plan_env" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"
```

期望证据：

```text
mpc_dcbf builds without requiring any semantic safety package change.
Existing launch files for controller modes 0-4 remain present.
```

## 任务 1：执行隔离与基线快照

**文件：**
- 修改：`.kiro/specs/semantic-safety-margin/change-record.md`

- [ ] **步骤 1：检测工作区隔离状态**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
GIT_DIR=$(cd "$(git rev-parse --git-dir)" && pwd -P)
GIT_COMMON=$(cd "$(git rev-parse --git-common-dir)" && pwd -P)
BRANCH=$(git branch --show-current)
SUPERPROJECT=$(git rev-parse --show-superproject-working-tree 2>/dev/null || true)
printf 'GIT_DIR=%s\nGIT_COMMON=%s\nBRANCH=%s\nSUPERPROJECT=%s\n' "$GIT_DIR" "$GIT_COMMON" "$BRANCH" "$SUPERPROJECT"
```

期望：输出路径能说明当前是否已经处在隔离工作区。如果尚未隔离，由于当前树里有重要未提交改动，创建 worktree 前必须询问用户。

- [ ] **步骤 2：记录当前 dirty tree**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
git status --short
```

期望：输出修改和未跟踪文件列表。将摘要复制到 `.kiro/specs/semantic-safety-margin/change-record.md`。

- [ ] **步骤 3：从临时 catkin 工作区构建语义包**

运行“验证命令”中的构建命令。

期望：`semantic_fusion_node`、`beta_guard_node`、`beta_ground_truth_node` 和 `mpc_secbf_node` 构建成功。

- [ ] **步骤 4：构建现有控制器栈**

运行“现有控制器回归构建”命令。

期望：`mpc_dcbf` 和现有仿真包在不编辑语义包的情况下构建成功。

- [ ] **步骤 5：追加基线结果**

编辑 `.kiro/specs/semantic-safety-margin/change-record.md` 并添加：

```markdown
### 2026-05-23 - 基线快照

阶段：任务 1
意图：实施前记录工作区状态。
改动文件：
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `git status --short`
- semantic package catkin build
- existing controller catkin build
验证：
- 已记录语义包构建结果。
- 已记录现有控制器构建结果。
结果：
- 只有在两个构建结果都被理解后才能继续执行。
回滚：
- 仅文档记录，无运行时代码回滚需求。
备注：
- 已保留无关用户改动。
```

## 任务 2：将规格文档与当前代码契约对齐

**文件：**
- 修改：`.kiro/specs/semantic-safety-margin/requirements.md`
- 修改：`.kiro/specs/semantic-safety-margin/design.md`
- 修改：`.kiro/specs/semantic-safety-margin/tasks.md`
- 修改：`.kiro/specs/semantic-safety-margin/change-record.md`

- [ ] **步骤 1：扫描过时命名和接口假设**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
rg -n "fixed safe|controller_type=5|8×N|safe_dist|R_robot|robot_radius|mpc_cbf.cpp|obs_matrix" .kiro/specs/semantic-safety-margin
```

期望：定位需要对齐措辞的规格文件位置。

- [ ] **步骤 2：更新 requirements**

进行以下明确编辑：

```text
FR-3.3：描述 beta_i 是独立 MPC-SECBF solver 使用的语义余量。
FR-3.4：描述 MPC-SECBF 是独立的 `mpc_secbf_node`，不是 legacy controller 内部的新模式。
FR-3.5：说明 legacy controller modes 0-4 仍位于 `mpc_dcbf`，不被语义模块修改。
FR-3.6：按 `mpc_secbf_node` 和 solver fallback 的当前实现描述 fallback 行为。
Interface section：描述 `/safety_margin/beta` 与 `/globalFsm_by_adsm/obs_predict_pub`，不再写扩展 legacy obstacle matrix。
Formula text：用 `robot_radius` 表示机器人本体半径，用 beta 表示语义余量。
```

- [ ] **步骤 3：更新 design**

进行以下明确编辑：

```text
Node graph：实车链路由 `/semantic_obstacles` 输入 `beta_guard_node`；数值仿真链路使用 `beta_ground_truth_node`。
MPC-SECBF section：写明 `h_EE = ||p_obs_pred - p_robot|| - R_obs - R_robot`，`h_SEE = h_EE - beta_i`。
Guard section：写明 `h_EE = ||p_rel + tau * v_rel|| - R_obs - R_robot`。
Compatibility section：写明 `planner/mpc_dcbf` 是回归目标，本实施不修改它。
```

- [ ] **步骤 4：更新 tasks**

进行以下明确编辑：

```text
只有当前构建或日志能证明完成的任务才保持 completed。
Phase 5.1、Phase 5.2 B2/S3、Phase 5.3 full pipeline latency、Phase 6.2、Phase 6.3 和 Phase 7 继续保持待执行。
每个 completed 项下添加 evidence line，写明证明它的命令或日志文件。
```

- [ ] **步骤 5：验证文档**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
rg -n "controller_type=5|8×N|mpc_cbf.cpp|safe_dist" .kiro/specs/semantic-safety-margin/requirements.md .kiro/specs/semantic-safety-margin/design.md .kiro/specs/semantic-safety-margin/tasks.md
```

期望：除清楚标注为 Zhang2026 或 legacy ACBF 历史 baseline 的描述外，不再出现过时集成说法。

## 任务 3：数值仿真脚手架

**文件：**
- 修改：`swarm_test/config/secbf_scenarios.yaml`
- 修改：`swarm_test/launch/secbf_planner.launch`
- 修改：`swarm_test/launch/start_test.launch`
- 创建：`swarm_test/scripts/run_secbf_sim_experiments.py`
- 修改：`swarm_test/scripts/verify_safety_bound.py`
- 修改：`.kiro/specs/semantic-safety-margin/change-record.md`

- [ ] **步骤 1：让场景和 baseline 参数可从 launch 传入**

在 `swarm_test/launch/secbf_planner.launch` 中添加 launch args：

```xml
<arg name="scenario_id" default="S4_mixed"/>
<arg name="baseline_id" default="B3_SECBF_with_guard"/>
<arg name="guard_enabled" default="true"/>
<arg name="output_dir" default="$(find swarm_test)/output/secbf_runs/manual"/>
```

用 `output_dir` 设置 Guard CSV 路径：

```xml
<param name="log_path" value="$(arg output_dir)/guard_log.csv"/>
```

将 `guard_enabled` 传入 `beta_ground_truth_node`：

```xml
<param name="guard/enabled" value="$(arg guard_enabled)"/>
```

- [ ] **步骤 2：让 start_test 输出目录可配置**

在 `swarm_test/launch/start_test.launch` 中添加 launch args：

```xml
<arg name="scenario_index" default="4"/>
<arg name="controller_index" default="6"/>
<arg name="output_dir" default="$(find swarm_test)/output/secbf_runs/manual"/>
```

在 `data_processor_node` 中使用这些 args：

```xml
<param name="scenario" value="$(arg scenario_index)" type="int"/>
<param name="controller" value="$(arg controller_index)" type="int"/>
<param name="output_csv" value="$(arg output_dir)/data_processor_summary.csv" type="string"/>
<param name="output_csv_dist" value="$(arg output_dir)/data_processor_distance.csv" type="string"/>
```

在 `global_path_data_processor.py` 中使用 `output_dir`：

```xml
<param name="output_dir" value="$(arg output_dir)" type="string"/>
```

- [ ] **步骤 3：添加实验 runner**

创建 `swarm_test/scripts/run_secbf_sim_experiments.py`，支持这些 CLI 选项：

```text
--scenario S1_pedestrian_crossing|S2_child_sudden|S3_feasibility_critical|S4_mixed|all
--baseline B1_ACBF_fixed|B2_SECBF_no_guard|B3_SECBF_with_guard|all
--duration-sec 90
--output-root /home/lxr20/lxr/panjian_ws/src/swarm_test/output/secbf_runs
--roscore auto|external
--dry-run
```

runner 行为：

```text
1. 为每个 scenario-baseline 组合创建带时间戳的输出目录。
2. 当 `--roscore auto` 时启动 `roscore`。
3. B2 和 B3 启动 `roslaunch swarm_test secbf_planner.launch`。
4. B1 启动 `roslaunch swarm_test acbf0_planner.launch`。
5. 启动 `roslaunch swarm_test start_test.launch`，传入匹配的 scenario 和 controller index。
6. 运行到 duration 结束或检测到到达证据。
7. 逆序停止子进程。
8. 对存在 `guard_log.csv` 的 B2/B3 运行 `verify_safety_bound.py`。
9. 写入 `summary.md` 和 `summary.csv`。
```

- [ ] **步骤 4：smoke 运行 B3 S4**

运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
cd /home/lxr20/lxr/panjian_ws
python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario S4_mixed --baseline B3_SECBF_with_guard --duration-sec 60 --roscore auto
```

期望：

```text
guard_log.csv exists.
data_processor_summary.csv exists.
verify_safety_bound.py reports Safety guaranteed: True.
No node exits with a segmentation fault.
```

- [ ] **步骤 5：smoke 运行 B2 S3**

运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
cd /home/lxr20/lxr/panjian_ws
python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario S3_feasibility_critical --baseline B2_SECBF_no_guard --duration-sec 60 --roscore auto
```

期望：

```text
guard_log.csv exists.
summary records that Guard was disabled.
run produces evidence for the feasibility-critical comparison.
如果没有出现预期 violation，则记录观察到的 min h，只调整场景参数，不调整安全公式。
```

## 任务 4：完整数值仿真矩阵

**文件：**
- 修改：`swarm_test/scripts/run_secbf_sim_experiments.py`
- 修改：`swarm_test/config/secbf_scenarios.yaml`
- 修改：`.kiro/specs/semantic-safety-margin/tasks.md`
- 修改：`.kiro/specs/semantic-safety-margin/change-record.md`

- [ ] **步骤 1：运行 S1-S4 与 B1-B3**

运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
cd /home/lxr20/lxr/panjian_ws
python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario all --baseline all --duration-sec 90 --roscore auto
```

期望：

```text
生成 12 个 run directories。
每个 B2 和 B3 run 包含 guard_log.csv。
每个 run 包含 summary.md 和 summary.csv。
```

- [ ] **步骤 2：验证所有 B3 场景安全下界**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
for csv in swarm_test/output/secbf_runs/*/B3_SECBF_with_guard*/guard_log.csv; do
  python3 swarm_test/scripts/verify_safety_bound.py --csv "$csv" --gamma 0.35 --eps_max 0.05 --delta_bar_beta 0.3
done
```

期望：

```text
每个 B3 run reports Safety guaranteed: True。
每个 B3 run reports rollback statistics。
```

- [ ] **步骤 3：验证 B2 S3 对照**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
python3 swarm_test/scripts/verify_safety_bound.py --csv "$(find swarm_test/output/secbf_runs -path '*S3_feasibility_critical*B2_SECBF_no_guard*guard_log.csv' | sort | tail -n 1)" --gamma 0.35 --eps_max 0.05 --delta_bar_beta 0.3
```

期望：

```text
B2 S3 结果被记录为 no-Guard 对照案例。
报告包含 min h、theoretical bound 和 Guard disabled status。
```

- [ ] **步骤 4：用证据更新任务状态**

编辑 `.kiro/specs/semantic-safety-margin/tasks.md`：

```text
只有当全部 12 个 run 存在时，才将 Phase 5.1 标记完成。
只有当 contrast case 已文档化时，才将 Phase 5.2 B2/S3 标记完成。
Phase 5.3 real end-to-end latency 在实车代码 dry-run 产生 latency log 前保持 pending。
```

## 任务 5：现有功能回归

**文件：**
- 修改：`.kiro/specs/semantic-safety-margin/change-record.md`

- [ ] **步骤 1：构建现有控制器包**

运行“现有控制器回归构建”命令。

期望：不修改 `planner/mpc_dcbf` 即可构建成功。

- [ ] **步骤 2：启动 legacy ACBF smoke test**

终端 A 运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
roslaunch swarm_test acbf0_planner.launch show_rviz:=false
```

终端 B 运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
roslaunch swarm_test start_test.launch scenario_index:=3 controller_index:=4 output_dir:=/home/lxr20/lxr/panjian_ws/src/swarm_test/output/regression_acbf
```

终端 C 运行：

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /cmd_vel1
rostopic hz /robot1/odom
```

期望：

```text
/cmd_vel1 publishes.
/robot1/odom publishes.
No semantic safety node is required for this run.
```

- [ ] **步骤 3：记录未改动 package 检查**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
git diff -- planner/mpc_dcbf
```

期望：

```text
除非用户明确批准 legacy-controller change，否则 planner/mpc_dcbf 无 diff。
```

## 任务 6：Gazebo 或硬件邻近 smoke 层

**文件：**
- 修改：`swarm_test/launch/start_gazebo_env.launch`
- 修改：`simulation_tools/dynamic_simulator/config/obstacles_param_gazebo.yaml`
- 修改：`.kiro/specs/semantic-safety-margin/change-record.md`

- [ ] **步骤 1：检查现有 Gazebo launch 接线**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
sed -n '1,240p' swarm_test/launch/start_gazebo_env.launch
sed -n '1,240p' simulation_tools/dynamic_simulator/config/obstacles_param_gazebo.yaml
```

期望：识别 obstacle count、obstacle topics、odometry topic 和 command topic。

- [ ] **步骤 2：为 Gazebo 障碍物添加语义类别参数**

在 `simulation_tools/dynamic_simulator/config/obstacles_param_gazebo.yaml` 中添加与 S1-S4 类别匹配的 semantic class 字段。

期望类别列表：

```text
S1: pedestrian
S2: child
S3: child
S4: pedestrian, vehicle, box
```

- [ ] **步骤 3：Gazebo smoke run**

运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
roslaunch swarm_test start_gazebo_env.launch
```

第二个终端运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
roslaunch swarm_test exp_secbf_planner.launch v_max:=0.3
```

期望：

```text
Gazebo starts.
Planner nodes start.
/safety_margin/beta publishes when semantic input is available.
No physical robot command topic is used in this step.
```

## 任务 7：实车代码 dry-run

**文件：**
- 仅当 topic 接线阻塞 dry-run 时修改：`swarm_test/launch_exp/exp_secbf_planner.launch`
- 仅当 topic 接线阻塞 dry-run 时修改：`swarm_test/launch_exp/start_perception.launch`
- 仅当 topic 接线阻塞 dry-run 时修改：`perception/semantic_detection/launch/yolo.launch`
- 仅当 topic 接线阻塞 dry-run 时修改：`perception/semantic_fusion/launch/fusion.launch`
- 修改：`.kiro/specs/semantic-safety-margin/change-record.md`

- [ ] **步骤 1：构建实车代码 package set**

运行：

```bash
source /opt/ros/noetic/setup.bash
cd /tmp/panjian_verify_ws
catkin_make -j1 -l1 -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver;fast_lio;dynamic_perception;semantic_detection;semantic_fusion;semantic_guard;mpc_secbf;swarm_test;traj_planner;map_generator;plan_env" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"
```

期望：

```text
Real perception, fusion, Guard, planner, and MPC-SECBF packages build.
Missing hardware-driver dependencies are recorded with exact package names.
```

- [ ] **步骤 2：在无自主运动条件下启动感知**

运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
roslaunch swarm_test start_perception.launch
```

期望 topics：

```text
/camera/color/image_raw
/camera/color/camera_info
/yolo/detections
/fastLIO/non_ground_points
/Odometry
```

- [ ] **步骤 3：启动 SECBF planner，并把命令 remap 到非机器人 topic**

运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
roslaunch swarm_test exp_secbf_planner.launch v_max:=0.2 cmd_vel_topic:=/cmd_vel_secbf_dryrun
```

期望：planner 发布到 `/cmd_vel_secbf_dryrun`，不发布到物理 `/cmd_vel`。

- [ ] **步骤 4：检查实车代码 topics**

运行：

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /semantic_obstacles
rostopic hz /safety_margin/beta
rostopic hz /safety_margin/guard_log
rostopic hz /globalFsm_by_adsm/obs_predict_pub
rostopic hz /cmd_vel_secbf_dryrun
```

期望：

```text
感知活跃时，/semantic_obstacles 接近 10 Hz。
/safety_margin/beta 接近 10 Hz。
/safety_margin/guard_log 接近 10 Hz。
/cmd_vel_secbf_dryrun 发布 planner 输出。
此 dry-run 命令不使用物理 /cmd_vel。
```

- [ ] **步骤 5：测量延迟**

运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
python3 /home/lxr20/lxr/panjian_ws/src/swarm_test/scripts/measure_latency.py --duration 60 --output /home/lxr20/lxr/panjian_ws/src/swarm_test/output/real_dryrun_latency.csv
```

期望：

```text
Latency CSV exists.
End-to-end latency is reported.
Any latency above 70 ms is recorded with the slowest stage.
```

## 任务 8：物理运动审批门槛

**文件：**
- 用户批准后才修改：`swarm_test/launch_exp/exp_secbf_planner.launch`
- 修改：`.kiro/specs/semantic-safety-margin/change-record.md`

- [ ] **步骤 1：向用户呈现实车 dry-run 证据**

报告：

```text
Build status.
Topic rates.
Latency summary.
Guard log path.
Planner command range on /cmd_vel_secbf_dryrun.
Any missing dependency or unstable topic.
```

- [ ] **步骤 2：等待明确的物理运动批准**

用户必须明确写出：

```text
批准实车接入 /cmd_vel
```

没有这条批准，不运行任何向物理机器人 command topic 发布的命令。

- [ ] **步骤 3：批准后只做低速 physical smoke**

运行：

```bash
source /opt/ros/noetic/setup.bash
source /tmp/panjian_verify_ws/devel/setup.bash
roslaunch swarm_test exp_secbf_planner.launch v_max:=0.2
```

期望：

```text
Robot receives bounded low-speed commands.
Operator can stop the robot externally.
Guard and beta logs are written.
```

## 任务 9：文档与证据包

**文件：**
- 修改：`README.md`
- 修改：`Record.md`
- 修改：`.kiro/specs/semantic-safety-margin/tasks.md`
- 修改：`.kiro/specs/semantic-safety-margin/change-record.md`
- 创建：`swarm_test/output/secbf_runs/latest_experiment_summary.md`

- [ ] **步骤 1：生成仿真摘要**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario all --baseline all --dry-run
```

期望：

```text
runner 打印准确的 launch matrix，但不启动 ROS 进程。
```

然后把已完成的 run 输出聚合到：

```text
swarm_test/output/secbf_runs/latest_experiment_summary.md
```

必要表格列：

```text
scenario, baseline, arrival_time_s, min_distance_m, h_min, beta_mean, guard_rollback_count, safety_bound_passed, output_dir
```

- [ ] **步骤 2：更新 README**

添加章节：

```text
MPC-SECBF formula contract.
Numerical simulation launch commands.
Experiment runner commands.
Real-code dry-run commands.
Physical motion approval rule.
Output file locations.
```

- [ ] **步骤 3：更新 Record**

添加简短日期条目：

```text
2026-05-23 Semantic safety margin simulation-to-real validation plan executed.
List build, simulation, regression, Gazebo or hardware-adjacent, and real-code dry-run results.
```

- [ ] **步骤 4：完成前最终验证**

运行：

```bash
cd /home/lxr20/lxr/panjian_ws
git status --short
rg -n "controller_type=5|8×N|mpc_cbf.cpp|safe_dist" .kiro/specs/semantic-safety-margin README.md Record.md
rg -n "cmd_vel_secbf_dryrun|批准实车接入 /cmd_vel|h_SEE|h_EE" .kiro/specs/semantic-safety-margin README.md
```

期望：

```text
dirty tree 只包含有意改动的文件和保留的用户文件。
文档包含当前公式契约。
文档包含实车运动审批规则。
除历史 baseline 描述外，不再有过时集成说法。
```

## 验收门槛

- [ ] 计划已由用户审阅并批准。
- [ ] 工作区隔离决策已记录。
- [ ] 语义包构建通过。
- [ ] 现有控制器栈构建通过。
- [ ] S4 B3 数值 smoke 通过安全下界验证。
- [ ] S3 B2 no-Guard 对照已记录。
- [ ] S1-S4 × B1-B3 数值矩阵产生 12 个输出目录。
- [ ] 现有 ACBF smoke test 在无语义节点条件下发布 `/cmd_vel1`。
- [ ] Gazebo 或硬件邻近 smoke 启动时不向物理机器人输出命令。
- [ ] 实车代码 dry-run 发布 `/cmd_vel_secbf_dryrun`，不发布物理 `/cmd_vel`。
- [ ] 端到端 latency log 存在。
- [ ] 物理 `/cmd_vel` 运行在用户明确批准前保持阻塞。
- [ ] README、Record、requirements、design、tasks 和 change record 均反映实测证据。

## 批准后的执行选项

1. 子代理驱动执行：为文档对齐、仿真脚手架、实车代码 dry-run 检查分别派发隔离 worker，并在每个任务后审查。
2. 当前会话内执行：在本会话中逐项执行，并在每个验收门槛后停下汇报。

推荐路径：任务 1 和任务 2 用当前会话内执行；任务 3 的仿真脚手架和任务 7 的实车代码 topic 审计彼此独立，可用子代理驱动执行。
