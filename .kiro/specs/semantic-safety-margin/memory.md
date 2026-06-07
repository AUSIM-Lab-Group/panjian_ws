# Semantic Safety Margin Memory

本文档是语义安全余量工作的长期记忆文件。每次修改代码、launch、实验脚本、论文公式或实验文档后，都要在本文档追加一条记录，避免后续忘记改动原因、验证证据和回滚方式。

原始逐条执行记录见：

- `.kiro/specs/semantic-safety-margin/change-record.md`

## 写入规则

- 任何代码更改前后都要追加记录；文档更改如果影响实验、公式、接口或运行流程，也要记录。
- 每条记录至少包含：日期、改动目的、改动文件、关键内容、验证命令、结果、回滚方式。
- 没有新鲜命令证据时，不写“构建通过”“仿真通过”“已验证实车”等结论。
- 不回退无关 dirty tree；遇到已有未提交改动时先记录状态，再只改本任务相关文件。
- 物理机器人相关命令必须继续遵守：用户明确批准前，不向真实 `/cmd_vel` 发布命令。

## 当前核心契约

- SEESM 安全函数以代码为准：

```text
h_SEE = ||p_rel + tau v_rel|| - R_obs - R_robot - beta_i
```

- 不再额外叠加固定 `R_safe`。固定安全距离 baseline 用 `beta_i ≡ d_safe` 表示。
- 相对量口径采用代码定义：`p_rel` 从机器人指向障碍物，`v_rel` 为障碍物相对机器人速度。
- Guard 使用带裕度的可接受集合：

```text
b_i^eta(X_t) = min(beta_bar_i, max(0, h_EE(X_t) - eta))
B_i^eta(X_t) = [0, b_i^eta(X_t)]
```

- Guard 更新策略按当前论文公式整理为：候选余量投影、上一帧余量投影回退、最后退到 `0`。
- 全局规划显式接入 `beta_i` 之前，`Global/Local` 实验只作为增强实验，不作为核心必做结论。
- `Group-aware` 也作为增强实验，当前投稿最小实验链优先完成日志、类别、上下文、Guard、主对比、实时性和小规模实车展示。

## 历史改动摘要

### 2026-05-23 - 建立语义安全余量执行记录体系

改动来源：
- `.kiro/specs/semantic-safety-margin/change-record.md`

关键内容：
- 创建执行计划、任务记录和审批门槛。
- 记录 dirty tree 快照，确认分支为 `feat/semantic-safety-margin`。
- 规定运行时代码和物理机器人操作前必须有记录与审批。

验证/证据：
- 原始记录包含 requirements/design/tasks 读取、相关路径扫描、工作区状态检查。

回滚：
- 删除 `.kiro/specs/semantic-safety-margin/execution-plan.md` 和 `.kiro/specs/semantic-safety-margin/change-record.md` 中对应条目。

### 2026-05-23 - 修复 catkin 消息生成依赖

改动文件：
- `swarm_test/CMakeLists.txt`
- `planner/vomp_planner/traj_planner/CMakeLists.txt`
- `state_estimation/FAST_LIO/CMakeLists.txt`

关键内容：
- 为依赖 `dynamic_simulator/DynTraj.h` 的 `swarm_test` 和 `traj_planner` target 增加 `${catkin_EXPORTED_TARGETS}` 依赖。
- 为 `fastlio_mapping` 增加 `fast_lio/Pose6D.h` 消息生成依赖。

验证/证据：
- 语义仿真相关包构建通过，包含 `semantic_fusion_node`、`mpc_secbf_node`、`beta_guard_node`、`beta_ground_truth_node`、`dynamic_planner_node`。
- legacy `mpc_dcbf` 控制器栈构建回归通过。
- 实车 package set 低并发构建通过，包含 `fastlio_mapping`、`semantic_fusion_node`、`mpc_secbf_node`、`beta_guard_node`。

回滚：
- 删除上述 CMakeLists 中新增的 `add_dependencies(...)` 行。

### 2026-05-23 - 对齐规格文档与当前代码契约

改动文件：
- `.kiro/specs/semantic-safety-margin/requirements.md`
- `.kiro/specs/semantic-safety-margin/design.md`
- `.kiro/specs/semantic-safety-margin/tasks.md`

关键内容：
- 移除旧的 `controller_type=5`、legacy obstacle matrix、`safe_dist/R_safe` 等描述。
- 规格改为描述独立 `mpc_secbf_node`。
- Guard 伪代码对齐 `p_rel + tau * v_rel`。
- MPC-SECBF 伪代码对齐 `robot_radius`、`h_EE`、`h_SEE` 和 `beta_i`。

验证/证据：
- 文档扫描确认旧字段无匹配。
- `rosmsg show semantic_fusion/SemanticObstacleArray` 和 `rosmsg show semantic_guard/GuardLog` 确认消息字段。
- 代码扫描确认 Guard 节点和 MPC-SECBF 公式字段存在。

回滚：
- 使用对应 git hunk 回退三个规格文档。

### 2026-05-23 - 增加 SECBF 仿真实验脚手架和 Guard 开关

改动文件：
- `planner/semantic_guard/src/beta_guard_node.cpp`
- `planner/semantic_guard/src/beta_ground_truth_node.cpp`
- `swarm_test/launch/secbf_planner.launch`
- `swarm_test/launch/start_test.launch`
- `simulation_tools/dynamic_simulator/launch/spawn_dynamic_obstacle.launch`
- `swarm_test/scripts/run_secbf_sim_experiments.py`

关键内容：
- Guard 节点新增 `guard/enabled` 参数，支持 no-Guard baseline。
- 仿真 launch 支持 `scenario_id`、`baseline_id`、`guard_enabled`、`output_dir`、`obstacle_classes`、`obstacle_params_file`。
- 实验脚本生成运行目录、障碍物 YAML、launch 日志、summary 和安全界验证。

验证/证据：
- `python3 -m py_compile swarm_test/scripts/run_secbf_sim_experiments.py`
- dry-run 生成 12 组 scenario/baseline 命令。
- B2 命令包含 `guard_enabled:=false`，B3 命令包含 `guard_enabled:=true`。

回滚：
- 删除实验脚本，并回退 Guard 节点和 launch 文件新增参数。

### 2026-05-23 - 修复仿真运行依赖和数据记录

改动文件：
- `.kiro/specs/semantic-safety-margin/execution-plan.md`
- `swarm_test/launch/start_test.launch`
- `swarm_test/scripts/run_secbf_sim_experiments.py`
- `planner/semantic_guard/src/beta_guard_node.cpp`
- `planner/semantic_guard/src/beta_ground_truth_node.cpp`

关键内容：
- 仿真构建白名单补充 `laser_simulator`。
- `start_test.launch` 新增 `record_data` 参数；实验脚本显式传 `record_data:=true`。
- Guard CSV 输出时间和数值改为固定小数精度，便于安全界验证脚本解析。

验证/证据：
- S4/B3 60 秒 smoke 暴露并修复 `laser_simulator/laser_sim_node` 白名单问题。
- `data_processor_summary.csv`、`data_processor_distance.csv`、Guard CSV 正常生成。

回滚：
- 回退白名单、`record_data` 参数和 Guard CSV 固定精度设置。

### 2026-05-23 - 仿真 smoke 链路验证

改动文件：
- `swarm_test/output/secbf_runs/latest_experiment_summary.md`

关键内容：
- 运行 SECBF with Guard、SECBF no Guard、legacy ACBF 三条基础链路。

验证/证据：
- S4/B3：`guard_log.csv`、`data_processor_summary.csv`、`data_processor_distance.csv` 存在；Guard 记录数 5028，时间跨度 27.90s，`min(h_EE)=0.3802`，rollback 63 次。
- S3/B2：`guard_enabled=false` 生效，rollback 0 次，Guard 记录数 559。
- S4/B1：legacy ACBF 可启动并生成数据 CSV。

回滚：
- 删除对应 `swarm_test/output/secbf_runs/20260523_*` 目录和 `latest_experiment_summary.md`。

### 2026-05-23 - 实车 dry-run 静态和短启动检查

改动文件：
- `swarm_test/launch_exp/exp_secbf_planner.launch`
- `.kiro/specs/semantic-safety-margin/execution-plan.md`
- `swarm_test/output/secbf_runs/latest_experiment_summary.md`

关键内容：
- `exp_secbf_planner.launch` 的可覆盖参数从固定 `value` 改为 `default`。
- 新增 `cmd_vel_topic` 参数，默认仍为 `/cmd_vel`。
- dry-run 使用 `/cmd_vel_secbf_dryrun`，避免发布到真实 `/cmd_vel`。

验证/证据：
- `roslaunch --nodes`、`roslaunch --dump-params`、`roslaunch --args` 静态验证通过。
- 12 秒 planner/perception dry-run 无 `ERROR`、`Exception`、`Traceback`、`process died`。

回滚：
- 将 launch arg 恢复为固定 `value`，并把 `/cmd_vel` remap 恢复为 `/cmd_vel`。

### 2026-05-24 - README 和仿真说明完善

改动文件：
- `README.md`
- `planner/README.md`
- `planner/mpc_secbf/launch/mpc_secbf.launch`
- `planner/semantic_guard/launch/beta_ground_truth.launch`
- `swarm_test/launch/secbf_planner.launch`
- `simulation_tools/robot_simulator/launch/vis_car.launch`
- `.kiro/specs/semantic-safety-margin/tasks.md`

关键内容：
- 明确本文核心代码在 `planner/mpc_secbf` 与 `planner/semantic_guard`。
- `secbf_planner.launch` 改为 include 本文包的 launch，保持包边界清楚。
- README 补充批量实验、RViz 可视化实验、Guard 日志、场景环境总览、两终端数值仿真流程、数据查看方式。
- `vis_car.launch` 补充 `time_res` 默认参数并转传。

验证/证据：
- 多个 `roslaunch --nodes`、`--dump-params`、`--args` 静态解析命令通过。
- README 中可找到 MPC-SECBF、动态语义环境、静态一字排开障碍物环境、Gazebo、数据记录查看等说明。

回滚：
- 回退 README、planner README、相关 launch 和 `vis_car.launch` 参数。

### 2026-06-07 - 论文公式口径按代码统一

改动文件：
- `论文公式.md`

关键内容：
- 统一安全函数为 `h_SEE = ||p_rel + tau v_rel|| - R_obs - R_robot - beta_i`。
- 删除固定 `R_safe/R_d` 口径。
- 将 `R_i` 明确为 `R_r + R_{o,i}`。
- 相对量改为代码口径：`l_i = p_i^o - p(t)`，`v_i = v_i^o - v(t)`。
- `T_i/tau_i` 改为基于闭合速度的非负前向时间。
- Guard 理论改为带 `eta` 的 `B_i^eta(X_t)`，并采用候选投影、上一帧投影回退、最后退到 `0` 的更新律。
- 同步更新 MPC-SECBF 安全函数、闭环 practical safety 推导和核心公式汇总。

验证/证据：
- 运行公式一致性检查，确认旧字符串 `R_{\mathrm{safe}}`、`R_d`、旧 `\mathcal B_i(X_t)` 不再存在。
- 检查确认新口径 `R_i=R_r+R_{o,i}`、`p_rel + tau v_rel`、`R_obs`、`R_robot`、`\mathcal B_i^\eta(X_t)`、投影 Guard 和 `d_tol` 存在。

回滚：
- 删除 `论文公式.md` 中 2026-06-07 相关修改 hunk，恢复旧公式草稿。

备注：

### 2026-06-07 - 推进 Exp3 上下文调制 sanity 实验

改动目的：
- 固定 `adult` 类别，只改变上下文运动状态，验证 `mu(phi)` 和 `beta_i` 会随 `static / same_direction / crossing / frontal_approaching` 变化。
- 为 Exp3 分析补齐 `TTC_norm`、`rho_norm`、`mu`、`beta_hat`、`beta`、`h_EE`、`h_SEE` 等可直接出图字段。

改动文件：
- `swarm_test/config/secbf_scenarios.yaml`
- `swarm_test/scripts/run_secbf_sim_experiments.py`
- `swarm_test/scripts/analyze_exp3_context.py`
- `planner/semantic_guard/src/beta_guard_node.cpp`
- `planner/semantic_guard/src/beta_ground_truth_node.cpp`
- `.kiro/specs/semantic-safety-margin/tasks.md`
- `experiments/实验流程.md`

关键内容：
- 新增 Exp3 场景索引和 `experiment_id=Exp3_context_modulation` 的 meta 写入。
- 新增 4 个 Exp3 单障碍物 adult 上下文场景：
  - `Exp3_context_static`
  - `Exp3_context_same_direction`
  - `Exp3_context_crossing`
  - `Exp3_context_frontal_approaching`
- 修正动态障碍物初始运动方向口径：`scale_x > 0` 对应初始 `+x` 同向运动，`scale_x < 0` 对应初始 `-x` 迎面接近。
- Guard CSV header 和行记录新增 `ttc_norm`、`rho_norm`，便于 Exp3 分析直接读取。
- 新增 `swarm_test/scripts/analyze_exp3_context.py`，输出：
  - `experiments/Exp3_context_modulation/analysis/exp3_run_metrics.csv`
  - `experiments/Exp3_context_modulation/analysis/exp3_summary_stats.csv`
  - `experiments/Exp3_context_modulation/analysis/context_mu_beta_curves.png`
  - `experiments/Exp3_context_modulation/analysis/context_beta_bar.png`
- 更新 `tasks.md`：Task 6.2 的场景配置、日志字段和三联图标记为完成；5 次 sanity 和 20 seed 仍未完成。
- 更新 `experiments/实验流程.md`：补充 Exp3 目的、场景、运行命令、分析命令、当前 sanity 结果和下一步 TODO。

验证/证据：
- `python3 -m py_compile swarm_test/scripts/run_secbf_sim_experiments.py swarm_test/scripts/analyze_exp3_context.py swarm_test/scripts/check_phase5_contract.py swarm_test/scripts/check_experiment_csv_fields.py`
- `python3 swarm_test/scripts/check_phase5_contract.py`
- `roslaunch --nodes swarm_test secbf_planner.launch show_rviz:=false scenario_id:=Exp3_context_frontal_approaching baseline_id:=SEESM_Ours obstacle_classes:='[adult]' map_size_x:=12 map_size_y:=6 goal_x:=12 output_dir:=/tmp/exp3_static`
- `roslaunch --nodes swarm_test start_test.launch output_dir:=/tmp/exp3_static record_data:=true obstacle_classes:='[adult]' goal_x:=12 goal_y:=0`
- `catkin_make --source . -DCATKIN_WHITELIST_PACKAGES="dynamic_simulator;robot_simulator;map_generator;laser_simulator;plan_env;traj_planner;semantic_fusion;semantic_guard;mpc_secbf;swarm_test" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"`
- Exp3 v5 sanity run：

```bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario Exp3_context_static,Exp3_context_same_direction,Exp3_context_crossing,Exp3_context_frontal_approaching \
  --baseline SEESM_Ours \
  --duration-sec 12 \
  --output-root /home/lxr20/lxr/panjian_ws/experiments/Exp3_context_modulation/runs_sanity_v5 \
  --roscore auto
```

- Exp3 v5 分析：

```bash
python3 swarm_test/scripts/analyze_exp3_context.py \
  experiments/Exp3_context_modulation/runs_sanity_v5 \
  --output-dir experiments/Exp3_context_modulation/analysis
```

结果：
- v5 run root：`experiments/Exp3_context_modulation/runs_sanity_v5/20260607_174908_*`
- 4 个 run 均通过 `check_experiment_csv_fields.py`。
- `beta_max` 单次 sanity 满足：

```text
frontal_approaching 0.359888
crossing            0.358382
same_direction      0.322340
static              0.312430
```

- `beta_mean` 单次 sanity 为：

```text
frontal_approaching 0.293768
same_direction      0.275147
crossing            0.272733
static              0.265653
```

注意事项：
- 当前只完成每个上下文 1 次 sanity。`beta_max` 趋势已经能证明上下文峰值调制存在；`beta_mean` 中 crossing 与 same_direction 仍接近，最终论文统计必须补 5 次 sanity 和至少 20 seed 后再下结论。
- 本次仿真使用数值仿真话题 `/cmd_vel1`，没有向真实 `/cmd_vel` 发布命令。

回滚：
- 删除 `swarm_test/config/secbf_scenarios.yaml` 中 Exp3 场景块和 runner 中 Exp3 索引/meta 逻辑。
- 删除 `swarm_test/scripts/analyze_exp3_context.py`。
- 回退 Guard CSV 中 `ttc_norm/rho_norm` 字段变更。
- 删除 `experiments/Exp3_context_modulation/runs_sanity_v*` 和 `experiments/Exp3_context_modulation/analysis/*` 生成物。

### 2026-06-07 - 上传 GitHub 前整理提交忽略规则

改动目的：
- 用户要求将当前状态上传到 GitHub；提交前避免误上传 catkin 构建目录、Python 缓存和 5GB 级临时仿真输出。

改动文件：
- `.gitignore`
- `.kiro/specs/semantic-safety-margin/memory.md`

关键内容：
- `.gitignore` 新增 `.catkin_workspace`、`build/`、`devel/`、`**/__pycache__/`。
- `.gitignore` 新增 `experiments/**/runs*/` 和 `swarm_test/output/secbf_runs/`，避免提交临时批量仿真输出。
- 保留 `experiments/` 下的实验 meta、流程文档、分析 CSV/PNG；原始 run 目录保留在本机，不上传 GitHub。
- 将提交内的分析 CSV 行尾从 CRLF 归一化为 LF，并去掉少量 YAML/脚本文件末尾多余空行，只做提交卫生整理，不改变实验数值。

验证/证据：
- `du -sh experiments swarm_test/output/secbf_runs build devel` 显示 `experiments` 约 11M，`swarm_test/output/secbf_runs` 约 5.3G，后者不适合上传。
- 后续通过 `git status -sb`、`git diff --cached --stat`、`git diff --cached --check` 检查提交范围；原始 run 目录从暂存区撤出。

回滚：
- 删除 `.gitignore` 中本条新增模式，并从 memory 删除本条记录。
- 当前 `论文公式.md` 在 git 状态中显示为未跟踪文件，后续提交前需确认是否加入版本控制。

### 2026-06-07 - 实验设置收缩为可执行投稿最小包

改动文件：
- `实验设置.md`

关键内容：
- 新增“2026-06-07 修订版：按代码口径收缩后的实验执行方案”。
- 明确实验记录不再使用额外固定 `R_safe`；固定距离 baseline 用 `beta_i ≡ d_safe`。
- Guard 口径统一为 `b_i^eta(X_t)` 和 `B_i^eta(X_t)`，日志字段 `guard_status` 至少区分 `accept/project/fallback/zero`。
- Baseline 收缩为 `Fixed-margin`、`Category-only`、`Context w/o Guard`、`SEESM Ours`。
- 将 `Global/Local` 与 `Group-aware` 降为增强实验。
- 投稿最小实验包定为：日志系统、类别感知、上下文调制、Guard 临界、主对比、实时性、小规模实车展示。
- 写入 Phase A--H Todo List，覆盖目录模板、日志字段、每组实验对比方法、统计指标、图表和验收标准。
- 修正旧文中的 `R_safe` 参数字段、`h_SEE` 公式、Guard 上界和投影集合。

验证/证据：
- 运行实验文档检查，确认修订版章节、Todo List、各 Phase、`h_i^{SEE}`、`R_obs`、`R_robot`、`\mathcal B_i^\eta(X_t)`、`guard_eta` 均存在。
- 检查确认旧 `| R_safe |` meta 字段和旧 `R_i + R_safe` 公式不再存在。

回滚：
- 删除 `实验设置.md` 中新增的 2026-06-07 修订版章节，并恢复旧 `R_safe` 字段和旧 Guard 口径。

### 2026-06-07 - 新建长期 memory 文件

改动文件：
- `.kiro/specs/semantic-safety-margin/memory.md`

关键内容：
- 建立长期记忆文件，规定之后每次代码、launch、实验脚本、公式或实验文档改动都要追加日志。
- 将 `.kiro/specs/semantic-safety-margin/change-record.md` 中 2026-05-23 至 2026-05-24 的关键实现、验证和回滚信息整理为摘要。
- 追加 2026-06-07 的论文公式修订和实验设置修订记录。

验证/证据：
- 本文件创建后需通过存在性和关键内容检查。

回滚：
- 删除 `.kiro/specs/semantic-safety-margin/memory.md`。

### 2026-06-07 - 将当前实验计划写入 Kiro tasks

改动文件：
- `.kiro/specs/semantic-safety-margin/tasks.md`
- `.kiro/specs/semantic-safety-margin/memory.md`

关键内容：
- 保留 Phase 1--4 的实现任务和既有完成记录。
- 将旧 Phase 5--7 的 12 组 S1--S4 数值仿真、Gazebo 和旧论文整理计划，替换为当前投稿最小实验链。
- 新 Phase 5：实验数据链打通，包含公式/日志契约、目录和 `meta.yaml` 模板、csv 字段检查。
- 新 Phase 6：组件证据实验，包含类别感知、上下文调制和 Safety Guard 临界实验。
- 新 Phase 7：主结果实验，包含主对比、实时性和小规模实车展示。
- 新 Phase 8：增强实验和论文收尾，包含 Global/Local、Group-aware、图表整理、记录提交和输出清理。
- 顶部执行证据更新为 2026-06-07 当前口径，强调 `h_SEE = ||p_rel + tau v_rel|| - R_obs - R_robot - beta_i`，不再额外叠加固定 `R_safe`。

验证/证据：
- 运行 `tasks and memory checks` 内容验证，确认 Phase 5--8、当前实验任务、Guard 口径和 memory 记录均存在。
- 检查确认旧 `Gazebo 仿真 + 实车准备`、旧 `Gazebo 语义障碍物场景`、旧 `Git 提交 + 推送` 章节不再作为当前待执行任务存在。

回滚：
- 恢复 `.kiro/specs/semantic-safety-margin/tasks.md` 中旧 Phase 5--7 内容，并删除本 memory 条目。

### 2026-06-07 - 开始执行 Phase 5 实验数据链

改动文件：
- `.kiro/specs/semantic-safety-margin/tasks.md`
- `.kiro/specs/semantic-safety-margin/memory.md`
- `experiments/`
- `planner/semantic_guard/src/beta_guard_node.cpp`
- `planner/semantic_guard/src/beta_ground_truth_node.cpp`
- `planner/mpc_secbf/src/mpc_secbf_node.cpp`
- `planner/mpc_secbf/launch/mpc_secbf.launch`
- `swarm_test/launch/secbf_planner.launch`
- `swarm_test/launch/start_test.launch`
- `swarm_test/scripts/run_secbf_sim_experiments.py`
- `swarm_test/scripts/check_phase5_contract.py`
- `swarm_test/scripts/phase5_csv_logger.py`
- `swarm_test/scripts/check_experiment_csv_fields.py`

关键内容：
- 按用户要求开始执行 `.kiro/specs/semantic-safety-margin/tasks.md`。
- 本轮优先范围为 Phase 5：统一实验公式和日志契约、建立实验目录和 `meta.yaml` 模板、完成最小日志字段检查。
- 由于当前 checkout 已包含未提交的实验计划、论文公式和 memory 文件，本轮在当前工作区执行，不创建新的 git worktree，避免丢失这些上下文。
- 将 Guard 上界改为 `min(beta_bar_i, max(0, h_EE - eta))`，并将 Guard 状态改为 `accept/project/fallback/zero`。
- 新增 Phase 5 静态契约检查脚本，检查 Guard 状态、Guard 上界、MPC-SECBF 安全函数和旧 `R_safe/safe_dist` 残留。
- 建立 `experiments/Exp0_log_check`、`Exp1_main_comparison`、`Exp2_category_aware`、`Exp3_context_modulation`、`Exp4_guard_critical`、`Exp7_runtime`、`Exp8_real_world_demo` 及各自 `meta.yaml` 模板。
- `mpc_secbf_node` 新增 `planner_log.csv` 和 `timing_log.csv` 输出，记录 `mpc_status`、`cmd_v/cmd_w`、`slack`、求解耗时和 fallback/zero 状态。
- `start_test.launch` 新增 `phase5_csv_logger.py`，运行时生成 `robot_log.csv`、`obstacle_log.csv` 和 `event_log.csv`。
- `secbf_planner.launch` 将 Guard 详细 CSV 改为 `margin_guard_log.csv`。
- 实验 runner 传递 `obstacle_classes`，并在每次 run 目录生成 run 级 `meta.yaml`。
- 新增字段检查脚本，确认论文图表字段 `t/id/class/d_i/rel_v/TTC/mu/beta_bar/beta_hat/beta/guard_upper_bound/h_EE/h_SEE/slack/mpc_status` 均可从 CSV 读取。
- `.kiro/specs/semantic-safety-margin/tasks.md` 已将 Phase 5.1、5.2、5.3 标记完成。

验证/证据：
- `python3 swarm_test/scripts/check_phase5_contract.py` 通过。
- `source /opt/ros/noetic/setup.bash && catkin_make --source . -DCATKIN_WHITELIST_PACKAGES="dynamic_simulator;robot_simulator;map_generator;laser_simulator;plan_env;traj_planner;semantic_fusion;semantic_guard;mpc_secbf;swarm_test" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"` 通过。
- `python3 -m py_compile swarm_test/scripts/phase5_csv_logger.py swarm_test/scripts/check_experiment_csv_fields.py swarm_test/scripts/run_secbf_sim_experiments.py swarm_test/scripts/check_phase5_contract.py` 通过。
- `roslaunch --nodes swarm_test secbf_planner.launch show_rviz:=false output_dir:=/tmp/seesm_phase5_static` 通过，节点包含 `mpc_secbf_node` 和 `beta_ground_truth_node`。
- `roslaunch --nodes swarm_test start_test.launch output_dir:=/tmp/seesm_phase5_static record_data:=true obstacle_classes:='[pedestrian,box]'` 通过，节点包含 `phase5_csv_logger`。
- `python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario S3_feasibility_critical --baseline B3_SECBF_with_guard --duration-sec 6 --roscore auto` 完成，生成 run：`swarm_test/output/secbf_runs/20260607_124616_S3_feasibility_critical_B3_SECBF_with_guard`。
- 该 run 包含 `meta.yaml`、`robot_log.csv`、`obstacle_log.csv`、`margin_guard_log.csv`、`planner_log.csv`、`timing_log.csv`、`event_log.csv`。
- `python3 swarm_test/scripts/check_experiment_csv_fields.py swarm_test/output/secbf_runs/20260607_124616_S3_feasibility_critical_B3_SECBF_with_guard` 通过。
- run 级 `meta.yaml` 必填字段检查通过，missing 字段为空。
- `verify_safety_bound.txt` 结果：`safety_bound_passed=True`，Guard 记录 86，类别 `child`，`min(h_EE)=0.954823`，`min(h_SEE)=0.328432`。

回滚：
- 删除本轮新增的 `experiments/` 文件、`swarm_test/scripts/check_phase5_contract.py`、`swarm_test/scripts/phase5_csv_logger.py`、`swarm_test/scripts/check_experiment_csv_fields.py` 和对应 output run 目录。
- 回退 Guard 节点、MPC 节点、launch、runner 和 tasks 的本轮 hunk。

### 2026-06-07 - 执行 Task 6.1 Exp2 类别感知 sanity matrix

改动文件：
- `.kiro/specs/semantic-safety-margin/tasks.md`
- `.kiro/specs/semantic-safety-margin/memory.md`
- `planner/semantic_guard/config/semantic_safety_margin.yaml`
- `planner/semantic_guard/src/beta_guard_node.cpp`
- `planner/semantic_guard/src/beta_ground_truth_node.cpp`
- `planner/semantic_guard/launch/beta_ground_truth.launch`
- `swarm_test/src/start_trigger.cpp`
- `swarm_test/launch/start_test.launch`
- `swarm_test/launch/secbf_planner.launch`
- `swarm_test/config/secbf_scenarios.yaml`
- `swarm_test/scripts/run_secbf_sim_experiments.py`
- `swarm_test/scripts/analyze_exp2_category.py`
- `experiments/Exp2_category_aware/meta.yaml`
- `experiments/Exp2_category_aware/runs_static/`
- `experiments/Exp2_category_aware/analysis/`

关键内容：
- 给 `semantic_guard` 增加 `adult` 与 `child_like` 类别别名，分别对齐 `pedestrian=0.4` 与 `child=0.7`。
- `beta_ground_truth.launch` 支持按 run 覆盖 `beta_bar_*` 和 `mu_weights/*`，用于构造 `Fixed_margin`、`Category_only` 和 `SEESM_Ours` 三种方法。
- `start_trigger_node` 改为可配置 `goal_x/goal_y/goal_z`，`start_test.launch` 支持传入目标点，满足 Exp2 的 `(0,0)->(12,0)`。
- `secbf_planner.launch` 支持传入地图尺寸、目标点、beta 表和 mu 权重。
- `secbf_scenarios.yaml` 增加 Exp2 四个单障碍物场景：`box/adult/child_like/cyclist`。第一轮 crossing sanity 暴露 `D_min` 不敏感后，改为静态路径障碍物以形成近距离交互。
- `run_secbf_sim_experiments.py` 增加 Exp2 场景索引、三种 baseline、逗号列表选择、`--repeat`、按 baseline 写 run 级 `meta.yaml`。
- 新增 `analyze_exp2_category.py`，从 run 目录聚合 `D_min, beta_mean, beta_max, path_length, travel_time, success`，输出 per-run 表、mean/std 表和三张图。
- `.kiro/specs/semantic-safety-margin/tasks.md` 已将 Task 6.1 中“地图/单障碍物/三方法对比/轨迹图/D_min-beta 图”标为完成；保留“每类 5 次 sanity”和“20 seed 统计”未完成。

验证/证据：
- `python3 -m py_compile swarm_test/scripts/run_secbf_sim_experiments.py swarm_test/scripts/analyze_exp2_category.py swarm_test/scripts/check_phase5_contract.py swarm_test/scripts/check_experiment_csv_fields.py swarm_test/scripts/phase5_csv_logger.py` 通过。
- `python3 swarm_test/scripts/check_phase5_contract.py` 通过。
- `roslaunch --nodes swarm_test secbf_planner.launch show_rviz:=false scenario_id:=Exp2_category_adult baseline_id:=SEESM_Ours obstacle_classes:='[adult]' map_size_x:=12 map_size_y:=6 goal_x:=12 output_dir:=/tmp/exp2_static` 通过。
- `roslaunch --nodes swarm_test start_test.launch output_dir:=/tmp/exp2_static record_data:=true obstacle_classes:='[adult]' goal_x:=12 goal_y:=0` 通过。
- `catkin_make --source .` 相关白名单构建通过，包含 `start_trigger_node`、`beta_guard_node`、`beta_ground_truth_node`。
- 第一轮 crossing sanity：12 个 run 完成并生成分析，但 `D_min` 约 2.5m，判定交互不足，不作为最终图表依据。
- 第二轮 static-on-path sanity：`python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario Exp2_category_box,Exp2_category_adult,Exp2_category_child_like,Exp2_category_cyclist --baseline Fixed_margin,Category_only,SEESM_Ours --duration-sec 12 --output-root /home/lxr20/lxr/panjian_ws/experiments/Exp2_category_aware/runs_static --roscore auto` 完成 12 个 run。
- 12 个 static sanity run 均通过 `check_experiment_csv_fields.py` 字段检查。
- 12 个 static sanity run 的 `summary.csv` 均为 `safety_bound_passed=True`，Guard records 约 204--207。
- `python3 swarm_test/scripts/analyze_exp2_category.py experiments/Exp2_category_aware/runs_static --output-dir experiments/Exp2_category_aware/analysis` 生成：
  - `experiments/Exp2_category_aware/analysis/exp2_run_metrics.csv`
  - `experiments/Exp2_category_aware/analysis/exp2_summary_stats.csv`
  - `experiments/Exp2_category_aware/analysis/dmin_by_class.png`
  - `experiments/Exp2_category_aware/analysis/beta_by_class.png`
  - `experiments/Exp2_category_aware/analysis/trajectory_by_class.png`
- Static sanity 结果：`Category_only` 中 `D_min(box)=0.328074`、`D_min(adult)=0.367867`、`D_min(child_like)=0.468005`；`SEESM_Ours` 中 `beta_mean(box)=0.071892`、`adult=0.282672`、`cyclist=0.418000`、`child_like=0.487842`。`SEESM_Ours` 的 `D_min` 仍需多 seed 均值确认。

回滚：
- 删除 `experiments/Exp2_category_aware/runs_static/` 与 `experiments/Exp2_category_aware/analysis/`。
- 删除 `swarm_test/scripts/analyze_exp2_category.py`。
- 回退本条所列代码、launch、config、runner、tasks 和 memory hunk。

### 2026-06-07 - 新增实验流程说明文档

改动文件：
- `experiments/实验流程.md`
- `.kiro/specs/semantic-safety-margin/memory.md`

关键内容：
- 在 `experiments/` 下新增实验流程说明，记录实验目的、总流程、核心代码入口、构建命令、静态检查命令、仿真运行方式、CSV 字段检查、分析出图步骤。
- 写明当前 Exp2 类别感知实验的目的、场景、三种 baseline 定义、已运行的 12 秒 static sanity 命令、输出目录、当前 sanity 结果和后续 5 次 sanity/20 seed 计划。
- 明确当前数值仿真使用 `/cmd_vel1`，不是真实机器人 `/cmd_vel`。

验证/证据：
- 运行内容检查确认 `实验总目的`、`核心代码入口`、`Exp2 类别感知实验`、`run_secbf_sim_experiments.py`、`analyze_exp2_category.py`、`runs_static`、`/cmd_vel1` 等关键内容存在。

回滚：
- 删除 `experiments/实验流程.md`，并删除本 memory 条目。

## 当前工作区注意事项

- `实验设置.md` 当前为已跟踪修改。
- `论文公式.md` 当前为未跟踪文件。
- `swarm_test/launch_exp/exp_secbf_planner.launch` 当前还有未提交修改，需确认是否属于用户已有改动或后续任务范围。
- `swarm_test/output/secbf_runs/` 下存在大量未跟踪实验输出，提交前需要决定是否保留、归档或加入 ignore。
