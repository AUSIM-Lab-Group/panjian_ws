# 语义安全余量改动记录

本文档记录语义安全余量“仿真到实车”工作中的每一次实现、验证和文档改动。

规则：

- 编辑运行时代码前先追加记录。
- 每个提升置信度的命令执行后，都追加验证结果。
- 记录改动文件、执行命令、观察结果和回滚路径。
- 保留无关用户改动。
- 没有命令证据时，不声称仿真、构建、dry-run 或实车结果已经通过。
- 用户明确批准前，不向物理机器人 `/cmd_vel` topic 发布命令。

## 记录格式

每条新记录使用以下格式：

```markdown
### 2026-05-23 HH:MM ET - 简短标题

阶段：任务编号或阶段名。
意图：一句话说明为什么做这个改动。
改动文件：
- `path/to/file`
命令：
- `command that was run`
验证：
- 观察到的输出或日志路径。
结果：
- 改动内容或得到的结论。
回滚：
- 精确到文件或命令级别的回滚方案。
备注：
- 约束、已保留的用户改动或后续证据。
```

## 记录

### 2026-05-23 - 创建计划文档

阶段：Planning
意图：在实施“仿真到实车”工作前，创建可审阅的执行计划和改动记录。
改动文件：
- `.kiro/specs/semantic-safety-margin/execution-plan.md`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- 阅读 Superpowers `writing-plans` 和 `using-git-worktrees` skill 说明。
- 阅读 `.kiro/specs/semantic-safety-margin/requirements.md`。
- 阅读 `.kiro/specs/semantic-safety-margin/design.md`。
- 阅读 `.kiro/specs/semantic-safety-margin/tasks.md`。
- 使用 `rg` 和 `sed` 阅读相关 SECBF launch、config 和 script 路径。
验证：
- 计划包含实施前审批门槛。
- 计划包含仿真优先、回归、Gazebo 或硬件邻近 smoke、实车代码 dry-run 和物理运动审批门槛。
- 计划包含使用 `h_EE`、`h_SEE`、`robot_radius` 和语义 `beta` 的公式契约。
结果：
- 本次请求未执行任何运行时代码实施。
- 用户审阅并批准计划前，执行保持阻塞。
回滚：
- 如果用户拒绝此计划结构，删除 `.kiro/specs/semantic-safety-margin/execution-plan.md` 和 `.kiro/specs/semantic-safety-margin/change-record.md`。
备注：
- 当前工作区已有未提交的用户和 assistant 改动；执行时必须保留无关文件。

### 2026-05-23 - 执行隔离检查与 dirty tree 快照

阶段：任务 1
意图：执行计划前确认工作区隔离状态，并记录当前未提交改动。
改动文件：
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `GIT_DIR=$(cd "$(git rev-parse --git-dir)" && pwd -P); GIT_COMMON=$(cd "$(git rev-parse --git-common-dir)" && pwd -P); BRANCH=$(git branch --show-current); SUPERPROJECT=$(git rev-parse --show-superproject-working-tree 2>/dev/null || true); printf ...`
- `git status --short`
验证：
- `GIT_DIR=/home/lxr20/lxr/panjian_ws/.git`
- `GIT_COMMON=/home/lxr20/lxr/panjian_ws/.git`
- `BRANCH=feat/semantic-safety-margin`
- `SUPERPROJECT=`，当前不是 linked worktree。
- dirty tree 包含 `README.md`、`planner/mpc_secbf/*`、`planner/semantic_guard/*`、`.kiro/specs/semantic-safety-margin/execution-plan.md`、`.kiro/specs/semantic-safety-margin/change-record.md`、`state_estimation/FAST_LIO/PCD/` 和 `论文思路.md`。
结果：
- 当前语义安全改动和计划文档均位于普通 checkout 的未提交状态。
- 为保留这些上下文，本轮先在当前工作区谨慎执行，不创建新的 git worktree。
回滚：
- 本条只记录状态；如后续需要隔离，可在用户确认后先提交或迁移当前改动，再创建 worktree。
备注：
- 不回退 `README.md`、`state_estimation/FAST_LIO/PCD/`、`论文思路.md` 或其他无关用户改动。

### 2026-05-23 - 修复 swarm_test 消息生成构建依赖

阶段：任务 1
意图：解决基线构建中 `swarm_test` 并行编译早于 `dynamic_simulator/DynTraj.h` 生成的问题。
改动文件：
- `swarm_test/CMakeLists.txt`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `catkin_make -DCATKIN_WHITELIST_PACKAGES="semantic_fusion;semantic_detection;semantic_guard;mpc_secbf;swarm_test;dynamic_simulator;map_generator;robot_simulator;traj_planner;plan_env" ...`
- `rg -n "DynTraj|generate_messages|catkin_EXPORTED_TARGETS|EXPORTED_TARGETS|add_dependencies" swarm_test simulation_tools/dynamic_simulator ...`
验证：
- 失败复现：`swarm_test/include/obs_manager_for_data_process/obs_manager.hpp:14:10: fatal error: dynamic_simulator/DynTraj.h: 没有那个文件或目录`。
- 根因：`data_processor_node` 和 `obs_vis_node` 包含 `dynamic_simulator/DynTraj.h`，但 target 没有依赖 `${catkin_EXPORTED_TARGETS}`，并行构建时消息头可能尚未生成。
结果：
- 为 `data_processor_node` 和 `obs_vis_node` 添加 `add_dependencies(... ${catkin_EXPORTED_TARGETS})`。
回滚：
- 删除 `swarm_test/CMakeLists.txt` 中新增的两行 `add_dependencies(...)`。
备注：
- 此改动只约束 catkin 构建顺序，不改变运行时代码行为。

### 2026-05-23 - 修复 traj_planner 消息生成构建依赖

阶段：任务 1
意图：解决同一基线构建中 `traj_planner` 并行编译早于 `dynamic_simulator/DynTraj.h` 生成的问题。
改动文件：
- `planner/vomp_planner/traj_planner/CMakeLists.txt`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- 重新运行语义包构建命令。
- `rg -n "DynTraj|globalFsm_by_adsm|global_path_by_adsm|add_dependencies|catkin_EXPORTED_TARGETS" planner/vomp_planner/traj_planner ...`
验证：
- 失败复现：`planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp:14:10: fatal error: dynamic_simulator/DynTraj.h: 没有那个文件或目录`。
- 根因：`dynamic_planner_node`、`obs_Manager_node`、`global_path_by_adsm` 和 `globalFsm_by_adsm` 均通过 `obs_manager.hpp` 或相关源文件依赖 `dynamic_simulator/DynTraj.h`，但 target 没有依赖 `${catkin_EXPORTED_TARGETS}`。
结果：
- 为四个 `traj_planner` target 添加 `add_dependencies(... ${catkin_EXPORTED_TARGETS})`。
回滚：
- 删除 `planner/vomp_planner/traj_planner/CMakeLists.txt` 中新增的四行 `add_dependencies(...)`。
备注：
- 此改动只约束 catkin 构建顺序，不改变规划器运行逻辑。

### 2026-05-23 - 语义仿真相关包构建通过

阶段：任务 1
意图：验证语义安全、仿真、全局规划和实验脚手架相关 package 的基线构建。
改动文件：
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `source /opt/ros/noetic/setup.bash`
- `cd /tmp/panjian_verify_ws`
- `catkin_make -DCATKIN_WHITELIST_PACKAGES="semantic_fusion;semantic_detection;semantic_guard;mpc_secbf;swarm_test;dynamic_simulator;map_generator;robot_simulator;traj_planner;plan_env" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"`
验证：
- `[83%] Built target semantic_fusion_node`
- `[85%] Built target mpc_secbf_node`
- `[98%] Built target beta_guard_node`
- `[100%] Built target beta_ground_truth_node`
- `[100%] Built target dynamic_planner_node`
结果：
- 语义仿真相关 package 构建通过。
- 构建中仍有既有 warning，包括 VTK imported target 缺失提示和 `traj_planner` 中未使用变量/可能未初始化变量警告；本任务未修改这些运行逻辑。
回滚：
- 如需回滚本阶段新增构建依赖，删除 `swarm_test/CMakeLists.txt` 与 `planner/vomp_planner/traj_planner/CMakeLists.txt` 中新增的 `add_dependencies(...)`。
备注：
- 当前验证只覆盖编译通过，不代表仿真行为已验证。

### 2026-05-23 - 现有控制器栈回归构建通过

阶段：任务 1
意图：确认语义安全相关改动没有破坏现有 `mpc_dcbf` 控制器构建。
改动文件：
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `source /opt/ros/noetic/setup.bash`
- `cd /tmp/panjian_verify_ws`
- `catkin_make -DCATKIN_WHITELIST_PACKAGES="mpc_dcbf;swarm_test;dynamic_simulator;map_generator;robot_simulator;traj_planner;plan_env" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"`
验证：
- `[98%] Built target mpc_horizen`
- `[100%] Built target mpc_node_c`
- 同一构建中 `swarm_test`、`dynamic_simulator`、`traj_planner` 和仿真基础包也构建通过。
结果：
- 现有控制器栈编译回归通过。
回滚：
- 本条只记录验证结果；无运行时代码回滚需求。
备注：
- 后续仍需运行 legacy ACBF smoke test 才能证明运行层面不受影响。

### 2026-05-23 - 对齐规格文档与当前代码契约

阶段：任务 2
意图：移除 requirements/design/tasks 中关于旧集成方式的描述，使文档与独立 `mpc_secbf_node`、Guard 前瞻公式和当前构建证据一致。
改动文件：
- `.kiro/specs/semantic-safety-margin/requirements.md`
- `.kiro/specs/semantic-safety-margin/design.md`
- `.kiro/specs/semantic-safety-margin/tasks.md`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `rg -n "fixed safe|controller_type=5|8×N|safe_dist|R_robot|robot_radius|mpc_cbf.cpp|obs_matrix" .kiro/specs/semantic-safety-margin`
- `rg -n "controller_type=5|8×N|mpc_cbf.cpp|safe_dist" .kiro/specs/semantic-safety-margin/requirements.md .kiro/specs/semantic-safety-margin/design.md .kiro/specs/semantic-safety-margin/tasks.md`
- `rg -n "R_safe|R_\\{safe\\}|fixed safe|controller_type=5|8×N|mpc_cbf.cpp" .kiro/specs/semantic-safety-margin/requirements.md .kiro/specs/semantic-safety-margin/design.md .kiro/specs/semantic-safety-margin/tasks.md`
- `source /opt/ros/noetic/setup.bash && source /tmp/panjian_verify_ws/devel/setup.bash && rosmsg show semantic_fusion/SemanticObstacleArray && rosmsg show semantic_guard/GuardLog`
- `rg -n "robot_radius|h_EE|h_SEE|tau_|lookahead_rel_pos|beta_i" planner/mpc_secbf planner/semantic_guard`
验证：
- 文档扫描中 `controller_type=5|8×N|mpc_cbf.cpp|safe_dist` 无匹配。
- 文档扫描中 `R_safe|R_{safe}|fixed safe|controller_type=5|8×N|mpc_cbf.cpp` 无匹配。
- `SemanticObstacleArray` 字段包含 `id`、`semantic_class`、`position`、`velocity`、`radius`、`beta_hat`、`heading_factor`、`ttc_norm`、`density_norm`、`guard_passed`。
- `GuardLog` 字段包含 `obstacle_ids`、`beta_requested`、`beta_applied`、`h_ee_values`、`guard_passed`、`total_rollbacks`。
- 代码扫描确认 `mpc_secbf` 使用 `robot_radius`、`h_EE`、`h_SEE`、`beta_i`，两个 Guard 节点包含 `lookahead_rel_pos = p_rel + tau_ * v_rel`。
结果：
- requirements 改为描述独立 `mpc_secbf_node`，不再描述向 legacy controller 注入新模式或扩展 legacy obstacle matrix。
- design 中 Guard 伪代码改为显式 `p_rel + tau * v_rel`，MPC-SECBF 伪代码改为 `robot_radius_`。
- tasks 中已完成项加入当前构建/消息证据，并把运行层面验证保留到 Phase 5-7。
回滚：
- 使用 git diff 中对应 hunk 恢复三个规格文档和本记录条目。
备注：
- 本阶段只修改文档，不改变运行时代码。

### 2026-05-23 - 增加仿真实验矩阵脚手架和 Guard 开关

阶段：任务 3
意图：让 B1/B2/B3 与 S1-S4 可以通过统一脚本启动，并支持 B2 no-Guard baseline。
改动文件：
- `planner/semantic_guard/src/beta_guard_node.cpp`
- `planner/semantic_guard/src/beta_ground_truth_node.cpp`
- `swarm_test/launch/secbf_planner.launch`
- `swarm_test/launch/start_test.launch`
- `simulation_tools/dynamic_simulator/launch/spawn_dynamic_obstacle.launch`
- `swarm_test/scripts/run_secbf_sim_experiments.py`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `python3 -m py_compile swarm_test/scripts/run_secbf_sim_experiments.py`
- `python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario S4_mixed --baseline B3_SECBF_with_guard --duration-sec 5 --dry-run`
- `python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario all --baseline all --duration-sec 5 --dry-run`
验证：
- dry-run 输出 12 组 scenario/baseline 命令。
- B2 命令包含 `guard_enabled:=false`。
- B3 命令包含 `guard_enabled:=true`。
- `start_test.launch` 通过 `obstacle_params_file` 加载每次运行生成的障碍物参数。
结果：
- Guard 节点新增 `guard/enabled` 参数；关闭时直接应用 `beta_hat`，不做 Guard rollback 和 rate limit。
- 仿真启动文件新增 `scenario_id`、`baseline_id`、`guard_enabled`、`output_dir`、`obstacle_classes`、`obstacle_params_file` 等参数。
- 实验脚本生成每次运行目录、障碍物 YAML、launch 日志、summary 和 Guard 安全界验证。
回滚：
- 删除 `swarm_test/scripts/run_secbf_sim_experiments.py`，并恢复上述 launch 与 Guard 节点中的新增参数 hunk。
备注：
- B1 使用 legacy `acbf0_planner.launch`，不产生 Guard log。

### 2026-05-23 - 修复仿真运行依赖和实验数据记录

阶段：任务 3
意图：让仿真 smoke test 真正启动完整链路，并生成可用于论文统计的数据 CSV。
改动文件：
- `.kiro/specs/semantic-safety-margin/execution-plan.md`
- `swarm_test/launch/start_test.launch`
- `swarm_test/scripts/run_secbf_sim_experiments.py`
- `planner/semantic_guard/src/beta_guard_node.cpp`
- `planner/semantic_guard/src/beta_ground_truth_node.cpp`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `catkin_make -DCATKIN_WHITELIST_PACKAGES="semantic_fusion;semantic_detection;semantic_guard;mpc_secbf;swarm_test;dynamic_simulator;map_generator;robot_simulator;laser_simulator;traj_planner;plan_env" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"`
- `python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario S4_mixed --baseline B3_SECBF_with_guard --duration-sec 60 --roscore auto`
- `python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario S3_feasibility_critical --baseline B2_SECBF_no_guard --duration-sec 30 --roscore auto`
验证：
- `laser_sim_node` 构建成功；此前 S4/B3 60 秒 smoke 暴露 `laser_simulator/laser_sim_node` 未编入白名单。
- `data_processor_summary.csv` 和 `data_processor_distance.csv` 在 `record_data:=true` 后生成。
- Guard CSV 时间戳改为固定小数精度后，`verify_safety_bound.py` 正确报告 `Time span`。
结果：
- 执行计划中的仿真构建白名单补充 `laser_simulator`。
- `start_test.launch` 新增 `record_data` 参数，默认 `false`；实验脚本显式传 `record_data:=true`，不改变手动启动默认行为。
- Guard CSV 使用 `std::fixed << std::setprecision(9)` 写入时间和数值。
回滚：
- 恢复 `execution-plan.md` 中白名单 hunk；删除 `record_data` 参数；移除 Guard CSV 固定精度设置。
备注：
- 主动关闭 roslaunch 时仍可能出现 `boost::mutex lock failed`，B1 与 B3 都出现，记录为关闭期残余问题。

### 2026-05-23 - 仿真烟测通过并生成摘要

阶段：任务 3/4/5
意图：验证 SECBF with Guard、SECBF no Guard 和 legacy ACBF 的基础运行链路。
改动文件：
- `swarm_test/output/secbf_runs/latest_experiment_summary.md`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario S4_mixed --baseline B3_SECBF_with_guard --duration-sec 30 --roscore auto`
- `python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario S3_feasibility_critical --baseline B2_SECBF_no_guard --duration-sec 30 --roscore auto`
- `python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario S4_mixed --baseline B1_ACBF_fixed --duration-sec 30 --roscore auto`
验证：
- S4/B3 输出目录：`swarm_test/output/secbf_runs/20260523_121359_S4_mixed_B3_SECBF_with_guard`
  - `guard_log.csv`、`data_processor_summary.csv`、`data_processor_distance.csv` 均存在。
  - Guard 记录数 5028，时间跨度 27.90s，`min(h_EE)=0.3802`，理论下界 `-1.0000`，rollback 63 次。
- S3/B2 输出目录：`swarm_test/output/secbf_runs/20260523_121203_S3_feasibility_critical_B2_SECBF_no_guard`
  - `guard_enabled=false` 生效，rollback 0 次。
  - Guard 记录数 559，`min(h_EE)=0.6788`，理论下界 `-1.0000`。
- S4/B1 输出目录：`swarm_test/output/secbf_runs/20260523_121507_S4_mixed_B1_ACBF_fixed`
  - legacy ACBF 可启动，数据 CSV 生成，不依赖语义安全节点。
结果：
- 三条 smoke 链路均能启动并生成预期日志/CSV。
- 最新执行摘要写入 `swarm_test/output/secbf_runs/latest_experiment_summary.md`。
回滚：
- 删除本次生成的 `swarm_test/output/secbf_runs/20260523_*` 目录和 `latest_experiment_summary.md`。
备注：
- 本轮未执行完整 12 组 90 秒实验矩阵；当前完成的是低成本 smoke 验证。

### 2026-05-23 - 修复 FAST_LIO 消息生成构建依赖

阶段：任务 7
意图：让实车代码 package set 在并行或低并发构建时不会早于 `fast_lio/Pose6D.h` 生成。
改动文件：
- `state_estimation/FAST_LIO/CMakeLists.txt`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver;fast_lio;dynamic_perception;semantic_detection;semantic_fusion;semantic_guard;mpc_secbf;swarm_test;traj_planner;map_generator;plan_env" ...`
- `catkin_make -j1 -l1 -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver;fast_lio;dynamic_perception;semantic_detection;semantic_fusion;semantic_guard;mpc_secbf;swarm_test;traj_planner;map_generator;plan_env" ...`
验证：
- 初次实车 package set 构建失败：`fatal error: fast_lio/Pose6D.h: 没有那个文件或目录`。
- 添加 `add_dependencies(fastlio_mapping ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})` 后，低并发构建通过。
- 构建证据包含 `[43%] Built target fastlio_mapping`、`[90%] Built target semantic_fusion_node`、`[92%] Built target mpc_secbf_node`、`[100%] Built target beta_guard_node`。
结果：
- `fastlio_mapping` 显式等待本包消息头和 catkin 导出 target。
回滚：
- 删除 `state_estimation/FAST_LIO/CMakeLists.txt` 中新增的 `add_dependencies(...)`。
备注：
- 高并发 `-j8` 在当前机器上会造成 `traj_planner` 大 C++ 单元编译时内存/IO 饱和，因此实车验证命令改为 `-j1 -l1`。

### 2026-05-23 - 实车代码 dry-run 静态和短启动检查

阶段：任务 7
意图：在不发布物理 `/cmd_vel` 的前提下验证实车代码链路可以解析并启动。
改动文件：
- `swarm_test/launch_exp/exp_secbf_planner.launch`
- `.kiro/specs/semantic-safety-margin/execution-plan.md`
- `swarm_test/output/secbf_runs/latest_experiment_summary.md`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `roslaunch --nodes swarm_test start_perception.launch`
- `roslaunch --nodes swarm_test exp_secbf_planner.launch v_max:=0.2 cmd_vel_topic:=/cmd_vel_secbf_dryrun`
- `roslaunch --dump-params swarm_test exp_secbf_planner.launch v_max:=0.2 cmd_vel_topic:=/cmd_vel_secbf_dryrun | rg -n "v_max|cmd_vel|mpc_frequency|gamma"`
- `roslaunch --args mpc_secbf_node swarm_test exp_secbf_planner.launch v_max:=0.2 cmd_vel_topic:=/cmd_vel_secbf_dryrun`
- `timeout 12s roslaunch swarm_test exp_secbf_planner.launch v_max:=0.2 cmd_vel_topic:=/cmd_vel_secbf_dryrun`
- `timeout 12s roslaunch swarm_test start_perception.launch use_sim_time:=false show_rviz:=false`
验证：
- `start_perception.launch` 静态节点：`/ri_dbscan_node`、`/L_shape_fitting`、`/obstacle_prediction_node`。
- `exp_secbf_planner.launch` 静态节点：`/mpc_secbf_node`、`/globalFsm_by_adsm`、`/random_forest`、`/semantic_fusion_node`、`/beta_guard_node`、`/odom_traj_visualization_node`、`/scout_visualizer_node`。
- `mpc_secbf_node` 参数中 `/mpc_secbf_node/mpc/v_max: 0.2`。
- `roslaunch --args` 显示 `/cmd_vel:=/cmd_vel_secbf_dryrun`。
- 12 秒 planner dry-run 启动后由 timeout 主动关闭，日志无 `ERROR`、`Exception`、`Traceback`、`process died`。
- 12 秒 perception dry-run 启动后由 timeout 主动关闭，日志无 `ERROR`、`Exception`、`Traceback`、`process died`。
结果：
- `exp_secbf_planner.launch` 的可覆盖参数从 `value` 改为 `default`，新增 `cmd_vel_topic` 参数，默认仍为 `/cmd_vel`。
- dry-run 命令改为 `cmd_vel_topic:=/cmd_vel_secbf_dryrun`，避免物理 `/cmd_vel`。
回滚：
- 将 `exp_secbf_planner.launch` 中相关 arg 恢复为固定 `value`，并把 `/cmd_vel` remap 恢复为 `/cmd_vel`。
备注：
- 当前环境没有实车传感器输入，因此未测 topic rate 和端到端延迟。
- 物理运动仍被审批门槛阻塞；没有运行任何向物理 `/cmd_vel` 发布的命令。

### 2026-05-24 - 固化 MPC-SECBF 论文代码目录与 RViz 数值仿真说明

阶段：任务 7
意图：按 `planner/dwa_planner`、`planner/mpc_dcbf`、`planner/mpc_secbf`、`planner/semantic_guard` 的并列结构说明本文代码归属，并补充可视化数值仿真命令。
改动文件：
- `README.md`
- `planner/README.md`
- `planner/mpc_secbf/launch/mpc_secbf.launch`
- `planner/semantic_guard/launch/beta_ground_truth.launch`
- `swarm_test/launch/secbf_planner.launch`
- `.kiro/specs/semantic-safety-margin/tasks.md`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- 未运行仿真；本次为文档和目录说明更新。
验证：
- `rg -n "MPC-SECBF|mpc_secbf|semantic_guard|controller_index:=6" README.md planner/README.md .kiro/specs/semantic-safety-margin/tasks.md .kiro/specs/semantic-safety-margin/change-record.md`
- `ls -d planner/dwa_planner planner/mpc_dcbf planner/mpc_secbf planner/semantic_guard`
- `roslaunch --nodes swarm_test secbf_planner.launch show_rviz:=false`
- `roslaunch --dump-params swarm_test secbf_planner.launch show_rviz:=false | rg -n "mpc_secbf_node/mpc/(v_max|gamma|robot_radius|beta_bar_unknown)|beta_ground_truth_node/(guard/enabled|baseline_id|scenario_id|log_path)|obstacle_classes"`
- `roslaunch --args mpc_secbf_node swarm_test secbf_planner.launch show_rviz:=false`
- `roslaunch --args beta_ground_truth_node swarm_test secbf_planner.launch show_rviz:=false`
结果：
- 根 README 新增 MPC-SECBF 批量实验、RViz 可视化实验、Guard 日志解释、目录结构和 controller 标签说明。
- 新增 `planner/README.md`，明确本文核心代码在 `planner/mpc_secbf` 与 `planner/semantic_guard`，`swarm_test` 只负责实验编排。
- `swarm_test/launch/secbf_planner.launch` 改为 include `mpc_secbf.launch` 和 `beta_ground_truth.launch`，让实验编排与本文包边界一致。
- 四个 planner 目录均已存在。
- `secbf_planner.launch` 可解析出 `/beta_ground_truth_node` 与 `/mpc_secbf_node`；`mpc_secbf_node` 仍 remap 到 `/robot1/odom` 和 `/cmd_vel1`。
回滚：
- 删除 `planner/README.md` 和 `planner/semantic_guard/launch/beta_ground_truth.launch`，并回退 README、三个 launch 文件、tasks 和本记录中的 2026-05-24 条目。

### 2026-05-24 - README 补充可选仿真环境与修改入口

阶段：任务 7
意图：让 README 不只给出主启动命令，也能作为仿真组件索引，说明现有可选仿真环境在哪里、怎么调用、常改参数在哪。
改动文件：
- `README.md`
- `simulation_tools/robot_simulator/launch/vis_car.launch`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `rg -n "仿真|simulation|Gazebo|数值|start_test|start_gazebo|spawn_dynamic|map_generator|robot_simulator|laser_simulator|secbf_planner|acbf0" README.md swarm_test/launch simulation_tools planner -g "*.launch" -g "*.xml" -g "*.yaml" -g "README.md"`
- `rg --files simulation_tools swarm_test/config | rg "(yaml|rviz|launch|xml)$"`
- `rg --files | rg "simple_env\\.world|spawn_scout_v2_use1\\.launch|obstacles_param_gazebo\\.yaml|obstacles_param\\.yaml$"`
验证：
- `rg -n "可选仿真环境与修改入口|常用入口总览|动态障碍物 YAML|单独调试仿真组件|推荐修改流程|spawn_dynamic_obstacle|obstacle_params_file|map_generator map.launch|robot_simulator vis_car|start_gazebo_env" README.md`
- `roslaunch --nodes map_generator map.launch rviz_vis:=false map_size_x_:=50.0 map_size_y_:=50.0 map_size_z_:=3.0 p_num:=20 c_num:=0`
- `roslaunch --nodes dynamic_simulator spawn_dynamic_obstacle.launch rviz:=false num_of_obs:=5 obstacle_params_file:=$(pwd)/simulation_tools/dynamic_simulator/config/obstacles_param.yaml`
- `roslaunch --nodes robot_simulator vis_car.launch rviz_vis:=false`
- `roslaunch --nodes swarm_test start_gazebo_env.launch gzclient:=false use_sim_time:=true`
结果：
- README 新增“可选仿真环境与修改入口”，覆盖 ACBF 数值、MPC-SECBF 数值、批量实验、动态障碍物、Gazebo、随机地图、差速车/激光和 RViz 配置。
- 补充了动态障碍物 YAML 字段含义、推荐复制配置后通过 `obstacle_params_file:=...` 指定的修改流程。
- 修正 README 底部“注意事项”编号顺序。
- 静态验证发现 `robot_simulator/launch/vis_car.launch` 漏传 `scout_simulator.xml` 必填的 `time_res_`，已新增 `time_res` 默认参数并转传。
- 随机地图、动态障碍物、机器人/激光、Gazebo 环境入口均通过 `roslaunch --nodes` 静态解析。
回滚：
- 删除 README 中“可选仿真环境与修改入口”章节，移除目录项，并回退“注意事项”编号调整、`vis_car.launch` 的 `time_res` 参数和本记录条目。

### 2026-05-24 - README 补充完整数值仿真终端顺序与数据查看

阶段：任务 7
意图：把手动 RViz 数值仿真的完整终端启动顺序、命令和数据记录查看方式写清楚，方便直接照 README 复现实验。
改动文件：
- `README.md`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `rg -n "data_processor_summary|data_processor_distance|guard_log|global_path|output_dir|record_data" README.md swarm_test/scripts swarm_test/launch`
- `head -n 5 swarm_test/output/secbf_runs/rviz_s4/guard_log.csv`
- `find swarm_test/output/secbf_runs/rviz_s4 -maxdepth 1 -type f | sort`
验证：
- `roslaunch --nodes swarm_test secbf_planner.launch show_rviz:=false scenario_id:=S4_mixed baseline_id:=B3_SECBF_with_guard guard_enabled:=true output_dir:=/home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/rviz_s4 obstacle_classes:='[pedestrian,vehicle,box]'`
- `roslaunch --nodes swarm_test start_test.launch scenario_index:=4 controller_index:=6 num_of_obs:=3 output_dir:=/home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/rviz_s4 obstacle_params_file:=/home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/rviz_s4/obstacles_param.yaml record_data:=true`
- `roslaunch --nodes swarm_test acbf0_planner.launch show_rviz:=false`
- `rg -n "完整终端启动顺序|终端 0|终端 1|终端 2|终端 3|数据记录怎么看|verify_safety_bound|data_processor_summary|planner.log" README.md`
结果：
- README 新增 MPC-SECBF 手动 RViz 运行的终端 0/1/2/3 顺序：准备/roscore、planner、环境+数据、运行时检查。
- README 新增 ACBF baseline 的完整终端顺序，并注明 `acbf0_planner.launch` 的 `controller` 与 `front_adsm` 当前是固定 `value`，命令行不能覆盖。
- README 新增数据记录查看说明，覆盖 `guard_log.csv`、`data_processor_summary.csv`、`data_processor_distance.csv`、全局路径分析、批量实验的 `summary.md`、`verify_safety_bound.txt` 和日志文件。
回滚：
- 删除 README 中新增的完整终端顺序和数据查看段落，并移除本记录条目。

### 2026-05-24 - README 补充 roscore 已运行时的处理说明

阶段：任务 7
意图：避免手动数值仿真终端 0 遇到已有 ROS master 时被误判为启动失败。
改动文件：
- `README.md`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `pgrep -af 'roscore|rosmaster|roslaunch'`
- `rostopic list | sed -n '1,40p'`
验证：
- 当前已有 `/usr/bin/python3 /opt/ros/noetic/bin/roscore` 与 `/opt/ros/noetic/bin/rosmaster --core -p 11311`。
- `rostopic list` 可返回 `/rosout`、`/rosout_agg`。
结果：
- README 在终端 0 的 `roscore` 后补充说明：若提示已有 roscore/master，先用 `rostopic list` 确认可用；能看到 `/rosout` 时直接继续终端 1/2；只有残留进程才 `pkill`。
回滚：
- 删除 README 中该说明段落，并移除本记录条目。

### 2026-05-24 - README 简化数值仿真启动为两终端默认流程

阶段：任务 7
意图：利用 `roslaunch` 自动启动 ROS master 的机制，把手动数值仿真说明改得更方便，避免把 `roscore` 写成必需步骤。
改动文件：
- `README.md`
- `.kiro/specs/semantic-safety-margin/change-record.md`
验证：
- 待执行：`roslaunch --nodes` 静态解析两终端命令。
结果：
- MPC-SECBF 手动 RViz 流程改为终端 1 启动 `secbf_planner.launch`，终端 2 准备障碍物 YAML 并启动 `start_test.launch`。
- `roscore` 改为“长期调试时可选”，并保留已有 master 的判断说明。
- ACBF baseline 流程移除必需 `roscore` 终端。
回滚：
- 恢复 README 中独立 `roscore` 终端说明，并移除本记录条目。

### 2026-05-24 - README 按论文实验场景重写可选仿真环境说明

阶段：任务 7
意图：把“可选仿真环境”从底层组件列表改成论文实验场景列表，明确原论文 RViz 动态障碍物环境、本文 MPC-SECBF 动态语义环境、静态一字排开障碍物环境和 Gazebo 环境。
改动文件：
- `README.md`
- `.kiro/specs/semantic-safety-margin/change-record.md`
命令：
- `sed -n '1,140p' swarm_test/launch/secbf_static_test.launch`
- `sed -n '1,120p' simulation_tools/dynamic_simulator/config/obstacles_param_static_beta.yaml`
- `rg -n "static|静态|obstacles_param_static|secbf_static|S4_mixed|S3_feasibility" README.md swarm_test simulation_tools planner -g "*.launch" -g "*.yaml" -g "*.md"`
验证：
- `roslaunch --nodes swarm_test secbf_static_test.launch show_rviz:=false`
- `rg -n "场景环境总览|原论文 RViz 动态障碍物环境|本文 MPC-SECBF RViz 动态语义环境|本文静态一字排开障碍物环境|三个一条线|secbf_static_test|obstacles_param_static_beta|底层组件入口" README.md`
结果：
- README 的“可选仿真环境与修改入口”新增“场景环境总览”。
- 明确原论文 RViz 动态障碍物环境：`acbf0_planner.launch + start_test.launch + obstacles_param.yaml`。
- 明确本文 MPC-SECBF 动态语义环境：`secbf_planner.launch + start_test.launch + secbf_scenarios.yaml`。
- 明确静态一字排开障碍物环境：`secbf_static_test.launch + obstacles_param_static_beta.yaml`，并写出如何从当前 5 个静态障碍物裁成 3 个。
- 保留底层组件入口作为修改参考。
- `secbf_static_test.launch` 可静态解析出 `dynamic_corridor`、`beta_ground_truth_node`、`mpc_secbf_node` 等节点。
回滚：
- 删除 README 中本次新增的场景环境总览和四个场景环境说明，并移除本记录条目。
