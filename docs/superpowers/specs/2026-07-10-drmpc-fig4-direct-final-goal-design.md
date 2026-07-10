# DR-MPC Fig. 4 直线场景最终目标发布设计

## 背景

`drmpc_fig4_scene_2_vertical` 和
`drmpc_fig4_scene_4_reverse_vertical` 当前通过一系列离散航点执行直线参考路径。
航点推进同时使用 `planner_goal_min_distance=3.0 m` 的初始前视距离和
`threshold=0.35 m` 的到达圆域。实验表明，毫米级初始里程计差异会改变首个
目标航点；绕障产生横向偏移后，机器人还可能已经越过航点却没有进入到达圆域，
从而被旧航点持续拉回并触发后退。

Scene 2 和 Scene 4 的参考路径本身都是从起点到终点的竖直直线，因此不需要
通过中间航点维持几何形状。Scene 1 和 Scene 3 是圆弧路径，仍然需要中间航点。

## 目标

- Scene 2 只向规划器发布最终目标 `(0, 6)`。
- Scene 4 只向规划器发布最终目标 `(0, -6)`。
- 绕障后由全局路径生成器从机器人当前位置重新连接最终目标，消除中间航点锁死。
- 保持 MPC-SECBF 的障碍物约束、语义边界和 Guard 行为不变，继续验证真实避障。
- Scene 1 和 Scene 3 继续使用现有圆弧航点发布机制。

## 方案

在场景配置中增加显式的参考路径目标模式：

```yaml
reference_path:
  type: line
  goal_mode: final_only
```

`run_secbf_sim_experiments.py` 根据 `goal_mode` 选择启动方式：

- `final_only`：不启动 `reference_path_goal_publisher.py`，沿用现有
  `start_trigger_node`，在启动延时后发布场景最终目标。
- 未配置或为 `waypoints`：保持现有参考航点文件和航点发布器行为。

只在 Scene 2 和 Scene 4 配置 `goal_mode: final_only`。Scene 1 和 Scene 3
不增加该字段，以保持向后兼容。

## 数据流

Scene 2/4 的运行链路变为：

1. `start_trigger_node` 发布最终目标到 `/move_base_simple/goal`。
2. `global_path_by_rviz` 按机器人当前位置和最终目标生成直线路径。
3. 机器人绕障产生横向位移后，全局路径继续从当前位置指向最终目标。
4. MPC-SECBF 使用动态全局路径和障碍物预测进行局部避障。
5. `data_processor_node` 和 Phase 5 日志继续以场景最终目标计算完成状态和终点误差。

Scene 1/3 的数据流保持不变，仍由 `reference_path_goal_publisher.py` 逐段发布
圆弧航点。

## 范围约束

- 不修改四个场景的机器人起终点。
- 不修改障碍物位置、速度、启动时间、语义类别和 `beta_bar`。
- 不修改 Scene 2 的 `planner_v_max: 0.9`。
- 不修改 corridor、地图范围、MPC 参数或安全边界。
- 不删除通用航点发布代码；圆弧场景和其他需要固定参考形状的场景继续使用它。
- 生成的 `reference_path.yaml` 可继续作为绘图和路径真值输入，但 Scene 2/4
  不再把其中间点逐个发布给在线规划器。

## 兼容性与失败处理

- `goal_mode` 缺省值为 `waypoints`，避免改变现有场景行为。
- 非法 `goal_mode` 在实验启动前直接报错，不允许静默回退。
- `final_only` 仅允许用于 `type: line`；若圆弧场景误配置为 `final_only`，
  配置校验必须失败，防止圆弧路径被直线截断。
- 即使使用 `final_only`，运行元数据仍记录完整参考路径和固定最终目标，保证
  后处理指标含义不变。

## 测试与验证

静态测试应验证：

1. Scene 2 和 Scene 4 的 `goal_mode` 为 `final_only`。
2. Scene 1 和 Scene 3 保持 waypoint 模式。
3. `final_only` 构建出的启动命令使用 `start_trigger_node`，最终目标分别为
   `(0, 6)` 和 `(0, -6)`。
4. `final_only` 不启动 `reference_path_goal_publisher.py`。
5. 圆弧路径不能配置为 `final_only`，未知模式会被拒绝。
6. 四个场景仍然生成完整参考路径元数据，绘图输入不受影响。

运行验证只重跑 Scene 2 和 Scene 4，输出目录遵循
`YYYYMMDD_rNN_<task_short_desc>` 命名。验收重点为：

- 日志中 Scene 2/4 只出现最终目标，不出现中间 waypoint 发布。
- 机器人绕障后不会因为错过中间航点而回头。
- 两场尽量到达最终目标，且不因本次目标发布调整增加碰撞或负安全余量。
- Scene 1/3 的现有测试保持通过，确认圆弧航点行为没有回归。
