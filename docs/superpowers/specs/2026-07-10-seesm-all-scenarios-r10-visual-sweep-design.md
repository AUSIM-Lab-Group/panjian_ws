# SEESM 全场景 r10 视觉巡检设计

## 目标

对当前 `secbf_scenarios.yaml` 中全部 29 个可运行场景执行一次
`SEESM_Ours` 单次仿真，集中检查机器人与动态/静态障碍物交互的视觉效果。

## 运行范围

- 场景：全部 29 个已注册场景。
- 方法：仅 `SEESM_Ours`。
- 单场时长：30 秒。
- 重复次数：1。
- ROS：共用一个外部 `roscore`，场景在 runner 内顺序执行。
- r10 中 `drmpc_fig4_scene_2_vertical` 和
  `drmpc_fig4_scene_4_reverse_vertical` 使用中线静态点
  `(0.0, -1.5)` 与 `(0.0, 3.0)`；其他场景参数不变。

本轮输出目录为：

- 后端：`swarm_test/output/secbf_runs/20260710_r10_all_scenarios_visual_sweep`
- 图层：`seesm_social_navigation/outputs/20260710_r10_all_scenarios_visual_sweep`

## 输出

1. 每个场景生成后端日志、`summary.csv` 和后处理 manifest。
2. 每个场景生成 methods-comparison PNG、GIF 和 `_slow2x.gif`。
3. 生成总览 PNG 与场景级路径指标 JSON。
4. 汇总每个场景的成功、碰撞、终点误差、最小安全余量和路径横向偏差。

## 边界与判读

- 这是单方法、单次重复的视觉巡检，不用于方法优劣比较或统计显著性结论。
- 若某场失败、碰撞或 GIF 缺失，保留原始日志和 GIF 结果，并在汇总中标注；
  不在同一轮自动调参重跑。
- Scene 2/4 的中线静态点用于观察显式绕行，单独记录其是否到达终点、
  是否碰撞和是否明显出 corridor。
- 新输出目录必须遵循用户约定的
  `YYYYMMDD_rNN_<task_short_desc>` 格式。
