# DR-MPC Fig. 4 风格 SEESM 补充场景设计

## 目标

在不复现 DR-MPC 控制算法、训练流程和 ORCA 人群模型的前提下，借鉴
DR-MPC Fig. 4 的四场景几何组织，构建由 `panjian_ws` 真实
SEESM-MPC-SECBF 控制器执行的补充实验，并由 `seesm_social_navigation`
从真实日志生成论文风格 PNG、四宫格和 GIF。

## 范围

- 保留 `panjian_ws` 的差速机器人模型、全局规划、MPC-SECBF、SEESM 和 Guard。
- 新增四个场景，不覆盖现有 `drmpc_scene_*` 场景：
  - `drmpc_fig4_scene_1_arc`
  - `drmpc_fig4_scene_2_vertical`
  - `drmpc_fig4_scene_3_reverse_arc`
  - `drmpc_fig4_scene_4_reverse_vertical`
- 每个场景使用 6 名脚本化动态 human；human 不使用 ORCA，也不响应机器人。
- 第一阶段只运行 `SEESM_Ours` smoke test。四场景稳定后，才决定是否运行
  4 methods 或 repeat=3。

## 非目标

- 不移植 DR-MPC 的 SAC、残差策略、OOD、HAN 或路径跟踪 MPC。
- 不让 human 之间进行 ORCA 或其他互惠避障。
- 不修改 `h_SEE`、语义安全裕度、Guard 投影或 MPC-SECBF 约束定义。
- 不把纯绘图轨迹冒充真实控制结果。

## 几何定义

统一使用 `world` 坐标系、`4.0 m` 圆弧半径、`1.5 m` 走廊总宽度。
圆弧 waypoint 最大间距为 `0.4 m`，直线 waypoint 间距为 `0.4 m`。

| 场景 | 机器人起点 | 机器人终点 | 参考路径 |
|---|---:|---:|---|
| `drmpc_fig4_scene_1_arc` | `(0, 4)` | `(0, -4)` | 圆心 `(0,0)`、半径 `4`、右半圆、顺时针 |
| `drmpc_fig4_scene_2_vertical` | `(0, -4)` | `(0, 4)` | `x=0`、由下向上 |
| `drmpc_fig4_scene_3_reverse_arc` | `(0, -4)` | `(0, 4)` | 与场景 1 相同的右半圆、逆时针 |
| `drmpc_fig4_scene_4_reverse_vertical` | `(0, 4)` | `(0, -4)` | `x=0`、由上向下 |

圆弧场景 1 和 3 必须使用相同几何、相反 waypoint 顺序；直线场景 2 和
4 也必须使用相同几何、相反 waypoint 顺序。这样方向变化不会同时改变
场景尺度。

## Human 运动

每个场景固定 6 名 human。运动由现有 `dynamic_simulator` 的直线往返轨迹
生成，使用固定坐标、固定 travel time 和统一 `start_delay=3.0 s`。
场景配置不得出现 `orca` 字段或 ORCA 节点。

语义类别按以下顺序复用：

1. `adult / cooperative`
2. `pedestrian / cooperative`
3. `child_like / noncooperative`
4. `cyclist / partial`
5. `adult / cooperative`
6. `child_like / noncooperative`

场景 1 在右半圆的上、中、下区域安排稀疏横穿和径向横穿；场景 2 在
`y={-2.7,-1.2,0.0,1.2,2.7}` 附近安排交替左右横穿，并加入一条对角横穿。
场景 3 将 6 条轨迹集中到右半圆中段，形成比场景 1 更密集的交互；场景 4
将 6 条横穿轨迹集中到竖直走廊中部，形成 crossing crowd。

所有 human 起点与机器人起点、终点的中心距离必须大于
`R_robot + R_human + 0.5 m`。同一场景、同一 repeat 的轨迹必须确定性一致。

## 后端路径跟踪

采用 waypoint 序列方案，不直接改写 MPC 参考轨迹接口：

1. 场景配置提供 `robot_start`、`robot_goal` 和 `reference_path`。
2. runner 根据路径几何生成有序 waypoint。
3. waypoint 发布器等待 `3.0 s`，订阅 `/robot1/odom`，在机器人距离当前
   waypoint 不超过 `0.35 m` 时发布下一点。
4. 最后一个 waypoint 必须等于场景的 `robot_goal`。
5. `secbf_planner.launch` 接收机器人初始位置和初始朝向；初始朝向取路径
   第一段切向。
6. `globalFsm_by_adsm` 仍负责 waypoint 之间的局部轨迹生成，并继续向
   MPC-SECBF 发布参考轨迹。
7. SEESM、Guard 和 MPC-SECBF 继续消费原有障碍预测及语义裕度，不增加
   DR-MPC 控制代码。

waypoint 发布失败、里程计超时或路径配置非法时，节点必须记录明确错误并
停止发布，不能退化成直接终点导航。

## 日志与成功判定

中间 waypoint 只用于路径推进，不能被记录器误判为实验终点：

- 数据记录器在第一条 waypoint 消息到达时开始计时。
- 成功判定固定使用场景的最终 `robot_goal`。
- runner 的最终成功阈值保持 `0.55 m`。
- 每个 run 继续输出 `robot_log.csv`、`obstacle_log.csv`、
  `margin_guard_log.csv`、`planner_log.csv`、`timing_log.csv` 和
  `event_log.csv`。
- 后处理新增路径横向误差 RMSE、最大横向误差和走廊外时间比例；这些是
  补充指标，不替代 SR、CR、最小距离、`h_SEE`、Guard 激活率和求解时间。

## 绘图

`seesm_social_navigation` 从 run metadata 读取与后端相同的路径几何：

- 黑色线：参考路径及方向箭头。
- 红色线：走廊两侧边界。
- 黄色：机器人当前位置；机器人真实轨迹使用现有 Proposed 配色。
- 灰色或语义配色小圆：human；不绘制障碍物虚线安全圈。
- GIF 只使用当前帧的人体位置，避免历史 artist 未清理造成虚影。

必须生成：

- 四场景各自的 `*_methods_comparison.png`
- 四场景各自的 `*_methods_comparison.gif`
- `drmpc_fig4_scenarios_overview.png`
- 每场景的 `*_beta_h_curves.png`
- 每场景的 `*_solver_guard_curves.png`

第一阶段虽然只有 `SEESM_Ours`，文件名继续沿用现有绘图契约，避免另建一套
后处理入口。

## 验证

### 静态测试

- 四个场景名称均被 runner、配置测试和绘图标签识别。
- 每场恰有 6 名 human，全部为脚本轨迹，且不存在 ORCA 配置。
- 场景 1/3 waypoint 逐点反序一致；场景 2/4 waypoint 逐点反序一致。
- waypoint 间距不超过 `0.4 m`，最后一点精确等于最终目标。
- 初始 human 与机器人起终点满足最小分离条件。

### Smoke test

按场景依次运行 `SEESM_Ours`，每场 `30 s`、`repeat=1`：

- 机器人和 human 在约 `3 s` 同步开始运动。
- `obstacle_log.csv` 中存在 6 个稳定 obstacle id。
- 机器人按 waypoint 顺序推进，最终目标距离不超过 `0.55 m`。
- 所有必需日志非空，MPC 求解记录存在。
- 图和 GIF 中路径、走廊、机器人轨迹与场景方向一致，且无图例交叉和人体虚影。

若某一场景未到达最终目标，只调整 waypoint 阈值、human 起终点或 travel
time；不得通过关闭 SEESM/Guard、缩小计算半径或伪造绘图轨迹获得成功。

## 交付顺序

1. 路径几何与 waypoint 单元测试。
2. runner、launch、机器人初始位姿和最终目标记录。
3. 四场景配置及 dry-run 检查。
4. 四场景 `SEESM_Ours` smoke test。
5. 真实日志转 PNG、四宫格和 GIF。
6. 汇总路径跟踪、安全性和求解性能，再决定是否扩展方法矩阵与 repeat。
