# DR-MPC Fig. 4 中线静态占位 r10 设计

## 目标

验证 SEESM-MPC-SECBF 在机器人竖直主路径被静态障碍物直接占据时的绕行效果，
并输出 Scene 2 和 Scene 4 的 r10 GIF。

## 变更

仅修改以下四个静态 line obstacle 的二维横坐标；每个静态点仍使用
`start == goal`：

| 场景 | 静态点 |
| --- | --- |
| `drmpc_fig4_scene_2_vertical` | `(0.0, -1.5)`、`(0.0, 3.0)` |
| `drmpc_fig4_scene_4_reverse_vertical` | `(0.0, -1.5)`、`(0.0, 3.0)` |

## 不变项

- 两场的机器人起点、最终目标和 `goal_mode: final_only`。
- 四条动态障碍物轨迹、`travel_time`、`start_delay`、类别和 `beta_bar`。
- 地图、`corridor_width`、`planner_v_max`、MPC 与 Guard 参数。
- Scene 1 和 Scene 3。

## 验证

1. 配置测试确认两场的静态点均为上述中线坐标，动态 crossing 数量不变。
2. r10 仅运行 Scene 2/4，输出根目录为
   `20260710_r10_scene24_static_centerline`。
3. 检查最终目标到达、碰撞、安全余量、路径横向偏差和后退情况。
4. 后处理并生成正常 GIF 与 `_slow2x.gif`；输出目录采用同名 r10 规范。

## 判读

静态点在 `x=0` 会迫使机器人偏离主路径。这一轮优先观察避障是否可见且安全，
不以轨迹最短为主要目标；若 Scene 4 继续大幅出 corridor，应在后续单独收紧
绕行幅度，而不在本轮修改中混入控制参数调整。
