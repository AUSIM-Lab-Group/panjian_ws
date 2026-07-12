# DR-MPC Fig. 4 竖直场景 Crossing 调整设计

## 目标

把 `drmpc_fig4_scene_2_vertical` 和
`drmpc_fig4_scene_4_reverse_vertical` 从“侧边干扰为主”调整成
“机器人必须面对真实横穿障碍物”的验证场景，用于更直接地检查
SEESM-MPC-SECBF 的动态避障能力。

## 调整原则

- 保持两场的机器人参考路径不变，仍然沿 `x=0` 竖直走廊前进。
- 保持每场 `6` 名脚本化动态 human、统一 `start_delay=3.0 s`、
  `planner_goal_min_distance=2.0`、不使用 ORCA。
- 不把全部障碍物都堆成封路墙，而是采用 `3` 个真正 crossing +
  `3` 个侧向干扰的混合布局。
- 场景 4 继续是场景 2 的反向版本，但不要求 human 轨迹逐点镜像复制；
  重点是保留“逆向通行 + crossing pressure”这一语义。

## 期望几何

### Scene 2

- 机器人从 `(0, -4)` 到 `(0, 4)`。
- 至少 `3` 个障碍物的起点与终点分别位于 `x=0` 两侧，也就是必须穿过
  机器人主路径。
- 推荐把三条主 crossing 放在 `y≈-2.2`、`y≈0.0`、`y≈2.2`，形成分层
  冲突，而不是集中在同一高度。
- 剩余 `3` 个障碍物放在左右侧做短程移动、斜向接近或边缘压迫，但不需要
  全部穿过中线。

### Scene 4

- 机器人从 `(0, 4)` 到 `(0, -4)`。
- 同样至少 `3` 个障碍物必须从 `x=0` 一侧横穿到另一侧。
- crossing 的高度分布仍覆盖上、中、下三个区段，但左右来向和 travel time
  顺序与场景 2 错开，避免两场只是简单翻转。
- 剩余 `3` 个障碍物继续承担侧向压迫和节奏干扰。

## 配置约束

- 横穿障碍物的 `start.x * goal.x` 必须小于 `0`，表示其起终点分居中线两侧。
- 障碍物起点仍需与机器人起点、终点保持最小安全分离，避免开局重叠。
- travel time 需要拉开少量差异，避免 `3` 个 crossing 在同一时刻完全重合。
- 保留当前语义类别顺序，不改 `beta_bar` 配置。
- `seesm_social_navigation/outputs` 下的新输出目录统一使用
  `YYYYMMDD_rNN_<task_short_desc>` 命名，例如
  `20260709_r03_scene2_smoother_crossing`，其中 `rNN` 表示当日第几轮，
  `<task_short_desc>` 使用简短 ASCII slug 描述本轮任务。

## 当前确认的路径占位静态障碍物调整

- 本轮同时修改 `drmpc_fig4_scene_2_vertical` 和
  `drmpc_fig4_scene_4_reverse_vertical`。
- 两场都保留现有的竖直机器人参考路径与总体 `6` 障碍物数量。
- `drmpc_fig4_scene_2_vertical` 继续保留当前 `planner_v_max: 0.9`，不再继续降低
  机器人速度；scene 4 本轮不额外引入新的速度调整逻辑。
- 两场各自选出 `2` 个原本承担外围侧向干扰的动态障碍物，替换为压在机器人
  主路径上的静态障碍物，固定坐标为 `(0, -1)` 和 `(0, 2)`。
- 为了复用当前脚本、渲染和测试链路，静态障碍物仍使用 `motion_type: "line"`，
  但写成 `start == goal` 的零长度轨迹，也就是“静态 line obstacle”。
- 两场都继续保留至少 `3` 个真正 crossing 的动态障碍物，用于维持可见的避障
  互动；新增静态障碍物是为了在主路径上提供持续占位压力，而不是替代全部动态
  冲突。
- 被替换成静态占位点的两个 obstacle 应尽量沿用原有语义类别顺序，不额外修改
  `beta_bar`、类别集合或 obstacle 总数。

## 当前确认的 1.5x 几何同比放大调整

- 在保留当前静态占位点布局与语义配置不变的前提下，同时修改
  `drmpc_fig4_scene_2_vertical` 和 `drmpc_fig4_scene_4_reverse_vertical`
  的二维几何尺度。
- 本轮以当前 `r06` 版本为基底，对两场的二维距离量统一乘 `1.5`，用于验证
  “语义边界在更大场景里的相对占比是否下降”，而不是继续单独放宽 corridor。
- 一起放大的量包括：`map.x`、`map.y`、`start/goal` 的 `x/y`、
  `reference_path` 的 `start/goal`、`corridor_width`、`spacing`、
  `planner_goal_min_distance`，以及所有障碍物 `start/goal` 的 `x/y`。
- 不修改 `map.z`、障碍物 `z`、`planner_v_max`、`travel_time`、
  `start_delay`、语义类别、`beta_bar` 和 obstacle 总数。
- `drmpc_fig4_scene_1_arc` 与 `drmpc_fig4_scene_3_reverse_arc` 不参与本轮同比放大，
  避免额外影响其他 Fig. 4 场景。
- 本轮的解释口径是：真实后端场景参数和图中的几何边界保持一致，
  不做“只改可视化、不改后端”的分离版本。

## 当前确认的 Scene 4 安全优先收敛调整

- 以 `r07` 的 `1.5x` 版本为基底，保留 `drmpc_fig4_scene_2_vertical`
  全部参数不变，不再继续调整 scene 2。
- 本轮只收敛 `drmpc_fig4_scene_4_reverse_vertical`，目标顺序明确为：
  先消除碰撞与 `h_see < 0`，再尽量减小终点误差。
- 不继续扩大整个场景，不新增障碍物，也不修改 `planner_v_max`、
  `beta_bar`、机器人起终点、reference path、corridor 宽度与 obstacle 总数。
- 调整重点放在 `scene 4` 起步上半区的三类高压因素解耦：
  上方 crossing、上方静态占位点、以及左侧斜向 noncooperative 干扰。

### Scene 4 具体改动

- 上方静态占位点从 `(-0.75, 3.0)` 外移到 `(-1.2, 3.0)`，释放机器人从
  `(0, 6)` 向下进入主通道时的左侧语义余量。
- 第一条上方 crossing 从 `y=3.45` 下移到 `y=2.7`，保持“上半区必须避障”的
  语义，但避免它与起步点过近。
- 同时将该 crossing 的 `travel_time` 从 `18.0 s` 放慢到约 `20.0 s`，
  降低其在上半区对机器人形成的瞬时横向压迫。
- 左侧斜向 `child_like` noncooperative 干扰保留原来的斜穿语义与较长轨迹，
  但将 `start_delay` 从 `3.0 s` 后移到 `4.5~5.0 s` 区间，避免其与第一条
  crossing 在起步阶段同时构成双重夹击。
- 中段 crossing、下方静态占位点、下半区 crossing 先保持不动，让后半程仍然
  具备可见干扰，避免场景被改成近似空走廊。

### 成功判据

- `scene 4` 不再出现 `nav_collision_count > 0`。
- `scene 4` 的 `h_see_min` 与 `h_ee_min` 回到非负区间。
- 在无碰撞前提下，`robot_final_goal_distance_m` 相比 `r07` 的 `4.457 m`
  有明显下降；本轮不强求一次到达完全成功，但必须优先摆脱碰撞态。

## 测试与验证

新增或调整静态测试，至少覆盖：

1. `scene 2` 和 `scene 4` 各自至少存在 `3` 个真正横穿 `x=0` 的障碍物。
2. crossing 障碍物的起点仍满足与机器人起终点的最小分离约束。
3. 原有 waypoint 反序关系、`6` 障碍物数量、无 ORCA、`planner_goal_min_distance`
   等约束保持成立。
4. `scene 2` 和 `scene 4` 都必须存在两个主路径静态占位障碍物，位置分别为
   `(0, -1)` 和 `(0, 2)`，并且以 `start == goal` 的零长度 line 形式表达。
5. `drmpc_fig4_scene_2_vertical` 当前的 `planner_v_max: 0.9` 保持不变。
6. `drmpc_fig4_scene_2_vertical` 和 `drmpc_fig4_scene_4_reverse_vertical`
   必须满足当前确认的 `1.5x` 几何比例关系：`start/goal` 变为 `y=±6`，
   `corridor_width=6.0`，`spacing=0.6`，`planner_goal_min_distance=3.0`，
   静态点坐标同步变为 `x=±0.75` 与 `y=-1.5/3.0`。
7. `scene 4` 新一轮安全优先收敛后，需要验证：
   第一条上方 crossing 已下移、上方静态点已外移、斜向 noncooperative
   obstacle 已延后启动，而 `scene 2` 的 `r07` 参数保持不变。

实现后先跑：

- `pytest -q swarm_test/tests/test_drmpc_fig4_scenarios.py`

若静态测试通过，再决定是否补跑 `scene 2` / `scene 4` 的 `30 s` smoke test
确认机器人确实需要在主路径上进行避障，而不是仅沿空走廊直行。
