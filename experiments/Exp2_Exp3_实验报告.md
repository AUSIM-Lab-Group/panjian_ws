# Exp2/Exp3 组件实验报告

报告日期：2026-06-07

本文整理 Phase 6.1 `Exp2_category_aware` 和 Phase 6.2 `Exp3_context_modulation` 的当前 sanity 实验结果。当前结果用于验证实验链路、日志字段、分析脚本和趋势是否合理；由于每组目前只有 1 次 run，本文不把结果作为最终统计结论。

统一安全函数为：

```text
h_SEE = ||p_rel + tau v_rel|| - R_obs - R_robot - beta_i
```

固定安全距离 baseline 不额外叠加 `R_safe`，而是使用 `beta_i = d_safe` 表示。

## 1. 数据来源

Exp2 数据：

```text
experiments/Exp2_category_aware/analysis/exp2_run_metrics.csv
experiments/Exp2_category_aware/analysis/exp2_summary_stats.csv
experiments/Exp2_category_aware/analysis/beta_by_class.png
experiments/Exp2_category_aware/analysis/dmin_by_class.png
experiments/Exp2_category_aware/analysis/trajectory_by_class.png
```

Exp3 数据：

```text
experiments/Exp3_context_modulation/analysis/exp3_run_metrics.csv
experiments/Exp3_context_modulation/analysis/exp3_summary_stats.csv
experiments/Exp3_context_modulation/analysis/context_mu_beta_curves.png
experiments/Exp3_context_modulation/analysis/context_beta_bar.png
```

原始 run 目录仍保留在本机，但不上传 GitHub：

```text
experiments/Exp2_category_aware/runs_static/20260607_130759_*
experiments/Exp3_context_modulation/runs_sanity_v5/20260607_174908_*
```

## 2. Exp2 类别感知实验

### 2.1 实验目的

Exp2 的目标是验证语义类别会改变安全余量 `beta_i`。实验固定地图、起点、终点、障碍物几何和运动方式，只改变单个障碍物类别：

```text
box / adult / child_like / cyclist
```

期望趋势是：

```text
beta(child_like) > beta(cyclist) > beta(adult) > beta(box)
```

如果实验链路有效，`Category_only` 应该直接体现类别查表差异；`SEESM_Ours` 应该在类别差异基础上叠加上下文调制和 Safety Guard，仍保留合理的类别顺序。

### 2.2 实验设置

```text
map: 12m x 6m
start: (0, 0)
goal: (12, 0)
obstacle_count: 1
obstacle_position: (6, 0)
motion_pattern: static_on_path
duration: 12 s
runs: 4 classes x 3 methods x 1 run
```

对比方法：

```text
Fixed_margin: 所有类别 beta_bar = 0.4, mu = 1.0
Category_only: 使用类别 beta_bar, mu = 1.0
SEESM_Ours: 使用类别 beta_bar, 使用 mu(phi), 启用 Safety Guard
```

运行命令：

```bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario Exp2_category_box,Exp2_category_adult,Exp2_category_child_like,Exp2_category_cyclist \
  --baseline Fixed_margin,Category_only,SEESM_Ours \
  --duration-sec 12 \
  --output-root /home/lxr20/lxr/panjian_ws/experiments/Exp2_category_aware/runs_static \
  --roscore auto
```

分析命令：

```bash
python3 swarm_test/scripts/analyze_exp2_category.py \
  experiments/Exp2_category_aware/runs_static \
  --output-dir experiments/Exp2_category_aware/analysis
```

### 2.3 结果

`Fixed_margin` 中，各类别的 `beta_max` 均为 0.400000，说明固定余量 baseline 没有类别差异。

| Method | Class | D_min | beta_mean | beta_max | path_length | travel_time |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Fixed_margin | box | 0.341295 | 0.382934 | 0.400000 | 9.874469 | 11.300055 |
| Fixed_margin | adult | 0.400336 | 0.387328 | 0.400000 | 9.700273 | 11.200068 |
| Fixed_margin | child_like | 0.342141 | 0.383486 | 0.400000 | 9.858692 | 11.320705 |
| Fixed_margin | cyclist | 0.361595 | 0.384368 | 0.400000 | 9.969653 | 11.260003 |

`Category_only` 的 `beta_mean/beta_max` 直接呈现类别查表顺序，并且单次 `D_min` 也呈现更高风险类别距离更大的趋势。

| Class | D_min | beta_mean | beta_max | path_length | travel_time |
| --- | ---: | ---: | ---: | ---: | ---: |
| box | 0.328074 | 0.100000 | 0.100000 | 9.878335 | 11.300096 |
| adult | 0.367867 | 0.385101 | 0.400000 | 9.874623 | 11.300191 |
| cyclist | 0.413029 | 0.561916 | 0.600000 | 9.892040 | 11.320063 |
| child_like | 0.468005 | 0.652300 | 0.700000 | 10.038605 | 11.300231 |

`SEESM_Ours` 的 `beta_mean` 顺序为：

```text
child_like(0.487842) > cyclist(0.418000) > adult(0.282672) > box(0.071892)
```

| Class | D_min | beta_mean | beta_max | path_length | travel_time |
| --- | ---: | ---: | ---: | ---: | ---: |
| box | 0.314448 | 0.071892 | 0.100000 | 9.886566 | 11.300093 |
| adult | 0.306218 | 0.282672 | 0.354577 | 9.909798 | 11.299924 |
| cyclist | 0.358951 | 0.418000 | 0.534290 | 9.925855 | 11.440172 |
| child_like | 0.442972 | 0.487842 | 0.623363 | 10.047914 | 11.280219 |

### 2.4 分析

Exp2 当前 sanity 结果支持“类别会调制安全余量”这一组件结论。`Fixed_margin` 的 `beta_max` 对所有类别相同；`Category_only` 完全符合查表预期；`SEESM_Ours` 在 Safety Guard 和上下文调制后，仍保持 `child_like > cyclist > adult > box` 的 `beta_mean` 顺序。

`D_min` 的解释需要更谨慎。`Category_only` 的单次 `D_min` 满足：

```text
child_like > cyclist > adult > box
```

这和类别风险顺序一致。`SEESM_Ours` 中 `child_like` 和 `cyclist` 的 `D_min` 明显高于 `box/adult`，但 `adult` 的单次 `D_min` 略低于 `box`。这可能来自短时 sanity 场景下 MPC 轨迹、Guard 投影、局部几何和单次随机扰动共同作用，不能据此否定类别感知机制；最终论文中应使用多 seed 的均值和标准差判断 `D_min`。

当前所有 run 的 `success` 字段为 0。该字段在本轮 12 秒 sanity 中不适合作为到达率结论，因为该批实验主要用于检查日志链路和类别趋势，并未按最终到达实验窗口设置。

### 2.5 当前结论

当前 Exp2 可以作为组件 sanity 结果写入实验进展：

```text
The category-aware margin generation behaves as expected: fixed-margin remains class-invariant, while both category-only and SEESM produce larger margins for higher-risk semantic classes.
```

但不能写成最终统计结论。最终论文表格仍需要补足每类至少 5 次 sanity，并进一步补到至少 20 个随机种子。

## 3. Exp3 上下文调制实验

### 3.1 实验目的

Exp3 的目标是固定类别为 `adult`，只改变上下文运动状态，验证 `beta_i` 不只是静态类别查表，而会随相对运动风险动态变化。

当前使用的上下文状态为：

```text
static
same_direction
crossing
frontal_approaching
```

期望趋势：

```text
beta_frontal_approaching > beta_crossing > beta_same_direction / beta_static
```

### 3.2 实验设置

```text
map: 12m x 6m
start: (0, 0)
goal: (12, 0)
obstacle_count: 1
semantic_class: adult
duration: 12 s
runs: 4 contexts x SEESM_Ours x 1 run
```

运行命令：

```bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario Exp3_context_static,Exp3_context_same_direction,Exp3_context_crossing,Exp3_context_frontal_approaching \
  --baseline SEESM_Ours \
  --duration-sec 12 \
  --output-root /home/lxr20/lxr/panjian_ws/experiments/Exp3_context_modulation/runs_sanity_v5 \
  --roscore auto
```

分析命令：

```bash
python3 swarm_test/scripts/analyze_exp3_context.py \
  experiments/Exp3_context_modulation/runs_sanity_v5 \
  --output-dir experiments/Exp3_context_modulation/analysis
```

### 3.3 结果

| Context | mu_mean | mu_max | beta_mean | beta_max | TTC_norm_mean | h_SEE_min |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| static | 0.664132 | 0.781076 | 0.265653 | 0.312430 | 0.011382 | 1.751974 |
| same_direction | 0.687867 | 0.805849 | 0.275147 | 0.322340 | 0.058224 | 1.747728 |
| crossing | 0.681831 | 0.895954 | 0.272733 | 0.358382 | 0.179675 | 0.584167 |
| frontal_approaching | 0.739023 | 0.899720 | 0.293768 | 0.359888 | 0.203874 | 0.100000 |

按 `beta_max` 排序：

```text
frontal_approaching(0.359888)
> crossing(0.358382)
> same_direction(0.322340)
> static(0.312430)
```

按 `beta_mean` 排序：

```text
frontal_approaching(0.293768)
> same_direction(0.275147)
> crossing(0.272733)
> static(0.265653)
```

### 3.4 分析

Exp3 当前 sanity 结果显示上下文峰值调制已经成立。`frontal_approaching` 和 `crossing` 的 `beta_max` 明显高于 `same_direction/static`，说明同一 `adult` 类别下，系统能根据相对运动上下文提高瞬时安全余量。

`TTC_norm_mean` 也符合风险直觉：

```text
frontal_approaching(0.203874) > crossing(0.179675) > same_direction(0.058224) > static(0.011382)
```

这说明迎面接近和横穿场景触发了更强的时间风险项。`mu_max` 中 `frontal_approaching` 和 `crossing` 也接近 0.9，说明上下文调制项在高风险瞬间能将 `mu` 推到较高水平。

需要注意的是，`beta_mean` 中 `crossing(0.272733)` 略低于 `same_direction(0.275147)`。这说明当前 12 秒单次 sanity 中，crossing 的高风险时段更像峰值事件，而不是全程平均风险；同向场景虽然峰值较低，但平均值略高。对于论文验收口径，如果要使用 `beta_mean` 做主指标，应继续调整统计窗口或补多 seed 后用均值判断；如果强调上下文瞬时风险响应，`beta_max` 和三联曲线更能表达机制。

`h_SEE_min` 方面，`frontal_approaching` 的最小值为 0.100000，说明 Guard 将安全余量压在可行边界附近；`crossing` 为 0.584167，仍有较大安全余量。当前 Exp3 没有暴露严重安全边界破坏，但 Guard 临界能力仍应在 Exp4 单独验证。

### 3.5 当前结论

当前 Exp3 可以作为组件 sanity 结果写入实验进展：

```text
The context modulation module reacts to dynamic risk: frontal approaching and crossing contexts produce higher peak safety margins than same-direction and static contexts under the same adult semantic class.
```

但如果最终论文表格要求 `beta_mean` 满足完整排序，目前证据还不够。应继续补每个上下文至少 5 次 sanity，并补到至少 20 seed，同时考虑把主图指标设为 `mu(t)/beta_hat(t)/beta(t)` 曲线和 `beta_max`，表格中再报告 `beta_mean` 作为辅助指标。

## 4. 总体判断

当前 Exp2/Exp3 的核心链路已经打通：

1. runner 能按类别和上下文批量生成 run。
2. 每个 run 能生成 `meta.yaml`、robot/obstacle/margin/planner/timing/event 日志。
3. 分析脚本能输出 CSV 和图表。
4. Exp2 的类别安全余量趋势清楚。
5. Exp3 的上下文峰值调制趋势清楚。

当前不能下的结论：

1. 不能报告 `mean ± std`，因为每组只有 1 次 run。
2. 不能用当前 `success_rate=0` 评价方法性能，因为 sanity 时长和目标到达统计尚未按最终实验设置。
3. 不能声称 Exp3 的 `beta_mean` 已满足完整排序；当前只满足 `beta_max` 排序。
4. 不能把 Exp2/Exp3 写成主结果实验，它们更适合作为组件证据实验。

## 5. 下一步实验建议

Exp2：

1. 保持当前 4 类别和 3 方法设置。
2. 每类每方法补到 5 次 sanity。
3. 如果曲线正常，再补到至少 20 seed。
4. 最终报告 `D_min, beta_mean, beta_max, path_length, travel_time` 的 `mean ± std`。
5. `success_rate` 需要延长仿真时间或调整到达判定后再使用。

Exp3：

1. 保持当前 4 上下文和 `SEESM_Ours` 设置。
2. 每个上下文补到 5 次 sanity。
3. 用 `context_mu_beta_curves.png` 判断高风险片段是否清楚。
4. 如果论文要求 `beta_mean` 排序，考虑设置风险窗口统计，例如只统计机器人与障碍物距离低于阈值或 TTC 有效的时间段。
5. 最终补到至少 20 seed，并报告 `mu_mean, mu_max, beta_mean, beta_max, TTC_norm, h_SEE_min`。

## 6. 可写入论文的保守表述

中文表述：

```text
在类别感知实验中，固定余量方法对不同语义类别给出相同的安全余量，而 Category-only 和 SEESM 均能根据类别产生差异化安全余量。当前单次 sanity 结果中，SEESM 的平均安全余量满足 child-like > cyclist > adult > box，说明语义类别已成功进入安全余量生成模块。

在上下文调制实验中，我们固定障碍物类别为 adult，仅改变其相对运动上下文。结果显示，迎面接近和横穿场景产生更高的峰值安全余量，说明 SEESM 能对动态风险作出响应。由于当前每个上下文仅完成 1 次 sanity run，均值排序仍需通过多随机种子实验进一步确认。
```

英文表述：

```text
In the category-aware experiment, the fixed-margin baseline remains class-invariant, whereas Category-only and SEESM generate class-dependent safety margins. In the current sanity run, the average SEESM margin follows child-like > cyclist > adult > box, indicating that semantic class information is correctly injected into the margin generation module.

In the context-modulation experiment, the semantic class is fixed to adult and only the motion context is changed. Frontal approaching and crossing contexts yield higher peak safety margins than same-direction and static contexts, demonstrating that SEESM reacts to dynamic interaction risk. Since the current result is based on one sanity run per context, the mean-value ordering should be confirmed with additional seeds.
```
