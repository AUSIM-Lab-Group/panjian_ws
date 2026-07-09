# DR-MPC Fig. 4 风格 SEESM 场景 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `panjian_ws` 中增加四个真实 waypoint 跟踪的 DR-MPC Fig. 4 风格场景，由 SEESM-MPC-SECBF 控制，并在 `seesm_social_navigation` 中从真实日志生成论文风格图和 GIF。

**Architecture:** runner 从场景 YAML 生成统一的参考路径文件，ROS waypoint 节点根据里程计依次发布目标，原有全局规划器继续生成 MPC 参考轨迹，SEESM/Guard/MPC-SECBF 保持不变。绘图层从 `meta.yaml` 读取同一组 waypoint 和走廊宽度，确保后端真实路径与图中参考路径一致。

**Tech Stack:** Ubuntu 20.04、ROS Noetic、C++14、Python 3、PyYAML、pytest、Matplotlib、Pillow、现有 CasADi MPC-SECBF。

## Global Constraints

- `panjian_ws` 是真实控制器和实验日志来源，`seesm_social_navigation` 只做后处理绘图。
- 场景名称固定为 `drmpc_fig4_scene_1_arc`、`drmpc_fig4_scene_2_vertical`、`drmpc_fig4_scene_3_reverse_arc`、`drmpc_fig4_scene_4_reverse_vertical`。
- 圆弧半径固定为 `4.0 m`，走廊总宽度固定为 `1.5 m`。
- waypoint 最大间距固定为 `0.4 m`，推进阈值固定为 `0.35 m`。
- 每场固定 6 名脚本化动态 human，统一 `start_delay=3.0 s`，不使用 ORCA。
- 不修改 `h_SEE`、SEESM、Guard 或 MPC-SECBF 的数学定义。
- smoke test 固定使用 `SEESM_Ours`、`30 s`、`repeat=1`。

---

### Task 1: 参考路径与 waypoint 几何

**Files:**
- Create: `swarm_test/scripts/reference_path_waypoints.py`
- Create: `swarm_test/tests/test_reference_path_waypoints.py`

**Interfaces:**
- Consumes: 场景中的 `reference_path` 字典。
- Produces: `generate_waypoints(reference_path) -> list[tuple[float, float]]`、`initial_yaw(waypoints) -> float`、`WaypointProgress`.

- [ ] **Step 1: Write the failing geometry tests**

```python
def test_arc_and_reverse_arc_are_exact_reverses():
    forward = generate_waypoints({
        "type": "arc", "center": [0.0, 0.0], "radius": 4.0,
        "start_angle": math.pi / 2, "end_angle": -math.pi / 2,
        "clockwise": True, "spacing": 0.4,
    })
    reverse = generate_waypoints({
        "type": "arc", "center": [0.0, 0.0], "radius": 4.0,
        "start_angle": -math.pi / 2, "end_angle": math.pi / 2,
        "clockwise": False, "spacing": 0.4,
    })
    assert reverse == pytest.approx(list(reversed(forward)))
    assert max(math.dist(a, b) for a, b in zip(forward, forward[1:])) <= 0.4


def test_vertical_paths_are_exact_reverses():
    up = generate_waypoints({
        "type": "line", "start": [0.0, -4.0], "goal": [0.0, 4.0],
        "spacing": 0.4,
    })
    down = generate_waypoints({
        "type": "line", "start": [0.0, 4.0], "goal": [0.0, -4.0],
        "spacing": 0.4,
    })
    assert down == pytest.approx(list(reversed(up)))
    assert initial_yaw(up) == pytest.approx(math.pi / 2)
    assert initial_yaw(down) == pytest.approx(-math.pi / 2)


def test_waypoint_progress_advances_only_inside_threshold():
    progress = WaypointProgress([(0.0, 0.0), (0.4, 0.0), (0.8, 0.0)], threshold=0.35)
    assert progress.current == (0.4, 0.0)
    assert not progress.update((0.0, 0.0))
    assert progress.update((0.1, 0.0))
    assert progress.current == (0.8, 0.0)
    assert progress.update((0.8, 0.0))
    assert progress.complete
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
cd /home/lxr20/lxr/panjian_ws
pytest -q swarm_test/tests/test_reference_path_waypoints.py
```

Expected: collection fails because `reference_path_waypoints.py` does not exist.

- [ ] **Step 3: Implement the pure geometry module**

Implement these exact rules:

```python
def generate_waypoints(path):
    spacing = float(path.get("spacing", 0.4))
    if spacing <= 0.0:
        raise ValueError("waypoint spacing must be positive")
    if path["type"] == "line":
        start = tuple(map(float, path["start"]))
        goal = tuple(map(float, path["goal"]))
        length = math.dist(start, goal)
        count = max(1, math.ceil(length / spacing))
        return [
            (
                start[0] + (goal[0] - start[0]) * index / count,
                start[1] + (goal[1] - start[1]) * index / count,
            )
            for index in range(count + 1)
        ]
    if path["type"] == "arc":
        center = tuple(map(float, path["center"]))
        radius = float(path["radius"])
        start_angle = float(path["start_angle"])
        end_angle = float(path["end_angle"])
        clockwise = bool(path["clockwise"])
        delta = end_angle - start_angle
        if clockwise and delta > 0.0:
            delta -= 2.0 * math.pi
        if not clockwise and delta < 0.0:
            delta += 2.0 * math.pi
        count = max(1, math.ceil(abs(delta) * radius / spacing))
        return [
            (
                center[0] + radius * math.cos(start_angle + delta * index / count),
                center[1] + radius * math.sin(start_angle + delta * index / count),
            )
            for index in range(count + 1)
        ]
    raise ValueError(f"unsupported reference path type: {path.get('type')}")
```

`initial_yaw` uses the first two waypoint differences. `WaypointProgress` starts at
index 1, advances at most one waypoint per `update`, exposes `current` and
`complete`, and rejects fewer than two waypoints or a non-positive threshold.

- [ ] **Step 4: Run tests and verify GREEN**

Run:

```bash
pytest -q swarm_test/tests/test_reference_path_waypoints.py
```

Expected: all tests pass.

- [ ] **Step 5: Commit Task 1**

```bash
git add swarm_test/scripts/reference_path_waypoints.py swarm_test/tests/test_reference_path_waypoints.py
git commit -m "feat: add Fig4 reference path waypoints"
```

### Task 2: ROS waypoint 执行与机器人初始朝向

**Files:**
- Create: `swarm_test/scripts/reference_path_goal_publisher.py`
- Create: `swarm_test/tests/test_reference_path_launch_contract.py`
- Modify: `swarm_test/CMakeLists.txt`
- Modify: `swarm_test/launch/start_test.launch`
- Modify: `swarm_test/launch/secbf_planner.launch`
- Modify: `swarm_test/src/data_processor.cpp`
- Modify: `simulation_tools/robot_simulator/launch/scout_simulator.xml`
- Modify: `simulation_tools/robot_simulator/src/scout_simulator.cpp`

**Interfaces:**
- Consumes: runner 生成的 `reference_path.yaml`，字段为 `waypoints`、`threshold`、`start_delay`、`final_goal`。
- Produces: `/move_base_simple/goal` waypoint 序列；机器人初始位姿参数；固定最终目标的记录契约。

- [ ] **Step 1: Write failing launch and source contract tests**

The test parses launch XML and source text, and asserts:

```python
assert "reference_path_file" in start_launch
assert "use_reference_path" in start_launch
assert "reference_path_goal_publisher.py" in start_launch
assert "final_goal_x" in start_launch and "final_goal_y" in start_launch
assert "init_x" in planner_launch and "init_y" in planner_launch and "init_yaw" in planner_launch
assert "p_init_yaw" in scout_xml
assert 'nh.param("p_init_yaw"' in scout_source
assert "use_fixed_final_goal" in data_processor_source
```

Also import the publisher module with stubbed `rospy` and verify that malformed
YAML without `waypoints` raises `ValueError`.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
pytest -q swarm_test/tests/test_reference_path_launch_contract.py
```

Expected: assertions fail because the launch arguments and node do not exist.

- [ ] **Step 3: Implement the waypoint ROS node**

The node must:

```python
path_file = Path(rospy.get_param("~path_file"))
config = yaml.safe_load(path_file.read_text(encoding="utf-8"))
waypoints = [tuple(map(float, point)) for point in config["waypoints"]]
progress = WaypointProgress(waypoints, float(config["threshold"]))
start_delay = float(config["start_delay"])
```

Subscribe to `/robot1/odom`, wait until `start_delay`, publish
`progress.current`, and publish the next waypoint only when
`progress.update((x, y))` returns true. Every `PoseStamped` uses
`frame_id="world"` and yaw equal to the next path segment tangent. Invalid YAML,
missing odometry for 5 seconds after activation, or publish failure logs a ROS
error and terminates the node.

Install both Python files beside each other so the ROS node can import the pure
geometry module:

```cmake
catkin_install_python(PROGRAMS
  scripts/reference_path_waypoints.py
  scripts/reference_path_goal_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

- [ ] **Step 4: Wire launch and fixed final goal recording**

`start_test.launch` adds:

```xml
<arg name="use_reference_path" default="false"/>
<arg name="reference_path_file" default=""/>
<arg name="final_goal_x" default="$(arg goal_x)"/>
<arg name="final_goal_y" default="$(arg goal_y)"/>
```

Run the old `start_trigger_node` only when `use_reference_path=false`; otherwise
run `reference_path_goal_publisher.py`. Pass
`use_fixed_final_goal/final_goal_x/final_goal_y` to `data_processor_node`.

In `data_processor.cpp`, fixed-final-goal mode initializes `end_pt_` from ROS
params. The first waypoint starts timing, but intermediate waypoint callbacks do
not replace `end_pt_`.

- [ ] **Step 5: Add initial yaw support**

Add `p_init_yaw` to `scout_simulator.xml` and initialize odometry orientation:

```cpp
double p_init_x, p_init_y, p_init_z, p_init_yaw;
nh.param("p_init_yaw", p_init_yaw, 0.0);
last_odom.pose.pose.orientation.w = std::cos(p_init_yaw / 2.0);
last_odom.pose.pose.orientation.z = std::sin(p_init_yaw / 2.0);
```

Expose `init_x`, `init_y`, and `init_yaw` from `secbf_planner.launch` and pass
them to `scout_simulator.xml`.

- [ ] **Step 6: Verify tests and ROS Python syntax**

Run:

```bash
pytest -q swarm_test/tests/test_reference_path_waypoints.py swarm_test/tests/test_reference_path_launch_contract.py
python3 -m py_compile swarm_test/scripts/reference_path_goal_publisher.py
```

Expected: all tests pass and `py_compile` exits 0.

- [ ] **Step 7: Commit Task 2**

```bash
git add swarm_test simulation_tools/robot_simulator
git commit -m "feat: execute reference paths through ROS waypoints"
```

### Task 3: 四场景配置与 runner 集成

**Files:**
- Modify: `swarm_test/config/secbf_scenarios.yaml`
- Modify: `swarm_test/scripts/run_secbf_sim_experiments.py`
- Create: `swarm_test/tests/test_drmpc_fig4_scenarios.py`

**Interfaces:**
- Consumes: Task 1 的 `generate_waypoints` 和 `initial_yaw`。
- Produces: 每次 run 的 `reference_path.yaml`、完整 `meta.yaml`、launch 参数。

- [ ] **Step 1: Write failing scenario tests**

Tests assert:

```python
FIG4 = (
    "drmpc_fig4_scene_1_arc",
    "drmpc_fig4_scene_2_vertical",
    "drmpc_fig4_scene_3_reverse_arc",
    "drmpc_fig4_scene_4_reverse_vertical",
)
assert [runner.SCENARIO_INDEX[name] for name in FIG4] == [216, 217, 218, 219]
for name in FIG4:
    scene = scenarios[name]
    assert len(scene["obstacles"]) == 6
    assert {float(item["start_delay"]) for item in scene["obstacles"]} == {3.0}
    assert all(item["motion_type"] == "line" for item in scene["obstacles"])
    assert "orca" not in yaml.safe_dump(scene).lower()
    assert scene["corridor_width"] == 1.5
```

Generate all four waypoint lists and assert scene 3 is the exact reverse of
scene 1, scene 4 is the exact reverse of scene 2, and every human start is at
least `1.3 m` from the robot start and goal.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
pytest -q swarm_test/tests/test_drmpc_fig4_scenarios.py
```

Expected: fails because the four scenario keys are absent.

- [ ] **Step 3: Add the exact four scene definitions**

Use these path definitions:

```yaml
scene_1: {type: arc, center: [0.0, 0.0], radius: 4.0, start_angle: 1.5707963267948966, end_angle: -1.5707963267948966, clockwise: true, spacing: 0.4}
scene_2: {type: line, start: [0.0, -4.0], goal: [0.0, 4.0], spacing: 0.4}
scene_3: {type: arc, center: [0.0, 0.0], radius: 4.0, start_angle: -1.5707963267948966, end_angle: 1.5707963267948966, clockwise: false, spacing: 0.4}
scene_4: {type: line, start: [0.0, 4.0], goal: [0.0, -4.0], spacing: 0.4}
```

Use these ordered human start/goal pairs:

```python
SCENE_1 = [
    ((-5.0, 2.8), (5.0, 2.8)),
    ((5.0, 1.4), (-1.5, 1.4)),
    ((-1.5, 0.0), (5.5, 0.0)),
    ((5.0, -1.4), (-1.5, -1.4)),
    ((-5.0, -2.8), (5.0, -2.8)),
    ((2.4, 5.0), (2.4, -5.0)),
]
SCENE_2 = [
    ((-4.5, -2.7), (4.5, -2.7)),
    ((4.5, -1.4), (-4.5, -1.4)),
    ((-4.5, 0.0), (4.5, 0.0)),
    ((4.5, 1.4), (-4.5, 1.4)),
    ((-4.5, 2.7), (4.5, 2.7)),
    ((-4.0, -3.5), (4.0, 3.5)),
]
SCENE_3 = [
    ((1.4, 3.2), (5.0, 1.5)),
    ((5.0, 2.4), (1.0, 1.2)),
    ((1.0, 0.8), (5.0, -0.5)),
    ((5.0, 0.2), (1.0, -1.0)),
    ((1.0, -1.6), (5.0, -2.8)),
    ((5.0, -3.4), (1.0, -2.0)),
]
SCENE_4 = [
    ((-4.5, -2.0), (4.5, -2.0)),
    ((4.5, -1.2), (-4.5, -1.2)),
    ((-4.5, -0.4), (4.5, -0.4)),
    ((4.5, 0.4), (-4.5, 0.4)),
    ((-4.5, 1.2), (4.5, 1.2)),
    ((4.5, 2.0), (-4.5, 2.0)),
]
```

Use semantic/cooperation order from the design spec, travel times
`[18, 19, 20, 21, 22, 24]`, `z=0.75`, and `start_delay=3.0`.

- [ ] **Step 4: Extend runner metadata and commands**

Add scenario indices 216-219. Implement:

```python
def write_reference_path_config(run_dir, scenario):
    waypoints = generate_waypoints(scenario["reference_path"])
    payload = {
        "waypoints": [[x, y] for x, y in waypoints],
        "threshold": 0.35,
        "start_delay": 3.0,
        "final_goal": list(waypoints[-1]),
    }
    path = run_dir / "reference_path.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return path, waypoints
```

For reference-path scenes, pass `use_reference_path:=true`,
`reference_path_file`, final goal, and robot initial pose to `start_test.launch`
and `secbf_planner.launch`. Write `start`, `goal`, `reference_path`,
`reference_waypoints`, and `corridor_width` into `meta.yaml`. Preserve the old
command path for every existing scene.

- [ ] **Step 5: Verify tests and dry-run**

Run:

```bash
pytest -q swarm_test/tests/test_reference_path_waypoints.py swarm_test/tests/test_reference_path_launch_contract.py swarm_test/tests/test_drmpc_fig4_scenarios.py
python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario drmpc_fig4_scene_1_arc,drmpc_fig4_scene_2_vertical,drmpc_fig4_scene_3_reverse_arc,drmpc_fig4_scene_4_reverse_vertical --baseline SEESM_Ours --duration-sec 30 --repeat 1 --dry-run
```

Expected: tests pass; dry-run prints four runs with reference path files and no
ORCA process.

- [ ] **Step 6: Commit Task 3**

```bash
git add swarm_test/config/secbf_scenarios.yaml swarm_test/scripts/run_secbf_sim_experiments.py swarm_test/tests
git commit -m "feat: add DR-MPC Fig4 SEESM scenarios"
```

### Task 4: Paper-style 路径、走廊和路径误差绘图

**Files:**
- Modify: `/home/lxr20/lxr/seesm_social_navigation/seesm_sim/visualization/panjian_backend.py`
- Modify: `/home/lxr20/lxr/seesm_social_navigation/tests/test_panjian_backend_figures.py`
- Modify: `/home/lxr20/lxr/seesm_social_navigation/tests/test_panjian_teacher_scenario_config.py`

**Interfaces:**
- Consumes: Task 3 `meta.yaml` 中的 `reference_waypoints` 和 `corridor_width`。
- Produces: 真实路径/走廊图、GIF、四宫格、`*_path_tracking_metrics.json`。

- [ ] **Step 1: Write failing renderer tests**

Add tests that build a temporary run with arc metadata and assert:

```python
geometry = _load_reference_geometry(run_path)
assert geometry["waypoints"][0] == pytest.approx([0.0, 4.0])
assert geometry["waypoints"][-1] == pytest.approx([0.0, -4.0])
assert geometry["corridor_width"] == 1.5
metrics = _path_tracking_metrics(robot_rows, geometry["waypoints"], 1.5)
assert metrics == {
    "lateral_error_rmse_m": pytest.approx(0.0),
    "lateral_error_max_m": pytest.approx(0.0),
    "outside_corridor_ratio": pytest.approx(0.0),
}
```

Render one PNG and one GIF, then assert non-blank pixel variance, expected
dimensions, `gif_obstacles_drawn_once=true`, and existence of the path metrics
JSON.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
cd /home/lxr20/lxr/seesm_social_navigation
/home/lxr20/miniconda3/envs/seesm_nav/bin/python -m pytest -q tests/test_panjian_backend_figures.py tests/test_panjian_teacher_scenario_config.py
```

Expected: fails because Fig. 4 scene titles and reference geometry helpers are absent.

- [ ] **Step 3: Implement shared metadata geometry**

Add four `SCENARIO_TITLES`. Extend `load_panjian_run_rows` so every row carries
`reference_waypoints`, `reference_path`, and `corridor_width` from the run
metadata. Replace the straight start-goal line in `_draw_scene_base` with
`_draw_reference_geometry` when metadata contains waypoints. The function draws:

```python
ax.plot(xs, ys, color="#202124", linewidth=1.6, label="reference path")
```

For arc paths, draw inner/outer radii `4.0 - 0.75` and `4.0 + 0.75` over the
same angle range. For line paths, draw the two parallel offset polylines at
`x=-0.75` and `x=0.75`. Both boundaries use `#d55e5e`, linewidth `1.2`, and no
fill. Add sparse direction arrows without changing axis limits.

`_axis_limits` must include reference waypoints and corridor boundaries.
Existing scenes without metadata keep the current straight-line fallback.

- [ ] **Step 4: Implement path tracking metrics**

For each robot point, compute minimum Euclidean distance to all waypoint line
segments. Report RMSE, maximum error, and fraction above `corridor_width/2`.
Write `<scenario>_path_tracking_metrics.json` beside the existing visual style
manifest, and include its filename plus metrics in the style manifest. When all
requested scenario names start with `drmpc_fig4_`, save the overview as
`drmpc_fig4_scenarios_overview.png`; preserve
`simulation_scenarios_overview.png` for existing calls.

- [ ] **Step 5: Verify renderer tests**

Run:

```bash
/home/lxr20/miniconda3/envs/seesm_nav/bin/python -m pytest -q tests/test_panjian_backend_figures.py tests/test_panjian_teacher_scenario_config.py
```

Expected: all tests pass.

- [ ] **Step 6: Commit Task 4 in the figure repository**

```bash
cd /home/lxr20/lxr/seesm_social_navigation
git add seesm_sim/visualization/panjian_backend.py tests/test_panjian_backend_figures.py tests/test_panjian_teacher_scenario_config.py
git commit -m "feat: render Fig4 SEESM reference paths"
```

### Task 5: Build、真实 smoke 实验与最终图片

**Files:**
- Generated run root: `swarm_test/output/secbf_runs/drmpc_fig4_seesm_smoke_20260709`
- Generated figure root: `seesm_social_navigation/outputs/drmpc_fig4_seesm_smoke_20260709`

**Interfaces:**
- Consumes: Tasks 1-4 的代码和四场景配置。
- Produces: 四个真实 run、四套 PNG/GIF、四宫格、验证摘要。

- [ ] **Step 1: Run the complete non-ROS test suite for changed behavior**

```bash
cd /home/lxr20/lxr/panjian_ws
pytest -q swarm_test/tests/test_reference_path_waypoints.py swarm_test/tests/test_reference_path_launch_contract.py swarm_test/tests/test_drmpc_fig4_scenarios.py
python3 -m py_compile swarm_test/scripts/reference_path_waypoints.py swarm_test/scripts/reference_path_goal_publisher.py swarm_test/scripts/run_secbf_sim_experiments.py
```

Expected: all tests pass and Python compilation exits 0.

- [ ] **Step 2: Build affected ROS packages**

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
catkin_make --source . -DCATKIN_WHITELIST_PACKAGES="robot_simulator;swarm_test" -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"
```

Expected: build exits 0.

- [ ] **Step 3: Run four real SEESM smoke experiments**

Use external `roscore`, then:

```bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario drmpc_fig4_scene_1_arc,drmpc_fig4_scene_2_vertical,drmpc_fig4_scene_3_reverse_arc,drmpc_fig4_scene_4_reverse_vertical \
  --baseline SEESM_Ours \
  --duration-sec 30 \
  --repeat 1 \
  --roscore external \
  --output-root swarm_test/output/secbf_runs/drmpc_fig4_seesm_smoke_20260709
```

Expected: four run directories; each contains six stable obstacle ids, non-empty
required logs, MPC timing rows, and final goal distance at most `0.55 m`.

- [ ] **Step 4: Build the figure manifest**

```bash
cd /home/lxr20/lxr/panjian_ws
python3 swarm_test/scripts/postprocess_teacher_canonical_runs.py \
  --output-root swarm_test/output/secbf_runs/drmpc_fig4_seesm_smoke_20260709
```

Expected: `manifest.csv` contains four `r01` rows and points at the four real
run directories.

- [ ] **Step 5: Generate paper-style figures and GIFs**

Use the existing panjian backend figure entry point with the smoke root and
output root:

```bash
cd /home/lxr20/lxr/seesm_social_navigation
/home/lxr20/miniconda3/envs/seesm_nav/bin/python scripts/run_panjian_backend_figures.py \
  --manifest /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/drmpc_fig4_seesm_smoke_20260709/manifest.csv \
  --output-dir /home/lxr20/lxr/seesm_social_navigation/outputs/drmpc_fig4_seesm_smoke_20260709 \
  --scenario drmpc_fig4_scene_1_arc,drmpc_fig4_scene_2_vertical,drmpc_fig4_scene_3_reverse_arc,drmpc_fig4_scene_4_reverse_vertical
```

Expected: four methods-comparison PNGs, four GIFs, four beta/h plots, four
solver/guard plots, four path metrics JSON files, and
`drmpc_fig4_scenarios_overview.png`.

- [ ] **Step 6: Inspect generated visuals and summarize evidence**

Use `view_image` on the overview and at least one keyframe from each GIF. Check:

- robot trajectory follows the configured direction;
- reference path and red corridor match the real run metadata;
- each frame contains six humans exactly once;
- no human ghosting, legend overlap, title clipping, or blank canvas;
- no plotted trajectory is synthesized independently of the logs.

Write a concise run summary with success, final goal distance, minimum distance,
minimum `h_SEE`, Guard activation rate, solve-time p95, path RMSE, maximum path
error, and corridor-outside ratio for each scene.
