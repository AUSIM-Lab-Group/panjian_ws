# DR-MPC Fig. 4 Vertical 1.5x Geometry Scaling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Update `drmpc_fig4_scene_2_vertical` and `drmpc_fig4_scene_4_reverse_vertical` so each scene keeps the current static-obstacle pattern while scaling the two-dimensional geometry by `1.5x`.

**Architecture:** Keep the current `scene 2` speed cap, obstacle count, semantic ordering, and timing unchanged while scaling the scene-2/scene-4 two-dimensional distances by `1.5`. Tighten the scene contract in tests first, then rewrite only the affected geometry fields in the two vertical scenarios, and finally rerun both real SEESM scenes plus paper-style GIF generation using the dated output naming rule.

**Tech Stack:** ROS Noetic, Python 3, PyYAML, pytest, existing `run_secbf_sim_experiments.py` runner, existing `run_panjian_backend_figures.py` renderer, `ffmpeg`.

## Global Constraints

- Keep both robot reference paths unchanged along the vertical `x=0` corridor.
- Keep each scene at exactly `6` obstacles and preserve the existing semantic class ordering and `beta_bar` configuration.
- Keep `drmpc_fig4_scene_2_vertical` at `planner_v_max: 0.9`; do not further reduce robot speed.
- Keep at least `3` true crossing dynamic obstacles in each of `drmpc_fig4_scene_2_vertical` and `drmpc_fig4_scene_4_reverse_vertical`.
- Represent the new static obstacles as `motion_type: "line"` with `start == goal`.
- `drmpc_fig4_scene_2_vertical` must contain static occupiers at `(-0.75, -1.5)` and `(0.75, 3.0)`.
- `drmpc_fig4_scene_4_reverse_vertical` must contain static occupiers at `(0.75, -1.5)` and `(-0.75, 3.0)`.
- `drmpc_fig4_scene_2_vertical` and `drmpc_fig4_scene_4_reverse_vertical` must use `map: {x: 15.0, y: 18.0}`, `start/goal` at `y=±6.0`, `corridor_width: 6.0`, `spacing: 0.6`, and `planner_goal_min_distance: 3.0`.
- Keep `drmpc_fig4_scene_1_arc` and `drmpc_fig4_scene_3_reverse_arc` unchanged from their current geometry.
- Keep `travel_time`, `start_delay`, the non-ORCA setup, `map.z`, and obstacle `z` unchanged.
- New directories under `seesm_social_navigation/outputs` must use `YYYYMMDD_rNN_<task_short_desc>` naming with an ASCII slug.

---

### Task 1: Lock the 1.5x scaled geometry contract in tests

**Files:**
- Modify: `swarm_test/tests/test_drmpc_fig4_scenarios.py`

**Interfaces:**
- Consumes: `load_scenarios() -> dict`
- Produces: `test_vertical_fig4_scenes_include_static_path_occupiers() -> None`

- [ ] **Step 1: Write the failing test**

Update the scene-contract tests so they require the scaled vertical geometry:

```python
def test_vertical_fig4_scaled_geometry_matches_1p5x_probe():
    scenarios = load_scenarios()
    expected = {
        "drmpc_fig4_scene_2_vertical": {
            "map": (15.0, 18.0),
            "start": (0.0, -6.0),
            "goal": (0.0, 6.0),
            "corridor_width": 6.0,
            "spacing": 0.6,
            "planner_goal_min_distance": 3.0,
        },
        "drmpc_fig4_scene_4_reverse_vertical": {
            "map": (15.0, 18.0),
            "start": (0.0, 6.0),
            "goal": (0.0, -6.0),
            "corridor_width": 6.0,
            "spacing": 0.6,
            "planner_goal_min_distance": 3.0,
        },
    }

    for name, cfg in expected.items():
        scene = scenarios[name]
        assert float(scene["map"]["x"]) == pytest.approx(cfg["map"][0])
        assert float(scene["map"]["y"]) == pytest.approx(cfg["map"][1])
        assert xy_from_mapping(scene["start"]) == pytest.approx(cfg["start"])
        assert xy_from_mapping(scene["goal"]) == pytest.approx(cfg["goal"])
        assert float(scene["corridor_width"]) == pytest.approx(cfg["corridor_width"])
        assert float(scene["reference_path"]["spacing"]) == pytest.approx(cfg["spacing"])
        assert float(scene["reference_path"]["planner_goal_min_distance"]) == pytest.approx(
            cfg["planner_goal_min_distance"]
        )
```

- [ ] **Step 2: Run the targeted test to verify it fails**

Run: `pytest -q /home/lxr20/lxr/panjian_ws/swarm_test/tests/test_drmpc_fig4_scenarios.py -k scaled_geometry_matches_1p5x_probe`

Expected: FAIL because the current vertical scenes still use the pre-scale coordinates.

### Task 2: Scale only the two vertical static-obstacle scenes by 1.5x

**Files:**
- Modify: `swarm_test/config/secbf_scenarios.yaml`

**Interfaces:**
- Consumes: Existing `drmpc_fig4_scene_2_vertical` and `drmpc_fig4_scene_4_reverse_vertical` scene blocks
- Produces: Scaled map/start/goal/reference-path/obstacle `x/y` geometry in both vertical scenes

- [ ] **Step 1: Write the minimal implementation**

Change the two scene blocks to the approved `1.5x` values. For `drmpc_fig4_scene_2_vertical`:

```yaml
drmpc_fig4_scene_2_vertical:
  map: {x: 15.0, y: 18.0, z: 3.0}
  start: {x: 0.0, y: -6.0}
  goal: {x: 0.0, y: 6.0}
  corridor_width: 6.0
  reference_path:
    start: [0.0, -6.0]
    goal: [0.0, 6.0]
    spacing: 0.6
    planner_goal_min_distance: 3.0
  obstacles:
    - start: {x: -4.5, y: -3.3}
      goal: {x: 3.0, y: -3.3}
    - start: {x: -0.75, y: -1.5}
      goal: {x: -0.75, y: -1.5}
    - start: {x: 3.9, y: 0.0}
      goal: {x: -2.7, y: 0.0}
    - start: {x: 0.75, y: 3.0}
      goal: {x: 0.75, y: 3.0}
    - start: {x: -4.2, y: 3.3}
      goal: {x: 2.7, y: 3.3}
    - start: {x: 3.3, y: -5.1}
      goal: {x: 5.7, y: 4.5}

drmpc_fig4_scene_4_reverse_vertical:
  map: {x: 15.0, y: 18.0, z: 3.0}
  start: {x: 0.0, y: 6.0}
  goal: {x: 0.0, y: -6.0}
  corridor_width: 6.0
  reference_path:
    start: [0.0, 6.0]
    goal: [0.0, -6.0]
    spacing: 0.6
    planner_goal_min_distance: 3.0
  obstacles:
    - start: {x: 6.3, y: 3.45}
      goal: {x: -3.0, y: 3.45}
    - start: {x: -0.75, y: 3.0}
      goal: {x: -0.75, y: 3.0}
    - start: {x: -6.3, y: 0.3}
      goal: {x: 3.0, y: 0.3}
    - start: {x: 0.75, y: -1.5}
      goal: {x: 0.75, y: -1.5}
    - start: {x: 6.3, y: -3.15}
      goal: {x: -3.0, y: -3.15}
    - start: {x: -3.3, y: 5.1}
      goal: {x: -5.7, y: -4.5}
```

- [ ] **Step 2: Re-run the targeted test to verify it passes**

Run: `pytest -q /home/lxr20/lxr/panjian_ws/swarm_test/tests/test_drmpc_fig4_scenarios.py -k scaled_geometry_matches_1p5x_probe`

Expected: PASS.

### Task 3: Re-run static verification for the vertical scene contract

**Files:**
- Reuse existing tests only

**Interfaces:**
- Consumes: Updated YAML scene layouts and existing launch/script contracts
- Produces: Passing scene-contract and launch-contract evidence

- [ ] **Step 1: Run the full vertical scene contract test file**

Run: `pytest -q /home/lxr20/lxr/panjian_ws/swarm_test/tests/test_drmpc_fig4_scenarios.py`

Expected: all tests pass.

- [ ] **Step 2: Run the planner launch regression**

Run: `pytest -q /home/lxr20/lxr/panjian_ws/swarm_test/tests/test_reference_path_launch_contract.py`

Expected: all tests pass.

### Task 4: Re-run both real vertical scenes with dated output naming

**Files:**
- Outputs only under `swarm_test/output/secbf_runs/`

**Interfaces:**
- Consumes: Existing `run_secbf_sim_experiments.py` CLI and external `roscore`
- Produces: `/home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260709_r07_scene24_scale15_static_offsets/summary.csv`

- [ ] **Step 1: Start external `roscore`**

Run: `source /opt/ros/noetic/setup.bash && roscore`

Expected: ROS master stays running without launch errors.

- [ ] **Step 2: Run the real two-scene SEESM experiment**

Run:

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/panjian_ws/devel/setup.bash
python3 /home/lxr20/lxr/panjian_ws/swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario drmpc_fig4_scene_2_vertical,drmpc_fig4_scene_4_reverse_vertical \
  --baseline SEESM_Ours \
  --duration-sec 30 \
  --repeat 1 \
  --roscore external \
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260709_r07_scene24_scale15_static_offsets
```

Expected: two run directories plus aggregate `summary.csv`.

- [ ] **Step 3: Inspect the aggregate summary**

Run: `sed -n '1,60p' /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260709_r07_scene24_scale15_static_offsets/summary.csv`

Expected: two populated rows with valid `success`, `robot_final_goal_distance_m`, `nav_min_distance_m`, `robot_mean_abs_w`, and `output_dir`.

### Task 5: Rebuild paper-style outputs and slow GIFs for both scenes

**Files:**
- Outputs only under `seesm_social_navigation/outputs/20260709_r07_scene24_scale15_static_offsets/`

**Interfaces:**
- Consumes: `manifest.csv` from Task 4, `run_panjian_backend_figures.py`, `ffmpeg`
- Produces: two methods GIFs, two slow GIFs, and two path metrics JSON files

- [ ] **Step 1: Build the run manifest**

Run:

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/panjian_ws/devel/setup.bash
python3 /home/lxr20/lxr/panjian_ws/swarm_test/scripts/postprocess_teacher_canonical_runs.py \
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260709_r07_scene24_scale15_static_offsets
```

Expected: writes `manifest.csv`, `teacher_run_metrics.csv`, and `teacher_table_summary.csv`.

- [ ] **Step 2: Render the paper-style figure set**

Run:

```bash
/home/lxr20/miniconda3/envs/seesm_nav/bin/python /home/lxr20/lxr/seesm_social_navigation/scripts/run_panjian_backend_figures.py \
  --manifest /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260709_r07_scene24_scale15_static_offsets/manifest.csv \
  --output-dir /home/lxr20/lxr/seesm_social_navigation/outputs/20260709_r07_scene24_scale15_static_offsets \
  --scenario drmpc_fig4_scene_2_vertical,drmpc_fig4_scene_4_reverse_vertical
```

Expected: writes methods PNGs, GIFs, metrics JSONs, and overview PNG under the dated output directory.

- [ ] **Step 3: Create the slow 2x GIFs**

Run:

```bash
ffmpeg -y \
  -i /home/lxr20/lxr/seesm_social_navigation/outputs/20260709_r07_scene24_scale15_static_offsets/drmpc_fig4_scene_2_vertical_methods_comparison.gif \
  -vf setpts=2.0*PTS \
  /home/lxr20/lxr/seesm_social_navigation/outputs/20260709_r07_scene24_scale15_static_offsets/drmpc_fig4_scene_2_vertical_methods_comparison_slow2x.gif

ffmpeg -y \
  -i /home/lxr20/lxr/seesm_social_navigation/outputs/20260709_r07_scene24_scale15_static_offsets/drmpc_fig4_scene_4_reverse_vertical_methods_comparison.gif \
  -vf setpts=2.0*PTS \
  /home/lxr20/lxr/seesm_social_navigation/outputs/20260709_r07_scene24_scale15_static_offsets/drmpc_fig4_scene_4_reverse_vertical_methods_comparison_slow2x.gif
```

Expected: writes both slow GIFs without errors.

- [ ] **Step 4: Inspect the updated path metrics**

Run:

```bash
sed -n '1,200p' /home/lxr20/lxr/seesm_social_navigation/outputs/20260709_r07_scene24_scale15_static_offsets/drmpc_fig4_scene_2_vertical_path_tracking_metrics.json
sed -n '1,200p' /home/lxr20/lxr/seesm_social_navigation/outputs/20260709_r07_scene24_scale15_static_offsets/drmpc_fig4_scene_4_reverse_vertical_path_tracking_metrics.json
```

Expected: valid JSON for both scenes, including `outside_corridor_ratio == 0.0` when the robot stays inside the corridor.
