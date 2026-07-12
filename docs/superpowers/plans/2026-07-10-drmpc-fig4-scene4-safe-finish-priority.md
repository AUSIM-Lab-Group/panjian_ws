# DR-MPC Fig. 4 Scene 4 Safe-Finish Priority Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Adjust only `drmpc_fig4_scene_4_reverse_vertical` so the scene stops colliding first and then reduces final goal error, while keeping `drmpc_fig4_scene_2_vertical` fixed at the current `r07` geometry.

**Architecture:** Tighten the approved `scene 4` contract in tests first, then minimally rewrite only the affected `scene 4` obstacle geometry/timing fields in `secbf_scenarios.yaml`. After the contract passes, rerun the two-scene SEESM batch so `scene 2` remains a control and `scene 4` can be judged against fresh backend metrics and paper-style GIFs.

**Tech Stack:** ROS Noetic, Python 3, PyYAML, pytest, existing `run_secbf_sim_experiments.py` runner, existing `postprocess_teacher_canonical_runs.py`, existing `run_panjian_backend_figures.py`, `ffmpeg`.

## Global Constraints

- Keep `drmpc_fig4_scene_2_vertical` unchanged from the current `r07` `1.5x` geometry.
- Modify only `drmpc_fig4_scene_4_reverse_vertical`.
- Prioritize `scene 4` safety: eliminate collision and `h_see < 0` before optimizing goal completion.
- Do not change `planner_v_max`, `beta_bar`, robot start/goal, reference path, corridor width, `map` size, or obstacle count.
- Keep `scene 4` at exactly `6` obstacles with the current semantic class ordering.
- Keep the lower static occupier, the middle crossing, and the lower crossing unchanged.
- Move the upper static occupier outward from `(-0.75, 3.0)` to `(-1.2, 3.0)`.
- Move the first upper crossing from `y=3.45` to `y=2.7` and slow its `travel_time` from `18.0` to `20.0`.
- Delay the diagonal `child_like` noncooperative obstacle start from `3.0 s` to a value in the approved `4.5~5.0 s` range.
- New output directories under both `swarm_test/output/secbf_runs` and `seesm_social_navigation/outputs` must use `YYYYMMDD_rNN_<task_short_desc>` ASCII naming.

---

### Task 1: Lock the new scene-4 safe-priority contract in tests

**Files:**
- Modify: `swarm_test/tests/test_drmpc_fig4_scenarios.py`

**Interfaces:**
- Consumes: `load_scenarios() -> dict`
- Produces: `test_scene4_safe_priority_adjustments_match_spec() -> None`

- [ ] **Step 1: Write the failing test**

Add a focused regression test that checks the approved `scene 4` values while asserting that `scene 2` still uses the `r07` upper static point:

```python
def test_scene4_safe_priority_adjustments_match_spec():
    scenarios = load_scenarios()
    scene2 = scenarios["drmpc_fig4_scene_2_vertical"]
    scene4 = scenarios["drmpc_fig4_scene_4_reverse_vertical"]

    assert (float(scene2["obstacles"][3]["start"]["x"]), float(scene2["obstacles"][3]["start"]["y"])) == pytest.approx(
        (0.75, 3.0)
    )

    upper_crossing = scene4["obstacles"][0]
    upper_static = scene4["obstacles"][1]
    diagonal_child = scene4["obstacles"][5]

    assert float(upper_crossing["start"]["y"]) == pytest.approx(2.7)
    assert float(upper_crossing["goal"]["y"]) == pytest.approx(2.7)
    assert float(upper_crossing["travel_time"]) == pytest.approx(20.0)
    assert float(upper_static["start"]["x"]) == pytest.approx(-1.2)
    assert float(upper_static["goal"]["x"]) == pytest.approx(-1.2)
    assert float(diagonal_child["start_delay"]) == pytest.approx(4.8)
```

- [ ] **Step 2: Run the targeted test to verify it fails**

Run: `pytest -q /home/lxr20/lxr/panjian_ws/swarm_test/tests/test_drmpc_fig4_scenarios.py -k scene4_safe_priority_adjustments_match_spec`

Expected: FAIL because `scene 4` still uses the old upper crossing height, static x-position, and diagonal start delay.

### Task 2: Apply the minimal scene-4 YAML changes

**Files:**
- Modify: `swarm_test/config/secbf_scenarios.yaml`

**Interfaces:**
- Consumes: Existing `drmpc_fig4_scene_4_reverse_vertical` obstacle list
- Produces: Updated `scene 4` upper-crossing, upper-static, and diagonal-child parameters

- [ ] **Step 1: Write the minimal implementation**

Update only the affected `scene 4` obstacle entries:

```yaml
drmpc_fig4_scene_4_reverse_vertical:
  obstacles:
    - {motion_type: "line", start: {x: 6.3, y: 2.7, z: 0.75}, goal: {x: -3.0, y: 2.7},
       travel_time: 20.0, start_delay: 3.0, semantic_class: "adult", cooperation_type: "cooperative"}
    - {motion_type: "line", start: {x: -1.2, y: 3.0, z: 0.75}, goal: {x: -1.2, y: 3.0},
       travel_time: 19.0, start_delay: 3.0, semantic_class: "pedestrian", cooperation_type: "cooperative"}
    - {motion_type: "line", start: {x: -3.3, y: 5.1, z: 0.75}, goal: {x: -5.7, y: -4.5},
       travel_time: 24.0, start_delay: 4.8, semantic_class: "child_like", cooperation_type: "noncooperative"}
```

Leave every other `scene 4` field and the entire `scene 2` block unchanged.

- [ ] **Step 2: Re-run the targeted test to verify it passes**

Run: `pytest -q /home/lxr20/lxr/panjian_ws/swarm_test/tests/test_drmpc_fig4_scenarios.py -k scene4_safe_priority_adjustments_match_spec`

Expected: PASS.

### Task 3: Re-run the full static verification suite

**Files:**
- Reuse existing tests only

**Interfaces:**
- Consumes: Updated YAML scene layouts and existing launch/script contracts
- Produces: Passing scenario-contract and launch-contract evidence

- [ ] **Step 1: Run the full scene contract test file**

Run: `pytest -q /home/lxr20/lxr/panjian_ws/swarm_test/tests/test_drmpc_fig4_scenarios.py`

Expected: all tests pass.

- [ ] **Step 2: Run the reference-path launch regression**

Run: `pytest -q /home/lxr20/lxr/panjian_ws/swarm_test/tests/test_reference_path_launch_contract.py`

Expected: all tests pass.

### Task 4: Re-run the two-scene SEESM batch with dated output naming

**Files:**
- Outputs only under `swarm_test/output/secbf_runs/20260710_r08_scene4_safe_finish_priority/`

**Interfaces:**
- Consumes: Existing `run_secbf_sim_experiments.py` CLI and external `roscore`
- Produces: Fresh run directories plus aggregate `summary.csv`

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
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r08_scene4_safe_finish_priority
```

Expected: two run directories plus aggregate `summary.csv`.

- [ ] **Step 3: Inspect the aggregate summary**

Run: `sed -n '1,20p' /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r08_scene4_safe_finish_priority/summary.csv`

Expected: two populated rows; compare `scene 4` against `r07` for `nav_collision_count`, `nav_min_distance_m`, `h_see_min`, and `robot_final_goal_distance_m`.

### Task 5: Rebuild paper-style outputs and slow GIFs

**Files:**
- Outputs only under `seesm_social_navigation/outputs/20260710_r08_scene4_safe_finish_priority/`

**Interfaces:**
- Consumes: `manifest.csv` from Task 4, `run_panjian_backend_figures.py`, `ffmpeg`
- Produces: rendered PNG/GIF/JSON outputs plus slow GIFs

- [ ] **Step 1: Build the postprocessed manifest**

Run:

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/panjian_ws/devel/setup.bash
python3 /home/lxr20/lxr/panjian_ws/swarm_test/scripts/postprocess_teacher_canonical_runs.py \
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r08_scene4_safe_finish_priority
```

Expected: writes `manifest.csv`, `teacher_run_metrics.csv`, and `teacher_table_summary.csv`.

- [ ] **Step 2: Render the paper-style figure set**

Run:

```bash
/home/lxr20/miniconda3/envs/seesm_nav/bin/python /home/lxr20/lxr/seesm_social_navigation/scripts/run_panjian_backend_figures.py \
  --manifest /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r08_scene4_safe_finish_priority/manifest.csv \
  --output-dir /home/lxr20/lxr/seesm_social_navigation/outputs/20260710_r08_scene4_safe_finish_priority \
  --scenario drmpc_fig4_scene_2_vertical,drmpc_fig4_scene_4_reverse_vertical
```

Expected: writes methods PNGs, GIFs, metrics JSONs, and overview PNG under the dated output directory.

- [ ] **Step 3: Create the slow 2x GIFs**

Run:

```bash
ffmpeg -y \
  -i /home/lxr20/lxr/seesm_social_navigation/outputs/20260710_r08_scene4_safe_finish_priority/drmpc_fig4_scene_2_vertical_methods_comparison.gif \
  -vf setpts=2.0*PTS \
  /home/lxr20/lxr/seesm_social_navigation/outputs/20260710_r08_scene4_safe_finish_priority/drmpc_fig4_scene_2_vertical_methods_comparison_slow2x.gif

ffmpeg -y \
  -i /home/lxr20/lxr/seesm_social_navigation/outputs/20260710_r08_scene4_safe_finish_priority/drmpc_fig4_scene_4_reverse_vertical_methods_comparison.gif \
  -vf setpts=2.0*PTS \
  /home/lxr20/lxr/seesm_social_navigation/outputs/20260710_r08_scene4_safe_finish_priority/drmpc_fig4_scene_4_reverse_vertical_methods_comparison_slow2x.gif
```

Expected: two playable slow GIFs in the dated output directory.
