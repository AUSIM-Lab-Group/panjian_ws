# DR-MPC Fig. 4 Direct Final Goal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Fig. 4 Scene 2 and Scene 4 publish only their final goal while preserving waypoint execution for the two arc scenes and preserving full reference-path metadata for plotting.

**Architecture:** Add a validated `reference_path.goal_mode` policy to the experiment runner. `final_only` line scenes continue generating reference-path files and metadata, but their ROS launch uses the existing final-goal trigger; waypoint scenes continue launching `reference_path_goal_publisher.py` through `use_reference_path:=true`.

**Tech Stack:** Python 3, pytest, PyYAML, ROS Noetic/roslaunch, existing SEESM experiment and figure scripts.

---

### Task 1: Add Failing Goal-Mode Contract Tests

**Files:**
- Modify: `swarm_test/tests/test_drmpc_fig4_scenarios.py`
- Test: `swarm_test/tests/test_drmpc_fig4_scenarios.py`

- [ ] **Step 1: Add the failing scene-mode and command-routing test**

Append this test after `test_fig4_waypoints_are_reversed_pairs_with_final_goals`:

```python
def test_vertical_scenes_use_final_only_goals_while_arcs_keep_waypoints(tmp_path):
    scenarios = load_scenarios()
    expected_modes = {
        "drmpc_fig4_scene_1_arc": "waypoints",
        "drmpc_fig4_scene_2_vertical": "final_only",
        "drmpc_fig4_scene_3_reverse_arc": "waypoints",
        "drmpc_fig4_scene_4_reverse_vertical": "final_only",
    }

    for name, expected_mode in expected_modes.items():
        scene = scenarios[name]
        assert runner.reference_goal_mode(scene) == expected_mode

        run_dir = tmp_path / name
        run_dir.mkdir()
        path_file, waypoints = runner.write_reference_path_config(run_dir, scene)
        classes_arg = runner.obstacle_classes(scene["obstacles"])
        _, start_cmd = runner.build_commands(
            name,
            "SEESM_Ours",
            run_dir,
            run_dir / "obstacles_param.yaml",
            classes_arg,
            len(scene["obstacles"]),
            scene,
        )

        assert path_file.exists()
        assert waypoints[-1] == pytest.approx(xy_from_mapping(scene["goal"]))
        if expected_mode == "final_only":
            assert "use_reference_path:=true" not in start_cmd
            assert f"goal_x:={float(scene['goal']['x'])}" in start_cmd
            assert f"goal_y:={float(scene['goal']['y'])}" in start_cmd
        else:
            assert "use_reference_path:=true" in start_cmd
            assert f"reference_path_file:={path_file}" in start_cmd
```

- [ ] **Step 2: Add failing validation tests**

Append these tests to the same file:

```python
def test_reference_goal_mode_rejects_unknown_mode():
    scene = {"reference_path": {"type": "line", "goal_mode": "unknown"}}
    with pytest.raises(ValueError, match="goal_mode"):
        runner.reference_goal_mode(scene)


def test_reference_goal_mode_rejects_final_only_arc():
    scene = {"reference_path": {"type": "arc", "goal_mode": "final_only"}}
    with pytest.raises(ValueError, match="line"):
        runner.reference_goal_mode(scene)


def test_reference_goal_mode_defaults_to_waypoints():
    scene = {"reference_path": {"type": "line"}}
    assert runner.reference_goal_mode(scene) == "waypoints"
```

- [ ] **Step 3: Run the focused tests and confirm RED**

Run:

```bash
pytest -q swarm_test/tests/test_drmpc_fig4_scenarios.py \
  -k 'goal_mode or final_only_goals'
```

Expected: FAIL because `reference_goal_mode` does not exist and Scene 2/4 do not yet declare `goal_mode: final_only`.

### Task 2: Implement Validated Final-Goal Routing

**Files:**
- Modify: `swarm_test/scripts/run_secbf_sim_experiments.py`
- Modify: `swarm_test/config/secbf_scenarios.yaml`
- Test: `swarm_test/tests/test_drmpc_fig4_scenarios.py`

- [ ] **Step 1: Add the goal-mode policy helpers**

Immediately after `is_reference_path_scene`, add:

```python
REFERENCE_GOAL_MODES = {"waypoints", "final_only"}


def reference_goal_mode(scenario: dict):
    reference_path = scenario.get("reference_path")
    if not reference_path:
        return None
    mode = str(reference_path.get("goal_mode", "waypoints"))
    if mode not in REFERENCE_GOAL_MODES:
        raise ValueError(f"unsupported reference_path goal_mode: {mode}")
    if mode == "final_only" and reference_path.get("type") != "line":
        raise ValueError("reference_path goal_mode final_only requires type: line")
    return mode


def uses_reference_waypoints(scenario: dict) -> bool:
    return reference_goal_mode(scenario) == "waypoints"
```

This keeps missing `reference_path` distinct from waypoint execution and validates the policy before launch.

- [ ] **Step 2: Route launch arguments through the validated mode**

In `build_commands`, keep `reference_path = is_reference_path_scene(scenario)` for waypoint generation and initial yaw, then add:

```python
    goal_mode = reference_goal_mode(scenario)
```

Replace:

```python
    if reference_path:
```

with:

```python
    if goal_mode == "waypoints":
```

Do not change `write_reference_path_config` or `write_run_meta`; both must continue receiving the full path for every Fig. 4 scene.

- [ ] **Step 3: Mark only the vertical line scenes as final-only**

In `swarm_test/config/secbf_scenarios.yaml`, add this field under the `reference_path` block of both `drmpc_fig4_scene_2_vertical` and `drmpc_fig4_scene_4_reverse_vertical`:

```yaml
      goal_mode: "final_only"
```

Do not add the field to Scene 1 or Scene 3. Do not alter any obstacle, speed, map, corridor, start, goal, spacing, or safety parameter.

- [ ] **Step 4: Run the focused tests and confirm GREEN**

Run:

```bash
pytest -q swarm_test/tests/test_drmpc_fig4_scenarios.py \
  -k 'goal_mode or final_only_goals'
```

Expected: `4 passed`, with the vertical commands omitting `use_reference_path:=true` and arc commands retaining it.

- [ ] **Step 5: Inspect the scoped diff**

Run:

```bash
git diff --check -- \
  swarm_test/config/secbf_scenarios.yaml \
  swarm_test/scripts/run_secbf_sim_experiments.py \
  swarm_test/tests/test_drmpc_fig4_scenarios.py
```

Expected: no output. Confirm the only new behavior is the goal-mode policy and the two `final_only` fields; preserve all pre-existing uncommitted changes.

### Task 3: Run Regression and Command-Contract Verification

**Files:**
- Test: `swarm_test/tests/test_drmpc_fig4_scenarios.py`
- Test: `swarm_test/tests/test_reference_path_launch_contract.py`
- Test: `swarm_test/tests/test_reference_path_waypoints.py`

- [ ] **Step 1: Run the complete Fig. 4 scenario tests**

Run:

```bash
pytest -q swarm_test/tests/test_drmpc_fig4_scenarios.py
```

Expected: all tests pass.

- [ ] **Step 2: Run reference-path regressions**

Run:

```bash
pytest -q \
  swarm_test/tests/test_reference_path_launch_contract.py \
  swarm_test/tests/test_reference_path_waypoints.py
```

Expected: all tests pass; the arc waypoint publisher and pure geometry helpers remain unchanged.

- [ ] **Step 3: Verify dry-run launch commands**

Run:

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/panjian_ws/devel/setup.bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario drmpc_fig4_scene_2_vertical,drmpc_fig4_scene_4_reverse_vertical \
  --baseline SEESM_Ours \
  --duration-sec 30 \
  --repeat 1 \
  --roscore external \
  --dry-run \
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r09_scene24_direct_final_goal
```

Expected: both `start_test.launch` commands contain `goal_x:=0.0` and the appropriate `goal_y`, and neither command contains `use_reference_path:=true`.

### Task 4: Execute r09 ROS Validation

**Files:**
- Create: `swarm_test/output/secbf_runs/20260710_r09_scene24_direct_final_goal/`

- [ ] **Step 1: Start an external ROS master**

Run in a dedicated terminal/session:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

Expected: ROS master starts on `http://localhost:11311` and remains running during both scenes.

- [ ] **Step 2: Run Scene 2 and Scene 4 sequentially**

Run:

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/panjian_ws/devel/setup.bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario drmpc_fig4_scene_2_vertical,drmpc_fig4_scene_4_reverse_vertical \
  --baseline SEESM_Ours \
  --duration-sec 30 \
  --repeat 1 \
  --roscore external \
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r09_scene24_direct_final_goal
```

Expected: two completed run directories and an aggregate `summary.csv`.

- [ ] **Step 3: Verify final-goal-only runtime evidence**

Run:

```bash
rg -n "Advanced reference target|Published reference waypoint" \
  swarm_test/output/secbf_runs/20260710_r09_scene24_direct_final_goal/*/start_test.log
```

Expected: no matches. Then run:

```bash
rg -n "goal has publisher" \
  swarm_test/output/secbf_runs/20260710_r09_scene24_direct_final_goal/*/start_test.log
```

Expected: one final-goal trigger message per scene.

- [ ] **Step 4: Compare r09 outcomes against r08**

Inspect:

```bash
column -s, -t < swarm_test/output/secbf_runs/20260710_r09_scene24_direct_final_goal/summary.csv
```

Acceptance checks:

- Scene 2 no longer remains locked near the first intermediate target and its final-goal error is materially below r08's `8.564408 m`.
- Scene 4 still reaches or remains close to `(0, -6)`; its final-goal error must not materially regress from r08's `0.003131 m`.
- Report `nav_collision_count`, `nav_min_distance_m`, `h_ee_min`, `h_see_min`, `guard_rollback_count`, and final-goal error without hiding metric disagreements.
- If either scene fails, stop after collecting evidence; do not tune obstacles or safety parameters in the same run.

- [ ] **Step 5: Stop the external ROS master**

Send Ctrl-C to the dedicated `roscore` session and confirm it exits.

### Task 5: Postprocess and Render Review GIFs

**Files:**
- Create: `seesm_social_navigation/outputs/20260710_r09_scene24_direct_final_goal/`

- [ ] **Step 1: Generate the paper manifest and tables**

Run:

```bash
python3 swarm_test/scripts/postprocess_teacher_canonical_runs.py \
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r09_scene24_direct_final_goal
```

Expected: `manifest.csv`, `teacher_run_metrics.csv`, and `teacher_table_summary.csv` are created.

- [ ] **Step 2: Render Scene 2 and Scene 4 outputs**

Run:

```bash
/home/lxr20/miniconda3/envs/seesm_nav/bin/python \
  /home/lxr20/lxr/seesm_social_navigation/scripts/run_panjian_backend_figures.py \
  --manifest /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r09_scene24_direct_final_goal/manifest.csv \
  --output-dir /home/lxr20/lxr/seesm_social_navigation/outputs/20260710_r09_scene24_direct_final_goal \
  --scenario drmpc_fig4_scene_2_vertical,drmpc_fig4_scene_4_reverse_vertical
```

Expected: PNG, normal GIF, visual-style JSON, path metrics, and diagnostic curves for both scenes.

- [ ] **Step 3: Create two-times slower review GIFs**

Run once per normal GIF:

```bash
ffmpeg -y -i INPUT.gif -filter_complex "[0:v]setpts=2.0*PTS[v]" \
  -map "[v]" OUTPUT_slow2x.gif
```

Use the exact Scene 2 and Scene 4 filenames generated in the r09 figure directory. Expected: both `_slow2x.gif` files exist and are non-empty.

- [ ] **Step 4: Review the final plots and metrics**

Open both methods-comparison PNGs and verify that Scene 2 no longer turns back toward an obsolete intermediate waypoint. Cross-check the plotted final error against backend `summary.csv`; explicitly report any difference between visualization clearance metrics and backend collision/safety metrics.
