# SEESM All-Scenarios r10 Visual Sweep Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run every registered scenario once with SEESM_Ours, with Scene 2/4 static obstacles centered on the main path, then generate reviewable PNG/GIF artifacts.

**Architecture:** The YAML configuration remains the backend source of truth. The experiment runner sequentially executes all 29 registered scenarios against one external ROS master; postprocessing produces a manifest, then the figure layer renders each real backend run into a same-date r10 output directory.

**Tech Stack:** Python 3, PyYAML, pytest, ROS Noetic, Gazebo, ffmpeg, SEESM figure renderer.

---

### Task 1: Center Static Obstacles for Scene 2 and Scene 4

**Files:**
- Modify: `swarm_test/config/secbf_scenarios.yaml`
- Modify: `swarm_test/tests/test_drmpc_fig4_scenarios.py`
- Test: `swarm_test/tests/test_drmpc_fig4_scenarios.py`

- [ ] **Step 1: Write the failing static-point expectation**

Replace the vertical-scene static-point expectation with:

```python
expected_points = {
    "drmpc_fig4_scene_2_vertical": {(0.0, -1.5), (0.0, 3.0)},
    "drmpc_fig4_scene_4_reverse_vertical": {(0.0, -1.5), (0.0, 3.0)},
}
```

Keep the existing check that identifies static obstacles by `start == goal`.

- [ ] **Step 2: Run the focused test and confirm RED**

Run:

```bash
pytest -q swarm_test/tests/test_drmpc_fig4_scenarios.py \
  -k vertical_fig4_scenes_include_static_path_occupiers
```

Expected: FAIL because the current static obstacle x coordinates are nonzero.

- [ ] **Step 3: Change only the four static x coordinates**

In the Scene 2 and Scene 4 `obstacles` lists, set both `start.x` and
`goal.x` to `0.0` for the two zero-length line obstacles at `y=-1.5`
and `y=3.0`. Preserve every other obstacle field and every non-static
obstacle unchanged.

- [ ] **Step 4: Run the focused and full configuration tests**

Run:

```bash
pytest -q swarm_test/tests/test_drmpc_fig4_scenarios.py
```

Expected: PASS. Then run:

```bash
git diff --check -- \
  swarm_test/config/secbf_scenarios.yaml \
  swarm_test/tests/test_drmpc_fig4_scenarios.py
```

Expected: no output.

### Task 2: Verify the Full 29-Scenario Launch Matrix

**Files:**
- Read: `swarm_test/config/secbf_scenarios.yaml`
- Read: `swarm_test/scripts/run_secbf_sim_experiments.py`

- [ ] **Step 1: Count configured scenarios**

Run:

```bash
/home/lxr20/miniconda3/envs/seesm_nav/bin/python - <<'PY'
from pathlib import Path
import yaml

scenarios = yaml.safe_load(
    Path("swarm_test/config/secbf_scenarios.yaml").read_text(encoding="utf-8")
)["scenarios"]
assert len(scenarios) == 29
print(len(scenarios))
PY
```

Expected: `29`.

- [ ] **Step 2: Dry-run the all-scenario command**

Run:

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/panjian_ws/devel/setup.bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario all \
  --baseline SEESM_Ours \
  --duration-sec 30 \
  --repeat 1 \
  --roscore external \
  --dry-run \
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r10_all_scenarios_visual_sweep
```

Expected: exactly 29 scenario headings, exactly 29 `start_test.launch`
commands, all using `SEESM_Ours`.

### Task 3: Execute the r10 Backend Sweep

**Files:**
- Create: `swarm_test/output/secbf_runs/20260710_r10_all_scenarios_visual_sweep/`

- [ ] **Step 1: Start the external ROS master**

Run in a dedicated terminal/session:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

Expected: master available at `http://localhost:11311`.

- [ ] **Step 2: Run all scenarios sequentially**

Run:

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/panjian_ws/devel/setup.bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario all \
  --baseline SEESM_Ours \
  --duration-sec 30 \
  --repeat 1 \
  --roscore external \
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r10_all_scenarios_visual_sweep
```

Expected: 29 completed run directories and aggregate `summary.csv`.

- [ ] **Step 3: Stop ROS master and audit backend output**

Stop the dedicated `roscore` session with Ctrl-C. Then run:

```bash
/home/lxr20/miniconda3/envs/seesm_nav/bin/python - <<'PY'
from pathlib import Path
import csv

root = Path("swarm_test/output/secbf_runs/20260710_r10_all_scenarios_visual_sweep")
with (root / "summary.csv").open(newline="", encoding="utf-8") as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) == 29
for row in rows:
    assert Path(row["output_dir"]).is_dir()
print(f"runs={len(rows)}")
PY
```

Expected: `runs=29`. Keep failed scenario rows; do not tune or rerun within r10.

### Task 4: Postprocess and Render All Visual Artifacts

**Files:**
- Create: `seesm_social_navigation/outputs/20260710_r10_all_scenarios_visual_sweep/`

- [ ] **Step 1: Generate the manifest and summary tables**

Run:

```bash
python3 swarm_test/scripts/postprocess_teacher_canonical_runs.py \
  --output-root /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r10_all_scenarios_visual_sweep
```

Expected: `manifest.csv`, `teacher_run_metrics.csv`, and
`teacher_table_summary.csv`.

- [ ] **Step 2: Render all manifest scenarios**

Run:

```bash
/home/lxr20/miniconda3/envs/seesm_nav/bin/python \
  /home/lxr20/lxr/seesm_social_navigation/scripts/run_panjian_backend_figures.py \
  --manifest /home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/20260710_r10_all_scenarios_visual_sweep/manifest.csv \
  --output-dir /home/lxr20/lxr/seesm_social_navigation/outputs/20260710_r10_all_scenarios_visual_sweep
```

Expected: one PNG, normal GIF, visual-style JSON, path metrics JSON, and
diagnostic curves for every manifest scenario, plus overview PNG.

- [ ] **Step 3: Create slow-review GIFs**

For every `*_methods_comparison.gif`, run:

```bash
ffmpeg -y -i INPUT.gif -filter_complex "[0:v]setpts=2.0*PTS[v]" \
  -map "[v]" OUTPUT_slow2x.gif
```

Expected: one non-empty `_slow2x.gif` per normal methods-comparison GIF.

- [ ] **Step 4: Verify artifact coverage**

Run a read-only script that compares manifest scenario names against the
figure directory and asserts each has a non-empty normal GIF and slow GIF.
Report the count of successful runs, collision runs, and missing artifacts;
do not rerun failed scenarios automatically.

### Task 5: Final Visual and Metric Review

**Files:**
- Read: `swarm_test/output/secbf_runs/20260710_r10_all_scenarios_visual_sweep/summary.csv`
- Read: `seesm_social_navigation/outputs/20260710_r10_all_scenarios_visual_sweep/`

- [ ] **Step 1: Inspect Scene 2 and Scene 4 output**

Open their methods-comparison PNG and slow GIF. Check that static points are
on `x=0`, avoidance is visible, and the displayed safety metrics match
backend summary values.

- [ ] **Step 2: Summarize the sweep**

Report:

```text
configured scenarios
completed backend rows
rendered normal GIFs
rendered slow GIFs
successful runs
collision runs
Scene 2 result
Scene 4 result
scenarios needing follow-up visual tuning
```

Do not claim this single-repeat, single-method sweep establishes comparative
performance.
