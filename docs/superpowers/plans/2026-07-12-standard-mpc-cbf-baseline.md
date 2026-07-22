# Standard MPC-CBF Baseline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an auditable fixed-distance Standard MPC-CBF baseline that shares the current MPC and logging stack without using obstacle future motion, EESM, semantic margins, Guard, global SEESM, or ADSM.

**Architecture:** `mpc_secbf` gains a `cbf_metric` switch. `seesm` preserves the existing per-step predicted-obstacle barrier; `distance` freezes each obstacle at its first position in the received horizon while retaining the same soft discrete CBF and solver. The batch runner routes `Standard_MPC_CBF` into `secbf_planner.launch` with fixed `0.4 m` beta, `cbf_metric=distance`, and all dynamic semantic/global switches disabled.

**Tech Stack:** ROS Noetic XML launch, C++14, CasADi, Python 3, PyYAML, pytest.

## Global Constraints

- Standard MPC-CBF uses `h = ||p_obs(t) - p_robot(t+k)|| - R_obs - R_robot - 0.4`.
- `cbf_metric=seesm` preserves current behavior exactly.
- Standard MPC-CBF disables semantic context, all Guard mechanisms, `global_seesm_enable`, and ADSM global dynamic checks.
- All methods retain the same MPC horizon, cost, solver, obstacle ordering, recorder, and seed materialization.
- `Standard_MPC_CBF` replaces the paper alias target; `B1_ACBF_fixed` remains legacy only.
- Never modify or import main-workspace uncommitted changes.

---

### Task 1: Add the Distance-Metric Controller Interface

**Files:**
- Modify: `planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h`
- Modify: `planner/mpc_secbf/src/mpc_secbf.cpp`
- Modify: `planner/mpc_secbf/launch/mpc_secbf.launch`
- Test: `swarm_test/tests/test_standard_mpc_cbf_contract.py`

**Consumes:** Existing `obs_matrix` layout `7 x (N * obstacle_count)`.
**Produces:** A `cbf_metric` constructor argument and `distance` constraints that use `obs_matrix->col(original_idx * N_)` for every horizon step.

- [ ] **Step 1: Write the failing test**

Create `swarm_test/tests/test_standard_mpc_cbf_contract.py`:

```python
import pathlib

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

def test_distance_metric_freezes_obstacle_position_for_the_horizon():
    source = (REPO_ROOT / "planner/mpc_secbf/src/mpc_secbf.cpp").read_text(encoding="utf-8")
    header = (REPO_ROOT / "planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h").read_text(encoding="utf-8")
    assert "cbf_metric" in header
    assert 'cbf_metric_ == "distance"' in source
    assert "obs_matrix->col(original_idx * N_)" in source
    assert "h_cbf" in source

def test_mpc_launch_exposes_seesm_default_metric():
    launch = (REPO_ROOT / "planner/mpc_secbf/launch/mpc_secbf.launch").read_text(encoding="utf-8")
    assert '<arg name="cbf_metric" default="seesm"/>' in launch
    assert '<param name="mpc/cbf_metric" value="$(arg cbf_metric)"/>' in launch
```

- [ ] **Step 2: Verify red**

Run: `pytest -q swarm_test/tests/test_standard_mpc_cbf_contract.py`

Expected: FAIL because `cbf_metric` does not exist.

- [ ] **Step 3: Implement the minimal controller switch**

In `mpc_secbf.h`, add `const std::string& cbf_metric` to the solver constructor, rename `h_secbf` to `h_cbf`, and store `std::string cbf_metric_;`.

In the CBF loop in `mpc_secbf.cpp`, use:

```cpp
Eigen::VectorXd obs_for_barrier = obs_k;
if (cbf_metric_ == "distance") {
    obs_for_barrier = obs_matrix->col(original_idx * N_);
}
casadi::MX hk = h_cbf(X_cur, obs_for_barrier, beta_i);
casadi::MX hk1 = h_cbf(X_nxt, obs_for_barrier, beta_i);
```

Keep the existing distance-and-radius expression in `h_cbf`; no velocity term is added. In `mpc_secbf.launch`, add:

```xml
<arg name="cbf_metric" default="seesm"/>
<param name="mpc/cbf_metric" value="$(arg cbf_metric)"/>
```

- [ ] **Step 4: Verify green and build**

Run:

```bash
pytest -q swarm_test/tests/test_standard_mpc_cbf_contract.py
source /opt/ros/noetic/setup.bash
catkin_make --pkg mpc_secbf
```

Expected: two tests pass and `mpc_secbf` builds.

- [ ] **Step 5: Commit**

```bash
git add planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h \
  planner/mpc_secbf/src/mpc_secbf.cpp \
  planner/mpc_secbf/launch/mpc_secbf.launch \
  swarm_test/tests/test_standard_mpc_cbf_contract.py
git commit -m "feat: add fixed-distance MPC-CBF metric"
```

### Task 2: Route Standard MPC-CBF Through the Common Stack

**Files:**
- Modify: `planner/mpc_secbf/src/mpc_secbf_node.cpp`
- Modify: `swarm_test/launch/secbf_planner.launch`
- Modify: `swarm_test/scripts/run_secbf_sim_experiments.py`
- Modify: `swarm_test/tests/test_standard_mpc_cbf_contract.py`

**Consumes:** `cbf_metric` from Task 1.
**Produces:** A runner baseline with fixed beta `0.4`, no Guard, no EESM global check, and no ADSM global dynamic check.

- [ ] **Step 1: Extend the failing runner test**

Append:

```python
import importlib.util

def load_runner():
    path = REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py"
    spec = importlib.util.spec_from_file_location("runner", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def test_standard_mpc_cbf_alias_uses_common_stack():
    runner = load_runner()
    baseline = runner.BASELINES["Standard_MPC_CBF"]
    switches = runner.baseline_switches("Standard_MPC_CBF")
    assert runner.resolve_baseline_alias("Standard_MPC_CBF") == "Standard_MPC_CBF"
    assert baseline["planner"] == "secbf_planner.launch"
    assert switches["semantic_mode"] == "fixed"
    assert switches["fixed_beta"] == 0.4
    assert switches["cbf_metric"] == "distance"
    assert switches["front_adsm"] == "false"
    assert switches["global_seesm_enable"] == "false"

def test_top_level_launch_forwards_distance_metric_and_adsm():
    launch = (REPO_ROOT / "swarm_test/launch/secbf_planner.launch").read_text(encoding="utf-8")
    assert '<arg name="cbf_metric" default="seesm"/>' in launch
    assert '<arg name="front_adsm" default="true"/>' in launch
    assert '<arg name="cbf_metric" value="$(arg cbf_metric)"/>' in launch
    assert '<arg name="used_adsm_" value="$(arg front_adsm)"/>' in launch
```

- [ ] **Step 2: Verify red**

Run: `pytest -q swarm_test/tests/test_standard_mpc_cbf_contract.py -k 'standard_mpc_cbf_alias or top_level_launch'`

Expected: FAIL because runner and top-level launch have no fixed-distance baseline switches.

- [ ] **Step 3: Implement node, launch, and runner plumbing**

In `mpc_secbf_node.cpp`, read and validate `mpc/cbf_metric` before solver construction:

```cpp
nh_.param<std::string>("mpc/cbf_metric", cbf_metric_, "seesm");
if (cbf_metric_ != "seesm" && cbf_metric_ != "distance") {
    ROS_FATAL_STREAM("Unsupported mpc/cbf_metric: " << cbf_metric_);
    throw std::runtime_error("unsupported mpc/cbf_metric");
}
```

Pass `cbf_metric_` into the solver constructor and add `std::string cbf_metric_;` to node state.

In `secbf_planner.launch`, change `front_adsm` to `default="true"`, add `cbf_metric`, then forward both:

```xml
<arg name="front_adsm" default="true"/>
<arg name="cbf_metric" default="seesm"/>
<arg name="used_adsm_" value="$(arg front_adsm)"/>
<arg name="cbf_metric" value="$(arg cbf_metric)"/>
```

In the runner, add:

```python
"Standard_MPC_CBF": {
    "planner": "secbf_planner.launch",
    "controller_index": 6,
    "guard_enabled": "false",
    "mpc_feasibility_guard_enabled": "false",
    "enable_rate_limit": "false",
    "enable_available_projection": "false",
    "enable_guard_fallback": "false",
    "experiment_label": "Standard_MPC_CBF",
    "semantic_mode": "fixed",
    "fixed_beta": 0.4,
    "cbf_metric": "distance",
    "front_adsm": "false",
},
```

Add `cbf_metric="seesm"` and `front_adsm="true"` to default switches. Change the `Standard_MPC_CBF` paper alias to `Standard_MPC_CBF`. Pass `cbf_metric` and `front_adsm` in planner commands and record both beside `global_seesm_enable` in metadata. Preserve the current global-SEESM enable rule: only `Unguarded_SEESM` and `SEESM_Ours` enable it.

- [ ] **Step 4: Verify green and regressions**

Run:

```bash
pytest -q \
  swarm_test/tests/test_standard_mpc_cbf_contract.py \
  swarm_test/tests/test_global_seesm_contract.py \
  swarm_test/tests/test_reference_path_launch_contract.py
```

Expected: PASS with no global-SEESM or launch regression.

- [ ] **Step 5: Commit**

```bash
git add planner/mpc_secbf/src/mpc_secbf_node.cpp \
  swarm_test/launch/secbf_planner.launch \
  swarm_test/scripts/run_secbf_sim_experiments.py \
  swarm_test/tests/test_standard_mpc_cbf_contract.py
git commit -m "feat: route standard MPC-CBF through common stack"
```

### Task 3: Build and Smoke-Test Standard MPC-CBF

**Files:**
- Verify: `planner/mpc_secbf`
- Verify: `swarm_test/scripts/run_secbf_sim_experiments.py`
- Create at runtime: `swarm_test/output/secbf_runs/20260712_r18_standard_mpc_cbf_smoke`

**Consumes:** Tasks 1 and 2 and existing `head_on_context_int`.
**Produces:** One normal recorder run with metadata proving the fixed-distance configuration.

- [ ] **Step 1: Build and inspect the generated command**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg mpc_secbf
source devel/setup.bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario head_on_context_int \
  --baseline Standard_MPC_CBF \
  --repeat 1 --duration-sec 30 --roscore external \
  --output-root swarm_test/output/secbf_runs/20260712_r18_standard_mpc_cbf_smoke \
  --dry-run
```

Expected: the planner command contains `cbf_metric:=distance`, `fixed_beta:=0.4`, `front_adsm:=false`, and `global_seesm_enable:=false`.

- [ ] **Step 2: Run the external-master smoke trial**

In one terminal:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

In another terminal:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario head_on_context_int \
  --baseline Standard_MPC_CBF \
  --repeat 1 --duration-sec 30 --roscore external \
  --output-root swarm_test/output/secbf_runs/20260712_r18_standard_mpc_cbf_smoke
```

Expected: one completed run with non-empty robot, obstacle, margin/Guard, planner, timing, and event CSVs.

- [ ] **Step 3: Verify the audit contract**

Run:

```bash
RUN=swarm_test/output/secbf_runs/20260712_r18_standard_mpc_cbf_smoke
rg -n 'resolved_baseline_id: Standard_MPC_CBF|cbf_metric: distance|fixed_beta: 0.4|front_adsm: false|global_seesm_enable: false' "$RUN"/*/meta.yaml
find "$RUN" -type f \( -name robot_log.csv -o -name obstacle_log.csv -o -name margin_guard_log.csv -o -name planner_log.csv -o -name timing_log.csv -o -name event_log.csv \) -size +1c -print
test "$(wc -l < "$RUN"/*/global_seesm_log.csv)" -eq 1
```

Expected: all six runtime log types are non-empty, while the disabled global
SEESM log contains only its canonical header and no fabricated data rows.

- [ ] **Step 4: Commit source and test changes only**

```bash
git status --short
git add planner/mpc_secbf swarm_test/launch/secbf_planner.launch \
  swarm_test/scripts/run_secbf_sim_experiments.py \
  swarm_test/tests/test_standard_mpc_cbf_contract.py
git commit -m "test: verify standard MPC-CBF smoke contract"
```

Do not commit generated run output.
