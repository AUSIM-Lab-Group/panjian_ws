# Global SEESM And Benchmark Foundations Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Route MPC-final semantic margins into global dynamic search, add exact paper baseline aliases and paired seed trials, and validate Head-on-Int, Crossing-Int, and Local-Crowding-Int visual closed loops.

**Architecture:** `mpc_secbf_node` publishes the exact beta vector used by its final successful solve. An ID-bearing message reaches `Obs_Manager`, whose new SEESM predicate supplements existing global dynamic checks in `ThetaAstar`. The batch runner materializes a manifest-defined scenario for every paired trial.

**Tech Stack:** ROS Noetic, C++14, catkin messages, CasADi, Python 3, PyYAML, pytest, Matplotlib, Pillow.

## Global Constraints

- Global search consumes only final MPC `beta_applied`, never candidate beta from `/safety_margin/beta`.
- `AppliedMarginArray` carries equal-length `obstacle_ids`, `beta_applied`, and `accepted_sources`; sources are `candidate`, `previous`, `zero`, or `no_cbf`.
- Global semantic rejection is `||p_obs - p_robot + 0.20 (v_obs - v_robot)|| - R_obs - R_robot - beta_applied <= 0`.
- `search/global_seesm_enable=false` preserves existing dynamic-check behavior.
- Missing, stale (`>0.50 s`), and `no_cbf` margins use beta zero while retaining physical checks and logging the reason.
- Seed perturbations are deterministic: x/y offsets `[-0.15,0.15] m`, speed `[0.95,1.05]`, delay `[-0.20,0.20] s`.
- Formal paper trials use `--seed-manifest`, not `--repeat` as statistical evidence.
- Never stage or revert unrelated dirty files. The first live output root is `20260710_r11_global_seesm_visual_closure`.

---

## Task 1: Create the Accepted-Margin ROS Message

**Files:**
- Create: `planner/semantic_guard/msg/AppliedMarginArray.msg`
- Modify: `planner/semantic_guard/CMakeLists.txt`
- Modify: `planner/semantic_guard/package.xml`
- Create: `swarm_test/tests/test_global_seesm_contract.py`

**Produces:** `semantic_guard::AppliedMarginArray` with a header, true obstacle IDs, final beta values, and accepted sources.

- [ ] **Step 1: Write the failing contract test**

```python
def test_applied_margin_message_contract():
    path = REPO_ROOT / "planner/semantic_guard/msg/AppliedMarginArray.msg"
    text = path.read_text(encoding="utf-8")
    assert "std_msgs/Header header" in text
    assert "uint32[] obstacle_ids" in text
    assert "float64[] beta_applied" in text
    assert "string[] accepted_sources" in text
```

- [ ] **Step 2: Verify the test is red**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py::test_applied_margin_message_contract`  
Expected: FAIL because the message file does not exist.

- [ ] **Step 3: Implement message generation**

Create the message with exactly:

```text
std_msgs/Header header
uint32[] obstacle_ids
float64[] beta_applied
string[] accepted_sources
```

Add `AppliedMarginArray.msg` to `add_message_files`, add
`generate_messages(DEPENDENCIES std_msgs)`, and add `message_generation` plus
`message_runtime` dependencies in the semantic_guard build files.

- [ ] **Step 4: Verify the green path**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py::test_applied_margin_message_contract && catkin_make --pkg semantic_guard`  
Expected: pytest PASS and the semantic_guard package builds.

- [ ] **Step 5: Commit**

Run: `git add planner/semantic_guard/msg/AppliedMarginArray.msg planner/semantic_guard/CMakeLists.txt planner/semantic_guard/package.xml swarm_test/tests/test_global_seesm_contract.py && git commit -m "feat: add accepted semantic margin message"`

## Task 2: Publish Ordered IDs and MPC-Final Margins

**Files:**
- Modify: `planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp`
- Modify: `planner/semantic_guard/src/beta_ground_truth_node.cpp`
- Modify: `planner/mpc_secbf/src/mpc_secbf_node.cpp`
- Modify: `planner/mpc_secbf/CMakeLists.txt`
- Modify: `planner/mpc_secbf/package.xml`
- Modify: `planner/mpc_secbf/launch/mpc_secbf.launch`
- Modify: `swarm_test/tests/test_global_seesm_contract.py`

**Consumes:** `AppliedMarginArray` and `/globalFsm_by_adsm/obs_predict_ids` (`std_msgs/UInt32MultiArray`).

**Produces:** `/safety_margin/beta_applied_final` with one final source/value per actual obstacle ID.

- [ ] **Step 1: Write failing publication tests**

```python
def test_id_topic_and_final_margin_publication_exist():
    obs = read("planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp")
    mpc = read("planner/mpc_secbf/src/mpc_secbf_node.cpp")
    assert "obs_predict_ids" in obs
    assert "semantic_guard::AppliedMarginArray" in mpc
    assert "beta_applied_final" in mpc
    assert "publishAcceptedMargins" in mpc
```

- [ ] **Step 2: Verify the tests are red**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py -k id_topic_and_final_margin_publication`  
Expected: FAIL because neither topic nor helper exists.

- [ ] **Step 3: Implement the ordered-ID stream**

In `Obs_Manager::pub_DCBF_traj`, publish `std_msgs::UInt32MultiArray` on
`/globalFsm_by_adsm/obs_predict_ids`. Its elements must be emitted in the
same order as the obstacle blocks of the existing flat prediction matrix.

Subscribe to this topic in `beta_ground_truth_node` and `mpc_secbf_node`.
Reject a candidate or final publication when ID count and beta count differ.

- [ ] **Step 4: Implement final-margin publication after the MPC decision**

Add this helper to `MpcSecbfNode`:

```cpp
void publishAcceptedMargins(const std::vector<double>& beta, const std::string& source) {
  if (beta.size() != obstacle_ids_.size()) return;
  semantic_guard::AppliedMarginArray out;
  out.header.stamp = ros::Time::now();
  out.obstacle_ids = obstacle_ids_;
  out.beta_applied.assign(beta.begin(), beta.end());
  out.accepted_sources.assign(beta.size(), source);
  pub_beta_applied_final_.publish(out);
}
```

Call it with the exact vector passed to the final successful solve:
`beta_list_`/`candidate`, `accepted_beta_list_`/`previous`, all-zero/`zero`,
or all-zero/`no_cbf`. Add `semantic_guard` to mpc_secbf catkin dependencies
and expose launch args `obs_predict_ids_topic` and
`beta_applied_final_topic` with the required default topic names.

- [ ] **Step 5: Verify and commit**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py -k id_topic_and_final_margin_publication && catkin_make --pkg mpc_secbf && catkin_make --pkg traj_planner`  
Expected: tests PASS and both packages compile.

Run: `git add planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp planner/semantic_guard/src/beta_ground_truth_node.cpp planner/mpc_secbf/src/mpc_secbf_node.cpp planner/mpc_secbf/CMakeLists.txt planner/mpc_secbf/package.xml planner/mpc_secbf/launch/mpc_secbf.launch swarm_test/tests/test_global_seesm_contract.py && git commit -m "feat: publish MPC-final semantic margins by obstacle ID"`

## Task 3: Add the Global SEESM Check and Global Evidence Log

**Files:**
- Modify: `planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp`
- Modify: `planner/vomp_planner/traj_planner/include/path_search/theta_astar.h`
- Modify: `planner/vomp_planner/traj_planner/src/theta_astar.cpp`
- Modify: `planner/vomp_planner/traj_planner/CMakeLists.txt`
- Modify: `planner/vomp_planner/traj_planner/package.xml`
- Modify: `planner/vomp_planner/traj_planner/launch/plan_global_fsm.launch`
- Modify: `swarm_test/launch/secbf_planner.launch`
- Modify: `swarm_test/tests/test_global_seesm_contract.py`

**Produces:** `Obs_Manager::is_SEESM_unsafe(...)`, `search/global_seesm_enable`, and `global_seesm_log.csv`.

- [ ] **Step 1: Write failing predicate and log tests**

```python
def test_global_seesm_predicate_contract():
    obs = read("planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp")
    astar = read("planner/vomp_planner/traj_planner/src/theta_astar.cpp")
    assert "is_SEESM_unsafe" in obs
    assert "p_rel + tau_global_ * v_rel" in obs
    assert "global_seesm_enable" in obs
    assert "is_SEESM_unsafe" in astar

def test_global_seesm_log_columns_exist():
    obs = read("planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp")
    for value in ("beta_applied", "accepted_source", "margin_age_ms", "primitive_rejected", "shot_rejected", "reason"):
        assert value in obs
```

- [ ] **Step 2: Verify the tests are red**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py -k 'global_seesm_predicate_contract or global_seesm_log_columns_exist'`  
Expected: FAIL because global SEESM does not exist.

- [ ] **Step 3: Implement the timestamped cache and predicate**

`Obs_Manager` subscribes to `/safety_margin/beta_applied_final`, validates
message lengths, duplicate IDs, finite values, and non-negative values, then
caches `{beta, source, message_stamp, receipt_time}` by obstacle ID.

Add:

```cpp
bool is_SEESM_unsafe(const Eigen::Vector4d& robot_state,
                     double robot_radius,
                     const ros::Time& prediction_time,
                     bool shot_check);
```

For every true dynamic obstacle ID, calculate `p_rel`, `v_rel`, `h_ee`, and
`h_see`. Use beta zero and reason `missing`, `stale`, or `no_cbf` when
required. A non-positive `h_see` rejects the sampled primitive. Append:

```text
t,replan_id,global_seesm_enable,obs_id,beta_applied,accepted_source,
margin_age_ms,h_ee,h_see,primitive_rejected,shot_rejected,reason,global_replan_ms
```

to the run-local log.

- [ ] **Step 4: Integrate without replacing existing global checks**

Add `is_used_global_seesm_` to `ThetaAstar`. When true, call
`is_SEESM_unsafe` after ESDF acceptance in both primitive expansion and
`computeShotTraj`. Retain the existing VO/DIS/ADSM branch exactly as-is.

Forward these launch arguments through `plan_global_fsm.launch` and
`secbf_planner.launch`:

```xml
<arg name="global_seesm_enable" default="false"/>
<arg name="global_seesm_margin_topic" default="/safety_margin/beta_applied_final"/>
<arg name="global_seesm_log_path" default=""/>
<param name="search/global_seesm_enable" value="$(arg global_seesm_enable)"/>
<param name="search/global_seesm_tau" value="0.20"/>
<param name="search/global_seesm_margin_timeout" value="0.50"/>
```

Set the planner log path to `$(arg output_dir)/global_seesm_log.csv`.

- [ ] **Step 5: Verify and commit**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py -k 'global_seesm_predicate_contract or global_seesm_log_columns_exist' && catkin_make --pkg traj_planner && pytest -q swarm_test/tests/test_reference_path_launch_contract.py`  
Expected: all tests PASS and traj_planner builds.

Run: `git add planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp planner/vomp_planner/traj_planner/include/path_search/theta_astar.h planner/vomp_planner/traj_planner/src/theta_astar.cpp planner/vomp_planner/traj_planner/CMakeLists.txt planner/vomp_planner/traj_planner/package.xml planner/vomp_planner/traj_planner/launch/plan_global_fsm.launch swarm_test/launch/secbf_planner.launch swarm_test/tests/test_global_seesm_contract.py && git commit -m "feat: apply accepted SEESM margins in global search"`

## Task 4: Add Exact Paper Aliases and Paired Seed Manifests

**Files:**
- Create: `swarm_test/scripts/generate_seed_manifest.py`
- Modify: `swarm_test/scripts/run_secbf_sim_experiments.py`
- Modify: `swarm_test/config/secbf_scenarios.yaml`
- Modify: `swarm_test/tests/test_global_seesm_contract.py`

**Produces:** typed seed-manifest runs, effective scenario YAML, and paper aliases.

- [ ] **Step 1: Write failing alias and manifest tests**

```python
def test_paper_aliases_have_exact_baseline_ids():
    assert runner.resolve_baseline_alias("Standard_MPC_CBF") == "B1_ACBF_fixed"
    assert runner.resolve_baseline_alias("EESM_MPC_ECBF") == "No_semantic"
    assert runner.resolve_baseline_alias("SEESM_Without_FPU") == "Unguarded_SEESM"
    assert runner.resolve_baseline_alias("Proposed_MPC_SECBF") == "SEESM_Ours"

def test_seed_manifest_materializes_the_same_paired_trial(tmp_path):
    trial = runner.load_seed_manifest(tmp_path / "seed.csv")[0]
    assert runner.materialize_trial(scene, trial) == runner.materialize_trial(scene, trial)
```

- [ ] **Step 2: Verify the tests are red**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py -k 'paper_aliases_have_exact or seed_manifest_materializes'`  
Expected: FAIL because helpers do not exist.

- [ ] **Step 3: Implement aliases and manifests**

Add:

```python
PAPER_BASELINE_ALIASES = {
    "Standard_MPC_CBF": "B1_ACBF_fixed",
    "EESM_MPC_ECBF": "No_semantic",
    "SEESM_Without_FPU": "Unguarded_SEESM",
    "Proposed_MPC_SECBF": "SEESM_Ours",
}
```

Implement `--seed-manifest PATH`; reject it together with `--repeat != 1`.
Validate the exact manifest header:

```text
trial_id,seed,scenario_id,obstacle_id,start_x_offset_m,start_y_offset_m,speed_scale,start_delay_offset_s
```

Group manifest rows by `(scenario_id, trial_id, seed)`; one group describes
one full obstacle script, with one row for each perturbed obstacle. Deep-copy
obstacles, apply every row in the group, reject an unknown ID, out-of-map
start, or start within `0.8 m` of the robot. Save
`trial_manifest_row.csv`, `effective_obstacles.yaml`, resolved baseline ID,
paper label, and perturbations under every run directory. Implement the
generator with `random.Random(seed)` and an explicit `--prefix`.

Set `global_seesm_enable=true` for `Unguarded_SEESM` and `SEESM_Ours` only;
keep it false for `B1_ACBF_fixed` and `No_semantic`.

- [ ] **Step 4: Verify dry-run behavior and commit**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py -k 'paper_aliases_have_exact or seed_manifest_materializes'`  
Expected: PASS.

Run: `python3 swarm_test/scripts/generate_seed_manifest.py --scenario head_on_context_int --count 1 --seed 20260710 --prefix pilot --output /tmp/head_on_pilot.csv && python3 swarm_test/scripts/run_secbf_sim_experiments.py --scenario head_on_context_int --baseline EESM_MPC_ECBF,SEESM_Without_FPU,Proposed_MPC_SECBF --seed-manifest /tmp/head_on_pilot.csv --dry-run --roscore external`  
Expected: three dry-run entries share `pilot_001` and resolve to No_semantic, Unguarded_SEESM, and SEESM_Ours.

Run: `git add swarm_test/scripts/generate_seed_manifest.py swarm_test/scripts/run_secbf_sim_experiments.py swarm_test/config/secbf_scenarios.yaml swarm_test/tests/test_global_seesm_contract.py && git commit -m "feat: add paired seed manifests and paper baseline aliases"`

## Task 5: Aggregate and Render Global Evidence

**Files:**
- Modify: `swarm_test/scripts/postprocess_teacher_canonical_runs.py`
- Modify: `/home/lxr20/lxr/seesm_social_navigation/seesm_sim/visualization/panjian_backend.py`
- Modify: `/home/lxr20/lxr/seesm_social_navigation/tests/test_panjian_backend_figures.py`

**Produces:** global metrics in trial tables and `<scenario>_global_seesm_curves.png`.

- [ ] **Step 1: Write failing postprocess and renderer tests**

```python
def test_global_log_metrics_count_semantic_and_stale_rows(tmp_path):
    write_global_log(tmp_path / "global_seesm_log.csv", [
        {"beta_applied": "0.4", "primitive_rejected": "1", "reason": "semantic"},
        {"beta_applied": "0.0", "primitive_rejected": "0", "reason": "stale"},
    ])
    metrics = postprocess.global_seesm_metrics(tmp_path)
    assert metrics["global_beta_applied_max"] == 0.4
    assert metrics["global_semantic_rejection_count"] == 1
    assert metrics["global_stale_margin_count"] == 1
```

- [ ] **Step 2: Verify tests are red**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py -k global_log_metrics_count_semantic_and_stale_rows`  
Expected: FAIL because no global log parser exists.

- [ ] **Step 3: Implement metrics and figures**

Add `global_seesm_metrics(run_dir)` to parse `global_seesm_log.csv`. Report
max final beta, semantic rejections, missing/stale counts, and mean replan
time in trial and summary tables while preserving `trial_id` and `seed`.

Add a four-panel renderer: final accepted beta by obstacle; global hEE/hSEE;
primitive/shot rejection events; global replan milliseconds. The figure title
includes scenario, paper method, trial ID, and seed.

- [ ] **Step 4: Verify and commit**

Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py && pytest -q /home/lxr20/lxr/seesm_social_navigation/tests/test_panjian_backend_figures.py`  
Expected: all tests PASS.

Run: `git add swarm_test/scripts/postprocess_teacher_canonical_runs.py swarm_test/tests/test_global_seesm_contract.py && git commit -m "feat: aggregate global SEESM evidence"`

Run: `git -C /home/lxr20/lxr/seesm_social_navigation add seesm_sim/visualization/panjian_backend.py tests/test_panjian_backend_figures.py && git -C /home/lxr20/lxr/seesm_social_navigation commit -m "feat: render global SEESM evidence"`

## Task 6: Execute the Three-Scenario Visual Acceptance Gate

**Files:**
- Create: `swarm_test/config/seed_manifests/20260710_visual_closure.csv`
- Create: `swarm_test/output/secbf_runs/20260710_r11_global_seesm_visual_closure/README.md`
- Create: `/home/lxr20/lxr/seesm_social_navigation/outputs/20260710_r11_global_seesm_visual_closure/README.md`
- Modify: `/home/lxr20/lxr/seesm_social_navigation/老师发的实验设置/我的实验安排.md`

**Consumes:** Tasks 1-5.

**Produces:** nine visual runs: three scenarios by `No_semantic`, `Unguarded_SEESM`, and `SEESM_Ours`.

- [ ] **Step 1: Create and test the fixed manifest**

Add a test asserting the manifest contains exactly three distinct
`(scenario_id, trial_id, seed)` groups for `head_on_context_int`,
`crossing_context_int`, and `local_crowding_context_int`; every row uses
`pilot_001` and seed `20260710`.
Run: `pytest -q swarm_test/tests/test_global_seesm_contract.py -k visual_closure_manifest`  
Expected: FAIL before creating the manifest, PASS after creating it.

- [ ] **Step 2: Build packages and run with external roscore**

Run: `catkin_make --pkg semantic_guard && catkin_make --pkg mpc_secbf && catkin_make --pkg traj_planner`  
Expected: all packages build.

Start `roscore` in one terminal, then run:

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/panjian_ws/devel/setup.bash
cd /home/lxr20/lxr/panjian_ws
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario head_on_context_int,crossing_context_int,local_crowding_context_int \
  --baseline EESM_MPC_ECBF,SEESM_Without_FPU,Proposed_MPC_SECBF \
  --seed-manifest swarm_test/config/seed_manifests/20260710_visual_closure.csv \
  --duration-sec 30 --roscore external \
  --output-root swarm_test/output/secbf_runs/20260710_r11_global_seesm_visual_closure
```

Verify every run has non-empty robot, obstacle, Guard, planner, timing, event,
and summary logs. Verify each SEESM run has non-empty `global_seesm_log.csv`.
Classify rather than retune collision, goal timeout, global no-path, MPC
infeasibility, and invalid infrastructure failures.

- [ ] **Step 3: Postprocess, render, review, and record the gate**

Run:

```bash
python3 swarm_test/scripts/postprocess_teacher_canonical_runs.py --output-root swarm_test/output/secbf_runs/20260710_r11_global_seesm_visual_closure
/home/lxr20/miniconda3/envs/seesm_nav/bin/python /home/lxr20/lxr/seesm_social_navigation/scripts/run_panjian_backend_figures.py --manifest swarm_test/output/secbf_runs/20260710_r11_global_seesm_visual_closure/manifest.csv --output-dir /home/lxr20/lxr/seesm_social_navigation/outputs/20260710_r11_global_seesm_visual_closure --scenario head_on_context_int,crossing_context_int,local_crowding_context_int
```

Confirm each scenario has methods PNG/GIF, beta/h, solver/Guard, and global
SEESM curves. Inspect the three PNGs and one frame from every GIF. Update
`我的实验安排.md` with output roots, manifest path, baseline aliases, command
results, and the explicit gate that blocks Exp2/Exp3 until this review passes.

- [ ] **Step 4: Commit owned artifacts and documentation**

Run: `git add swarm_test/config/seed_manifests/20260710_visual_closure.csv swarm_test/tests/test_global_seesm_contract.py && git commit -m "test: validate global SEESM visual closure"`

Run: `git -C /home/lxr20/lxr/seesm_social_navigation add '老师发的实验设置/我的实验安排.md' && git -C /home/lxr20/lxr/seesm_social_navigation commit -m "docs: record global SEESM visual closure"`

## Plan Self-Review

- Tasks 1-3 cover final-margin identity, global predicate, compatibility, stale behavior, and logs.
- Task 4 covers exact aliases and paired seeds.
- Task 5 covers evidence extraction and plotting.
- Task 6 is the first live gate; Exp2/Exp3, 9-condition pilot, formal batch, dense stress, and real-world work remain intentionally deferred.
- `AppliedMarginArray`, `/safety_margin/beta_applied_final`, `global_seesm_enable`, `is_SEESM_unsafe`, and the four accepted sources are named consistently in every task.
