# Collision Episode Postprocessing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Report physical collision episodes from obstacle clearance logs without treating every control-cycle contact sample as an independent collision.

**Architecture:** The canonical postprocessor will derive one episode count per run from `obstacle_log.csv`. For each obstacle ID, a collision begins when `d_i - radius - R_robot < 0`; consecutive negative samples remain one episode if their timestamp gap is at most 2.5 times that obstacle's P90 positive sample interval. The P90 estimate is robust to duplicate prediction-message bursts within one physical control cycle. The derived count is added to trial metrics and summarized per condition, while raw `nav_collision_count` remains unchanged.

**Tech Stack:** Python 3 standard library, existing CSV/YAML postprocessor, pytest.

## Global Constraints

- Clearance uses the logged obstacle radius and `robot_radius` from `meta.yaml` (default `0.4 m`).
- Episode grouping is per obstacle ID; different obstacles never merge into one event.
- Missing or malformed obstacle samples yield zero episodes only when no valid clearance samples exist; the raw logs remain the audit source.
- Paper Success remains the existing reproducible rule: goal reached and `nav_collision_count == 0`.

---

### Task 1: Derive per-run collision episodes

**Files:**
- Modify: `swarm_test/scripts/postprocess_teacher_canonical_runs.py`
- Test: `swarm_test/tests/test_global_seesm_postprocess.py`

**Interfaces:**
- Consumes: `obstacle_log.csv` columns `t,id,radius,d_i` and `meta.yaml.robot_radius`.
- Produces: `collision_episode_metrics(run_dir) -> {"collision_episode_count": int}`.

- [x] Write a failing test with two separated negative intervals for one obstacle and one interval for a second obstacle; expect three episodes.
- [x] Run `pytest -q swarm_test/tests/test_global_seesm_postprocess.py` and confirm the missing helper fails.
- [x] Implement clearance parsing, P90 sample-period estimation, and per-obstacle episode grouping.
- [x] Run the focused test and confirm it passes.

### Task 2: Export and aggregate the metric

**Files:**
- Modify: `swarm_test/scripts/postprocess_teacher_canonical_runs.py`
- Test: `swarm_test/tests/test_global_seesm_postprocess.py`

**Interfaces:**
- Consumes: `collision_episode_metrics`.
- Produces: `collision_episode_count` in `teacher_run_metrics.csv` and `collision_episode_mean` in `teacher_table_summary.csv`.

- [x] Write a failing aggregation test for mean episode count.
- [x] Run the focused pytest target and confirm it fails for the new field.
- [x] Add the two stable CSV fields without changing existing raw collision or Success fields.
- [x] Run the focused test and confirm it passes.

### Task 3: Replay the completed pilot and document the frozen rule

**Files:**
- Modify: `seesm_social_navigation/老师发的实验设置/我的实验安排.md`

- [x] Run the postprocessor on `20260712_r23_standard_mpc_cbf_pilot5`.
- [x] Verify 45 metric rows, 9 table rows, and the derived episode fields.
- [x] Record the grouping rule and r23 episode result in the experiment plan.
- [ ] Run the relevant contract/postprocessor test suite and `git diff --check`.
