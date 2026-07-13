# Dynamic Tau for EESM and SEESM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the fixed look-ahead time in EESM-MPC-ECBF, Unguarded SEESM, and Proposed MPC-SECBF with one auditable, velocity-aware `tau_i(k)` policy while preserving the Standard MPC-CBF and legacy ACBF baselines.

**Architecture:** Put the numeric policy in the public `semantic_guard` header so Guard, ground-truth margin generation, and global SEESM use exactly the same implementation. Add an algebraically equivalent CasADi expression inside `mpc_secbf` for every obstacle and prediction stage, with the existing `h_EE`, `h_SEE`, slack, obstacle-selection, and `beta_applied_final` contracts unchanged. Extend logs and trial metadata with the factors that produced each `tau`, then validate the three dynamic methods on the same seed before running a larger benchmark.

**Tech Stack:** C++14, ROS Noetic/catkin, Eigen, CasADi/Opti, GoogleTest through `catkin_add_gtest`, Python 3/pytest, CSV/JSON trial artifacts.

---

## Scope Map

The implementation is split by ownership so each change has one testable responsibility:

| File | Responsibility in this plan |
|---|---|
| `planner/semantic_guard/include/semantic_guard/dynamic_tau.hpp` | Header-only numeric dynamic-`tau` policy and result metadata |
| `planner/semantic_guard/src/beta_guard_node.cpp` | Guard-side accepted margin and audit log using the shared policy |
| `planner/semantic_guard/src/beta_ground_truth_node.cpp` | Unguarded/reference margin path using the same policy |
| `planner/semantic_guard/launch/beta_guard.launch` | Guard dynamic-`tau` parameters |
| `planner/semantic_guard/launch/beta_ground_truth.launch` | Ground-truth/reference dynamic-`tau` parameters |
| `planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp` | Global SEESM check using dynamic `tau` and final accepted beta |
| `planner/vomp_planner/traj_planner/launch/plan_global_fsm.launch` | Global dynamic-`tau` parameters |
| `planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h` | MPC solver configuration and symbolic helper declarations |
| `planner/mpc_secbf/src/mpc_secbf.cpp` | CasADi dynamic-`tau` expression inside `h_cbf` |
| `planner/mpc_secbf/src/mpc_secbf_node.cpp` | Solver parameter plumbing and MPC audit fields |
| `planner/mpc_secbf/launch/mpc_secbf.launch` | MPC dynamic-`tau` parameters |
| `swarm_test/launch/secbf_planner.launch` | Forward the common parameters into all nodes |
| `swarm_test/scripts/run_secbf_sim_experiments.py` | Method mapping, launch arguments, metadata, and summary fields |
| `swarm_test/scripts/check_experiment_csv_fields.py` | Required-field contract for the new audit columns |
| `semantic_guard/test/test_dynamic_tau_policy.cpp` | Numeric policy unit tests |
| `swarm_test/tests/test_dynamic_tau_contract.py` | Static contract for policy usage and method switches |

The following remain unchanged and must be checked after implementation: `planner/mpc_dcbf/src/mpc_cbf.cpp` and its legacy `B1_ACBF_fixed` launch path, Standard MPC-CBF's fixed-distance behavior, the `beta_applied_final` topic semantics, obstacle selection limits, CBF slack handling, and the existing seed manifest format.

## Shared Definitions

The implementation uses the code-standard safety functions:

```text
h_EE  = ||l + tau v||_2 - R_obs - R_robot
h_SEE = h_EE - beta
```

`beta` is the explicit semantic margin. Do not add a hidden `R_safe` term to either function. The three methods map as follows:

| Runner method | Dynamic `tau` | `beta` passed to local MPC | Guard |
|---|---:|---:|---:|
| `Standard_MPC_CBF` | no | fixed distance baseline (`0.4`) | no |
| `No_semantic` / `EESM_MPC_ECBF` | yes | `0.0` | no semantic effect |
| `Unguarded_SEESM` / `SEESM_Without_FPU` | yes | candidate semantic beta | disabled |
| `SEESM_Ours` / `Proposed_MPC_SECBF` | yes | Guard-accepted `beta_applied_final` | enabled |

The paper factors are recorded as `tau = f_r * f_v * f_T * Ke * T_i`. The code conversion must guarantee a finite, non-negative result: stationary, receding, non-threatening, invalid, or out-of-horizon cases return `tau = 0`. Closing cases may return a positive value no larger than `tau_max`. The same convention must be used by the numeric helper, Guard, global checker, and CasADi expression.

## Task 1: Add Failing Policy and Contract Tests

**Files:**
- Create: `planner/semantic_guard/test/test_dynamic_tau_policy.cpp`
- Create: `swarm_test/tests/test_dynamic_tau_contract.py`
- Modify: `planner/semantic_guard/CMakeLists.txt`
- Modify: `planner/semantic_guard/package.xml`

- [ ] **Step 1: Write the numeric unit test before adding the helper.**

Create the test with these cases and assertions. The test deliberately includes the public header that does not exist yet, so the first build proves that the test is connected to the intended API.

```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "semantic_guard/dynamic_tau.hpp"

namespace {
using semantic_guard::DynamicTauParams;
using semantic_guard::DynamicTauResult;
using semantic_guard::computeDynamicTau;

DynamicTauParams params() {
  DynamicTauParams p;
  p.ke = 0.30;
  p.t_max = 2.0;
  p.min_speed = 1e-6;
  p.min_distance = 1e-6;
  p.max_tau = 2.0;
  return p;
}

TEST(DynamicTauPolicy, StationaryAndRecedingReturnZero) {
  const auto stationary = computeDynamicTau(5.0, 0.0, 0.0, 0.0, 0.8, params());
  const auto receding = computeDynamicTau(5.0, 0.0, 1.0, 0.0, 0.8, params());
  EXPECT_DOUBLE_EQ(stationary.tau, 0.0);
  EXPECT_DOUBLE_EQ(receding.tau, 0.0);
  EXPECT_TRUE(std::isfinite(stationary.tau));
  EXPECT_TRUE(std::isfinite(receding.tau));
}

TEST(DynamicTauPolicy, ClosingMotionProducesFiniteNonNegativeTau) {
  const auto result = computeDynamicTau(5.0, 0.0, -1.0, 0.0, 0.8, params());
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.closing);
  EXPECT_GE(result.tau, 0.0);
  EXPECT_LE(result.tau, params().max_tau);
  EXPECT_TRUE(std::isfinite(result.T_i));
}

TEST(DynamicTauPolicy, VelocityObstacleGateRejectsOutsideCone) {
  const auto result = computeDynamicTau(5.0, 0.0, -0.1, 2.0, 0.8, params());
  EXPECT_DOUBLE_EQ(result.tau, 0.0);
  EXPECT_EQ(result.reason, "velocity_gate");
}

TEST(DynamicTauPolicy, HorizonGateRejectsTiAfterTmax) {
  DynamicTauParams p = params();
  p.t_max = 0.5;
  const auto result = computeDynamicTau(5.0, 0.0, -0.1, 0.0, 0.8, p);
  EXPECT_DOUBLE_EQ(result.tau, 0.0);
  EXPECT_EQ(result.reason, "time_gate");
}

TEST(DynamicTauPolicy, DegenerateInputsAreFinite) {
  const auto zero_speed = computeDynamicTau(5.0, 0.0, 0.0, 0.0, 0.8, params());
  const auto near_zero_distance = computeDynamicTau(0.0, 0.0, -1.0, 0.0, 0.8, params());
  const auto non_finite = computeDynamicTau(NAN, 0.0, -1.0, 0.0, 0.8, params());
  for (const DynamicTauResult* result : {&zero_speed, &near_zero_distance, &non_finite}) {
    EXPECT_DOUBLE_EQ(result->tau, 0.0);
    EXPECT_TRUE(std::isfinite(result->tau));
    EXPECT_FALSE(result->valid);
  }
}

TEST(DynamicTauPolicy, ReferenceFactorsAreAuditable) {
  const auto result = computeDynamicTau(2.0, 0.0, -0.5, 0.0, 0.5, params());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.f_r, 1.0);
  EXPECT_DOUBLE_EQ(result.f_v, 1.0);
  EXPECT_DOUBLE_EQ(result.f_T, 1.0);
  EXPECT_NEAR(result.T_i, 3.0, 1e-9);
  EXPECT_NEAR(result.tau, 0.9, 1e-9);
}
}  // namespace
```

- [ ] **Step 2: Connect the test to catkin.**

Add this exact test block to `planner/semantic_guard/CMakeLists.txt` after the two node targets:

```cmake
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(dynamic_tau_policy_test
    test/test_dynamic_tau_policy.cpp)
  if(TARGET dynamic_tau_policy_test)
    target_link_libraries(dynamic_tau_policy_test ${catkin_LIBRARIES})
  endif()
endif()
```

Add the following package dependency before `</package>` in `planner/semantic_guard/package.xml`:

```xml
<test_depend>rostest</test_depend>
```

- [ ] **Step 3: Add the Python source contract.**

Create `swarm_test/tests/test_dynamic_tau_contract.py`:

```python
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def read(relative):
    return (REPO_ROOT / relative).read_text(encoding="utf-8")


def test_shared_policy_is_the_numeric_source_of_truth():
    header = read("planner/semantic_guard/include/semantic_guard/dynamic_tau.hpp")
    assert "struct DynamicTauParams" in header
    assert "struct DynamicTauResult" in header
    assert "computeDynamicTau" in header
    assert "f_r" in header and "f_v" in header and "f_T" in header
    assert "max_tau" in header


def test_dynamic_methods_have_the_same_switch_and_beta_contract():
    runner = read("swarm_test/scripts/run_secbf_sim_experiments.py")
    for method in ("No_semantic", "Unguarded_SEESM", "SEESM_Ours"):
        assert method in runner
    assert '"dynamic_tau_enabled": True' in runner
    assert '"dynamic_tau_enabled": False' in runner
    assert "beta_applied_final" in runner
    assert "tau_valid" in runner
    assert "tau_reason" in runner


def test_standard_and_legacy_baselines_remain_separate():
    runner = read("swarm_test/scripts/run_secbf_sim_experiments.py")
    legacy = read("planner/mpc_dcbf/src/mpc_cbf.cpp")
    assert '"Standard_MPC_CBF"' in runner
    assert "cbf_metric" in runner
    assert "set_tau_value" in legacy
    assert "B1_ACBF_fixed" in runner
```

- [ ] **Step 4: Run the new tests and record the expected failure.**

Run:

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
python3 -m pytest -q swarm_test/tests/test_dynamic_tau_contract.py
catkin_make --pkg semantic_guard
```

Expected result: pytest fails because the shared header and new metadata fields are absent, and the C++ target fails to compile because `semantic_guard/dynamic_tau.hpp` is absent. Do not proceed until the test is demonstrably connected to the build.

- [ ] **Step 5: Commit the failing-test scaffold.**

```bash
git add planner/semantic_guard/test/test_dynamic_tau_policy.cpp \
  planner/semantic_guard/CMakeLists.txt planner/semantic_guard/package.xml \
  swarm_test/tests/test_dynamic_tau_contract.py
git commit -m "test: define dynamic tau contracts"
```

## Task 2: Implement the Shared Numeric Dynamic-Tau Policy

**Files:**
- Create: `planner/semantic_guard/include/semantic_guard/dynamic_tau.hpp`
- Test: `planner/semantic_guard/test/test_dynamic_tau_policy.cpp`

- [ ] **Step 1: Add the header-only policy with a stable result schema.**

Create the header with this complete API and implementation. `reason` is intentionally a stable string so CSV logs and post-processing can group failure causes without parsing free-form ROS messages.

```cpp
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace semantic_guard {

struct DynamicTauParams {
  double ke = 0.30;
  double t_max = 2.0;
  double min_speed = 1e-6;
  double min_distance = 1e-6;
  double max_tau = 2.0;
};

struct DynamicTauResult {
  double tau = 0.0;
  double T_i = 0.0;
  double f_r = 0.0;
  double f_v = 0.0;
  double f_T = 0.0;
  double cos_delta = 0.0;
  bool closing = false;
  bool valid = false;
  std::string reason = "invalid";
};

inline bool finite(double value) {
  return std::isfinite(value);
}

inline double positiveSign(double value) {
  return value > 0.0 ? 1.0 : 0.0;
}

inline DynamicTauResult computeDynamicTau(double lx, double ly,
                                          double vx, double vy,
                                          double inflated_radius,
                                          const DynamicTauParams& params) {
  DynamicTauResult result;
  const double values[] = {lx, ly, vx, vy, inflated_radius, params.ke,
                           params.t_max, params.min_speed,
                           params.min_distance, params.max_tau};
  for (double value : values) {
    if (!finite(value)) {
      result.reason = "non_finite_input";
      return result;
    }
  }

  const double distance = std::hypot(lx, ly);
  const double speed = std::hypot(vx, vy);
  if (distance <= params.min_distance) {
    result.reason = "distance_degenerate";
    return result;
  }
  if (speed <= params.min_speed) {
    result.reason = "speed_degenerate";
    return result;
  }

  const double dot = lx * vx + ly * vy;
  result.cos_delta = dot / (distance * speed);
  if (!finite(result.cos_delta)) {
    result.reason = "angle_invalid";
    return result;
  }

  // The workspace relative-vector convention is p_obstacle-p_robot and
  // v_obstacle-v_robot. A closing pair has cos_delta < 0.
  result.closing = result.cos_delta < 0.0;
  result.f_r = positiveSign(-result.cos_delta);

  const double cone_value =
      dot * dot + (inflated_radius * inflated_radius - distance * distance) *
                       speed * speed;
  result.f_v = positiveSign(cone_value);

  const double approach_cos = std::max(0.0, -result.cos_delta);
  const double clearance = distance - inflated_radius;
  result.T_i = clearance > 0.0 ? (clearance * approach_cos) / speed : 0.0;
  if (!finite(result.T_i) || result.T_i <= 0.0 || result.f_r == 0.0) {
    result.reason = "receding_or_nonclosing";
    return result;
  }

  result.f_T = positiveSign(params.t_max - result.T_i);
  if (result.f_v == 0.0) {
    result.reason = "velocity_gate";
    return result;
  }
  if (result.f_T == 0.0) {
    result.reason = "time_gate";
    return result;
  }

  const double raw_tau = result.f_r * result.f_v * result.f_T * params.ke * result.T_i;
  if (!finite(raw_tau) || raw_tau <= 0.0) {
    result.reason = "tau_invalid";
    return result;
  }

  result.tau = std::min(raw_tau, std::max(0.0, params.max_tau));
  result.valid = finite(result.tau);
  result.reason = result.valid ? "active" : "tau_invalid";
  if (!result.valid) result.tau = 0.0;
  return result;
}

}  // namespace semantic_guard
```

- [ ] **Step 2: Run the unit test and resolve only policy-level failures.**

Run:

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg semantic_guard
catkin_make --pkg semantic_guard --make-args run_tests_semantic_guard_gtest_dynamic_tau_policy_test
```

Expected result: the target builds and all six policy tests pass. If the reference-factor test fails, correct the test's hand calculation or the helper's factor conversion before integrating any ROS node; do not weaken the test.

- [ ] **Step 3: Commit the shared policy.**

```bash
git add planner/semantic_guard/include/semantic_guard/dynamic_tau.hpp
git commit -m "feat: add shared dynamic tau policy"
```

## Task 3: Integrate Guard and Ground-Truth Margin Paths

**Files:**
- Modify: `planner/semantic_guard/src/beta_guard_node.cpp`
- Modify: `planner/semantic_guard/src/beta_ground_truth_node.cpp`
- Modify: `planner/semantic_guard/launch/beta_guard.launch`
- Modify: `planner/semantic_guard/launch/beta_ground_truth.launch`
- Modify: `swarm_test/launch/secbf_planner.launch`
- Test: `planner/semantic_guard/test/test_dynamic_tau_policy.cpp`

- [ ] **Step 1: Add one common parameter set to both margin nodes.**

Include `semantic_guard/dynamic_tau.hpp` and replace the fixed-only parameter block with these parameters, while retaining `guard/tau` as the disabled-mode compatibility value:

```cpp
nh_.param("dynamic_tau_enabled", dynamic_tau_enabled_, false);
nh_.param("dynamic_tau/Ke", dynamic_tau_params_.ke, 0.30);
nh_.param("dynamic_tau/Tmax", dynamic_tau_params_.t_max, 2.0);
nh_.param("dynamic_tau/min_speed", dynamic_tau_params_.min_speed, 1e-6);
nh_.param("dynamic_tau/min_distance", dynamic_tau_params_.min_distance, 1e-6);
nh_.param("dynamic_tau/max_tau", dynamic_tau_params_.max_tau, 2.0);
nh_.param("guard/tau", tau_, 0.20);
```

Add these members to each node's private section:

```cpp
bool dynamic_tau_enabled_ = false;
semantic_guard::DynamicTauParams dynamic_tau_params_;
```

- [ ] **Step 2: Replace fixed look-ahead evaluation with the shared result.**

In both `beta_guard_node.cpp` and `beta_ground_truth_node.cpp`, use the obstacle and robot relative state already available in the callback. The exact replacement must preserve the existing beta selection, rate-limit, projection, fallback, and `beta_applied_final` publication logic:

```cpp
const Eigen::Vector2d rel_pos = obstacle.position - robot_position;
const Eigen::Vector2d rel_vel = obstacle.velocity - robot_velocity;
const double inflated_radius = obstacle.radius + robot_radius_;

semantic_guard::DynamicTauResult tau_result;
if (dynamic_tau_enabled_) {
  tau_result = semantic_guard::computeDynamicTau(
      rel_pos.x(), rel_pos.y(), rel_vel.x(), rel_vel.y(),
      inflated_radius, dynamic_tau_params_);
} else {
  tau_result.tau = tau_;
  tau_result.valid = true;
  tau_result.reason = "fixed_config";
}

const Eigen::Vector2d lookahead_rel_pos = rel_pos + tau_result.tau * rel_vel;
const double h_ee = lookahead_rel_pos.norm() - obstacle.radius - robot_radius_;
const double guard_upper_bound =
    std::min(beta_bar_val, std::max(0.0, h_ee - eta_));
```

Use `tau_result.tau` for the existing `h_see = h_ee - beta_final` calculation. Do not recompute a second tau later in the callback. For `semantic_mode == "none"`, continue to force the semantic beta to zero; dynamic tau remains available to the physical EESM check.

- [ ] **Step 3: Extend the Guard CSV without changing existing columns.**

Append these columns after the existing `R_sem` field in the CSV header in both nodes:

```text
,tau,T_i,f_r,f_v,f_T,tau_valid,tau_reason
```

Append the corresponding values on every row:

```cpp
<< "," << tau_result.tau
<< "," << tau_result.T_i
<< "," << tau_result.f_r
<< "," << tau_result.f_v
<< "," << tau_result.f_T
<< "," << (tau_result.valid ? 1 : 0)
<< "," << tau_result.reason
```

The ROS `GuardLog` message remains compatible; the audit CSV is the authoritative source for factor-level analysis.

- [ ] **Step 4: Expose the parameters in both launch files.**

Add these arguments to `beta_guard.launch` and `beta_ground_truth.launch`, then pass each argument into the node with the same parameter names:

```xml
<arg name="dynamic_tau_enabled" default="false"/>
<arg name="dynamic_tau_ke" default="0.30"/>
<arg name="dynamic_tau_tmax" default="2.0"/>
<arg name="dynamic_tau_min_speed" default="1e-6"/>
<arg name="dynamic_tau_min_distance" default="1e-6"/>
<arg name="dynamic_tau_max_tau" default="2.0"/>

<param name="dynamic_tau_enabled" value="$(arg dynamic_tau_enabled)"/>
<param name="dynamic_tau/Ke" value="$(arg dynamic_tau_ke)"/>
<param name="dynamic_tau/Tmax" value="$(arg dynamic_tau_tmax)"/>
<param name="dynamic_tau/min_speed" value="$(arg dynamic_tau_min_speed)"/>
<param name="dynamic_tau/min_distance" value="$(arg dynamic_tau_min_distance)"/>
<param name="dynamic_tau/max_tau" value="$(arg dynamic_tau_max_tau)"/>
```

Forward the same six arguments from `swarm_test/launch/secbf_planner.launch` into both included margin-node launch files. Keep defaults disabled so an old launch invocation remains behaviorally fixed until the runner explicitly enables the method.

- [ ] **Step 5: Build both margin nodes and commit.**

Run:

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg semantic_guard
```

Expected result: both `beta_guard_node` and `beta_ground_truth_node` compile, and no existing message-generation target regresses.

```bash
git add planner/semantic_guard/src/beta_guard_node.cpp \
  planner/semantic_guard/src/beta_ground_truth_node.cpp \
  planner/semantic_guard/launch/beta_guard.launch \
  planner/semantic_guard/launch/beta_ground_truth.launch \
  swarm_test/launch/secbf_planner.launch
git commit -m "feat: use dynamic tau in semantic margin nodes"
```

## Task 4: Integrate Global SEESM with Final Accepted Beta

**Files:**
- Modify: `planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp`
- Modify: `planner/vomp_planner/traj_planner/launch/plan_global_fsm.launch`
- Modify: `swarm_test/launch/secbf_planner.launch`
- Test: `swarm_test/tests/test_global_seesm_contract.py`

- [ ] **Step 1: Add the shared policy and global parameters.**

Include `semantic_guard/dynamic_tau.hpp` in `obs_manager.hpp`. Add these members near `tau_global_`, and load them from the existing `search/` namespace:

```cpp
bool dynamic_tau_enabled_ = false;
semantic_guard::DynamicTauParams dynamic_tau_params_;
```

Load them with:

```cpp
nh.param("search/dynamic_tau_enabled", dynamic_tau_enabled_, false);
nh.param("search/dynamic_tau/Ke", dynamic_tau_params_.ke, 0.30);
nh.param("search/dynamic_tau/Tmax", dynamic_tau_params_.t_max, 2.0);
nh.param("search/dynamic_tau/min_speed", dynamic_tau_params_.min_speed, 1e-6);
nh.param("search/dynamic_tau/min_distance", dynamic_tau_params_.min_distance, 1e-6);
nh.param("search/dynamic_tau/max_tau", dynamic_tau_params_.max_tau, 2.0);
```

- [ ] **Step 2: Compute global `h_SEE` from the same dynamic policy.**

In `is_SEESM_unsafe`, keep `beta_applied_final` as the only semantic-margin input. Replace the fixed `tau_global_` expression with:

```cpp
const Eigen::Vector2d p_rel = robot_state.head<2>() - obstacle_state.head<2>();
const Eigen::Vector2d v_rel = robot_state.tail<2>() - obstacle_state.tail<2>();
const double inflated_radius = obstacle_state(2) + robot_R;

semantic_guard::DynamicTauResult tau_result;
if (dynamic_tau_enabled_) {
  tau_result = semantic_guard::computeDynamicTau(
      p_rel.x(), p_rel.y(), v_rel.x(), v_rel.y(), inflated_radius,
      dynamic_tau_params_);
} else {
  tau_result.tau = tau_global_;
  tau_result.valid = true;
  tau_result.reason = "fixed_config";
}

const double h_ee = p_rel.norm() - obstacle_state(2) - robot_R;
const double h_see = (p_rel + tau_result.tau * v_rel).norm()
                   - obstacle_state(2) - robot_R - beta_applied;
```

Preserve the existing physical rejection, SEESM rejection, reason aggregation, timeout handling, and global replan counters. Log `tau_result` beside `h_ee` and `h_see`; do not substitute `beta_candidate`, `beta_requested`, or a stale beta cache for `beta_applied_final`.

- [ ] **Step 3: Extend the global CSV schema.**

Append the same seven fields to the global header and writer:

```text
,tau,T_i,f_r,f_v,f_T,tau_valid,tau_reason
```

The row writer must receive the `DynamicTauResult` from the same check that generated `h_see`, ensuring the audit row and decision cannot disagree.

- [ ] **Step 4: Expose the global launch arguments.**

Add these arguments and params to both copies of the global planner node in `plan_global_fsm.launch`:

```xml
<arg name="dynamic_tau_enabled" default="false"/>
<arg name="dynamic_tau_ke" default="0.30"/>
<arg name="dynamic_tau_tmax" default="2.0"/>
<arg name="dynamic_tau_min_speed" default="1e-6"/>
<arg name="dynamic_tau_min_distance" default="1e-6"/>
<arg name="dynamic_tau_max_tau" default="2.0"/>

<param name="search/dynamic_tau_enabled" value="$(arg dynamic_tau_enabled)"/>
<param name="search/dynamic_tau/Ke" value="$(arg dynamic_tau_ke)"/>
<param name="search/dynamic_tau/Tmax" value="$(arg dynamic_tau_tmax)"/>
<param name="search/dynamic_tau/min_speed" value="$(arg dynamic_tau_min_speed)"/>
<param name="search/dynamic_tau/min_distance" value="$(arg dynamic_tau_min_distance)"/>
<param name="search/dynamic_tau/max_tau" value="$(arg dynamic_tau_max_tau)"/>
```

Forward the arguments from `secbf_planner.launch` only when the runner selects a semantic method; Standard MPC-CBF keeps global SEESM disabled.

- [ ] **Step 5: Run the existing global contract test and commit.**

Run:

```bash
cd /home/lxr20/lxr/panjian_ws
python3 -m pytest -q swarm_test/tests/test_global_seesm_contract.py
```

Expected result: all assertions pass, including the final-beta topic, both global launch instances, and the unchanged alias mapping.

```bash
git add planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp \
  planner/vomp_planner/traj_planner/launch/plan_global_fsm.launch \
  swarm_test/launch/secbf_planner.launch \
  swarm_test/tests/test_global_seesm_contract.py
git commit -m "feat: apply dynamic tau to global seesm checks"
```

## Task 5: Put Dynamic Tau Inside the CasADi MPC Safety Kernel

**Files:**
- Modify: `planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h`
- Modify: `planner/mpc_secbf/src/mpc_secbf.cpp`
- Modify: `planner/mpc_secbf/src/mpc_secbf_node.cpp`
- Modify: `planner/mpc_secbf/launch/mpc_secbf.launch`
- Modify: `planner/mpc_secbf/CMakeLists.txt`
- Modify: `planner/mpc_secbf/package.xml`
- Test: `swarm_test/tests/test_standard_mpc_cbf_contract.py`

- [ ] **Step 1: Extend solver configuration without changing the solve API.**

Include the shared numeric header in `mpc_secbf.h`. Add the following optional arguments to `init_solver` after `cbf_metric`, preserving all existing call sites through defaults:

```cpp
void init_solver(double Ts, int N, double v_max, double v_min, double o_max,
                 std::vector<double> Q, std::vector<double> R,
                 double gamma, double beta_bar_unknown, double robot_radius,
                 double epsilon_max = 0.05, double slack_weight = 1000.0,
                 int max_cbf_obstacles = 6,
                 const std::string& cbf_metric = "seesm",
                 bool dynamic_tau_enabled = false,
                 semantic_guard::DynamicTauParams dynamic_tau_params = {});
```

Store `dynamic_tau_enabled_` and `dynamic_tau_params_` as private members. Keep the public `solve` signature and the obstacle matrix layout unchanged.

- [ ] **Step 2: Add a CasADi helper with the same gates as the numeric policy.**

Add this private declaration:

```cpp
casadi::MX dynamicTauCasadi(const casadi::MX& lx,
                            const casadi::MX& ly,
                            const casadi::MX& vx,
                            const casadi::MX& vy,
                            double inflated_radius) const;
```

Implement the helper in `mpc_secbf.cpp` with guarded denominators and `if_else` gates:

```cpp
casadi::MX MPC_SECBF_SOLVE::dynamicTauCasadi(
    const casadi::MX& lx, const casadi::MX& ly,
    const casadi::MX& vx, const casadi::MX& vy,
    double inflated_radius) const {
  const double eps = dynamic_tau_params_.min_distance;
  casadi::MX distance = casadi::MX::sqrt(lx * lx + ly * ly + eps * eps);
  casadi::MX speed = casadi::MX::sqrt(vx * vx + vy * vy +
                                      dynamic_tau_params_.min_speed *
                                      dynamic_tau_params_.min_speed);
  casadi::MX dot = lx * vx + ly * vy;
  casadi::MX cos_delta = dot / (distance * speed);
  casadi::MX closing = casadi::MX::if_else(cos_delta < 0.0, 1.0, 0.0);
  casadi::MX cone_value = dot * dot +
      (inflated_radius * inflated_radius - distance * distance) *
      speed * speed;
  casadi::MX velocity_gate = casadi::MX::if_else(cone_value > 0.0, 1.0, 0.0);
  casadi::MX clearance = casadi::MX::fmax(0.0, distance - inflated_radius);
  casadi::MX approach_cos = casadi::MX::fmax(0.0, -cos_delta);
  casadi::MX T_i = clearance * approach_cos / speed;
  casadi::MX time_gate = casadi::MX::if_else(
      T_i < dynamic_tau_params_.t_max, 1.0, 0.0);
  casadi::MX raw_tau = closing * velocity_gate * time_gate *
      dynamic_tau_params_.ke * T_i;
  return casadi::MX::fmin(dynamic_tau_params_.max_tau,
                          casadi::MX::fmax(0.0, raw_tau));
}
```

The expression above is the CasADi translation of the shared policy. Keep the same sign convention, gate inequalities, `Ke`, `Tmax`, and cap. The tiny denominator regularization is only for symbolic numerical stability; the numeric helper remains the audit reference.

- [ ] **Step 3: Replace the frozen-position `h_cbf` calculation for dynamic methods.**

Change `h_cbf` so the obstacle matrix columns `[x,y,radius,radius,theta,vx,vy]` supply both obstacle position and velocity, while `curpos(3:4)` supplies the MPC-predicted robot velocity:

```cpp
casadi::MX MPC_SECBF_SOLVE::h_cbf(casadi::MX& curpos,
                                  Eigen::VectorXd obs,
                                  double beta_i) {
  casadi::MX lx = obs(0) - curpos(0);
  casadi::MX ly = obs(1) - curpos(1);
  casadi::MX vx = obs(5) - curpos(3);
  casadi::MX vy = obs(6) - curpos(4);
  const double obs_radius = obs(2);

  casadi::MX tau = 0.0;
  if (dynamic_tau_enabled_) {
    tau = dynamicTauCasadi(lx, ly, vx, vy,
                           obs_radius + robot_radius_);
  }

  casadi::MX lookahead_x = lx + tau * vx;
  casadi::MX lookahead_y = ly + tau * vy;
  casadi::MX h_ee = casadi::MX::sqrt(
      lookahead_x * lookahead_x + lookahead_y * lookahead_y) -
      obs_radius - robot_radius_;
  return h_ee - beta_i;
}
```

For `dynamic_tau_enabled_ == false`, this preserves Standard MPC-CBF's current instantaneous distance behavior. For dynamic methods, do not use an external fixed-`tau` position as a substitute for the symbolic expression. Keep the existing `cbf_metric == "distance"` obstacle freezing only for the Standard baseline; the runner must pass `cbf_metric == "seesm"` to all three dynamic methods.

- [ ] **Step 4: Pass parameters from the node and launch file.**

Read these parameters in `mpc_secbf_node.cpp` and pass them to `init_solver`:

```cpp
bool dynamic_tau_enabled = false;
semantic_guard::DynamicTauParams dynamic_tau_params;
nh_.param("dynamic_tau_enabled", dynamic_tau_enabled, false);
nh_.param("dynamic_tau/Ke", dynamic_tau_params.ke, 0.30);
nh_.param("dynamic_tau/Tmax", dynamic_tau_params.t_max, 2.0);
nh_.param("dynamic_tau/min_speed", dynamic_tau_params.min_speed, 1e-6);
nh_.param("dynamic_tau/min_distance", dynamic_tau_params.min_distance, 1e-6);
nh_.param("dynamic_tau/max_tau", dynamic_tau_params.max_tau, 2.0);
```

Add the same XML arguments and params to `mpc_secbf.launch`:

```xml
<arg name="dynamic_tau_enabled" default="false"/>
<arg name="dynamic_tau_ke" default="0.30"/>
<arg name="dynamic_tau_tmax" default="2.0"/>
<arg name="dynamic_tau_min_speed" default="1e-6"/>
<arg name="dynamic_tau_min_distance" default="1e-6"/>
<arg name="dynamic_tau_max_tau" default="2.0"/>
<param name="dynamic_tau_enabled" value="$(arg dynamic_tau_enabled)"/>
<param name="dynamic_tau/Ke" value="$(arg dynamic_tau_ke)"/>
<param name="dynamic_tau/Tmax" value="$(arg dynamic_tau_tmax)"/>
<param name="dynamic_tau/min_speed" value="$(arg dynamic_tau_min_speed)"/>
<param name="dynamic_tau/min_distance" value="$(arg dynamic_tau_min_distance)"/>
<param name="dynamic_tau/max_tau" value="$(arg dynamic_tau_max_tau)"/>
```

Add `semantic_guard` to the `find_package(catkin REQUIRED COMPONENTS ...)`, `catkin_package`, and package dependencies in `mpc_secbf` if it is not already present. This makes the shared header dependency explicit rather than relying on an incidental include path.

- [ ] **Step 5: Add MPC audit fields without changing the solver result contract.**

At the MPC node's existing CSV writer, append `dynamic_tau_enabled`, `tau`, `T_i`, `f_r`, `f_v`, `f_T`, `tau_valid`, and `tau_reason`. Compute these fields from the first constrained obstacle at the current robot state with `computeDynamicTau`; if no obstacle is constrained, write `tau=0`, `tau_valid=0`, and `tau_reason="no_constrained_obstacle"`. The local MPC `beta` field must remain the value supplied to `solve`, and the Proposed method must continue to source that value from `beta_applied_final`.

- [ ] **Step 6: Build all MPC dependencies and commit.**

Run:

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg semantic_guard mpc_secbf traj_planner
python3 -m pytest -q swarm_test/tests/test_standard_mpc_cbf_contract.py
```

Expected result: all three packages build, Standard MPC-CBF still reports `semantic_mode=fixed`, `fixed_beta=0.4`, `cbf_metric=distance`, and the contract test confirms no hidden `R_safe` term was introduced.

```bash
git add planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h \
  planner/mpc_secbf/src/mpc_secbf.cpp planner/mpc_secbf/src/mpc_secbf_node.cpp \
  planner/mpc_secbf/launch/mpc_secbf.launch planner/mpc_secbf/CMakeLists.txt \
  planner/mpc_secbf/package.xml swarm_test/tests/test_standard_mpc_cbf_contract.py
git commit -m "feat: add dynamic tau to mpc secbf kernel"
```

## Task 6: Update Experiment Mapping, Metadata, and CSV Contracts

**Files:**
- Modify: `swarm_test/scripts/run_secbf_sim_experiments.py`
- Modify: `swarm_test/scripts/check_experiment_csv_fields.py`
- Modify: `swarm_test/tests/test_global_seesm_contract.py`
- Modify: `swarm_test/tests/test_standard_mpc_cbf_contract.py`
- Test: `swarm_test/tests/test_dynamic_tau_contract.py`

- [ ] **Step 1: Add one parameter block to the runner.**

Add these values to the default switch dictionary and expose them in the generated launch arguments:

```python
"dynamic_tau_enabled": False,
"dynamic_tau_ke": 0.30,
"dynamic_tau_tmax": 2.0,
"dynamic_tau_min_speed": 1e-6,
"dynamic_tau_min_distance": 1e-6,
"dynamic_tau_max_tau": 2.0,
```

Set method-specific switches with this exact mapping:

```python
if baseline_id in {"No_semantic", "Unguarded_SEESM", "SEESM_Ours"}:
    switches["dynamic_tau_enabled"] = True
    switches["cbf_metric"] = "seesm"
else:
    switches["dynamic_tau_enabled"] = False
    switches["cbf_metric"] = "distance"

switches["global_seesm_enable"] = baseline_id in {"Unguarded_SEESM", "SEESM_Ours"}
```

The EESM baseline uses dynamic tau with `beta=0`; it does not enable the semantic Guard or global beta rejection. Proposed MPC-SECBF uses only `beta_applied_final` for the local semantic margin and global check.

- [ ] **Step 2: Add auditable metadata.**

Extend each `trial_meta.json` with:

```json
{
  "dynamic_tau": {
    "enabled": true,
    "Ke": 0.30,
    "Tmax": 2.0,
    "min_speed": 1e-6,
    "min_distance": 1e-6,
    "max_tau": 2.0,
    "formula": "tau=f_r*f_v*f_T*Ke*T_i",
    "h_ee": "||l+tau*v||-R_obs-R_robot",
    "h_see": "h_ee-beta",
    "beta_source": "zero|candidate|beta_applied_final"
  }
}
```

Set `beta_source` to `zero` for EESM, `candidate` for Unguarded SEESM, and `beta_applied_final` for Proposed MPC-SECBF. Add the same fields to the human-readable trial README generated by the runner.

- [ ] **Step 3: Extend CSV required fields and summary aggregation.**

Add these required fields to `check_experiment_csv_fields.py` for margin and global logs:

```python
"tau", "T_i", "f_r", "f_v", "f_T", "tau_valid", "tau_reason"
```

Require all numeric tau fields to parse as finite values. Add summary keys `tau_mean`, `tau_max`, `tau_active_fraction`, and `tau_invalid_count`; group `tau_reason` values without failing on the known reasons `active`, `fixed_config`, `velocity_gate`, `time_gate`, `receding_or_nonclosing`, `speed_degenerate`, `distance_degenerate`, `non_finite_input`, `tau_invalid`, and `no_constrained_obstacle`.

- [ ] **Step 4: Add runner contract assertions.**

Extend the existing Python contracts with these assertions:

```python
assert runner_switches("No_semantic")["dynamic_tau_enabled"] is True
assert runner_switches("Unguarded_SEESM")["dynamic_tau_enabled"] is True
assert runner_switches("SEESM_Ours")["dynamic_tau_enabled"] is True
assert runner_switches("Standard_MPC_CBF")["dynamic_tau_enabled"] is False
assert runner_switches("No_semantic")["fixed_beta"] == 0.0
assert runner_switches("SEESM_Ours")["global_seesm_enable"] is True
assert "beta_applied_final" in proposed_metadata["dynamic_tau"]["beta_source"]
```

Use the runner's existing switch-construction helper rather than duplicating a second mapping in the test.

- [ ] **Step 5: Run contracts and commit.**

Run:

```bash
cd /home/lxr20/lxr/panjian_ws
python3 -m pytest -q \
  swarm_test/tests/test_dynamic_tau_contract.py \
  swarm_test/tests/test_global_seesm_contract.py \
  swarm_test/tests/test_standard_mpc_cbf_contract.py
```

Expected result: all mapping, topic, metadata, and CSV-schema assertions pass.

```bash
git add swarm_test/scripts/run_secbf_sim_experiments.py \
  swarm_test/scripts/check_experiment_csv_fields.py \
  swarm_test/tests/test_dynamic_tau_contract.py \
  swarm_test/tests/test_global_seesm_contract.py \
  swarm_test/tests/test_standard_mpc_cbf_contract.py
git commit -m "test: audit dynamic tau experiment mapping"
```

## Task 7: Build, Cross-Check Numeric and CasADi Paths, and Run One Smoke Trial

**Files:**
- Modify: `swarm_test/scripts/check_experiment_csv_fields.py` if validation finds a schema mismatch
- Create: `swarm_test/tests/test_dynamic_tau_casadi_reference.py` if a separate numeric/CasADi comparison is needed
- Create: `outputs/20260713_r13_dynamic_tau_smoke/README.md` through the runner, not by hand

- [ ] **Step 1: Run the complete build and unit suite.**

Run:

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg semantic_guard mpc_secbf traj_planner
python3 -m pytest -q swarm_test/tests
```

Expected result: the dynamic-tau unit test, source contracts, and existing scenario/runner contracts pass. A failure in the legacy ACBF or Standard baseline is a regression and must be resolved before the smoke run.

- [ ] **Step 2: Cross-check a representative closing state.**

Use the shared helper's reference case `(lx,ly,vx,vy,R)=(2.0,0.0,-0.5,0.0,0.5)` and compare the MPC symbolic expression at the same state. Assert that both paths produce `tau=0.9` within `1e-6`, then compare `h_EE` and `h_SEE` for `beta=0` and `beta=0.4`. The test must also include a receding state and assert that both paths produce `tau=0`.

If the existing CasADi test harness cannot evaluate a private solver helper directly, expose a small test-only `casadi::Function` factory under `#ifdef SEMANTIC_GUARD_TESTING` rather than duplicating the formula in Python. The numeric helper remains the only scalar implementation used by runtime logging.

- [ ] **Step 3: Run the common one-seed smoke experiment.**

Run from the built workspace with the existing external roscore convention:

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
export ROS_HOME=/tmp/ros_dynamic_tau_smoke
python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario head_on_context_bl \
  --baseline Standard_MPC_CBF,EESM_MPC_ECBF,SEESM_Without_FPU,Proposed_MPC_SECBF \
  --seed-manifest swarm_test/config/seed_manifests/20260712_nine_condition_pilot5.csv \
  --duration-sec 30 --roscore external \
  --output-root swarm_test/output/secbf_runs/20260713_dynamic_tau_smoke
```

Expected result: four trial directories complete with `trial_meta.json`, margin/global/MPC CSVs, summary JSON, and no orphaned ROS processes. Do not start the 5-seed pilot or 30-seed formal benchmark at this stage.

- [ ] **Step 4: Verify the smoke artifact contract.**

Run:

```bash
cd /home/lxr20/lxr/panjian_ws
python3 swarm_test/scripts/check_experiment_csv_fields.py \
  swarm_test/output/secbf_runs/20260713_dynamic_tau_smoke
rg -n '"dynamic_tau"|"beta_source"|"tau_active_fraction"' \
  swarm_test/output/secbf_runs/20260713_dynamic_tau_smoke
```

Check manually that:

- all three dynamic methods have finite `tau` fields;
- their identical robot/obstacle states produce identical `tau`, `T_i`, `f_r`, `f_v`, and `f_T` streams;
- EESM has `beta=0`, Unguarded SEESM records candidate beta, and Proposed records accepted beta;
- Standard MPC-CBF has `dynamic_tau_enabled=false` and no dynamic look-ahead;
- Proposed global logs consume `beta_applied_final` and never `beta_requested`;
- collision, solver failure, empty-obstacle fallback, and no-CBF cycles remain explicitly counted.

- [ ] **Step 5: Commit validation-only changes and write the handoff note.**

If the smoke validation required a schema or test correction, commit it separately:

```bash
git add swarm_test/scripts/check_experiment_csv_fields.py swarm_test/tests
git commit -m "test: validate dynamic tau smoke artifacts"
```

The next experiment action after this plan is a user-visible review of the four-method, one-seed artifacts. Only after that review should the implementation be promoted to the 5-seed pilot.

## Completion Checklist

- [ ] `semantic_guard` numeric tests pass, including degenerate and non-finite inputs.
- [ ] Guard, ground-truth margin, global SEESM, and CasADi MPC use the same dynamic-`tau` definition.
- [ ] EESM, Unguarded SEESM, and Proposed MPC-SECBF share identical tau factors for identical states.
- [ ] Proposed MPC-SECBF uses `beta_applied_final` at both local and global safety checks.
- [ ] Standard MPC-CBF remains fixed-distance and the legacy ACBF path remains untouched.
- [ ] CSV and JSON artifacts expose `tau`, `T_i`, `f_r`, `f_v`, `f_T`, validity, reason, and beta source.
- [ ] All builds, contracts, cross-checks, and the common one-seed smoke trial pass.
- [ ] No 5-seed or 30-seed benchmark is started before the smoke artifacts are reviewed.

## Self-Review Before Execution

- [ ] Compare each task against `docs/superpowers/specs/2026-07-13-dynamic-tau-seesm-design.md`, especially the method table, beta semantics, final-beta contract, configuration fields, and staged validation.
- [ ] Run a repository-wide scan for prohibited planning tokens and remove every match from this plan.
- [ ] Confirm the names `DynamicTauParams`, `DynamicTauResult`, `computeDynamicTau`, `dynamicTauCasadi`, `dynamic_tau_enabled`, `tau_valid`, and `tau_reason` are consistent across all tasks.
- [ ] Run `git diff --check` before committing the plan.
