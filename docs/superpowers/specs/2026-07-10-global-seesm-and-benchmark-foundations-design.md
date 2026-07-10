# Global SEESM And Benchmark Foundations Design

**Date:** 2026-07-10  
**Status:** Approved for specification  
**Scope:** First implementation subproject of the paper-experiment roadmap

## Goal

Make the accepted semantic margin a single, auditable contract for both the
global dynamic search and the local MPC-SECBF controller. In the same vertical
slice, make the simulation runner reproduce paired seed-based trials and map
paper method names to exact baseline configurations. The first acceptance
output is a visual closed loop for Head-on-Int, Crossing-Int, and
Local-Crowding-Int.

This subproject deliberately stops before Exp2/Exp3 batch validation, the
nine-condition pilot, the 30-seed formal benchmark, dense-corridor stress
testing, and real-robot trials. Those phases consume the foundations produced
here and must not begin before their gates pass.

## Current Facts

- The global planner is `globalFsm_by_adsm`. `ThetaAstar` currently checks
  dynamic obstacles through `Obs_Manager::is_VO_unsafe`, `is_collide`, or
  `is_Adsm_unsafe`; none receives a semantic margin.
- `beta_ground_truth_node` computes and publishes the Guard-projected value
  on `/safety_margin/beta`, but the message is positional and does not carry
  obstacle IDs.
- `mpc_secbf_node` can make a later feasibility fallback from the candidate
  beta list to the previous accepted list, zero beta, or no-CBF fallback. The
  global planner must not use a beta that differs from this final local
  decision.
- The runner currently writes `random_seed: 1` in every run and `--repeat`
  only repeats directory creation. Such repeats are not independent trials.

## Decisions

1. **One margin source of truth:** the global search uses only the final beta
   accepted by MPC after its feasibility fallback. This is called
   `beta_applied` in all paper-facing logs and figures.
2. **Stable obstacle identity:** every margin message carries the dynamic
   simulator obstacle ID. Position in an array is never used as identity.
3. **Matching safety model:** global dynamic rejection evaluates the same
   velocity-aware SEESM geometry as local control:

   ```text
   h_global_SEESM = ||p_obs - p_robot + tau_global (v_obs - v_robot)||
                    - R_obs - R_robot - beta_applied
   ```

   A sampled motion primitive is rejected when `h_global_SEESM <= 0` for any
   dynamic obstacle at the sampled prediction time. `tau_global` defaults to
   the existing Guard time constant of `0.20 s` and is written to metadata.
4. **Strict compatibility switch:** `global_seesm_enable=false` preserves the
   existing global dynamic-check behavior byte-for-byte. It is the
   global/local ablation switch.
5. **No hidden semantic fallback:** a missing, stale, or MPC no-CBF margin
   record uses `beta_applied=0` for global checking and emits a log reason.
   The physical dynamic check remains active; the run is not silently treated
   as semantic-safe.
6. **Formal repeats are seed trials:** a pre-generated seed manifest defines
   all scenario perturbations. Every compared method receives the same
   `scenario_id`, `trial_id`, `seed`, and effective obstacle script.

## Architecture

### 1. Accepted-margin ROS interface

Add a generated `semantic_guard/AppliedMarginArray` message with:

```text
std_msgs/Header header
uint32[] obstacle_ids
float64[] beta_applied
string[] accepted_sources
```

`accepted_sources` uses only `candidate`, `previous`, `zero`, and `no_cbf`.
All three arrays must have identical length; malformed messages are rejected
and counted in the global log.

The data flow is:

```text
Obs_Manager ordered predicted obstacle IDs
  -> beta_ground_truth_node candidate/Guard beta
  -> mpc_secbf_node final feasibility decision
  -> /safety_margin/beta_applied_final (AppliedMarginArray)
  -> globalFsm_by_adsm / Obs_Manager
  -> ThetaAstar global SEESM primitive rejection
```

The current `/safety_margin/beta` `Float32MultiArray` remains available only
as the candidate input to MPC. It is not a global-planning input after this
change.

`Obs_Manager` also publishes the obstacle IDs in the exact order used for the
existing flat prediction matrix. `beta_ground_truth_node` and
`mpc_secbf_node` consume that order to create ID-bearing messages. The
simulator's dynamic IDs are therefore preserved through the whole pipeline.

### 2. Global semantic dynamic check

`Obs_Manager` owns a timestamped `obstacle_id -> AppliedMargin` cache. Each
entry contains beta, source, receipt time, and message stamp. It exposes a
new `is_SEESM_unsafe(robot_state, robot_radius, prediction_time)` method that
iterates dynamic obstacle trajectories by their true IDs and evaluates
`h_global_SEESM` above.

`ThetaAstar` calls this method only when `search/global_seesm_enable=true`.
The check is applied both while expanding primitives and while validating the
shot trajectory. Its existing ESDF and selected VO/DIS/ADSM checks remain in
place; SEESM adds a semantic rejection and never removes a physical rejection.

The global planner subscribes to accepted margins before its first replan. If
there is no fresh accepted value for an obstacle, it uses beta zero for that
obstacle and writes `missing` or `stale` to the log. A margin is stale after
`0.50 s`.

### 3. Global evidence log

Each global replan appends `global_seesm_log.csv` with:

```text
t,replan_id,global_seesm_enable,obs_id,beta_applied,accepted_source,
margin_age_ms,h_ee,h_see,primitive_rejected,shot_rejected,reason,
global_replan_ms
```

The runner requires this file only for global-SEESM-enabled runs. It adds the
file path and switch values to `run_meta.json`, and the postprocessor includes
global margin, rejection, and replan-time summaries in the trial table.

### 4. Exact baseline registry

The runner exposes paper aliases and writes their resolved switches to every
`run_meta.json`:

| Paper label | Runner baseline | Exact behavior |
|---|---|---|
| Standard MPC-CBF | `B1_ACBF_fixed` | Legacy fixed-distance ACBF controller; reported as an external-controller baseline |
| EESM-MPC-ECBF | `No_semantic` | Same MPC-SECBF stack, `semantic_mode=none`, therefore `beta_applied=0` |
| SEESM w/o Feasibility-Preserving Update | `Unguarded_SEESM` | Full candidate margin with rate limit, projection, Guard fallback, and MPC feasibility guard disabled |
| Proposed MPC-SECBF | `SEESM_Ours` | Full semantic margin, Guard projection/rate limit/fallback, and MPC feasibility guard enabled |
| No semantic margin | `No_semantic` | Ablation alias of EESM-MPC-ECBF |
| Category only | `Category_only` | Category prior active; heading, TTC, and density weights zero |
| Context only | `Context_only` | Context modulation active; category prior is not used as the category-dependent multiplier |
| No rate limit | `No_rate_limit` | Positive-increment limiter disabled |
| No projection | `No_projection` | Available-margin projection disabled |
| No MPC feasibility guard | `No_mpc_guard` | MPC candidate-to-previous/zero feasibility fallback disabled |
| Unguarded SEESM | `Unguarded_SEESM` | All feasibility-preserving update mechanisms disabled |

`J_side` is absent from the current MPC cost. The paper must not claim a
`full w/o J_side` result until a separately specified soft side-passing cost
and its `No_J_side` baseline exist. It is not part of this subproject.

### 5. Auditable seed trials

Replace formal use of `--repeat` with a seed manifest. A manifest row has:

```text
trial_id,seed,scenario_id,obstacle_id,start_x_offset_m,start_y_offset_m,
speed_scale,start_delay_offset_s
```

The runner accepts `--seed-manifest`. For every row it materializes the
effective obstacle YAML under the run directory, passes it to the simulator,
and writes the full row to `run_meta.json` and `summary.csv`. The same row is
reused across every baseline for paired comparison.

Seed-generated perturbations are bounded and deterministic:

```text
start_x_offset_m in [-0.15, 0.15]
start_y_offset_m in [-0.15, 0.15]
speed_scale in [0.95, 1.05]
start_delay_offset_s in [-0.20, 0.20]
```

An invalid perturbation that starts in collision or exits the map is rejected
when the manifest is generated, not during a method run. Seed manifests are
immutable once a pilot starts.

## First Acceptance Stage: Three Visual Closed Loops

The first live deliverable uses one fixed manifest row for each condition:

```text
head_on_context_int
crossing_context_int
local_crowding_context_int
```

Each condition runs three methods:

```text
No_semantic (EESM-MPC-ECBF)
Unguarded_SEESM
SEESM_Ours
```

The stage produces, for every condition:

```text
<scenario>_methods_comparison.png
<scenario>_methods_comparison.gif
<scenario>_beta_h_curves.png
<scenario>_solver_guard_curves.png
<scenario>_global_seesm_curves.png
```

The last figure shows final accepted beta, global `h_see`, semantic primitive
rejections, and global replan time. The overview figure uses the existing
DR-MPC-inspired visual style. Failed trials remain visible and receive a
collision/deadlock annotation.

## Tests And Acceptance Criteria

### Automated tests

1. `AppliedMarginArray` validation rejects mismatched arrays, duplicate IDs,
   non-finite beta values, and negative beta values.
2. The final MPC source message maps `candidate`, `previous`, `zero`, and
   `no_cbf` to the exact beta list used by the solver.
3. A global primitive that is physically clear but violates
   `h_global_SEESM` is rejected only with `global_seesm_enable=true`.
4. With the switch disabled, the global collision result matches the current
   DIS/ADSM result.
5. A seed manifest produces identical effective YAML and metadata for every
   method of one `trial_id`, while distinct seed rows differ only within the
   declared bounds.
6. Paper aliases resolve to the table above, and `run_meta.json` captures all
   resolved switches.

### ROS smoke tests

For each of the three visual conditions, run all three methods with one fixed
seed and external `roscore`. A valid run requires non-empty robot, obstacle,
Guard, planner, timing, event, and summary logs; global-SEESM-enabled runs
also require non-empty `global_seesm_log.csv`.

The stage passes only when the global log proves that the beta value and source
used in global rejection equal the final accepted MPC value for matching
obstacle ID and timestamp. It does not require every method to succeed; all
failures must be classified as collision, goal timeout, global no-path, MPC
infeasibility, or infrastructure invalidity.

## Deferred Stages And Gates

| Stage | Starts only after | Output |
|---|---|---|
| Exp2 + Exp3 component validation | Three visual closed loops pass | Category/context matrix and beta/h curves |
| Nine-condition pilot | Exp2 + Exp3 evidence is valid | 9 conditions x 4 methods x 5 paired seeds |
| Formal benchmark | Pilot scenario difficulty and failure taxonomy are frozen | 9 conditions x 4 methods x 30 paired seeds |
| Dense-corridor stress | Formal baseline registry is verified | Guard-focused stress evidence |
| Real-world verification | Simulation main table and runtime gate pass | 3 scenarios x 2 methods x 5 trials |

No formal batch may include a new scenario, baseline definition, seed range,
or success threshold after its pilot begins.

## Non-Goals

- Do not claim global SEESM in the paper before this global/local consistency
  check passes.
- Do not run the 9-condition pilot or 30-seed benchmark in this subproject.
- Do not add DR-MPC reinforcement learning, AVOCADO cooperation models,
  semantic-recognition accuracy experiments, DWA/TEB/ORCA baselines, or
  real-robot trials.
