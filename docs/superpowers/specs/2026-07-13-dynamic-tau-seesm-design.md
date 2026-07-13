# Dynamic Tau for EESM and SEESM Design

## 1. Goal

Align the local MPC safety kernel used by EESM-MPC-ECBF, Unguarded SEESM,
and Proposed MPC-SECBF with the velocity-aware EESM definition in the
reference paper. Standard MPC-CBF and the legacy ACBF branch remain separate
baselines.

The three dynamic methods must share the same dynamic forward-integration
time `tau_i(k)`. Their only safety-kernel difference is the semantic margin:

- EESM-MPC-ECBF: `beta_i = 0`.
- Unguarded SEESM: the candidate semantic margin is used directly.
- Proposed MPC-SECBF: the Guard/MPC-accepted margin is used.

## 2. Reference Semantics

The paper defines

```text
h_EE(X_k) = ||l_i(k) + tau_i(k) v_i(k)||_2 - R_obs - R_robot
h_SEE(X_k) = h_EE(X_k) - beta_i(k)
tau_i = f_r f_v f_T K_e T_i
```

The paper's fixed `R_safe` is not added as a second hidden term in this
workspace. When a fixed-margin baseline needs it, the value is represented
explicitly as a fixed `beta`; this preserves the current code-standard
meaning of `beta=0` for the no-semantic EESM baseline and `beta=0.4` for the
fixed-distance baseline.

where `l_i` and `v_i` are relative position and relative velocity, `T_i`
is the time-to-collision-boundary term, and `f_r`, `f_v`, and `f_T` gate
convergent motion, the velocity-obstacle cone, and the configured time
horizon. The implementation will preserve the paper's vector convention by
using the equivalent workspace convention

```text
l_code = p_obstacle - p_robot
v_code = v_obstacle - v_robot
```

Both vectors are sign-reversed together, so the EE norm and dot-product
tests are unchanged. Closing and non-finite cases must produce a finite,
non-negative `tau`; receding or non-threatening cases produce `tau = 0`.
The exact paper factors and their configured bounds are recorded in metadata
and tested against hand-computed reference cases.

## 3. Architecture

### 3.1 Shared numeric policy

Add a header-only dynamic-tau policy under `semantic_guard/include/`.
It exposes a small scalar/numeric API independent of CasADi and returns:

```text
tau, T_i, f_r, f_v, f_T, closing, valid, reason
```

The policy owns `K_e`, `T_max`, finite-value handling, and the non-negative
output rule. Guard and `Obs_Manager` use this API for their numeric checks.

### 3.2 Symbolic MPC implementation

`mpc_secbf` implements the same lookahead formula for each obstacle and
prediction stage. At every receding-horizon solve, the shared numeric policy
is evaluated from the measured robot state and that stage's obstacle
prediction. The resulting `stage_tau` is then frozen as a CasADi constant while
building that stage's `h_cbf()` constraint; the next solve recomputes it.
This solver-time freezing avoids putting the non-smooth `tau` gates inside the
NLP while preserving the dynamic, auditable policy. `dynamicTauCasadi()` is
retained only as an algebraic reference helper and is not used by production
solves.

The existing discrete CBF constraint remains:

```text
-h_{k+1} + (1 - gamma) h_k <= epsilon_k
```

The existing obstacle selection limit, slack variables, control bounds, and
MPC objective are unchanged in this change.

### 3.3 Guard and global check

Guard and global SEESM use the same numeric tau policy and the same physical
radius convention. The global check continues to consume only
`beta_applied_final`; it must not consume candidate or requested beta.

## 4. Method Mapping

The experiment runner keeps the existing aliases and changes only the local
safety-kernel input:

| Method | Dynamic tau | Semantic margin | Guard |
|---|---:|---:|---:|
| Standard MPC-CBF | no | fixed distance margin | no |
| EESM-MPC-ECBF | yes | zero | no semantic effect |
| Unguarded SEESM | yes | candidate beta | disabled |
| Proposed MPC-SECBF | yes | accepted beta | enabled |

The legacy `B1_ACBF_fixed` implementation in `mpc_dcbf` is not migrated in
this change. Its behavior remains available for historical comparison.

## 5. Configuration and Logging

Expose the dynamic-tau parameters through the existing launch/config path:

- `Ke` / relaxation factor;
- `Tmax` / time-horizon gate;
- minimum numerical speed and distance tolerances;
- a finite upper bound used only as an implementation guard;
- an explicit `dynamic_tau_enabled` switch for the three dynamic methods.

Each Guard and planner record adds or preserves:

```text
tau, T_i, f_r, f_v, f_T, tau_valid, tau_reason
```

The trial metadata records the effective values and method mapping. Existing
`beta_requested`, `beta_applied`, `h_ee`, `h_see`, solver status, slack, and
fallback fields remain unchanged.

## 6. Validation

Validation is staged:

1. Build-level check for all three dependent packages.
2. Unit tests for: stationary obstacle, receding motion, closing motion,
   velocity-obstacle boundary, `T_i >= T_max`, zero-speed, near-zero distance,
   and non-finite input.
3. A numerical cross-check comparing the shared numeric policy with the
   CasADi expression at representative states.
4. One common `head_on_context_bl` seed for all four formal methods.
5. Confirm that the three dynamic methods report identical tau factors for
   identical robot/obstacle states, while their beta streams differ as
   intended.
6. Confirm no missing logs, no NaN/Inf values, and explicit accounting of
   infeasible or no-CBF fallback cycles.

Success means the formula path is auditable, the dynamic methods share the
same tau kernel, and the Standard and legacy ACBF baselines remain unchanged.
