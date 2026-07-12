# Standard MPC-CBF Baseline Design

## Goal

Add a stable Standard MPC-CBF baseline to the existing MPC-SECBF experiment
stack. It must use the same robot model, MPC objective, solver, obstacle
ordering, recorder, and seed materialization as the other methods while
removing dynamic EESM and semantic-margin behavior.

## Scope

The new runner baseline is `Standard_MPC_CBF` and is implemented entirely in
the current `secbf_planner.launch` pipeline. The legacy `mpc_dcbf` controller
is not launched by this baseline.

The baseline uses the fixed safety function

```text
h_CBF = ||p_obs(t) - p_robot(t+k)|| - R_obs - R_robot - 0.4
```

For every local MPC prediction step `k`, `p_obs(t)` is the first predicted
obstacle position received in the current control cycle. The controller does
not consume later obstacle predictions for this barrier. This removes the
future-motion component used by EESM while retaining the same discrete soft
CBF constraint, horizon, cost, and solver as the other methods.

## Runtime Configuration

`mpc_secbf` receives a `cbf_metric` parameter with two values:

- `seesm`: existing behavior; each horizon step uses its corresponding
  predicted obstacle position and per-obstacle beta.
- `distance`: Standard MPC-CBF behavior; each horizon step uses the current
  obstacle position and the fixed beta value.

The baseline receives a fixed beta vector of `0.4 m` for every obstacle. Its
semantic producer is configured with `semantic_mode=fixed`,
`fixed_beta=0.4`, and all Guard mechanisms disabled. The Guard log remains
enabled so beta provenance remains auditable.

For `Standard_MPC_CBF`, global SEESM and ADSM dynamic global checks are both
disabled. The global layer therefore supplies the static reference path and
does not add a second dynamic-risk mechanism to the standard local-CBF
baseline.

## Runner Mapping And Evidence

The paper alias `Standard_MPC_CBF` resolves to a new in-stack baseline ID,
`Standard_MPC_CBF`, rather than the unstable external `B1_ACBF_fixed` chain.
Run metadata records the resolved ID, `cbf_metric=distance`, fixed beta, and
the disabled global/Guard switches.

Existing `No_semantic`, `Unguarded_SEESM`, and `SEESM_Ours` mappings are
unchanged. Existing EESM/SEESM launch defaults remain `cbf_metric=seesm`.

## Failure Handling

The existing MPC feasibility fallback remains available to preserve the common
logging contract. A run that uses it must report the existing
`used_fallback`/`mpc_feasibility_guard_used` fields. It does not silently
switch to an EESM constraint.

## Validation

1. Contract tests prove that `Standard_MPC_CBF` resolves to the new in-stack
   baseline and emits `cbf_metric=distance` with fixed `0.4 m` beta.
2. Contract tests prove the distance path uses the first obstacle position for
   all horizon constraints, while the SEESM path remains unchanged.
3. Build `mpc_secbf` and execute one Head-on-Int smoke trial using the normal
   recorder.
4. The smoke output must contain non-empty robot, obstacle, margin/Guard,
   planner, timing, and event logs. Metadata must show the fixed-distance
   configuration and `global_seesm_log.csv` must be absent or empty because
   global SEESM is disabled.

## Non-Goals

- Reproducing or repairing the legacy `mpc_dcbf` controller.
- Claiming strict source-code reproduction of any external MPC-CBF paper.
- Changing scenario geometry, seeds, common MPC cost, or the current
  EESM/SEESM controller behavior.
