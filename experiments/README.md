# SEESM Experiment Workspace

This directory stores reproducible experiment plans and run artifacts for the
semantic safety margin paper.

Each experiment keeps a `meta.yaml` template. For an actual run, copy the
template into the run directory and fill in `run_id`, `timestamp`, exact command,
git commit, and output file paths.

Core safety-function contract:

```text
h_SEE = ||p_rel + tau v_rel|| - R_obs - R_robot - beta_i
```

No extra fixed `R_safe` term should be added. A fixed-margin baseline is encoded
as `beta_i = d_safe`.
