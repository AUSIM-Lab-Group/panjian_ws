#!/usr/bin/env bash
set -eo pipefail
W="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PILOT="$W/swarm_test/output/secbf_runs/20260721_r40_runtime_stability_pilot2"
while systemctl --user is-active --quiet r40-runtime-stability-pilot-retry.service; do sleep 10; done
test -f "$PILOT/RUNTIME_STABILITY_PILOT_AUDIT_PASS.txt" || { echo "r40 audit missing; serial chain stopped" >&2; exit 1; }
python3 "$W/swarm_test/scripts/analyze_runtime_stability_thresholds.py" --input-root "$PILOT" --output "$PILOT/runtime_stability_thresholds.csv"
/bin/bash "$W/swarm_test/scripts/run_20260721_r41_runtime_stability_formal30.sh"
test -f "$W/swarm_test/output/secbf_runs/20260721_r41_runtime_stability_formal30/RUNTIME_STABILITY_FORMAL_AUDIT_PASS.txt"
/bin/bash "$W/swarm_test/scripts/run_20260721_r42_context_int_ablation_formal30.sh"
test -f "$W/swarm_test/output/secbf_runs/20260721_r42_context_int_ablation_formal30/ABLATION_FORMAL_AUDIT_PASS.txt"
echo "r40-r42 serial chain completed successfully."
