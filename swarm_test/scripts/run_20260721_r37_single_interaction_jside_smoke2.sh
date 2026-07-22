#!/usr/bin/env bash
set -eo pipefail
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTPUT_ROOT="${OUTPUT_ROOT:-$WORKSPACE/swarm_test/output/secbf_runs/20260721_r37_single_interaction_jside_smoke2}"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260719_context_int_ablation_pilot2.csv"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
set -u
python3 -u "$WORKSPACE/swarm_test/scripts/run_secbf_sim_experiments.py" \
  --scenario head_on_context_int,crossing_context_int,local_crowding_context_int \
  --baseline No_J_side,SideWeight_005 --duration-sec 30 \
  --seed-manifest "$MANIFEST" --output-root "$OUTPUT_ROOT" \
  --roscore external --skip-existing-complete
mapfile -d '' RUN_DIRS < <(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -type d ! -name '.*' -print0 | sort -z)
[[ "${#RUN_DIRS[@]}" -eq 12 ]] || { echo "expected 12 runs, found ${#RUN_DIRS[@]}" >&2; exit 1; }
for run_dir in "${RUN_DIRS[@]}"; do python3 "$WORKSPACE/swarm_test/scripts/check_experiment_csv_fields.py" "$run_dir" >/dev/null; done
OUTPUT_ROOT="$OUTPUT_ROOT" python3 - <<'PY'
from collections import defaultdict
import csv, hashlib, os
from pathlib import Path
import yaml
root=Path(os.environ['OUTPUT_ROOT']); pairs=defaultdict(list); expected={'No_J_side','SideWeight_005'}
for d in sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith('.')):
    meta=yaml.safe_load((d/'meta.yaml').read_text()); row=next(csv.DictReader((d/'summary.csv').open()))
    if row.get('termination_reason')=='invalid' or int(float(row.get('robot_records') or 0))<=0: raise SystemExit(f'invalid: {d}')
    key=(meta['scenario'],meta['trial_manifest']['trial_id']); digest=hashlib.sha256((d/'obstacles_param.yaml').read_bytes()).hexdigest()
    pairs[key].append((meta['baseline_id'],digest))
if len(pairs)!=6: raise SystemExit(f'expected 6 pairs, found {len(pairs)}')
for key,values in pairs.items():
    if {v[0] for v in values}!=expected or len({v[1] for v in values})!=1: raise SystemExit(f'pair audit failed: {key}')
print('Single-interaction J_side smoke audit passed: 12/12 valid and 6/6 paired hashes.')
PY
python3 "$WORKSPACE/swarm_test/scripts/analyze_jside_weight_pilot.py" --input-root "$OUTPUT_ROOT" --output "$OUTPUT_ROOT/jside_weight_summary.csv"
echo 'Single-interaction J_side smoke audit passed: 12/12 valid and 6/6 paired hashes.' > "$OUTPUT_ROOT/SINGLE_INTERACTION_JSIDE_AUDIT_PASS.txt"
