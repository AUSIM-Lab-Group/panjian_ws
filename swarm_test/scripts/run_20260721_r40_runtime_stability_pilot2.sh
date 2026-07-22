#!/usr/bin/env bash
set -eo pipefail
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTPUT_ROOT="$WORKSPACE/swarm_test/output/secbf_runs/20260721_r40_runtime_stability_pilot2"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260721_runtime_stability_pilot2.csv"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
set -u
python3 -u "$WORKSPACE/swarm_test/scripts/run_secbf_sim_experiments.py" \
  --scenario runtime_scaling_n1,runtime_scaling_n2,runtime_scaling_n4,runtime_scaling_n6 \
  --baseline No_semantic,SEESM_Ours --duration-sec 30 \
  --seed-manifest "$MANIFEST" --output-root "$OUTPUT_ROOT" \
  --roscore external --skip-existing-complete
mapfile -d '' RUN_DIRS < <(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -type d ! -name '.*' -print0 | sort -z)
[[ "${#RUN_DIRS[@]}" -eq 16 ]] || { echo "expected 16 runs, found ${#RUN_DIRS[@]}" >&2; exit 1; }
for d in "${RUN_DIRS[@]}"; do python3 "$WORKSPACE/swarm_test/scripts/check_experiment_csv_fields.py" "$d" >/dev/null; done
OUTPUT_ROOT="$OUTPUT_ROOT" python3 - <<'PY'
from collections import defaultdict
import csv, hashlib, os
from pathlib import Path
import yaml
root=Path(os.environ['OUTPUT_ROOT']); pairs=defaultdict(list); expected={'No_semantic','SEESM_Ours'}
for d in sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith('.')):
 meta=yaml.safe_load((d/'meta.yaml').read_text()); row=next(csv.DictReader((d/'summary.csv').open()))
 if row.get('termination_reason')=='invalid' or int(float(row.get('robot_records') or 0))<=0: raise SystemExit(f'invalid: {d}')
 key=(meta['scenario'],meta['trial_manifest']['trial_id']); digest=hashlib.sha256((d/'obstacles_param.yaml').read_bytes()).hexdigest(); pairs[key].append((meta['baseline_id'],digest))
if len(pairs)!=8: raise SystemExit(f'expected 8 pairs, found {len(pairs)}')
for key,v in pairs.items():
 if {x[0] for x in v}!=expected or len({x[1] for x in v})!=1: raise SystemExit(f'pair audit failed: {key}')
print('Runtime/stability pilot audit passed: 16/16 valid and 8/8 paired hashes.')
PY
echo 'Runtime/stability pilot audit passed: 16/16 valid and 8/8 paired hashes.' > "$OUTPUT_ROOT/RUNTIME_STABILITY_PILOT_AUDIT_PASS.txt"
