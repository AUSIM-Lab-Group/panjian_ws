#!/usr/bin/env bash
set -eo pipefail
W="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; O="$W/swarm_test/output/secbf_runs/20260721_r42_context_int_ablation_formal30"; M="$W/swarm_test/config/seed_manifests/20260712_nine_condition_formal30.csv"
source /opt/ros/noetic/setup.bash; source "$W/devel/setup.bash"; set -u
python3 -u "$W/swarm_test/scripts/run_secbf_sim_experiments.py" --scenario head_on_context_int,crossing_context_int,local_crowding_context_int --baseline No_semantic,Category_only,Unguarded_SEESM,No_J_side,SEESM_Ours --duration-sec 30 --seed-manifest "$M" --output-root "$O" --roscore external --skip-existing-complete
mapfile -d '' D < <(find "$O" -mindepth 1 -maxdepth 1 -type d ! -name '.*' -print0 | sort -z); [[ ${#D[@]} -eq 450 ]] || { echo "expected 450, found ${#D[@]}" >&2; exit 1; }
for d in "${D[@]}"; do python3 "$W/swarm_test/scripts/check_experiment_csv_fields.py" "$d" >/dev/null; done
OUTPUT_ROOT="$O" python3 - <<'PY'
from collections import defaultdict
import csv,hashlib,os,yaml
from pathlib import Path
r=Path(os.environ['OUTPUT_ROOT']); p=defaultdict(list); e={'No_semantic','Category_only','Unguarded_SEESM','No_J_side','SEESM_Ours'}
for d in sorted(x for x in r.iterdir() if x.is_dir() and not x.name.startswith('.')):
 m=yaml.safe_load((d/'meta.yaml').read_text()); s=next(csv.DictReader((d/'summary.csv').open()));
 if s.get('termination_reason')=='invalid' or int(float(s.get('robot_records') or 0))<=0: raise SystemExit(f'invalid {d}')
 p[(m['scenario'],m['trial_manifest']['trial_id'])].append((m['baseline_id'],hashlib.sha256((d/'obstacles_param.yaml').read_bytes()).hexdigest()))
if len(p)!=90: raise SystemExit(f'expected 90 groups, found {len(p)}')
for k,v in p.items():
 if {x[0] for x in v}!=e or len({x[1] for x in v})!=1: raise SystemExit(f'pair failed {k}')
print('Ablation formal audit passed: 450/450 valid and 90/90 paired hashes.')
PY
echo 'Ablation formal audit passed: 450/450 valid and 90/90 paired hashes.' > "$O/ABLATION_FORMAL_AUDIT_PASS.txt"
