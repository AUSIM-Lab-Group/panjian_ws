#!/usr/bin/env bash
set -eo pipefail
W="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; T="$(cd "$W/.." && pwd)"; O="$T/seesm_social_navigation/新计划实验输出目录/04_formal/runtime_v3_clean_20260728"; M="$W/swarm_test/config/seed_manifests/20260721_runtime_stability_formal30.csv"; F="$W/swarm_test/config/experiment_freezes/runtime_formal.yaml"
source /opt/ros/noetic/setup.bash; source "$W/devel/setup.bash"; set -u
python3 -u "$W/swarm_test/scripts/run_secbf_sim_experiments.py" --scenario runtime_scaling_n1,runtime_scaling_n2,runtime_scaling_n4,runtime_scaling_n6 --baseline EESM_MPC_ECBF,Proposed_MPC_SECBF --duration-sec 30 --seed-manifest "$M" --campaign runtime --execution-tier formal --parameter-freeze "$F" --protocol-id teacher_v1_runtime_formal_v3_clean_20260728 --output-root "$O" --roscore external --skip-existing-complete
mapfile -d '' D < <(find "$O" -mindepth 1 -maxdepth 1 -type d ! -name '.*' -print0 | sort -z); [[ ${#D[@]} -eq 240 ]] || { echo "expected 240, found ${#D[@]}" >&2; exit 1; }
for d in "${D[@]}"; do python3 "$W/swarm_test/scripts/check_experiment_csv_fields.py" "$d" >/dev/null; done
OUTPUT_ROOT="$O" python3 - <<'PY'
from collections import defaultdict
import csv,hashlib,os,yaml
from pathlib import Path
r=Path(os.environ['OUTPUT_ROOT']); p=defaultdict(list); e={'No_semantic','SEESM_Ours'}
for d in sorted(x for x in r.iterdir() if x.is_dir() and not x.name.startswith('.')):
 m=yaml.safe_load((d/'meta.yaml').read_text()); s=next(csv.DictReader((d/'summary.csv').open()));
 if s.get('termination_reason')=='invalid' or int(float(s.get('robot_records') or 0))<=0: raise SystemExit(f'invalid {d}')
 p[(m['scenario'],m['trial_manifest']['trial_id'])].append((m['baseline_id'],hashlib.sha256((d/'obstacles_param.yaml').read_bytes()).hexdigest()))
if len(p)!=120: raise SystemExit(f'expected 120 pairs, found {len(p)}')
for k,v in p.items():
 if {x[0] for x in v}!=e or len({x[1] for x in v})!=1: raise SystemExit(f'pair failed {k}')
print('Runtime formal audit passed: 240/240 valid and 120/120 paired hashes.')
PY
python3 "$W/swarm_test/scripts/analyze_runtime_stability_thresholds.py" --input-root "$O" --output "$O/runtime_stability_thresholds.csv"
echo 'Runtime formal audit passed: 240/240 valid and 120/120 paired hashes.' > "$O/RUNTIME_STABILITY_FORMAL_AUDIT_PASS.txt"
