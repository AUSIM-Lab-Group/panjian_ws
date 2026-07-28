#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TEACHER_ROOT="$(cd "$WORKSPACE/.." && pwd)"
OUTPUT_ROOT="$TEACHER_ROOT/seesm_social_navigation/新计划实验输出目录/04_formal/stress_v3_clean_20260728"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260720_feasibility_stress_formal30.csv"
FREEZE="$WORKSPACE/swarm_test/config/experiment_freezes/stress_formal.yaml"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
set -u

python3 - <<'PY'
import socket
sock = socket.socket()
try:
    sock.bind(("127.0.0.1", 0))
finally:
    sock.close()
print("Local ROS socket preflight passed.")
PY

python3 -u "$WORKSPACE/swarm_test/scripts/run_secbf_sim_experiments.py" \
  --scenario stress_high_candidate_margin,stress_short_ttc,stress_local_crowding \
  --baseline Unguarded_SEESM,SEESM_Ours \
  --duration-sec 30 \
  --seed-manifest "$MANIFEST" \
  --campaign stress \
  --execution-tier formal \
  --parameter-freeze "$FREEZE" \
  --protocol-id teacher_v1_stress_formal_v3_clean_20260728 \
  --output-root "$OUTPUT_ROOT" \
  --roscore external \
  --skip-existing-complete

mapfile -d '' RUN_DIRS < <(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 \
  -type d ! -name '.*' -print0 | sort -z)
if [[ "${#RUN_DIRS[@]}" -ne 180 ]]; then
  echo "Formal stress audit failed: expected 180 run directories, found ${#RUN_DIRS[@]}." >&2
  exit 1
fi

for run_dir in "${RUN_DIRS[@]}"; do
  python3 "$WORKSPACE/swarm_test/scripts/check_experiment_csv_fields.py" "$run_dir" >/dev/null
done

OUTPUT_ROOT="$OUTPUT_ROOT" python3 - <<'PY'
from collections import defaultdict
import csv
import hashlib
import os
from pathlib import Path
import yaml

root = Path(os.environ["OUTPUT_ROOT"])
paired = defaultdict(list)
expected = {"Unguarded_SEESM", "SEESM_Ours"}
for run_dir in sorted(path for path in root.iterdir() if path.is_dir() and not path.name.startswith(".")):
    meta = yaml.safe_load((run_dir / "meta.yaml").read_text(encoding="utf-8"))
    trial_summary = next(csv.DictReader((run_dir / "summary.csv").open()))
    if trial_summary.get("termination_reason") == "invalid" or int(float(trial_summary.get("robot_records") or 0)) <= 0:
        raise SystemExit(f"invalid formal stress trial: {run_dir}")
    trial = meta["trial_manifest"]["trial_id"]
    digest = hashlib.sha256((run_dir / "obstacles_param.yaml").read_bytes()).hexdigest()
    paired[(meta["scenario"], trial)].append((meta["baseline_id"], digest))

if len(paired) != 90:
    raise SystemExit(f"expected 90 paired scenario/seed groups, found {len(paired)}")
for key, values in paired.items():
    if {baseline for baseline, _ in values} != expected:
        raise SystemExit(f"incomplete formal stress pair: {key}: {values}")
    if len({digest for _, digest in values}) != 1:
        raise SystemExit(f"formal stress paired hash mismatch: {key}: {values}")

(root / "FORMAL_STRESS_AUDIT_PASS.txt").write_text(
    "180/180 valid field contracts passed\n"
    "90/90 paired obstacle hashes passed\n",
    encoding="utf-8",
)
print("Formal stress audit passed: 180/180 valid contracts and 90/90 paired hashes.")
PY
