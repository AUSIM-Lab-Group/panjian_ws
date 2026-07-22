#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTPUT_ROOT="$WORKSPACE/swarm_test/output/secbf_runs/20260720_r33_jside_weight_pilot2"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260719_context_int_ablation_pilot2.csv"

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
  --scenario head_on_context_int,crossing_context_int,local_crowding_context_int \
  --baseline No_J_side,SideWeight_005,SideWeight_010,SideWeight_020,SideWeight_050 \
  --duration-sec 30 \
  --seed-manifest "$MANIFEST" \
  --output-root "$OUTPUT_ROOT" \
  --roscore external \
  --skip-existing-complete

mapfile -d '' RUN_DIRS < <(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 \
  -type d ! -name '.*' -print0 | sort -z)
if [[ "${#RUN_DIRS[@]}" -ne 30 ]]; then
  echo "J_side pilot audit failed: expected 30 run directories, found ${#RUN_DIRS[@]}." >&2
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
expected = {"No_J_side", "SideWeight_005", "SideWeight_010", "SideWeight_020", "SideWeight_050"}
for run_dir in sorted(path for path in root.iterdir() if path.is_dir() and not path.name.startswith(".")):
    meta = yaml.safe_load((run_dir / "meta.yaml").read_text(encoding="utf-8"))
    trial_summary = next(csv.DictReader((run_dir / "summary.csv").open()))
    if trial_summary.get("termination_reason") == "invalid" or int(float(trial_summary.get("robot_records") or 0)) <= 0:
        raise SystemExit(f"invalid J_side trial: {run_dir}")
    trial = meta["trial_manifest"]["trial_id"]
    digest = hashlib.sha256((run_dir / "obstacles_param.yaml").read_bytes()).hexdigest()
    paired[(meta["scenario"], trial)].append((meta["baseline_id"], digest))

if len(paired) != 6:
    raise SystemExit(f"expected 6 paired scenario/seed groups, found {len(paired)}")
for key, values in paired.items():
    if {baseline for baseline, _ in values} != expected:
        raise SystemExit(f"incomplete J_side weight set: {key}: {values}")
    if len({digest for _, digest in values}) != 1:
        raise SystemExit(f"J_side paired obstacle hash mismatch: {key}: {values}")
print("J_side pilot audit passed: 30/30 valid contracts and 6/6 paired hashes.")
PY

python3 "$WORKSPACE/swarm_test/scripts/analyze_jside_weight_pilot.py" \
  --input-root "$OUTPUT_ROOT" \
  --output "$OUTPUT_ROOT/jside_weight_summary.csv"
