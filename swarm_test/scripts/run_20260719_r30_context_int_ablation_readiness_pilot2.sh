#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTPUT_ROOT="$WORKSPACE/swarm_test/output/secbf_runs/20260719_r30_context_int_ablation_readiness_pilot2"
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

# This readiness gate intentionally covers only the four ablation variants that
# currently exist in the controller. No_J_side must not be added here until the
# separately switchable J_side cost and its frozen coefficient are implemented.
python3 -u "$WORKSPACE/swarm_test/scripts/run_secbf_sim_experiments.py" \
  --scenario head_on_context_int,crossing_context_int,local_crowding_context_int \
  --baseline No_semantic,Category_only,Unguarded_SEESM,SEESM_Ours \
  --duration-sec 30 \
  --seed-manifest "$MANIFEST" \
  --output-root "$OUTPUT_ROOT" \
  --roscore auto \
  --skip-existing-complete

mapfile -d '' RUN_DIRS < <(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 \
  -type d ! -name '.*' -print0 | sort -z)
if [[ "${#RUN_DIRS[@]}" -ne 24 ]]; then
  echo "Ablation readiness audit failed: expected 24 run directories, found ${#RUN_DIRS[@]}." >&2
  exit 1
fi

for run_dir in "${RUN_DIRS[@]}"; do
  python3 "$WORKSPACE/swarm_test/scripts/check_experiment_csv_fields.py" "$run_dir"
done

OUTPUT_ROOT="$OUTPUT_ROOT" python3 - <<'PY'
from collections import defaultdict
import hashlib
import os
from pathlib import Path
import yaml

root = Path(os.environ["OUTPUT_ROOT"])
paired = defaultdict(list)
expected = {"No_semantic", "Category_only", "Unguarded_SEESM", "SEESM_Ours"}
for run_dir in sorted(path for path in root.iterdir() if path.is_dir() and not path.name.startswith(".")):
    meta = yaml.safe_load((run_dir / "meta.yaml").read_text(encoding="utf-8"))
    baseline = meta["baseline_id"]
    if baseline not in expected:
        raise SystemExit(f"unexpected ablation baseline: {baseline}")
    trial = meta["trial_manifest"]["trial_id"]
    digest = hashlib.sha256((run_dir / "obstacles_param.yaml").read_bytes()).hexdigest()
    paired[(meta["scenario"], trial)].append((baseline, digest))

if len(paired) != 6:
    raise SystemExit(f"expected 6 paired scenario/seed groups, found {len(paired)}")
for key, values in paired.items():
    if {baseline for baseline, _ in values} != expected:
        raise SystemExit(f"incomplete ablation variants: {key}: {values}")
    if len({digest for _, digest in values}) != 1:
        raise SystemExit(f"paired obstacle hash mismatch: {key}: {values}")
print("Ablation readiness audit passed: 24/24 contracts and 6/6 paired hashes.")
PY

