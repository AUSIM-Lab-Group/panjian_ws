#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTPUT_ROOT="$WORKSPACE/swarm_test/output/secbf_runs/20260715_r27_nine_condition_pilot2_current_protocol"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260714_nine_condition_pilot2.csv"

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
  --scenario head_on_context_bl,head_on_context_int,head_on_context_ext,crossing_context_bl,crossing_context_int,crossing_context_ext,local_crowding_context_bl,local_crowding_context_int,local_crowding_context_ext \
  --baseline Standard_MPC_CBF,EESM_MPC_ECBF,SEESM_Without_FPU,Proposed_MPC_SECBF \
  --duration-sec 30 \
  --seed-manifest "$MANIFEST" \
  --output-root "$OUTPUT_ROOT" \
  --roscore auto \
  --skip-existing-complete

mapfile -d '' RUN_DIRS < <(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 \
  -type d ! -name '.*' -print0 | sort -z)
if [[ "${#RUN_DIRS[@]}" -ne 72 ]]; then
  echo "Pilot audit failed: expected 72 run directories, found ${#RUN_DIRS[@]}." >&2
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

root = Path(os.environ["OUTPUT_ROOT"])
paired = defaultdict(list)
for run_dir in sorted(path for path in root.iterdir() if path.is_dir() and not path.name.startswith(".")):
    parts = run_dir.name.rsplit("_", 1)
    if len(parts) != 2:
        raise SystemExit(f"unexpected run directory name: {run_dir.name}")
    meta = (run_dir / "meta.yaml").read_text(encoding="utf-8")
    values = {}
    for line in meta.splitlines():
        if line.startswith(("scenario:", "baseline_id:")):
            key, value = line.split(":", 1)
            values[key] = value.strip()
    trial = next(
        line.split(":", 1)[1].strip()
        for line in meta.splitlines()
        if line.strip().startswith("trial_id:")
    )
    digest = hashlib.sha256((run_dir / "obstacles_param.yaml").read_bytes()).hexdigest()
    paired[(values["scenario"], trial)].append((values["baseline_id"], digest))

if len(paired) != 18:
    raise SystemExit(f"expected 18 paired scenario/seed groups, found {len(paired)}")
for key, values in paired.items():
    if len(values) != 4 or len({digest for _, digest in values}) != 1:
        raise SystemExit(f"paired obstacle hash mismatch: {key}: {values}")
print("Pilot audit passed: 72/72 field contracts and 18/18 paired obstacle hashes.")
PY
