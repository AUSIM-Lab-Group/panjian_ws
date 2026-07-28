#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TEACHER_ROOT="$(cd "$WORKSPACE/.." && pwd)"
OUTPUT_ROOT="$TEACHER_ROOT/seesm_social_navigation/新计划实验输出目录/04_formal/main_v3_clean_20260728"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260712_nine_condition_formal30.csv"
FREEZE="$WORKSPACE/swarm_test/config/experiment_freezes/main_formal.yaml"

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
  --campaign main \
  --execution-tier formal \
  --parameter-freeze "$FREEZE" \
  --protocol-id teacher_v1_main_formal_v3_clean_20260728 \
  --output-root "$OUTPUT_ROOT" \
  --roscore auto \
  --skip-existing-complete

mapfile -d '' RUN_DIRS < <(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 \
  -type d ! -name '.*' -print0 | sort -z)
if [[ "${#RUN_DIRS[@]}" -ne 1080 ]]; then
  echo "Formal audit failed: expected 1080 run directories, found ${#RUN_DIRS[@]}." >&2
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
for run_dir in sorted(path for path in root.iterdir() if path.is_dir() and not path.name.startswith(".")):
    meta = yaml.safe_load((run_dir / "meta.yaml").read_text(encoding="utf-8"))
    trial = meta["trial_manifest"]["trial_id"]
    scenario = meta["scenario"]
    baseline = meta["baseline_id"]
    if "adult" in str(meta.get("obstacle_classes", "")):
        raise SystemExit(f"legacy adult label in formal run: {run_dir.name}")
    digest = hashlib.sha256((run_dir / "obstacles_param.yaml").read_bytes()).hexdigest()
    paired[(scenario, trial)].append((baseline, digest))

if len(paired) != 270:
    raise SystemExit(f"expected 270 paired scenario/seed groups, found {len(paired)}")
for key, values in paired.items():
    if len(values) != 4 or len({digest for _, digest in values}) != 1:
        raise SystemExit(f"paired obstacle hash mismatch: {key}: {values}")
print("Formal pairing audit passed: 1080/1080 contracts and 270/270 paired hashes.")
PY

python3 "$WORKSPACE/swarm_test/scripts/postprocess_teacher_canonical_runs.py" \
  --output-root "$OUTPUT_ROOT"

OUTPUT_ROOT="$OUTPUT_ROOT" python3 - <<'PY'
import os
from pathlib import Path

root = Path(os.environ["OUTPUT_ROOT"])
(root / "FORMAL_AUDIT_PASS.txt").write_text(
    "1080/1080 field contracts passed\n"
    "270/270 paired obstacle hashes passed\n"
    "teacher post-processing completed\n",
    encoding="utf-8",
)
print(f"Formal batch and audit completed: {root}")
PY
