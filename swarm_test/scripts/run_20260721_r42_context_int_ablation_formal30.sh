#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_ROOT="$(cd "$WORKSPACE/.." && pwd)"
SEESM_REPO="$PROJECT_ROOT/seesm_social_navigation"
OUTPUT_ROOT="$SEESM_REPO/新计划实验输出目录/04_formal/ablation_formal_current_20260730"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260712_nine_condition_formal30.csv"
FREEZE="$WORKSPACE/swarm_test/config/experiment_freezes/ablation_formal.yaml"
RUNNER="$WORKSPACE/swarm_test/scripts/run_secbf_sim_experiments.py"
OVERLAY="$WORKSPACE/devel_current"
EXPECTED_BRANCH="formal/logging-repair-v4-20260729"
EXPECTED_PANJIAN_ALGORITHM_COMMIT="5adfce86b8e1b05d39abaeabd8a10620f394ff03"
EXPECTED_SEESM_COMMIT="afe29b65673a84cb884059c25a04d4a80930a006"
EXPECTED_MPC_SHA256="0a07dc1e47c5ef30598ab6e886c7cf1aa377b95d019acc0bcfd372684dc51ac3"
EXPECTED_GUARD_SHA256="9423f22d806b7819904ef47909517cacb38bde797b741a04fdcfbd900e5994fd"

source /opt/ros/noetic/setup.bash
source "$OVERLAY/setup.bash"
set -u
export ROS_HOME="${ROS_HOME:-/tmp/ros_teacher_ablation_current}"
mkdir -p "$ROS_HOME"

[[ "$(git -C "$WORKSPACE" branch --show-current)" == "$EXPECTED_BRANCH" ]]
[[ "$(git -C "$SEESM_REPO" branch --show-current)" == "$EXPECTED_BRANCH" ]]
[[ -z "$(git -C "$WORKSPACE" status --porcelain=v1 --untracked-files=all)" ]]
[[ -z "$(git -C "$SEESM_REPO" status --porcelain=v1 --untracked-files=all)" ]]
git -C "$WORKSPACE" merge-base --is-ancestor "$EXPECTED_PANJIAN_ALGORITHM_COMMIT" HEAD
[[ "$(git -C "$SEESM_REPO" rev-parse HEAD)" == "$EXPECTED_SEESM_COMMIT" ]]
[[ "$(sha256sum "$OVERLAY/lib/mpc_secbf/mpc_secbf_node" | awk '{print $1}')" == "$EXPECTED_MPC_SHA256" ]]
[[ "$(sha256sum "$OVERLAY/lib/semantic_guard/beta_ground_truth_node" | awk '{print $1}')" == "$EXPECTED_GUARD_SHA256" ]]

DRY_RUN_ARGS=()
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN_ARGS+=(--dry-run)
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--dry-run]" >&2
  exit 2
else
  python3 - <<'PY'
import socket

sock = socket.socket()
try:
    sock.bind(("127.0.0.1", 0))
finally:
    sock.close()
print("Local ROS socket preflight passed.")
PY
fi

python3 -u "$RUNNER" \
  --scenario head_on_context_int,crossing_context_int,local_crowding_context_int \
  --baseline No_semantic,Category_only,Unguarded_SEESM,No_J_side,SEESM_Ours \
  --duration-sec 30 \
  --seed-manifest "$MANIFEST" \
  --campaign ablation \
  --execution-tier formal \
  --parameter-freeze "$FREEZE" \
  --protocol-id teacher_v1_ablation_formal_current_20260730 \
  --output-root "$OUTPUT_ROOT" \
  --roscore auto \
  --skip-existing-complete \
  "${DRY_RUN_ARGS[@]}"

if [[ ${#DRY_RUN_ARGS[@]} -gt 0 ]]; then
  exit 0
fi

mapfile -d '' RUN_DIRS < <(
  find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -type d ! -name '.*' -print0 | sort -z
)
[[ ${#RUN_DIRS[@]} -eq 450 ]] || {
  echo "Expected 450 run directories, found ${#RUN_DIRS[@]}." >&2
  exit 1
}

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
expected = {
    "No_semantic", "Category_only", "Unguarded_SEESM", "No_J_side",
    "SEESM_Ours",
}
paired = defaultdict(list)

for run_dir in sorted(path for path in root.iterdir() if path.is_dir()):
    meta = yaml.safe_load((run_dir / "meta.yaml").read_text(encoding="utf-8"))
    with (run_dir / "summary.csv").open(encoding="utf-8") as stream:
        summary = next(csv.DictReader(stream))
    if summary.get("termination_reason") == "invalid":
        raise SystemExit(f"Invalid trial: {run_dir}")
    if int(float(summary.get("robot_records") or 0)) <= 0:
        raise SystemExit(f"Empty robot log: {run_dir}")
    key = (meta["scenario"], meta["trial_manifest"]["trial_id"])
    digest = hashlib.sha256((run_dir / "obstacles_param.yaml").read_bytes()).hexdigest()
    paired[key].append((meta["baseline_id"], digest))

if len(paired) != 90:
    raise SystemExit(f"Expected 90 paired groups, found {len(paired)}")
for key, values in paired.items():
    if {method for method, _ in values} != expected:
        raise SystemExit(f"Method pairing failed: {key}")
    if len({digest for _, digest in values}) != 1:
        raise SystemExit(f"Obstacle hash pairing failed: {key}")

print("Ablation formal audit passed: 450/450 valid and 90/90 paired hashes.")
PY

printf '%s\n' \
  'Ablation formal audit passed: 450/450 valid and 90/90 paired hashes.' \
  > "$OUTPUT_ROOT/ABLATION_FORMAL_AUDIT_PASS.txt"
