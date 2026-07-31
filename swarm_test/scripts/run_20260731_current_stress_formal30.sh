#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_ROOT="$(cd "$WORKSPACE/.." && pwd)"
SEESM_REPO="$PROJECT_ROOT/seesm_social_navigation"
OUTPUT_ROOT="$SEESM_REPO/新计划实验输出目录/04_formal/stress_formal_current_20260731"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260720_feasibility_stress_formal30.csv"
FREEZE="$WORKSPACE/swarm_test/config/experiment_freezes/stress_formal.yaml"
EVALUATOR="$WORKSPACE/swarm_test/config/common_offline_evaluation_v1.yaml"
RUNNER="$WORKSPACE/swarm_test/scripts/run_secbf_sim_experiments.py"
POSTPROCESS="$WORKSPACE/swarm_test/scripts/postprocess_teacher_canonical_runs.py"
OVERLAY="$WORKSPACE/devel_current"
EXPECTED_BRANCH="formal/logging-repair-v4-20260729"
EXPECTED_PANJIAN_ALGORITHM_COMMIT="f01451c0076e12742bdc3e1149de2138d283e75a"
EXPECTED_SEESM_COMMIT="cefd47d"
EXPECTED_MPC_SHA256="0a07dc1e47c5ef30598ab6e886c7cf1aa377b95d019acc0bcfd372684dc51ac3"
EXPECTED_GUARD_SHA256="9423f22d806b7819904ef47909517cacb38bde797b741a04fdcfbd900e5994fd"

source /opt/ros/noetic/setup.bash
source "$OVERLAY/setup.bash"
set -u
export ROS_HOME="${ROS_HOME:-/tmp/ros_teacher_stress_current}"
mkdir -p "$ROS_HOME"

[[ "$(git -C "$WORKSPACE" branch --show-current)" == "$EXPECTED_BRANCH" ]]
[[ "$(git -C "$SEESM_REPO" branch --show-current)" == "$EXPECTED_BRANCH" ]]
[[ -z "$(git -C "$WORKSPACE" status --porcelain=v1 --untracked-files=all)" ]]
[[ -z "$(git -C "$SEESM_REPO" status --porcelain=v1 --untracked-files=all)" ]]
git -C "$WORKSPACE" merge-base --is-ancestor "$EXPECTED_PANJIAN_ALGORITHM_COMMIT" HEAD
git -C "$SEESM_REPO" merge-base --is-ancestor "$EXPECTED_SEESM_COMMIT" HEAD
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
  --scenario stress_high_candidate_margin,stress_short_ttc,stress_local_crowding \
  --baseline Unguarded_SEESM,SEESM_Ours \
  --duration-sec 30 \
  --seed-manifest "$MANIFEST" \
  --campaign stress \
  --execution-tier formal \
  --parameter-freeze "$FREEZE" \
  --protocol-id teacher_v1_stress_formal_current_20260731 \
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
[[ ${#RUN_DIRS[@]} -eq 180 ]] || {
  echo "Expected 180 run directories, found ${#RUN_DIRS[@]}." >&2
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
expected = {"Unguarded_SEESM", "SEESM_Ours"}
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

message = "Stress formal audit passed: 180/180 valid and 90/90 paired hashes.\n"
(root / "STRESS_FORMAL_AUDIT_PASS.txt").write_text(message, encoding="utf-8")
(root / "FORMAL_STRESS_AUDIT_PASS.txt").write_text(message, encoding="utf-8")
print(message.strip())
PY

python3 "$POSTPROCESS" \
  --output-root "$OUTPUT_ROOT" \
  --config "$WORKSPACE/swarm_test/config/secbf_scenarios.yaml" \
  --evaluation-contract "$EVALUATOR"
