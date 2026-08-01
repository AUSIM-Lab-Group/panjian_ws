#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_ROOT="$(cd "$WORKSPACE/.." && pwd)"
SEESM_REPO="$PROJECT_ROOT/seesm_social_navigation"
CONFIG="$WORKSPACE/swarm_test/config/active_set_comparisons/local_crowding_max3_formal30.yaml"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260720_feasibility_stress_formal30.csv"
FREEZE="$WORKSPACE/swarm_test/config/experiment_freezes/active_set_formal30.yaml"
EVALUATOR="$WORKSPACE/swarm_test/config/common_offline_evaluation_v1.yaml"
OUTPUT_BASE="$SEESM_REPO/新计划实验输出目录/04_formal/local_crowding_active_set_max3_formal30_20260801"
REFERENCE_ROOT="$OUTPUT_BASE/max6"
CANDIDATE_ROOT="$OUTPUT_BASE/max3"
ANALYSIS_ROOT="$OUTPUT_BASE/analysis"
EXPECTED_BRANCH="formal/logging-repair-v4-20260729"
EXPECTED_ALGORITHM_COMMIT="f01451c0076e12742bdc3e1149de2138d283e75a"
EXPECTED_BATCH_COMMIT="22deff4fa1daf5e340b0f9fd448d97977d3c01eb"
EXPECTED_SEESM_COMMIT="948a4d1ffe901259f5cd00e728553bd57254b58d"
EXPECTED_MPC_SHA256="0a07dc1e47c5ef30598ab6e886c7cf1aa377b95d019acc0bcfd372684dc51ac3"
EXPECTED_GUARD_SHA256="9423f22d806b7819904ef47909517cacb38bde797b741a04fdcfbd900e5994fd"
DRY_RUN_ARGS=()

if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN_ARGS+=(--dry-run)
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--dry-run]" >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel_current/setup.bash"
set -u
export ROS_HOME="${ROS_HOME:-/tmp/ros_teacher_active_set_max3}"
mkdir -p "$ROS_HOME"

[[ "$(git -C "$WORKSPACE" branch --show-current)" == "$EXPECTED_BRANCH" ]]
[[ "$(git -C "$SEESM_REPO" branch --show-current)" == "$EXPECTED_BRANCH" ]]
git -C "$WORKSPACE" merge-base --is-ancestor "$EXPECTED_ALGORITHM_COMMIT" HEAD
git -C "$WORKSPACE" merge-base --is-ancestor "$EXPECTED_BATCH_COMMIT" HEAD
git -C "$SEESM_REPO" merge-base --is-ancestor "$EXPECTED_SEESM_COMMIT" HEAD
[[ "$(sha256sum "$WORKSPACE/devel_current/lib/mpc_secbf/mpc_secbf_node" | awk '{print $1}')" == "$EXPECTED_MPC_SHA256" ]]
[[ "$(sha256sum "$WORKSPACE/devel_current/lib/semantic_guard/beta_ground_truth_node" | awk '{print $1}')" == "$EXPECTED_GUARD_SHA256" ]]

if [[ ${#DRY_RUN_ARGS[@]} -eq 0 ]]; then
  [[ -z "$(git -C "$WORKSPACE" status --porcelain=v1 --untracked-files=all)" ]]
  [[ -z "$(git -C "$SEESM_REPO" status --porcelain=v1 --untracked-files=all)" ]]
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

run_variant() {
  local label="$1"
  local active_set_max="$2"
  local output_root="$3"
  python3 -u "$WORKSPACE/swarm_test/scripts/run_teacher_active_set_pilot.py" \
    --active-set-max "$active_set_max" \
    --scenario stress_local_crowding \
    --baseline SEESM_Ours \
    --duration-sec 30 \
    --seed-manifest "$MANIFEST" \
    --campaign active_set_formal30 \
    --execution-tier formal \
    --parameter-freeze "$FREEZE" \
    --protocol-id "teacher_v1_local_crowding_active_set_${label}_formal30_20260801" \
    --output-root "$output_root" \
    --roscore auto \
    --skip-existing-complete \
    "${DRY_RUN_ARGS[@]}"
}

run_variant max6 6 "$REFERENCE_ROOT"
run_variant max3 3 "$CANDIDATE_ROOT"

if [[ ${#DRY_RUN_ARGS[@]} -gt 0 ]]; then
  exit 0
fi

audit_variant() {
  local output_root="$1"
  local expected_max="$2"
  mapfile -d '' run_dirs < <(
    find "$output_root" -mindepth 1 -maxdepth 1 -type d -name 'formal_*' -print0 | sort -z
  )
  [[ ${#run_dirs[@]} -eq 30 ]] || {
    echo "Expected 30 run directories under $output_root, found ${#run_dirs[@]}." >&2
    return 1
  }
  for run_dir in "${run_dirs[@]}"; do
    [[ -f "$run_dir/RUN_COMPLETE.txt" ]]
    [[ ! -e "$run_dir/RUN_INVALID.txt" ]]
    [[ "$(awk '/^max_cbf_obstacles:/{print $2; exit}' "$run_dir/meta.yaml")" == "$expected_max" ]]
    (cd "$run_dir" && sha256sum -c trial_integrity.sha256 >/dev/null)
    python3 "$WORKSPACE/swarm_test/scripts/check_experiment_csv_fields.py" "$run_dir" >/dev/null
  done
}

audit_variant "$REFERENCE_ROOT" 6
audit_variant "$CANDIDATE_ROOT" 3

for output_root in "$REFERENCE_ROOT" "$CANDIDATE_ROOT"; do
  python3 "$WORKSPACE/swarm_test/scripts/postprocess_teacher_canonical_runs.py" \
    --output-root "$output_root" \
    --config "$WORKSPACE/swarm_test/config/secbf_scenarios.yaml" \
    --evaluation-contract "$EVALUATOR"
done

python3 "$WORKSPACE/swarm_test/scripts/analyze_local_crowding_active_set_compare.py" \
  --config "$CONFIG" \
  --reference-root "$REFERENCE_ROOT" \
  --candidate-root "$CANDIDATE_ROOT" \
  --output-dir "$ANALYSIS_ROOT"
