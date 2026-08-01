#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_ROOT="$(cd "$WORKSPACE/.." && pwd)"
CONFIG="$WORKSPACE/swarm_test/config/active_set_comparisons/local_crowding_max3_pilot.yaml"
MANIFEST="$WORKSPACE/swarm_test/config/seed_manifests/20260801_local_crowding_max3_pilot5.csv"
FREEZE="$WORKSPACE/swarm_test/config/experiment_freezes/active_set_formal.yaml"
EVALUATOR="$WORKSPACE/swarm_test/config/common_offline_evaluation_v1.yaml"
OUTPUT_BASE="$PROJECT_ROOT/seesm_social_navigation/新计划实验输出目录/03_smoke/local_crowding_active_set_max3_pilot_20260801"
REFERENCE_ROOT="$OUTPUT_BASE/max6"
CANDIDATE_ROOT="$OUTPUT_BASE/max3"
ANALYSIS_ROOT="$OUTPUT_BASE/analysis"
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
    --campaign active_set \
    --execution-tier formal \
    --parameter-freeze "$FREEZE" \
    --protocol-id "teacher_v1_local_crowding_active_set_${label}_pilot_20260801" \
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
