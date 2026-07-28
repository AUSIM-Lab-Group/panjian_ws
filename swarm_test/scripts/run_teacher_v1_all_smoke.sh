#!/usr/bin/env bash
set -eo pipefail

W="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
T="$(cd "$W/.." && pwd)"
R="$T/seesm_social_navigation/新计划实验输出目录/03_smoke"
RUNNER="$W/swarm_test/scripts/run_secbf_sim_experiments.py"
AUDIT="$W/swarm_test/scripts/audit_teacher_campaign.py"

source /opt/ros/noetic/setup.bash
source "$W/devel/setup.bash"
set -u
export ROS_HOME="${ROS_HOME:-/tmp/teacher_v1_ros_home}"

run_campaign() {
  local campaign="$1"
  local scenarios="$2"
  local methods="$3"
  local manifest="$4"
  local output="$R/${campaign}_v1"
  local freeze="$W/swarm_test/config/experiment_freezes/${campaign}_smoke.yaml"

  python3 -u "$RUNNER" \
    --campaign "$campaign" \
    --execution-tier smoke \
    --parameter-freeze "$freeze" \
    --protocol-id "teacher_v1_${campaign}_smoke_v1" \
    --scenario "$scenarios" \
    --baseline "$methods" \
    --duration-sec 30 \
    --seed-manifest "$manifest" \
    --output-root "$output" \
    --roscore auto \
    --skip-existing-complete

  python3 "$AUDIT" --root "$output" --campaign "$campaign" --tier smoke
}

run_campaign \
  main \
  head_on_context_bl,head_on_context_int,head_on_context_ext,crossing_context_bl,crossing_context_int,crossing_context_ext,local_crowding_context_bl,local_crowding_context_int,local_crowding_context_ext \
  Standard_MPC_CBF,EESM_MPC_ECBF,SEESM_Without_FPU,Proposed_MPC_SECBF \
  "$W/swarm_test/config/seed_manifests/20260714_nine_condition_pilot2.csv"

run_campaign \
  ablation \
  head_on_context_int,crossing_context_int,local_crowding_context_int \
  No_semantic,Category_only,Unguarded_SEESM,No_J_side,SEESM_Ours \
  "$W/swarm_test/config/seed_manifests/20260714_nine_condition_pilot2.csv"

run_campaign \
  stress \
  stress_high_candidate_margin,stress_short_ttc,stress_local_crowding \
  Unguarded_SEESM,SEESM_Ours \
  "$W/swarm_test/config/seed_manifests/20260720_feasibility_stress_pilot2.csv"

run_campaign \
  runtime \
  runtime_scaling_n1,runtime_scaling_n2,runtime_scaling_n4,runtime_scaling_n6 \
  EESM_MPC_ECBF,Proposed_MPC_SECBF \
  "$W/swarm_test/config/seed_manifests/20260721_runtime_stability_pilot2.csv"
