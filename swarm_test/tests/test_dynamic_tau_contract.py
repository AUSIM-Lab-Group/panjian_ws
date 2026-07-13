from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def read(relative):
    return (REPO_ROOT / relative).read_text(encoding="utf-8")


def test_shared_policy_is_the_numeric_source_of_truth():
    header = read("planner/semantic_guard/include/semantic_guard/dynamic_tau.hpp")
    assert "struct DynamicTauParams" in header
    assert "struct DynamicTauResult" in header
    assert "computeDynamicTau" in header
    assert "f_r" in header and "f_v" in header and "f_T" in header
    assert "max_tau" in header


def test_dynamic_methods_have_the_same_switch_and_beta_contract():
    runner = read("swarm_test/scripts/run_secbf_sim_experiments.py")
    for method in ("No_semantic", "Unguarded_SEESM", "SEESM_Ours"):
        assert method in runner
    assert '"dynamic_tau_enabled": True' in runner
    assert '"dynamic_tau_enabled": False' in runner
    assert "beta_applied_final" in runner
    assert "tau_valid" in runner
    assert "tau_reason" in runner


def test_standard_and_legacy_baselines_remain_separate():
    runner = read("swarm_test/scripts/run_secbf_sim_experiments.py")
    legacy = read("planner/mpc_dcbf/src/mpc_cbf.cpp")
    assert '"Standard_MPC_CBF"' in runner
    assert "cbf_metric" in runner
    assert "set_tau_value" in legacy
    assert "B1_ACBF_fixed" in runner
