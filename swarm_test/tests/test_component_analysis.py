import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
ANALYZER_PATH = REPO_ROOT / "swarm_test/scripts/analyze_exp3_context.py"


def load_analyzer():
    spec = importlib.util.spec_from_file_location("exp3_component_analyzer", ANALYZER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def row(method, context, beta_mean):
    return {
        "method": method,
        "context": context,
        "n": 10,
        "mu_mean": 0.5,
        "mu_max": 0.8,
        "beta_hat_mean": beta_mean,
        "beta_hat_max": beta_mean,
        "beta_mean": beta_mean,
        "beta_max": beta_mean,
        "cos_delta_mean": 0.0,
        "ttc_mean": 1.0,
        "ttc_norm_mean": 0.5,
        "rho_norm_mean": 0.0,
        "h_EE_min": 0.6,
        "h_SEE_min": 0.2,
        "run_dir": f"/{method}/{context}",
    }


def test_exp3_aggregate_keeps_methods_separate():
    analyzer = load_analyzer()
    rows = [
        row("Category_only", "crossing", 0.4),
        row("SEESM_Ours", "crossing", 0.3),
    ]

    stats = analyzer.aggregate(rows)

    assert {(item["method"], item["context"]) for item in stats} == {
        ("Category_only", "crossing"),
        ("SEESM_Ours", "crossing"),
    }
    assert {item["beta_mean_mean"] for item in stats} == {0.4, 0.3}


def test_exp3_curve_exemplars_use_only_proposed_runs():
    analyzer = load_analyzer()
    rows = [
        row("Category_only", "static", 0.4),
        row("SEESM_Ours", "static", 0.25),
        row("SEESM_Ours", "crossing", 0.35),
    ]

    exemplars = analyzer.proposed_exemplars(rows)

    assert exemplars == {
        "static": Path("/SEESM_Ours/static"),
        "crossing": Path("/SEESM_Ours/crossing"),
    }
