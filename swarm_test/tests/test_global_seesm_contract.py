import pathlib


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def test_applied_margin_message_contract():
    path = REPO_ROOT / "planner/semantic_guard/msg/AppliedMarginArray.msg"
    text = path.read_text(encoding="utf-8")
    assert "std_msgs/Header header" in text
    assert "uint32[] obstacle_ids" in text
    assert "float64[] beta_applied" in text
    assert "string[] accepted_sources" in text
