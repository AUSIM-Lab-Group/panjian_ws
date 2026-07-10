import pathlib


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def test_applied_margin_message_contract():
    path = REPO_ROOT / "planner/semantic_guard/msg/AppliedMarginArray.msg"
    text = path.read_text(encoding="utf-8")
    lines = [line for line in text.splitlines() if line.strip()]
    assert lines == [
        "std_msgs/Header header",
        "uint32[] obstacle_ids",
        "float64[] beta_applied",
        "string[] accepted_sources",
    ]
