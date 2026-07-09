import math

import pytest

from swarm_test.scripts.reference_path_waypoints import (
    WaypointProgress,
    generate_waypoints,
    initial_yaw,
)


def test_arc_and_reverse_arc_are_exact_reverses():
    forward = generate_waypoints(
        {
            "type": "arc",
            "center": [0.0, 0.0],
            "radius": 4.0,
            "start_angle": math.pi / 2,
            "end_angle": -math.pi / 2,
            "clockwise": True,
            "spacing": 0.4,
        }
    )
    reverse = generate_waypoints(
        {
            "type": "arc",
            "center": [0.0, 0.0],
            "radius": 4.0,
            "start_angle": -math.pi / 2,
            "end_angle": math.pi / 2,
            "clockwise": False,
            "spacing": 0.4,
        }
    )
    assert reverse == pytest.approx(list(reversed(forward)))
    assert max(math.dist(a, b) for a, b in zip(forward, forward[1:])) <= 0.4


def test_vertical_paths_are_exact_reverses():
    up = generate_waypoints(
        {
            "type": "line",
            "start": [0.0, -4.0],
            "goal": [0.0, 4.0],
            "spacing": 0.4,
        }
    )
    down = generate_waypoints(
        {
            "type": "line",
            "start": [0.0, 4.0],
            "goal": [0.0, -4.0],
            "spacing": 0.4,
        }
    )
    assert down == pytest.approx(list(reversed(up)))
    assert initial_yaw(up) == pytest.approx(math.pi / 2)
    assert initial_yaw(down) == pytest.approx(-math.pi / 2)


def test_waypoint_progress_advances_only_inside_threshold():
    progress = WaypointProgress(
        [(0.0, 0.0), (0.4, 0.0), (0.8, 0.0)], threshold=0.35
    )
    assert progress.current == (0.4, 0.0)
    assert not progress.update((0.0, 0.0))
    assert progress.update((0.1, 0.0))
    assert progress.current == (0.8, 0.0)
    assert progress.update((0.8, 0.0))
    assert progress.complete
