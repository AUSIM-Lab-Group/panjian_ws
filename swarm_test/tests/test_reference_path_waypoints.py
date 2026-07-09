import math

import pytest

from swarm_test.scripts.reference_path_waypoints import (
    WaypointProgress,
    generate_waypoints,
    initial_yaw,
)


def _flatten(points):
    return [coordinate for point in points for coordinate in point]


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
    assert _flatten(reverse) == pytest.approx(_flatten(list(reversed(forward))))
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
    assert _flatten(down) == pytest.approx(_flatten(list(reversed(up))))
    assert initial_yaw(up) == pytest.approx(math.pi / 2)
    assert initial_yaw(down) == pytest.approx(-math.pi / 2)


def test_arc_waypoints_follow_raw_formula():
    waypoints = generate_waypoints(
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
    index = 2
    delta = -math.pi
    count = max(1, math.ceil(abs(delta) * 4.0 / 0.4))
    expected = (
        4.0 * math.cos(math.pi / 2 + delta * index / count),
        4.0 * math.sin(math.pi / 2 + delta * index / count),
    )
    assert waypoints[index] == pytest.approx(expected, abs=1e-15)


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


def test_generate_waypoints_rejects_unsupported_path_type():
    with pytest.raises(ValueError, match="unsupported reference path type"):
        generate_waypoints({"type": "spiral", "spacing": 0.4})


def test_generate_waypoints_rejects_non_positive_spacing():
    with pytest.raises(ValueError, match="waypoint spacing must be positive"):
        generate_waypoints(
            {
                "type": "line",
                "start": [0.0, 0.0],
                "goal": [1.0, 0.0],
                "spacing": 0.0,
            }
        )


def test_initial_yaw_rejects_fewer_than_two_waypoints():
    with pytest.raises(ValueError, match="at least two waypoints are required"):
        initial_yaw([(0.0, 0.0)])


def test_waypoint_progress_rejects_invalid_threshold():
    with pytest.raises(ValueError, match="threshold must be positive"):
        WaypointProgress([(0.0, 0.0), (1.0, 0.0)], threshold=0.0)


def test_waypoint_progress_rejects_fewer_than_two_waypoints():
    with pytest.raises(ValueError, match="at least two waypoints are required"):
        WaypointProgress([(0.0, 0.0)], threshold=0.35)
