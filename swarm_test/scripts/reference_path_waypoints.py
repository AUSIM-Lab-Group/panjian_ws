#!/usr/bin/env python3

"""Pure geometry helpers for Fig.4 reference paths."""

import math


def _point(x, y):
    return (round(x, 12), round(y, 12))


def generate_waypoints(path):
    spacing = float(path.get("spacing", 0.4))
    if spacing <= 0.0:
        raise ValueError("waypoint spacing must be positive")
    if path["type"] == "line":
        start = tuple(map(float, path["start"]))
        goal = tuple(map(float, path["goal"]))
        length = math.dist(start, goal)
        count = max(1, math.ceil(length / spacing))
        return [
            _point(
                start[0] + (goal[0] - start[0]) * index / count,
                start[1] + (goal[1] - start[1]) * index / count,
            )
            for index in range(count + 1)
        ]
    if path["type"] == "arc":
        center = tuple(map(float, path["center"]))
        radius = float(path["radius"])
        start_angle = float(path["start_angle"])
        end_angle = float(path["end_angle"])
        clockwise = bool(path["clockwise"])
        delta = end_angle - start_angle
        if clockwise and delta > 0.0:
            delta -= 2.0 * math.pi
        if not clockwise and delta < 0.0:
            delta += 2.0 * math.pi
        count = max(1, math.ceil(abs(delta) * radius / spacing))
        return [
            _point(
                center[0] + radius * math.cos(start_angle + delta * index / count),
                center[1] + radius * math.sin(start_angle + delta * index / count),
            )
            for index in range(count + 1)
        ]
    raise ValueError(f"unsupported reference path type: {path.get('type')}")


def initial_yaw(waypoints):
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    start = waypoints[0]
    next_point = waypoints[1]
    return math.atan2(next_point[1] - start[1], next_point[0] - start[0])


class WaypointProgress:
    def __init__(self, waypoints, threshold):
        if len(waypoints) < 2:
            raise ValueError("at least two waypoints are required")
        threshold = float(threshold)
        if threshold <= 0.0:
            raise ValueError("threshold must be positive")
        self._waypoints = list(waypoints)
        self._threshold = threshold
        self._index = 1
        self.complete = False

    @property
    def current(self):
        return self._waypoints[self._index]

    def update(self, position):
        if self.complete:
            return True
        if math.dist(position, self.current) <= self._threshold:
            if self._index < len(self._waypoints) - 1:
                self._index += 1
            if self._index >= len(self._waypoints) - 1:
                self.complete = True
            return True
        return False
