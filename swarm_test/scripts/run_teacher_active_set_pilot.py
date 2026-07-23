#!/usr/bin/env python3
"""Run a Teacher-v1 active-set/cache diagnostic without changing the runner defaults.

The wrapper injects only diagnostic launch switches into the existing runner.
The canonical Teacher-v1 path remains max_cbf_obstacles=6, active_set_distance=8 m,
and graph_cache_enabled=false unless the caller explicitly overrides them.
"""

import argparse
import sys

import run_secbf_sim_experiments as runner


def main() -> int:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--active-set-max", type=int, default=None)
    parser.add_argument("--active-set-distance", type=float, default=None)
    parser.add_argument("--graph-cache", action="store_true")
    known, remaining = parser.parse_known_args()
    if known.active_set_max is not None and known.active_set_max < 1:
        parser.error("--active-set-max must be positive")
    if known.active_set_distance is not None and known.active_set_distance <= 0.0:
        parser.error("--active-set-distance must be positive")

    original_switches = runner.scenario_switches

    def overridden_switches(baseline_id, scenario):
        switches = original_switches(baseline_id, scenario)
        if known.active_set_max is not None:
            switches["max_cbf_obstacles"] = known.active_set_max
        if known.active_set_distance is not None:
            switches["active_set_distance_m"] = known.active_set_distance
        if known.graph_cache:
            switches["graph_cache_enabled"] = "true"
        return switches

    runner.scenario_switches = overridden_switches
    sys.argv = [sys.argv[0]] + remaining
    return int(runner.main())


if __name__ == "__main__":
    raise SystemExit(main())
