#!/usr/bin/env python3
"""Create a recoverable source/binary snapshot for Teacher-v1 experiments."""

import argparse
import datetime as dt
import hashlib
import json
import subprocess
import tarfile
from pathlib import Path


SCRIPT = Path(__file__).resolve()
PANJIAN = SCRIPT.parents[2]
TEACHER = PANJIAN.parent
SEESM = TEACHER / "seesm_social_navigation"

SOURCE_ROOTS = (
    (PANJIAN / "planner/mpc_secbf", "panjian_ws/planner/mpc_secbf"),
    (PANJIAN / "planner/semantic_guard", "panjian_ws/planner/semantic_guard"),
    (
        PANJIAN / "planner/vomp_planner/traj_planner/include/obs_manager",
        "panjian_ws/planner/vomp_planner/traj_planner/include/obs_manager",
    ),
    (
        PANJIAN / "simulation_tools/dynamic_simulator/launch",
        "panjian_ws/simulation_tools/dynamic_simulator/launch",
    ),
    (
        PANJIAN / "simulation_tools/dynamic_simulator/scripts",
        "panjian_ws/simulation_tools/dynamic_simulator/scripts",
    ),
    (PANJIAN / "swarm_test/config", "panjian_ws/swarm_test/config"),
    (PANJIAN / "swarm_test/launch", "panjian_ws/swarm_test/launch"),
    (PANJIAN / "swarm_test/scripts", "panjian_ws/swarm_test/scripts"),
    (PANJIAN / "swarm_test/tests", "panjian_ws/swarm_test/tests"),
    (SEESM / "seesm_sim", "seesm_social_navigation/seesm_sim"),
    (SEESM / "scripts", "seesm_social_navigation/scripts"),
    (SEESM / "tests", "seesm_social_navigation/tests"),
)

REPOSITORY_SCOPES = {
    "panjian_ws": (
        "planner/mpc_secbf",
        "planner/semantic_guard",
        "planner/vomp_planner/traj_planner/include/obs_manager",
        "simulation_tools/dynamic_simulator/launch",
        "simulation_tools/dynamic_simulator/scripts",
        "swarm_test",
    ),
    "seesm_social_navigation": (
        "seesm_sim",
        "scripts",
        "tests",
    ),
}

PINNED_FILES = (
    PANJIAN / "devel/lib/mpc_secbf/mpc_secbf_node",
    PANJIAN / "swarm_test/config/common_offline_evaluation_v1.yaml",
    PANJIAN / "swarm_test/config/experiment_freezes/main_smoke.yaml",
    PANJIAN / "swarm_test/config/experiment_freezes/main_formal.yaml",
    PANJIAN / "swarm_test/config/experiment_freezes/ablation_formal.yaml",
    PANJIAN / "swarm_test/config/experiment_freezes/stress_formal.yaml",
    PANJIAN / "swarm_test/config/experiment_freezes/runtime_formal.yaml",
    TEACHER / "老师发的实验设置/最新指示/draft_V7_071.tex",
    TEACHER / "老师发的实验设置/最新指示/draft_V6.tex",
    TEACHER / "老师发的实验设置/最新指示/lu-xuran-mpc-secbf-experiment-design.md",
)

COMPONENT_REQUIRED_FILES = (
    "component-validation.csv",
    "component_trials.csv",
    "component_summary.csv",
    "meta.json",
    "COMPONENT_VALIDATION_AUDIT_PASS.txt",
)

COMPONENT_OPTIONAL_FILES = (
    "figures/E1_seesm_component_validation.png",
    "figures/E1_seesm_component_validation.pdf",
)

REGRESSION_REQUIRED_FILES = (
    "main_smoke/MAIN_SMOKE_AUDIT_PASS.txt",
    "stress_smoke/STRESS_SMOKE_AUDIT_PASS.txt",
    "完整回归结论_2026-07-28.md",
)


def sha256_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_bytes(repository, *arguments):
    result = subprocess.run(
        ["git", "-C", str(repository), *arguments],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.decode("utf-8", errors="replace"))
    return result.stdout


def repo_record(repository, scopes):
    pathspec = list(scopes) + [
        ":(exclude,glob)**/__pycache__/**",
        ":(exclude,glob)**/*.pyc",
    ]
    status = git_bytes(
        repository, "status", "--porcelain=v1", "-z", "--untracked-files=all",
        "--", *pathspec,
    )
    patch = git_bytes(
        repository, "diff", "--binary", "HEAD", "--", *pathspec,
    )
    return {
        "path": str(repository),
        "commit": git_bytes(repository, "rev-parse", "HEAD").decode().strip(),
        "tree": git_bytes(repository, "rev-parse", "HEAD^{tree}").decode().strip(),
        "branch": git_bytes(
            repository, "branch", "--show-current"
        ).decode().strip(),
        "dirty": bool(status),
        "scope": list(scopes),
        "status_sha256": sha256_bytes(status),
        "tracked_patch_sha256": sha256_bytes(patch),
    }, patch


def archive_filter(info):
    parts = Path(info.name).parts
    if "__pycache__" in parts or ".pytest_cache" in parts:
        return None
    if info.name.endswith(".pyc"):
        return None
    if "/output/" in info.name or "/outputs/" in info.name:
        return None
    return info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--component-dir", type=Path, required=True)
    parser.add_argument(
        "--regression-dir", type=Path,
        help="Regression root containing main/stress PASS sentinels",
    )
    args = parser.parse_args()
    output = args.output_dir.resolve()
    component = args.component_dir.resolve()
    regression = (
        args.regression_dir.resolve()
        if args.regression_dir else component.parent.resolve()
    )
    output.mkdir(parents=True, exist_ok=True)

    repositories = {}
    for name, repository in (
        ("panjian_ws", PANJIAN), ("seesm_social_navigation", SEESM)
    ):
        record, patch = repo_record(repository, REPOSITORY_SCOPES[name])
        patch_path = output / f"{name}_tracked_changes.patch"
        patch_path.write_bytes(patch)
        status_path = output / f"{name}_status.txt"
        status_path.write_bytes(git_bytes(
            repository, "status", "--short", "--branch",
            "--untracked-files=all",
        ))
        record["tracked_patch_file"] = patch_path.name
        record["status_file"] = status_path.name
        record["status_file_sha256"] = sha256_file(status_path)
        repositories[name] = record

    archive = output / "teacher_v1_source_snapshot.tar.gz"
    with tarfile.open(str(archive), "w:gz") as handle:
        for source, archive_name in SOURCE_ROOTS:
            handle.add(
                str(source), arcname=archive_name, recursive=True,
                filter=archive_filter,
            )

    pinned = {}
    for path in PINNED_FILES:
        path = path.resolve(strict=True)
        pinned[str(path.relative_to(TEACHER))] = {
            "sha256": sha256_file(path),
            "size_bytes": path.stat().st_size,
        }

    component_files = {}
    for relative in COMPONENT_REQUIRED_FILES:
        path = (component / relative).resolve(strict=True)
        component_files[relative] = {
            "sha256": sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
    missing_optional = []
    for relative in COMPONENT_OPTIONAL_FILES:
        path = component / relative
        if not path.exists():
            missing_optional.append(relative)
            continue
        path = path.resolve(strict=True)
        component_files[relative] = {
            "sha256": sha256_file(path),
            "size_bytes": path.stat().st_size,
        }

    regression_files = {}
    for relative in REGRESSION_REQUIRED_FILES:
        path = (regression / relative).resolve(strict=True)
        regression_files[relative] = {
            "sha256": sha256_file(path),
            "size_bytes": path.stat().st_size,
        }

    manifest = {
        "id": "teacher_v1_source_freeze_candidate_001",
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "status": "exact_snapshot_dirty_worktrees",
        "formal_execution_ready": False,
        "formal_blocker": (
            "Formal runner requires clean teacher-v1 commits; this snapshot "
            "is recoverable but preserves pre-existing dirty worktrees."
        ),
        "repositories": repositories,
        "source_snapshot": {
            "file": archive.name,
            "sha256": sha256_file(archive),
            "size_bytes": archive.stat().st_size,
            "roots": [archive_name for _, archive_name in SOURCE_ROOTS],
        },
        "pinned_files": pinned,
        "component_validation": component_files,
        "component_optional_files_missing": missing_optional,
        "regression_evidence": regression_files,
    }
    manifest_path = output / "SOURCE_FREEZE_MANIFEST.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    sentinel = output / "SOURCE_FREEZE_CREATED.txt"
    sentinel.write_text(
        "\n".join([
            "TEACHER_V1_SOURCE_FREEZE_CREATED",
            f"manifest_sha256={sha256_file(manifest_path)}",
            f"source_snapshot_sha256={manifest['source_snapshot']['sha256']}",
            "formal_execution_ready=false",
            "",
        ]),
        encoding="utf-8",
    )
    print(manifest_path)
    print(sentinel)


if __name__ == "__main__":
    main()
