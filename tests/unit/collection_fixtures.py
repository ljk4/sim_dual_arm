"""Synthetic collection scenario trees for record/campaign/worker tests.

The binding, recipes and campaign ledger live in a temporary project tree.  No
real campaign file, seed plan or backend is touched.
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from scenario_fixtures import ROOT, build_scenario, sha  # noqa: F401 - re-exported

ROBOT = "synthetic_sim"
TASK = "stack"
SCENARIO_ID = f"{ROBOT}/{TASK}"


@dataclass
class CollectionTree:
    repo: Path
    shared: Path
    scenario: Any
    campaign_recipe: Path
    runtime_recipe: Path
    backend_recipe: Path
    campaign: dict[str, Any]


def campaign_document(repo: Path, recipe_paths: list[Path]) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "budget": {
            "pilot_seed_first": 1000,
            "pilot_seed_last": 1009,
            "pilot_max_attempts": 10,
            "pilot_act_reserved_attempts": 2,
            "pilot_teacher_seed_last": 1007,
            "pilot_label_replay_max": 2,
            "formal_seed_first": 10000,
            "formal_seed_last": 10099,
            "formal_max_attempts": 100,
            "formal_target_qualified": 50,
            "formal_validation_qualified_indices": [9, 19, 29, 39, 49],
            "test_seed_first": 20000,
            "test_seed_last": 20019,
            "test_max_attempts": 20,
            "evaluation_max_attempts": 20,
            "max_raw_bytes": 100_000_000,
            "reserve_bytes": 1_000,
            "estimated_attempt_bytes": 1_000,
            "min_free_bytes": 0,
        },
        "paths": {
            "attempts_root": "var/synthetic_sim/stack/datasets/raw",
            "ledger": "var/synthetic_sim/stack/logs/collection/attempts.jsonl",
            "lock": "var/synthetic_sim/stack/logs/collection/.ledger.lock",
            "run_lock": "var/synthetic_sim/stack/logs/collection/.run.lock",
            "formal_select": "var/synthetic_sim/stack/datasets/raw/formal-select.json",
            "required_paths": [str(path.relative_to(repo)) for path in recipe_paths],
            "sdk_marker": "build/ros-dev/contracts.json",
        },
        "launch": {"command": [], "timeout_s": 10.0},
        "quality": {"sidecar": "export/quality.json"},
    }


def _write_runtime_recipe(root: Path) -> Path:
    (root / "configs/control.yaml").write_text("schema_version: 1\n", encoding="utf-8")
    (root / "configs/follower.yaml").write_text("schema_version: 1\n", encoding="utf-8")
    document = {
        "schema_version": 1,
        "robot_profile": "../profiles/robot.json",
        "observation_schema": "../profiles/observation.json",
        "action_schema": "../profiles/action.json",
        "control_constraints": "control.yaml",
        "follower_config": "follower.yaml",
        "robot_namespace": "/robot",
        "action_source": "teacher",
        "teacher_api": "marvin_stack_api_msgs.teacher_chunk.v1",
        "reset": {"enabled": True, "seed": 0},
    }
    path = root / "configs/runtime_teacher.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def _write_backend_recipe(root: Path) -> Path:
    for name in ("scene.yaml", "stream.yaml", "task.yaml"):
        (root / "configs" / name).write_text("schema_version: 1\n", encoding="utf-8")
    document = {
        "robot": {
            "namespace": "/robot",
            "profile": "../profiles/robot.json",
            "schemas": "../../../../common/interfaces/schemas",
            "scene": "scene.yaml",
            "control": "control.yaml",
            "stream": "stream.yaml",
            "follower": "follower.yaml",
            "task": "task.yaml",
            "render_quality": {"enabled": True},
            "task_mode": "teacher",
        },
        "run": {"realtime_speed": 1.0},
    }
    path = root / "configs/simulation_teacher.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def build_collection_tree(tmp_path: Path) -> CollectionTree:
    repo = tmp_path / "repo"
    shared = repo / "shared"
    scenario = build_scenario(shared / "scenarios" / SCENARIO_ID)
    shutil.copytree(
        ROOT / "shared/common/interfaces/schemas", shared / "common/interfaces/schemas"
    )
    scenario.document["interfaces"]["common_ros"] = ["robot_api_msgs", "sim_api_msgs"]
    scenario.write()
    runtime_recipe = _write_runtime_recipe(scenario.root)
    backend_recipe = _write_backend_recipe(scenario.root)
    campaign_recipe = scenario.root / "configs/campaign.yaml"
    campaign = campaign_document(
        repo, [runtime_recipe, backend_recipe, campaign_recipe]
    )
    campaign_recipe.write_text(yaml.safe_dump(campaign, sort_keys=False), encoding="utf-8")
    ros = shared / "common/interfaces/ros"
    (ros / "robot_api_msgs/srv").mkdir(parents=True, exist_ok=True)
    (ros / "sim_api_msgs/srv").mkdir(parents=True, exist_ok=True)
    (repo / "build/ros-dev").mkdir(parents=True, exist_ok=True)
    (repo / "build/ros-dev/contracts.json").write_text("{}\n", encoding="utf-8")
    return CollectionTree(
        repo, shared, scenario, campaign_recipe, runtime_recipe, backend_recipe, campaign
    )
