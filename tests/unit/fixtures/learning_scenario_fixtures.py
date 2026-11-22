"""Shared construction helpers for learning scenario-selection tests.

The helpers build synthetic bindings, recipes and CLI argument lists; they do
not assert behaviour.  Tests that need a real bundle still create it through
``act_fixtures``.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import yaml
from scenario_fixtures import ROOT, build_scenario, sha  # noqa: F401 - re-exported

SHA = "a" * 64


@dataclass
class Audit:
    synthetic: bool = True


@dataclass
class Tree:
    repo: Path
    shared: Path
    scenario: object
    dataset_recipe: Path
    training_recipe: Path
    service_recipe: Path


def write_dataset_recipe(scenario, tmp_path: Path, *, task_text: str) -> Path:
    document = yaml.safe_load(
        (ROOT / "shared/scenarios/marvin_sim/stack/configs/learning/act_dataset.yaml").read_text()
    )
    document["profile"] = str(scenario.root / "profiles/robot.json")
    document["select"] = str(tmp_path / "select.json")
    document["output_root"] = str(tmp_path / "data")
    document["cache_root"] = str(tmp_path / "cache")
    document["task_text"] = task_text
    path = scenario.root / "configs/dataset.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def write_training_recipe(
    scenario, tmp_path: Path, dataset_recipe: Path, *, mode: str = "smoke"
) -> Path:
    document = yaml.safe_load(
        (ROOT / "shared/scenarios/marvin_sim/stack/configs/learning/act_training.yaml").read_text()
    )
    document["mode"] = mode
    document["cache_root"] = str(tmp_path / "cache")
    document["data"] = {
        "config": str(dataset_recipe),
        "root": str(tmp_path / "data"),
        "manifest": "manifest.json",
        "train_dir": "train",
        "validation_dir": "validation",
    }
    document["model"]["weights"] = {"path": str(tmp_path / "resnet18.pth"), "sha256": ""}
    document["checkpoint"] = {"root": str(tmp_path / "models")}
    document["logging"]["root"] = str(tmp_path / "logs")
    path = scenario.root / "configs/training.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def write_service_recipe(scenario, *, task_text: str = "scenario service task") -> Path:
    document = yaml.safe_load(
        (ROOT / "shared/scenarios/marvin_sim/stack/configs/learning/policy.yaml").read_text()
    )
    document["service"]["task_text"] = task_text
    path = scenario.root / "configs/policy_service.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def make_tree(tmp_path: Path, *, adapter: str = "act") -> Tree:
    repo = tmp_path / "repo"
    shared = repo / "shared"
    scenario = build_scenario(shared / "scenarios/synthetic_sim/stack")
    shutil.copytree(
        ROOT / "shared/common/interfaces/schemas", shared / "common/interfaces/schemas"
    )
    scenario.document["learning"] = {"implementation": "lerobot", "adapter": adapter}
    scenario.document["artifact_roots"] = [str(tmp_path)]
    scenario.write()
    dataset = write_dataset_recipe(scenario, tmp_path, task_text="scenario dataset task")
    training = write_training_recipe(scenario, tmp_path, dataset)
    service = write_service_recipe(scenario)
    return Tree(repo, shared, scenario, dataset, training, service)


def fake_convert(capture: dict):
    def convert(**kwargs):
        capture.update(kwargs)
        return {
            "status": "complete",
            "synthetic": True,
            "splits": {
                "training": {"episode_count": 45, "frame_count": 1},
                "validation": {"episode_count": 5, "frame_count": 1},
            },
        }

    return convert


def fake_train(capture: dict):
    def train(config, **kwargs):
        capture["config"] = config
        capture["kwargs"] = kwargs
        return {
            "reason": "completed",
            "model_name": "act_20260101_000000",
            "model_dir": str(config.checkpoint.root / "act_20260101_000000"),
            "mode": config.mode,
            "steps": 0,
            "device": config.device,
            "best_step": None,
            "best_val_l1": None,
            "duration_seconds": 0.0,
        }

    return train


def scenario_args(tree: Tree, command: str, *extra: str) -> list[str]:
    return [
        "--project-root",
        str(tree.repo),
        command,
        "--robot",
        "synthetic_sim",
        "--task",
        "stack",
        "--shared-root",
        str(tree.shared),
        *extra,
    ]


def convert_args(tree: Tree, tmp_path: Path) -> list[str]:
    return scenario_args(
        tree,
        "convert",
        "--select",
        str(tmp_path / "select.json"),
        "--output-root",
        str(tmp_path / "out"),
        "--synthetic",
    )


def run_dispatcher(tmp_path: Path, script_name: str, *arguments: str) -> list[str]:
    """Run a learning script with stub setup/python3 so only argument injection is observed."""
    setup = tmp_path / "setup.sh"
    setup.write_text(f'export PROJECT_ROOT="{ROOT}"\n', encoding="utf-8")
    binary = tmp_path / "bin"
    binary.mkdir(exist_ok=True)
    capture = tmp_path / "argv.txt"
    capture.unlink(missing_ok=True)
    stub = binary / "python3"
    stub.write_text(
        "#!/bin/sh\n"
        'case "$*" in\n'
        "  *policy_sdk.py*) exit 0 ;;\n"
        "esac\n"
        'printf "%s\\n" "$@" >> "$CAPTURE"\n'
        "exit 0\n",
        encoding="utf-8",
    )
    stub.chmod(0o755)
    environment = os.environ | {
        "PROJECT_CONTAINER": "1",
        "PROJECT_SETUP": str(setup),
        "PATH": f"{binary}:{os.environ['PATH']}",
        "CAPTURE": str(capture),
    }
    result = subprocess.run(
        ["bash", str(ROOT / "learning/lerobot/scripts" / script_name), *arguments],
        env=environment,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr
    if not capture.is_file():
        return []
    return capture.read_text(encoding="utf-8").splitlines()
