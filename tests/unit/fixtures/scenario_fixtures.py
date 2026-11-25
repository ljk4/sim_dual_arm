"""Synthetic scenario trees for binding tests.

The real recipes and extension packages are deliberately not used here: phase
02 must not imply that the example binding is runnable yet.
"""

from __future__ import annotations

import hashlib
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[3]
SHARED = ROOT / "shared"
MARVIN_STACK_PROFILES = SHARED / "scenarios/marvin/mujoco/stack/profiles"
MARVIN_SHARED_PROFILES = SHARED / "robots/marvin/profiles"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

RECIPES = (
    "simulation_teacher",
    "simulation_policy",
    "runtime_teacher",
    "runtime_policy",
    "campaign",
    "dataset",
    "training",
    "policy_service",
)


@dataclass
class ScenarioTree:
    root: Path
    binding_path: Path
    document: dict[str, Any]

    def write(self, document: dict[str, Any] | None = None) -> Path:
        if document is not None:
            self.document = document
        self.binding_path.write_text(
            yaml.safe_dump(self.document, sort_keys=False), encoding="utf-8"
        )
        return self.binding_path

    def copy(self) -> dict[str, Any]:
        return yaml.safe_load(yaml.safe_dump(self.document))


def base_binding() -> dict[str, Any]:
    return {
        "robot_id": "synthetic_sim",
        "task_id": "stack",
        "description": "Synthetic phase-02 binding",
        "backend": {"kind": "simulation", "implementation": "mujoco", "task_type": "stack"},
        "runtime": {"implementation": "ros2"},
        "learning": {"implementation": "lerobot", "adapter": "act"},
        "profiles": {
            "robot": "profiles/robot.json",
            "observation": "profiles/observation.json",
            "action": "profiles/action.json",
        },
        "recipes": {name: f"configs/{name}.yaml" for name in RECIPES},
        "interfaces": {
            "common_ros": ["robot_api_msgs"],
            "extension_ros": ["extensions/ros/marvin_teacher_api_msgs"],
            "teacher_api": "marvin_teacher_api_msgs.teacher_chunk",
            "policy_api": "bimanual.policy",
        },
        "source_roots": [],
        "artifact_roots": [],
    }


def _copy_profile_documents(root: Path) -> None:
    profiles = root / "profiles"
    profiles.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(
        MARVIN_STACK_PROFILES / "robot/profile.json",
        profiles / "robot.json",
    )
    shutil.copyfile(
        MARVIN_SHARED_PROFILES / "observations/marvin_rgb_joint.json",
        profiles / "observation.json",
    )
    shutil.copyfile(
        MARVIN_SHARED_PROFILES / "actions/marvin_joint_position.json",
        profiles / "action.json",
    )


def build_scenario(root: Path, document: dict[str, Any] | None = None) -> ScenarioTree:
    root.mkdir(parents=True, exist_ok=True)
    _copy_profile_documents(root)
    configs = root / "configs"
    configs.mkdir(exist_ok=True)
    for name in RECIPES:
        (configs / f"{name}.yaml").write_text("{}\n", encoding="utf-8")
    extension = root / "extensions/ros/marvin_teacher_api_msgs"
    (extension / "msg").mkdir(parents=True, exist_ok=True)
    (extension / "srv").mkdir(parents=True, exist_ok=True)
    (extension / "package.xml").write_text(
        "<package format='3'><name>marvin_teacher_api_msgs</name></package>\n", encoding="utf-8"
    )
    (extension / "msg/TeacherCapability.msg").write_text(
        "string ID=marvin_teacher_api_msgs.teacher_chunk\n", encoding="utf-8"
    )
    (extension / "srv/GetTeacherChunk.srv").write_text("---\n", encoding="utf-8")
    tree = ScenarioTree(root, root / "binding.yaml", document or base_binding())
    tree.write()
    return tree
