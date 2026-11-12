"""Strict optional-follower selection in the runnable backend configuration."""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/panda_stack"
PROFILES = SCENARIO / "profiles"
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.config.backend_config import (  # noqa: E402
    BackendConfigError,
    load_backend_config,
)
from robot_backend_sim.config.follower_config import load_follower_config  # noqa: E402

BASE = SCENARIO / "configs/simulation/backend/backend.yaml"
FOLLOWER = ROOT / "tests/fixtures/simulation/backend_follower.yaml"
FOLLOWER_PARAMS = PROFILES / "control/panda_follower_v1.yaml"


def write_variant(tmp_path: Path, mutate) -> Path:
    document = copy.deepcopy(yaml.safe_load(FOLLOWER.read_text(encoding="utf-8")))
    document["robot"].update(
        {
            "profile": str(PROFILES / "robots/panda_stack_v1/profile.json"),
            "schemas": str(ROOT / "shared/common/interfaces/schemas"),
            "scene": str(SCENARIO / "configs/simulation/scene/panda_stack.yaml"),
            "control": str(PROFILES / "control/panda_control_v1.yaml"),
            "stream": str(PROFILES / "control/panda_stream_v1.yaml"),
            "follower": str(FOLLOWER_PARAMS),
        }
    )
    mutate(document)
    path = tmp_path / "backend_follower.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def test_base_config_keeps_follower_disabled():
    config = load_backend_config(BASE)
    assert config.follower_path is None
    assert config.follower_reference_queue_capacity == config.accepted_reference_queue_capacity


def test_follower_config_resolves_relative_follower_file_and_queue():
    config = load_backend_config(FOLLOWER)
    assert config.follower_path == FOLLOWER_PARAMS.resolve()
    assert config.follower_reference_queue_capacity == 64
    assert load_follower_config(config.follower_path).target_count == 10
    expected_scene = SCENARIO / "configs/simulation/scene/panda_stack.yaml"
    assert config.scene_path == expected_scene.resolve()


def test_missing_follower_file_is_rejected_not_disabled(tmp_path):
    path = write_variant(
        tmp_path, lambda document: document["robot"].update({"follower": "missing.yaml"})
    )
    with pytest.raises(BackendConfigError) as caught:
        load_backend_config(path)
    assert caught.value.field == "robot.follower"


def test_unknown_robot_key_still_rejected(tmp_path):
    path = write_variant(
        tmp_path, lambda document: document["robot"].update({"follower_extra": "x"})
    )
    with pytest.raises(BackendConfigError) as caught:
        load_backend_config(path)
    assert caught.value.field == "robot.follower_extra"


def test_follower_queue_must_be_positive(tmp_path):
    path = write_variant(
        tmp_path, lambda document: document["queues"].update({"follower_references": 0})
    )
    with pytest.raises(BackendConfigError) as caught:
        load_backend_config(path)
    assert caught.value.field == "queues.follower_references"
