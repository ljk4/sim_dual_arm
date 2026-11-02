from __future__ import annotations

import copy
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "components/backend_sim/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.scene_config import load_scene_config  # noqa: E402
from robot_backend_sim.task_config import (  # noqa: E402
    StackTaskConfigError,
    load_task_config,
    validate_scene_geometry,
)

CONFIG_PATH = ROOT / "configs/simulation/panda_stack_task.yaml"


def document() -> dict:
    return copy.deepcopy(yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8")))


def write_config(tmp_path: Path, value: dict) -> Path:
    directory = tmp_path / "configs" / "simulation"
    directory.mkdir(parents=True)
    (directory / "kinematics_check.yaml").write_text("dummy: true\n", encoding="utf-8")
    path = directory / "panda_stack_task.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    return path


def test_repository_task_config_has_all_thresholds_and_capacity():
    config = load_task_config(CONFIG_PATH)
    assert config.success.stable_duration_s == 2.0
    assert config.success.max_task_sim_duration_s == 30.0
    assert config.success.support_xy_margin_m == 0.002
    assert config.success.support_height_tolerance_m == 0.005
    assert config.teacher.chunk_size == 60
    assert config.teacher.action_period_ns == 50_000_000
    assert config.capacity.expected_chunks >= 120
    assert config.capacity.max_episode_chunks >= config.capacity.expected_chunks
    assert config.capacity.backend_chunk_history >= config.capacity.max_episode_chunks
    assert len(config.config_digest) == 64
    assert len(config.file_sha256) == 64


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("stable_duration_s", 1.0, "stable_duration_s"),
        ("max_task_sim_duration_s", 31.0, "max_task_sim_duration_s"),
        ("support_xy_margin_m", -0.001, "support_xy_margin_m"),
        ("min_support_normal_force_n", 0.0, "min_support_normal_force_n"),
        ("min_gripper_opening_m", 0.081, "min_gripper_opening_m"),
        ("min_vertical_normal_component", 0.0, "min_vertical_normal_component"),
    ],
)
def test_invalid_success_threshold_is_rejected(tmp_path, field, value, message):
    value_document = document()
    value_document["success"][field] = value
    with pytest.raises(StackTaskConfigError, match=message):
        load_task_config(write_config(tmp_path, value_document))


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("expected_chunks", 64, "expected_chunks"),
        ("max_episode_chunks", 119, "max_episode_chunks"),
        ("max_teacher_requests", 64, "max_teacher_requests"),
        ("rpc_cache_entries", 64, "rpc_cache_entries"),
        ("backend_chunk_history", 119, "backend_chunk_history"),
    ],
)
def test_capacity_cannot_truncate_a_120_chunk_task(tmp_path, field, value, message):
    value_document = document()
    value_document["capacity"][field] = value
    with pytest.raises(StackTaskConfigError, match=message):
        load_task_config(write_config(tmp_path, value_document))


def test_unknown_key_is_rejected(tmp_path):
    value_document = document()
    value_document["teacher"]["unknown"] = 1
    with pytest.raises(StackTaskConfigError, match="teacher.unknown"):
        load_task_config(write_config(tmp_path, value_document))


def test_phase_minimum_must_cover_success_window(tmp_path):
    value_document = document()
    value_document["teacher"]["phase_min_duration_s"]["stable"] = 0.5
    with pytest.raises(StackTaskConfigError, match="phase_min_duration_s.stable"):
        load_task_config(write_config(tmp_path, value_document))


def test_wrong_geometry_is_rejected_against_configured_cube(tmp_path):
    value_document = document()
    value_document["objects"]["source_half_extents_m"] = [0.02, 0.02, 0.03]
    with pytest.raises(StackTaskConfigError, match="source_half_extents_m"):
        load_task_config(write_config(tmp_path, value_document))


def test_task_geometry_must_match_the_frozen_scene():
    scene_config = load_scene_config(ROOT / "configs/simulation/panda_stack.yaml")
    config = load_task_config(CONFIG_PATH)
    validate_scene_geometry(config, scene_config)
    wrong = replace(config, source_half_extents_m=(0.021, 0.02, 0.02))
    with pytest.raises(StackTaskConfigError, match="half extents"):
        validate_scene_geometry(wrong, scene_config)
