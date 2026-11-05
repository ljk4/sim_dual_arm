from __future__ import annotations

import copy
import math
import sys
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.motion_config import (  # noqa: E402
    CONTROL_JOINT_NAMES,
    MotionConfigError,
    load_control_config,
    parse_control_config,
)

CONFIG_PATH = ROOT / "simulation/mujoco/configs/control/panda_control.yaml"
DEFAULT_DOCUMENT = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def write_document(tmp_path: Path, document: object, name: str = "panda_control.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def changed(**changes: object) -> dict[str, object]:
    document = copy.deepcopy(DEFAULT_DOCUMENT)
    document.update(changes)
    return document


def test_candidate_initial_values_and_si_configuration():
    config = load_control_config(CONFIG_PATH)
    assert config.joint_names == CONTROL_JOINT_NAMES
    assert config.max_accelerations == ((1.0,) * 7 + (0.5,) + (1.0,) * 7 + (0.5,))
    assert config.position_tolerances == ((0.012,) * 7 + (0.002,) + (0.012,) * 7 + (0.002,))
    assert config.velocity_tolerances == ((0.025,) * 7 + (0.004,) + (0.025,) * 7 + (0.004,))
    assert config.stable_duration_s == pytest.approx(0.2)
    assert config.stable_duration_ns == 200_000_000
    assert config.settle_timeout_ns == 2_000_000_000
    assert config.motion_wall_timeout_ns == 10_000_000_000
    assert config.stop_wall_timeout_ns == 3_000_000_000
    assert config.contact_force_min_n == pytest.approx(0.5)
    assert config.finger_velocity_max_m_s == pytest.approx(0.003)


def test_config_is_deeply_immutable_and_round_trips_through_a_fresh_snapshot():
    config = load_control_config(CONFIG_PATH)
    snapshot = config.to_document()
    assert snapshot["joint_names"] == list(CONTROL_JOINT_NAMES)
    assert snapshot["max_accelerations"][7] == 0.5
    snapshot["joint_names"].append("mutated")
    snapshot["max_accelerations"][0] = 99.0
    assert config.to_document()["joint_names"] == list(CONTROL_JOINT_NAMES)
    assert config.max_accelerations[0] == 1.0

    restored = parse_control_config(config.to_document(), "<snapshot>")
    assert restored == config
    with pytest.raises(FrozenInstanceError):
        config.stable_duration_s = 0.1


@pytest.mark.parametrize(
    ("mutate", "field"),
    (
        (lambda d: d.update({"unknown": 1}), "unknown"),
        (lambda d: d.pop("stable_duration_s"), "stable_duration_s"),
        (lambda d: d["joint_names"].reverse(), "joint_names"),
        (lambda d: d["joint_names"].pop(), "joint_names"),
        (lambda d: d["max_accelerations"].pop(), "max_accelerations"),
        (lambda d: d.__setitem__("max_accelerations", True), "max_accelerations"),
        (lambda d: d["position_tolerances"].__setitem__(3, True), "position_tolerances[3]"),
        (lambda d: d["velocity_tolerances"].__setitem__(4, math.nan), "velocity_tolerances[4]"),
        (lambda d: d.__setitem__("stable_duration_s", 0.0), "stable_duration_s"),
        (lambda d: d.__setitem__("settle_timeout_s", -1.0), "settle_timeout_s"),
        (lambda d: d.__setitem__("motion_wall_timeout_s", True), "motion_wall_timeout_s"),
        (lambda d: d.__setitem__("stop_wall_timeout_s", math.inf), "stop_wall_timeout_s"),
        (lambda d: d.__setitem__("stop_wall_timeout_s", 1e300), "stop_wall_timeout_s"),
        (lambda d: d.__setitem__("contact_force_min_n", 0.0), "contact_force_min_n"),
        (lambda d: d.__setitem__("finger_velocity_max_m_s", -0.1), "finger_velocity_max_m_s"),
    ),
)
def test_invalid_documents_are_rejected_with_source_and_field(tmp_path: Path, mutate, field: str):
    document = copy.deepcopy(DEFAULT_DOCUMENT)
    mutate(document)
    path = write_document(tmp_path, document)
    with pytest.raises(MotionConfigError) as caught:
        load_control_config(path)
    assert caught.value.field == field
    assert str(path) in str(caught.value)
    assert field in str(caught.value)


def test_duplicate_yaml_key_and_non_mapping_are_rejected(tmp_path: Path):
    path = tmp_path / "duplicate.yaml"
    path.write_text(
        "joint_names: []\njoint_names: []\nmax_accelerations: []\n",
        encoding="utf-8",
    )
    with pytest.raises(MotionConfigError) as caught:
        load_control_config(path)
    assert caught.value.field == "joint_names"
    assert "duplicate" in caught.value.message

    with pytest.raises(MotionConfigError) as caught:
        parse_control_config([], "<root>")
    assert caught.value.field == "<root>"


def test_unknown_and_bool_are_rejected_before_partial_configuration():
    document = changed(stable_duration_s=True)
    with pytest.raises(MotionConfigError) as caught:
        parse_control_config(document, "inline")
    assert caught.value.field == "stable_duration_s"
    assert "booleans" in caught.value.message
