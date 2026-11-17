from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.config.backend_config import (  # noqa: E402
    BackendConfigError,
    load_backend_config,
)

CONFIG_PATH = ROOT / "shared/scenarios/panda_sim/stack/configs/simulation/backend/backend.yaml"
DEFAULT_DOCUMENT = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def write_document(tmp_path: Path, document: dict) -> Path:
    directory = tmp_path / "simulation" / "mujoco" / "configs"
    directory.mkdir(parents=True)
    path = directory / "backend.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    (directory / "panda_stack.yaml").write_text("scene: x\n", encoding="utf-8")
    (directory / "panda_control.yaml").write_text("control: x\n", encoding="utf-8")
    (directory / "panda_stream.yaml").write_text("stream: x\n", encoding="utf-8")
    (directory / "panda_stack_task.yaml").write_text("task: x\n", encoding="utf-8")
    (directory / "kinematics_check.yaml").write_text("ik: x\n", encoding="utf-8")
    profile = directory / "profile.json"
    profile.write_text("{}\n", encoding="utf-8")
    schemas = directory / "schemas"
    schemas.mkdir()
    return path


def document_with_paths(tmp_path: Path) -> dict:
    document = copy.deepcopy(DEFAULT_DOCUMENT)
    document["robot"].update(
        {
            "profile": "profile.json",
            "schemas": "schemas",
            "scene": "panda_stack.yaml",
            "control": "panda_control.yaml",
            "stream": "panda_stream.yaml",
        }
    )
    return document


def test_valid_config_resolves_every_path_against_the_config_file(tmp_path):
    path = write_document(tmp_path, document_with_paths(tmp_path))
    config = load_backend_config(path)
    assert config.source == path.resolve()
    assert config.profile_path == (path.parent / "profile.json").resolve()
    assert config.schemas_path == (path.parent / "schemas").resolve()
    assert config.scene_path == (path.parent / "panda_stack.yaml").resolve()
    assert config.control_path == (path.parent / "panda_control.yaml").resolve()
    assert config.stream_path == (path.parent / "panda_stream.yaml").resolve()
    assert config.namespace == "/robot"
    assert config.realtime_speed == 1.0
    assert config.capture_hz == 20.0
    assert config.owner_queue_capacity == 64


def test_repository_config_loads():
    config = load_backend_config(CONFIG_PATH)
    assert config.scene_path.is_file()
    assert config.profile_path.is_file()
    assert config.log.max_event_bytes <= config.log.max_file_bytes


def test_visualization_can_be_disabled_without_disabling_cameras(tmp_path):
    document = document_with_paths(tmp_path)
    document["robot"]["visualization_enabled"] = False
    config = load_backend_config(write_document(tmp_path, document))
    assert config.visualization_enabled is False
    assert config.capture_hz == 20.0


def test_unknown_key_is_rejected(tmp_path):
    document = document_with_paths(tmp_path)
    document["run"]["unknown"] = 1
    with pytest.raises(BackendConfigError) as caught:
        load_backend_config(write_document(tmp_path, document))
    assert caught.value.field == "run.unknown"


def test_unknown_nested_key_is_rejected(tmp_path):
    document = document_with_paths(tmp_path)
    document["logging"]["extra"] = True
    with pytest.raises(BackendConfigError) as caught:
        load_backend_config(write_document(tmp_path, document))
    assert caught.value.field == "logging.extra"


def test_duplicate_key_is_rejected(tmp_path):
    path = write_document(tmp_path, document_with_paths(tmp_path))
    text = path.read_text(encoding="utf-8").replace(
        "  capture_hz: 20.0\n", "  capture_hz: 20.0\n  capture_hz: 10.0\n"
    )
    path.write_text(text, encoding="utf-8")
    with pytest.raises(BackendConfigError) as caught:
        load_backend_config(path)
    assert caught.value.field == "capture_hz"


@pytest.mark.parametrize("field,value", [
    ("seed", True), ("capture_hz", float("nan")),
    ("realtime_speed", 0.0), ("max_catchup_steps", 0),
])
def test_run_values_are_strict(tmp_path, field, value):
    document = document_with_paths(tmp_path)
    document["run"][field] = value
    with pytest.raises(BackendConfigError) as caught:
        load_backend_config(write_document(tmp_path, document))
    assert caught.value.field == f"run.{field}"


@pytest.mark.parametrize("value", [0, 5000, True])
def test_queue_bounds_are_strict(tmp_path, value):
    document = document_with_paths(tmp_path)
    document["queues"]["owner"] = value
    with pytest.raises(BackendConfigError, match="queues.owner"):
        load_backend_config(write_document(tmp_path, document))


def test_boolean_cannot_stand_in_for_a_number(tmp_path):
    document = document_with_paths(tmp_path)
    document["limits"]["shutdown_s"] = True
    with pytest.raises(BackendConfigError, match="limits.shutdown_s"):
        load_backend_config(write_document(tmp_path, document))


def test_log_cross_field_bounds_are_rejected(tmp_path):
    document = document_with_paths(tmp_path)
    document["logging"]["max_event_bytes"] = 100
    with pytest.raises(BackendConfigError, match="logging"):
        load_backend_config(write_document(tmp_path, document))


def test_missing_referenced_file_is_rejected(tmp_path):
    document = document_with_paths(tmp_path)
    document["robot"]["scene"] = "missing.yaml"
    with pytest.raises(BackendConfigError, match="robot.scene: must be an existing file"):
        load_backend_config(write_document(tmp_path, document))


@pytest.mark.parametrize(
    "namespace",
    ["", "robot", "/", "/robot/", "/robot//arm", "/robot-arm"],
)
def test_namespace_must_be_a_non_empty_ros_name(tmp_path, namespace):
    document = document_with_paths(tmp_path)
    document["robot"]["namespace"] = namespace
    with pytest.raises(BackendConfigError, match="robot.namespace"):
        load_backend_config(write_document(tmp_path, document))
