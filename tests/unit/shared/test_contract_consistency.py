"""Text-level consistency checks; deliberately runnable without generated SDKs.

Pairs: RunContext -> RunContext.msg + policy.RunContext; Target -> policy.JointTarget;
ControlEvent/ExecutionFeedback/FingerFeedback/GripperFeedback/StreamChunkStatus ->
matching ROS messages; RobotProfile/JointSpec/CameraSpec -> profile JSON documents;
RecordingResult/RuntimeResult/TaskResult -> episode manifest sections.
Action/observation schema files describe bindings, not ActionChunk/Observation
payloads, so those are validated as profiles rather than falsely paired.
"""
from __future__ import annotations

import json
import re
from copy import deepcopy
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator
from robot_contracts.episodes import RecordingResult, RuntimeResult, TaskResult
from robot_contracts.execution_feedback import (
    MAX_FINGER_INDEX,
    MAX_FINGER_OBJECTS,
    MAX_GRIPPER_FEEDBACK,
    MAX_GRIPPER_FINGERS,
    ControlEvent,
    ExecutionFeedback,
    FingerFeedback,
    GripperFeedback,
    StreamChunkStatus,
)
from robot_contracts.models import RunContext, Target
from robot_contracts.profiles import load_profile
from robot_contracts.task_state import TaskState

ROOT = Path(__file__).resolve().parents[3]
SCHEMAS = ROOT / "shared/common/interfaces/schemas"


def _document(name):
    return json.loads((SCHEMAS / name).read_text())


def _ros_fields(text):
    fields = {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or line == "---":
            continue
        field_type, name = line.split()[:2]
        if "=" not in name:  # Ignore IDL constants, never treat them as fields.
            assert name not in fields, name
            fields[name] = field_type
    return fields


def _proto_fields(text, message):
    text = re.sub(r"//[^\n]*", "", text)
    body = re.search(r"\bmessage\s+" + message + r"\s*\{([^{}]*)\}", text)
    assert body is not None, message
    matches = re.findall(r"\b(?:repeated\s+)?([\w.]+)\s+(\w+)\s*=\s*\d+\s*;", body[1])
    result = {name: kind for kind, name in matches}
    assert len(result) == len(matches)
    return result


def _same_fields(expected, actual):
    assert set(expected) == set(actual), (
        f"missing={set(expected) - set(actual)}, extra={set(actual) - set(expected)}"
    )


@pytest.mark.parametrize("model", [RunContext, ControlEvent, ExecutionFeedback,
                                    FingerFeedback, GripperFeedback, StreamChunkStatus])
def test_ros_message_fields_match_models(model):
    text = (
        ROOT / f"shared/common/interfaces/ros/robot_api_msgs/msg/{model.__name__}.msg"
    ).read_text()
    actual = _ros_fields(text)
    _same_fields(model.model_fields, actual)
    schema = model.model_json_schema()
    # Sentinel zero/empty-string encodings for optional fields are intentional in ROS.
    for name, field in schema["properties"].items():
        kind = field.get("type")
        ros_type = actual[name]
        if kind == "array":
            assert "[" in ros_type
        elif kind == "boolean":
            assert ros_type == "bool"
        elif kind == "string":
            assert ros_type == "string"
        elif kind == "integer":
            assert re.fullmatch(r"u?int(?:8|16|32|64)", ros_type)
        elif kind == "number":
            assert ros_type == "float64"


def test_ros_bounded_feedback_bounds_match_the_contract():
    base = ROOT / "shared/common/interfaces/ros/robot_api_msgs/msg"
    execution = _ros_fields((base / "ExecutionFeedback.msg").read_text())
    gripper = _ros_fields((base / "GripperFeedback.msg").read_text())
    finger = _ros_fields((base / "FingerFeedback.msg").read_text())
    assert execution["grippers"] == (
        f"robot_api_msgs/GripperFeedback[<={MAX_GRIPPER_FEEDBACK}]"
    )
    assert gripper["fingers"] == (
        f"robot_api_msgs/FingerFeedback[<={MAX_GRIPPER_FINGERS}]"
    )
    assert gripper["side"] == "string"
    assert finger["side"] == "string"
    assert finger["finger_index"] == "uint8"
    assert MAX_FINGER_INDEX == MAX_GRIPPER_FINGERS
    assert finger["object_names"] == f"string[<={MAX_FINGER_OBJECTS}]"


@pytest.mark.parametrize(("model", "message", "rename"), [
    (RunContext, "RunContext", {}),
    (Target, "JointTarget", {"time_ns": "time_from_observation_ns"}),
])
def test_proto_fields_match_models(model, message, rename):
    text = (ROOT / "shared/common/interfaces/policy/policy.proto").read_text()
    fields = _proto_fields(text, message)
    _same_fields({rename.get(name, name) for name in model.model_fields}, fields)
    for name, field in model.model_json_schema()["properties"].items():
        expected_type = {"string": "string", "integer": "uint64", "array": "double"}[field["type"]]
        assert fields[rename.get(name, name)] == expected_type


def _resolved(field, schema):
    if "$ref" in field:
        return schema["$defs"][field["$ref"].split("/")[-1]]
    return field


def _schema_pair(model, declared, *, excluded=(), optional=(), enum_differences=None):
    generated = model.model_json_schema()
    props = {key: value for key, value in generated["properties"].items() if key not in excluded}
    _same_fields(props, declared["properties"])
    assert (set(generated.get("required", [])) - set(excluded)) | set(optional) == set(
        declared["required"]
    )
    for name, raw in props.items():
        field = _resolved(raw, generated)
        other = declared["properties"][name]
        if "enum" in field or "enum" in other:
            difference = (enum_differences or {}).get(name)
            actual = (field.get("enum"), other.get("enum"))
            assert actual == difference if difference else actual[0] == actual[1]
        if "type" in field and "type" in other:
            assert field["type"] == other["type"], name


@pytest.mark.parametrize(("model", "section", "optional"), [
    (RecordingResult, "recording", ("failure_reason",)),
    (RuntimeResult, "runtime", ("reason",)),
    (TaskResult, "task", ("outcome", "reason")),
])
def test_episode_result_schema_matches_models(model, section, optional):
    # Models default empty reasons/outcomes for construction; serialized manifests require them.
    _schema_pair(
        model, _document("episode_manifest.schema.json")["properties"][section], optional=optional
    )


def test_all_profile_documents_validate():
    documents = sorted(
        [
            *(ROOT / "shared/robots/marvin/profiles").glob("**/*.json"),
            *(ROOT / "shared/scenarios/marvin/mujoco").glob("*/profiles/**/*.json"),
        ]
    )
    assert documents
    for path in documents:
        document = json.loads(path.read_text())
        if path.name == "profile.json":
            load_profile(path, SCHEMAS)
            continue
        assert path.parent.name in {"observations", "actions"}, path
        name = "observation" if path.parent.name == "observations" else "action"
        schema = _document(f"{name}.schema.json")
        Draft202012Validator.check_schema(schema)
        Draft202012Validator(schema).validate(document)


@pytest.mark.parametrize("format", ["ros", "proto", "schema"])
def test_pair_check_detects_a_deleted_field(format):
    # Mutation check runs in memory: production contracts are never edited for validation.
    if format == "ros":
        text = (ROOT / "shared/common/interfaces/ros/robot_api_msgs/msg/RunContext.msg").read_text()
        changed = re.sub(r"^string robot_id\n", "", text, flags=re.M)
        with pytest.raises(AssertionError):
            _same_fields(RunContext.model_fields, _ros_fields(changed))
    elif format == "proto":
        text = (ROOT / "shared/common/interfaces/policy/policy.proto").read_text()
        changed = text.replace("string robot_id = 1;", "")
        with pytest.raises(AssertionError):
            _same_fields(RunContext.model_fields, _proto_fields(changed, "RunContext"))
    else:
        changed = deepcopy(_document("episode_manifest.schema.json")["properties"]["runtime"])
        del changed["properties"]["outcome"]
        with pytest.raises(AssertionError):
            _schema_pair(RuntimeResult, changed, optional=("reason",))


def test_task_state_uses_the_simulation_contract():
    text = (ROOT / "shared/common/interfaces/ros/sim_api_msgs/msg/TaskState.msg").read_text()
    rename = {"sample_time_ns": "sample_time", "metrics": "metrics_json"}
    _same_fields({rename.get(name, name) for name in TaskState.model_fields}, _ros_fields(text))
    assert _ros_fields(text)["sample_time"] == "builtin_interfaces/Time"
    assert _ros_fields(text)["metrics_json"] == "string"


def test_teacher_service_is_a_scenario_extension_not_a_common_package():
    common = ROOT / "shared/common/interfaces/ros/sim_api_msgs"
    extension = (
        ROOT
        / "shared/robots/marvin/extensions/ros/marvin_teacher_api_msgs"
    )
    assert not (common / "srv/GetTeacherChunk.srv").exists()
    assert "GetTeacherChunk" not in (common / "CMakeLists.txt").read_text()
    assert (extension / "srv/GetTeacherChunk.srv").is_file()
    assert "srv/GetTeacherChunk.srv" in (extension / "CMakeLists.txt").read_text()
    capability = (extension / "msg/TeacherCapability.msg").read_text()
    assert "marvin_teacher_api_msgs.teacher_chunk" in capability
