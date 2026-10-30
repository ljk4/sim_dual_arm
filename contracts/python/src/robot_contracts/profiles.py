"""Load schema-validated profiles and compute semantic content hashes."""

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator

from robot_contracts.models import CameraSpec, ContractError, JointSpec, RobotProfile


def validate_document(document: dict[str, Any], schema_path: Path) -> None:
    schema = json.loads(schema_path.read_text())
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(document), key=lambda e: str(e.path))
    if errors:
        error = errors[0]
        path = ".".join(str(part) for part in error.absolute_path) or "<root>"
        raise ContractError("SCHEMA_MISMATCH", f"{path}: {error.message}")


def load_document(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict):
        raise ContractError("SCHEMA_MISMATCH", f"Expected an object: {path}")
    return document


def semantic_hash(document: dict[str, Any]) -> str:
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode()).hexdigest()


def load_profile(path: Path, schema_root: Path) -> RobotProfile:
    document = load_document(path)
    validate_document(document, schema_root / "robot_profile.schema.json")
    names = [joint["name"] for joint in document["joints"]]
    if len(names) != len(set(names)):
        raise ContractError("SCHEMA_MISMATCH", "Duplicate joint names")
    joints = []
    for joint in document["joints"]:
        values = [joint[key] for key in ("min", "max", "max_velocity", "initial")]
        if not all(math.isfinite(value) for value in values):
            raise ContractError("SCHEMA_MISMATCH", "Non-finite joint parameters")
        if not joint["min"] <= joint["initial"] <= joint["max"]:
            raise ContractError("SCHEMA_MISMATCH", f"Invalid limits for {joint['name']}")
        if joint["min"] >= joint["max"]:
            raise ContractError("SCHEMA_MISMATCH", f"Empty limits for {joint['name']}")
        joints.append(
            JointSpec(
                joint["name"],
                joint["unit"],
                joint["min"],
                joint["max"],
                joint["max_velocity"],
                joint["initial"],
            )
        )
    camera = document["camera"]
    return RobotProfile(
        document["robot_id"],
        document["profile_id"],
        semantic_hash(document),
        tuple(joints),
        CameraSpec(**camera),
    )


def validate_bindings(
    profile: RobotProfile,
    observations: dict[str, Any],
    actions: dict[str, Any],
    schema_root: Path,
) -> None:
    validate_document(observations, schema_root / "observation.schema.json")
    validate_document(actions, schema_root / "action.schema.json")
    for document in (observations, actions):
        if tuple(document["joint_names"]) != profile.joint_names:
            raise ContractError("SCHEMA_MISMATCH", "Joint ordering differs from profile")
    if observations["camera_names"] != [profile.camera.name]:
        raise ContractError("SCHEMA_MISMATCH", "Required cameras differ from profile")
