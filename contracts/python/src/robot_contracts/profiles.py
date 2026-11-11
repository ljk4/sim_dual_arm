"""Load schema-validated profiles and compute semantic content hashes."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

from .models import CameraSpec, ContractError, RobotProfile
from .validation import UniqueKeyError, read_unique_yaml

__all__ = [
    "load_document",
    "load_profile",
    "normalize_cameras",
    "profile_from_document",
    "profile_schema_path",
    "semantic_hash",
    "validate_bindings",
    "validate_document",
]


def validate_document(document: dict[str, Any], schema_path: Path) -> None:
    schema = json.loads(schema_path.read_text())
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(document), key=lambda e: str(e.path))
    if errors:
        error = errors[0]
        path = ".".join(str(part) for part in error.absolute_path) or "<root>"
        raise ContractError("SCHEMA_MISMATCH", f"{path}: {error.message}")


def load_document(path: Path) -> dict[str, Any]:
    try:
        document = read_unique_yaml(path)
    except UniqueKeyError as error:
        raise ContractError(
            "SCHEMA_MISMATCH", f"{error.source}: {error.field}: {error.message}"
        ) from error
    if not isinstance(document, dict):
        raise ContractError("SCHEMA_MISMATCH", f"{path}: expected an object")
    return document


def semantic_hash(document: dict[str, Any]) -> str:
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode()).hexdigest()


def _schema_directory(schema_root: Path) -> Path:
    if schema_root.name == "v1":
        return schema_root
    return schema_root / "v1"


def profile_schema_path(schema_root: Path) -> Path:
    """Return the sole Profile v2 schema in the shared schema collection."""
    return _schema_directory(schema_root) / "robot_profile.schema.json"


def normalize_cameras(
    document: dict[str, Any], source: Path | str = "<profile>"
) -> tuple[CameraSpec, ...]:
    """Validate the named camera list of a Profile v2 document."""
    return RobotProfile.validate_dict(document, source=str(source)).cameras


def load_profile(path: Path, schema_root: Path) -> RobotProfile:
    return profile_from_document(load_document(path), schema_root, source=path)


def profile_from_document(
    document: dict[str, Any], schema_root: Path, *, source: Path | str = "<profile>"
) -> RobotProfile:
    """Validate a recorded profile snapshot with the same rules as a file."""
    try:
        validate_document(document, profile_schema_path(schema_root))
    except ContractError as error:
        raise ContractError(error.reason or "SCHEMA_MISMATCH",
                            f"{source}: {str(error).split(': ', 1)[-1]}") from error
    except OSError as error:
        raise ContractError("SCHEMA_MISMATCH", f"{source}: {error}") from error
    profile = RobotProfile.validate_dict(document, source=str(source))
    return profile.model_copy(update={"digest": semantic_hash(document)})


def validate_bindings(
    profile: RobotProfile,
    observations: dict[str, Any],
    actions: dict[str, Any],
    schema_root: Path,
) -> None:
    schemas = _schema_directory(schema_root)
    validate_document(observations, schemas / "observation.schema.json")
    validate_document(actions, schemas / "action.schema.json")
    for name, document in (("observation", observations), ("action", actions)):
        if tuple(document["joint_names"]) != profile.joint_names:
            raise ContractError("SCHEMA_MISMATCH", f"{name}: Joint ordering differs from profile")
    if tuple(observations["camera_names"]) != profile.camera_names:
        raise ContractError(
            "SCHEMA_MISMATCH", "observation: camera_names order differs from profile cameras"
        )
