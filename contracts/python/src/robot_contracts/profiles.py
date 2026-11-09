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
    "profile_schema_version",
    "semantic_hash",
    "validate_bindings",
    "validate_document",
]

_PROFILE_SCHEMA_FILES = {1: "robot_profile.schema.json", 2: "robot_profile_v2.schema.json"}


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


def profile_schema_version(document: dict[str, Any], source: Path | str = "<profile>") -> int:
    """Return a strictly typed profile version; booleans are never version numbers."""
    if "schema_version" not in document:
        raise ContractError("SCHEMA_MISMATCH", f"{source}: schema_version: field is required")
    version = document["schema_version"]
    if isinstance(version, bool) or not isinstance(version, int) or version not in (1, 2):
        raise ContractError(
            "SCHEMA_MISMATCH", f"{source}: schema_version: must be the integer 1 or 2"
        )
    return version


def _schema_directory(schema_root: Path) -> Path:
    if schema_root.name in {"v1", "v2"}:
        return schema_root.parent / "v1"
    return schema_root / "v1"


def profile_schema_path(schema_root: Path, version: int) -> Path:
    filename = _PROFILE_SCHEMA_FILES.get(version)
    if filename is None:
        raise ContractError("SCHEMA_MISMATCH", f"unsupported profile schema version {version!r}")
    return _schema_directory(schema_root) / filename


def normalize_cameras(
    document: dict[str, Any], source: Path | str = "<profile>"
) -> tuple[CameraSpec, ...]:
    """Normalize v1's single camera or v2's named list through RobotProfile."""
    return RobotProfile.validate_dict(document, source=str(source)).cameras


def load_profile(path: Path, schema_root: Path) -> RobotProfile:
    return profile_from_document(load_document(path), schema_root, source=path)


def profile_from_document(
    document: dict[str, Any], schema_root: Path, *, source: Path | str = "<profile>"
) -> RobotProfile:
    """Validate a recorded profile snapshot with the same rules as a file."""
    version = profile_schema_version(document, source)
    try:
        validate_document(document, profile_schema_path(schema_root, version))
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
