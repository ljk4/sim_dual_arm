"""Load schema-validated profiles and compute semantic content hashes."""

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator

from robot_contracts.models import CameraSpec, ContractError, JointSpec, RobotProfile

_CAMERA_COUNT_MAX = 8
_CAMERA_WIDTH_MAX = 640
_CAMERA_HEIGHT_MAX = 480
_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_CAMERA_NAME_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]{0,63}$")
_V1_CAMERA_FIELDS = frozenset({"name", "width", "height", "frame_id"})
_V2_CAMERA_FIELDS = _V1_CAMERA_FIELDS | {"calibration_digest"}


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe loader that rejects duplicate mapping keys instead of dropping them."""


def _construct_unique_mapping(
    loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as error:
            raise ContractError(
                "SCHEMA_MISMATCH",
                f"{getattr(loader, 'source_name', '<document>')}: mapping keys must be hashable",
            ) from error
        if duplicate:
            raise ContractError(
                "SCHEMA_MISMATCH",
                f"{getattr(loader, 'source_name', '<document>')}: duplicate key: {key}",
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def validate_document(document: dict[str, Any], schema_path: Path) -> None:
    schema = json.loads(schema_path.read_text())
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(document), key=lambda e: str(e.path))
    if errors:
        error = errors[0]
        path = ".".join(str(part) for part in error.absolute_path) or "<root>"
        raise ContractError("SCHEMA_MISMATCH", f"{path}: {error.message}")


def _with_source(source: Path | str, error: ContractError) -> ContractError:
    detail = str(error)
    prefix = f"{error.reason}: "
    if detail.startswith(prefix):
        detail = detail[len(prefix) :]
    source_text = str(source)
    if source_text and not detail.startswith(f"{source_text}:"):
        detail = f"{source_text}: {detail}"
    return ContractError(error.reason or "SCHEMA_MISMATCH", detail)


def load_document(path: Path) -> dict[str, Any]:
    try:
        loader = _UniqueKeyLoader(path.read_text())
        loader.source_name = str(path)
        try:
            document = loader.get_single_data()
        finally:
            loader.dispose()
    except ContractError:
        raise
    except (OSError, yaml.YAMLError) as error:
        raise ContractError("SCHEMA_MISMATCH", f"{path}: {error}") from error
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
    if isinstance(version, bool) or not isinstance(version, int):
        raise ContractError(
            "SCHEMA_MISMATCH", f"{source}: schema_version: must be the integer 1 or 2"
        )
    if version not in (1, 2):
        raise ContractError(
            "SCHEMA_MISMATCH", f"{source}: schema_version: unsupported version {version}"
        )
    return version


def _schema_directory(schema_root: Path, version: int) -> Path:
    """Accept either contracts/schemas or one version-specific schema directory."""
    directory = f"v{version}"
    if schema_root.name in {"v1", "v2"}:
        return schema_root.parent / directory
    return schema_root / directory


def profile_schema_path(schema_root: Path, version: int) -> Path:
    return _schema_directory(schema_root, version) / "robot_profile.schema.json"


def _field(document: dict[str, Any], name: str, source: Path | str) -> Any:
    if name not in document:
        raise ContractError("SCHEMA_MISMATCH", f"{source}: {name}: field is required")
    return document[name]


def _text(value: Any, source: Path | str, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise ContractError("SCHEMA_MISMATCH", f"{source}: {field}: must be a non-empty string")
    return value


def _integer(value: Any, source: Path | str, field: str, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError("SCHEMA_MISMATCH", f"{source}: {field}: must be an integer")
    if not 1 <= value <= maximum:
        raise ContractError("SCHEMA_MISMATCH", f"{source}: {field}: must be within [1, {maximum}]")
    return value


def _joint_specs(document: dict[str, Any], source: Path | str) -> tuple[JointSpec, ...]:
    joints = _field(document, "joints", source)
    if not isinstance(joints, list) or not joints:
        raise ContractError("SCHEMA_MISMATCH", f"{source}: joints: must be a non-empty list")
    names = []
    result = []
    for index, joint in enumerate(joints):
        location = f"joints[{index}]"
        if not isinstance(joint, dict):
            raise ContractError("SCHEMA_MISMATCH", f"{source}: {location}: must be an object")
        name = _text(_field(joint, "name", source), source, f"{location}.name")
        names.append(name)
        values = []
        for key in ("min", "max", "max_velocity", "initial"):
            value = _field(joint, key, source)
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise ContractError(
                    "SCHEMA_MISMATCH", f"{source}: {location}.{key}: must be a finite number"
                )
            try:
                number = float(value)
            except OverflowError as error:
                raise ContractError(
                    "SCHEMA_MISMATCH",
                    f"{source}: {location}.{key}: must be a finite representable number",
                ) from error
            if not math.isfinite(number):
                raise ContractError(
                    "SCHEMA_MISMATCH", f"{source}: {location}.{key}: must be a finite number"
                )
            values.append(number)
        minimum, maximum, max_velocity, initial = values
        if not minimum <= initial <= maximum:
            raise ContractError("SCHEMA_MISMATCH", f"{source}: invalid limits for {name}")
        if minimum >= maximum:
            raise ContractError("SCHEMA_MISMATCH", f"{source}: empty limits for {name}")
        if max_velocity <= 0:
            raise ContractError("SCHEMA_MISMATCH", f"{source}: invalid max_velocity for {name}")
        result.append(
            JointSpec(
                name,
                _text(_field(joint, "unit", source), source, f"{location}.unit"),
                minimum,
                maximum,
                max_velocity,
                initial,
            )
        )
    if len(names) != len(set(names)):
        raise ContractError("SCHEMA_MISMATCH", f"{source}: duplicate joint names")
    return tuple(result)


def _check_camera_fields(
    camera: dict[str, Any], source: Path | str, location: str, version: int
) -> None:
    expected = _V1_CAMERA_FIELDS if version == 1 else _V2_CAMERA_FIELDS
    unknown = sorted(set(camera) - expected)
    if unknown:
        raise ContractError("SCHEMA_MISMATCH", f"{source}: {location}: unknown field: {unknown[0]}")
    missing = sorted(expected - set(camera))
    if missing:
        raise ContractError(
            "SCHEMA_MISMATCH", f"{source}: {location}: field is required: {missing[0]}"
        )


def _camera_spec(camera: Any, source: Path | str, location: str, version: int) -> CameraSpec:
    if not isinstance(camera, dict):
        raise ContractError("SCHEMA_MISMATCH", f"{source}: {location}: must be an object")
    _check_camera_fields(camera, source, location, version)
    name = _text(camera["name"], source, f"{location}.name")
    if version == 2 and _CAMERA_NAME_PATTERN.fullmatch(name) is None:
        raise ContractError(
            "SCHEMA_MISMATCH",
            f"{source}: {location}.name: must be a ROS topic basename "
            "(letter/underscore start, then letters, digits or underscores, at most 64)",
        )
    width = _integer(camera["width"], source, f"{location}.width", _CAMERA_WIDTH_MAX)
    height = _integer(camera["height"], source, f"{location}.height", _CAMERA_HEIGHT_MAX)
    frame_id = _text(camera["frame_id"], source, f"{location}.frame_id")
    calibration_digest = ""
    if version == 2:
        calibration_digest = _text(
            camera["calibration_digest"], source, f"{location}.calibration_digest"
        )
        if _SHA256_PATTERN.fullmatch(calibration_digest) is None:
            raise ContractError(
                "SCHEMA_MISMATCH",
                f"{source}: {location}.calibration_digest: must be a lowercase SHA256 hex digest",
            )
    return CameraSpec(name, width, height, frame_id, calibration_digest)


def normalize_cameras(
    document: dict[str, Any], source: Path | str = "<profile>"
) -> tuple[CameraSpec, ...]:
    """Normalize v1's single camera or v2's named list into the immutable domain form.

    Configuration loading uses this after schema validation; audit reuses it on the
    recorded profile snapshot so there is never a second camera parser.
    """
    version = profile_schema_version(document, source)
    if version == 1:
        raw_cameras = [_field(document, "camera", source)]
        locations = ["camera"]
    else:
        raw_cameras = _field(document, "cameras", source)
        if not isinstance(raw_cameras, list):
            raise ContractError("SCHEMA_MISMATCH", f"{source}: cameras: must be a list")
        if not 1 <= len(raw_cameras) <= _CAMERA_COUNT_MAX:
            raise ContractError(
                "SCHEMA_MISMATCH",
                f"{source}: cameras: must contain 1 to {_CAMERA_COUNT_MAX} entries",
            )
        locations = [f"cameras[{index}]" for index in range(len(raw_cameras))]
    cameras = tuple(
        _camera_spec(camera, source, location, version)
        for camera, location in zip(raw_cameras, locations, strict=True)
    )
    names = [camera.name for camera in cameras]
    if len(names) != len(set(names)):
        raise ContractError("SCHEMA_MISMATCH", f"{source}: duplicate camera names")
    frame_ids = [camera.frame_id for camera in cameras]
    if len(frame_ids) != len(set(frame_ids)):
        raise ContractError("SCHEMA_MISMATCH", f"{source}: duplicate camera frame_id values")
    return cameras


def load_profile(path: Path, schema_root: Path) -> RobotProfile:
    return profile_from_document(load_document(path), schema_root, source=path)


def profile_from_document(
    document: dict[str, Any], schema_root: Path, *, source: Path | str = "<profile>"
) -> RobotProfile:
    """Validate a recorded profile snapshot with the same rules as a file."""
    path = source
    version = profile_schema_version(document, path)
    try:
        validate_document(document, profile_schema_path(schema_root, version))
    except ContractError as error:
        raise _with_source(path, error) from error
    except OSError as error:
        raise ContractError("SCHEMA_MISMATCH", f"{path}: {error}") from error
    try:
        joints = _joint_specs(document, path)
        cameras = normalize_cameras(document, path)
    except ContractError as error:
        raise _with_source(path, error) from error
    return RobotProfile(
        robot_id=_text(document["robot_id"], path, "robot_id"),
        profile_id=_text(document["profile_id"], path, "profile_id"),
        digest=semantic_hash(document),
        test_only=document["test_only"],
        joints=joints,
        cameras=cameras,
    )


def validate_bindings(
    profile: RobotProfile,
    observations: dict[str, Any],
    actions: dict[str, Any],
    schema_root: Path,
) -> None:
    schemas = _schema_directory(schema_root, 1)
    validate_document(observations, schemas / "observation.schema.json")
    validate_document(actions, schemas / "action.schema.json")
    for name, document in (("observation", observations), ("action", actions)):
        if tuple(document["joint_names"]) != profile.joint_names:
            raise ContractError("SCHEMA_MISMATCH", f"{name}: Joint ordering differs from profile")
    if tuple(observations["camera_names"]) != profile.camera_names:
        raise ContractError(
            "SCHEMA_MISMATCH",
            "observation: camera_names order differs from profile cameras",
        )
