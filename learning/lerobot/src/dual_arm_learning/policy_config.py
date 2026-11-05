"""Strict, framework-free configuration for one Policy RPC service.

Only the Python standard library and PyYAML are used.  Torch, ROS, MuJoCo and
the runtime package are deliberately absent; a fake provider and a future ACT
provider share this one service configuration.
"""

from __future__ import annotations

import math
import re
from dataclasses import asdict, dataclass
from dataclasses import field as dc_field
from pathlib import Path
from typing import Any

import yaml
from robot_run_logging import LogSettings

_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")
_TOP_LEVEL = {
    "schema_version",
    "bind_address",
    "port",
    "max_workers",
    "max_message_bytes",
    "max_sessions",
    "max_open_requests",
    "max_request_cache",
    "predict_deadline_s",
    "open_deadline_s",
    "close_deadline_s",
    "test_only",
    "protocol_major",
    "stateful",
    "model_bundle_digest",
    "processor_digest",
    "observation_schema_hash",
    "action_schema_hash",
    "robot_profile_hash",
    "joint_names",
    "joint_lower_bounds",
    "joint_upper_bounds",
    "action_joint_names",
    "camera_names",
    "camera_width",
    "camera_height",
    "action_period_ns",
    "chunk_size",
    "task_id",
    "task_text",
    "command_space",
    "logging",
}


class PolicyConfigError(ValueError):
    """One actionable configuration rejection with its field attached."""

    def __init__(self, field: str, message: str) -> None:
        self.field = field
        super().__init__(f"{field}: {message}")


def _mapping(value: object, location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PolicyConfigError(location, "must be an object")
    for key in value:
        if not isinstance(key, str):
            raise PolicyConfigError(location, "keys must be strings")
    return value


def _reject_non_finite(value: object, location: str) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            _reject_non_finite(item, f"{location}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _reject_non_finite(item, f"{location}[{index}]")
    elif isinstance(value, float) and not math.isfinite(value):
        raise PolicyConfigError(location, "must be finite")


def _integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise PolicyConfigError(location, "must be an integer")
    if not minimum <= value <= maximum:
        raise PolicyConfigError(location, f"must be within [{minimum}, {maximum}]")
    return value


def _number(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise PolicyConfigError(location, "must be a finite number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise PolicyConfigError(location, "must be a finite positive number")
    return result


def _text(value: object, location: str) -> str:
    if not isinstance(value, str) or not value:
        raise PolicyConfigError(location, "must be a non-empty string")
    return value


def _boolean(value: object, location: str) -> bool:
    if not isinstance(value, bool):
        raise PolicyConfigError(location, "must be a boolean")
    return value


def _numbers(value: object, location: str, *, count: int) -> tuple[float, ...]:
    if isinstance(value, str | bytes) or not isinstance(value, list):
        raise PolicyConfigError(location, "must be an array of finite numbers")
    if len(value) != count:
        raise PolicyConfigError(location, f"must contain exactly {count} numbers")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, int | float):
            raise PolicyConfigError(f"{location}[{index}]", "must be a finite number")
        number = float(item)
        if not math.isfinite(number):
            raise PolicyConfigError(f"{location}[{index}]", "must be finite")
        result.append(number)
    return tuple(result)


def _digest(value: object, location: str) -> str:
    result = _text(value, location)
    if _SHA256.fullmatch(result) is None:
        raise PolicyConfigError(location, "must be a lowercase SHA-256 hex digest")
    return result


def _names(value: object, location: str, *, count: int | None = None) -> tuple[str, ...]:
    if isinstance(value, str | bytes) or not isinstance(value, list):
        raise PolicyConfigError(location, "must be an array of names")
    result: list[str] = []
    for index, item in enumerate(value):
        if not isinstance(item, str) or _NAME.fullmatch(item) is None:
            raise PolicyConfigError(f"{location}[{index}]", "must be a valid name")
        if item in result:
            raise PolicyConfigError(f"{location}[{index}]", "must be unique")
        result.append(item)
    if count is not None and len(result) != count:
        raise PolicyConfigError(location, f"must contain exactly {count} names")
    return tuple(result)


@dataclass(frozen=True, slots=True)
class PolicyServiceConfig:
    """Every bound and identity needed by the service, with SI time in ns."""

    schema_version: int
    bind_address: str
    port: int
    max_workers: int
    max_message_bytes: int
    max_sessions: int
    max_open_requests: int
    max_request_cache: int
    predict_deadline_s: float
    open_deadline_s: float
    close_deadline_s: float
    test_only: bool
    protocol_major: int
    stateful: bool
    model_bundle_digest: str
    processor_digest: str
    observation_schema_hash: str
    action_schema_hash: str
    robot_profile_hash: str
    joint_names: tuple[str, ...]
    joint_lower_bounds: tuple[float, ...]
    joint_upper_bounds: tuple[float, ...]
    action_joint_names: tuple[str, ...]
    camera_names: tuple[str, ...]
    camera_width: int
    camera_height: int
    action_period_ns: int
    chunk_size: int
    task_id: str
    task_text: str
    command_space: str
    logging: LogSettings = dc_field(default_factory=LogSettings)

    def __post_init__(self) -> None:
        if self.schema_version != 1:
            raise PolicyConfigError("schema_version", "must be 1")
        if not isinstance(self.test_only, bool):
            raise PolicyConfigError("test_only", "must be a boolean")
        if not isinstance(self.stateful, bool):
            raise PolicyConfigError("stateful", "must be a boolean")
        if self.protocol_major != 1:
            raise PolicyConfigError("protocol_major", "unsupported protocol major")
        if not 0 <= self.port <= 65535:
            raise PolicyConfigError("port", "must be within [1, 65535]")
        if self.chunk_size != 60:
            raise PolicyConfigError("chunk_size", "v1 requires exactly 60 targets")
        if self.action_period_ns != 50_000_000:
            raise PolicyConfigError("action_period_ns", "v1 requires a uniform 50 ms period")
        if len(self.joint_names) != 16 or self.action_joint_names != self.joint_names:
            raise PolicyConfigError(
                "action_joint_names", "v1 requires the same complete 16-joint order"
            )
        if len(self.joint_lower_bounds) != 16 or len(self.joint_upper_bounds) != 16:
            raise PolicyConfigError("joint_lower_bounds", "must contain 16 finite bounds")
        if any(
            not low < high
            for low, high in zip(self.joint_lower_bounds, self.joint_upper_bounds, strict=True)
        ):
            raise PolicyConfigError("joint_upper_bounds", "every upper bound must exceed its lower")
        if len(self.camera_names) != 3:
            raise PolicyConfigError("camera_names", "v1 requires exactly three RGB cameras")
        if (self.camera_width, self.camera_height) != (320, 240):
            raise PolicyConfigError("camera_width", "v1 requires 320x240 RGB captures")
        if self.command_space != "joint_position":
            raise PolicyConfigError("command_space", "v1 accepts only joint_position")
        if self.max_message_bytes < 1024:
            raise PolicyConfigError("max_message_bytes", "must be at least 1024 bytes")

    @property
    def chunk_duration_ns(self) -> int:
        return self.chunk_size * self.action_period_ns

    @property
    def required_digests(self) -> tuple[str, ...]:
        return (
            self.model_bundle_digest,
            self.processor_digest,
            self.observation_schema_hash,
            self.action_schema_hash,
            self.robot_profile_hash,
        )

    def to_document(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "bind_address": self.bind_address,
            "port": self.port,
            "max_workers": self.max_workers,
            "max_message_bytes": self.max_message_bytes,
            "max_sessions": self.max_sessions,
            "max_open_requests": self.max_open_requests,
            "max_request_cache": self.max_request_cache,
            "predict_deadline_s": self.predict_deadline_s,
            "open_deadline_s": self.open_deadline_s,
            "close_deadline_s": self.close_deadline_s,
            "test_only": self.test_only,
            "protocol_major": self.protocol_major,
            "stateful": self.stateful,
            "model_bundle_digest": self.model_bundle_digest,
            "processor_digest": self.processor_digest,
            "observation_schema_hash": self.observation_schema_hash,
            "action_schema_hash": self.action_schema_hash,
            "robot_profile_hash": self.robot_profile_hash,
            "joint_names": list(self.joint_names),
            "joint_lower_bounds": list(self.joint_lower_bounds),
            "joint_upper_bounds": list(self.joint_upper_bounds),
            "action_joint_names": list(self.action_joint_names),
            "camera_names": list(self.camera_names),
            "camera_width": self.camera_width,
            "camera_height": self.camera_height,
            "action_period_ns": self.action_period_ns,
            "chunk_size": self.chunk_size,
            "task_id": self.task_id,
            "task_text": self.task_text,
            "command_space": self.command_space,
            "logging": asdict(self.logging),
        }


def parse_policy_config(document: object) -> PolicyServiceConfig:
    value = _mapping(document, "<policy config>")
    _reject_non_finite(value, "<policy config>")
    unknown = sorted(set(value) - _TOP_LEVEL)
    if unknown:
        raise PolicyConfigError("<policy config>", f"unknown field(s): {', '.join(unknown)}")
    for field in _TOP_LEVEL:
        if field != "logging" and field not in value:
            raise PolicyConfigError(field, "field is required")
    return PolicyServiceConfig(
        schema_version=_integer(value["schema_version"], "schema_version", 1, 1),
        bind_address=_text(value["bind_address"], "bind_address"),
        port=_integer(value["port"], "port", 0, 65535),
        max_workers=_integer(value["max_workers"], "max_workers", 1, 256),
        max_message_bytes=_integer(
            value["max_message_bytes"], "max_message_bytes", 1024, 256 * 1024 * 1024
        ),
        max_sessions=_integer(value["max_sessions"], "max_sessions", 1, 4096),
        max_open_requests=_integer(value["max_open_requests"], "max_open_requests", 1, 65536),
        max_request_cache=_integer(value["max_request_cache"], "max_request_cache", 1, 65536),
        predict_deadline_s=_number(value["predict_deadline_s"], "predict_deadline_s"),
        open_deadline_s=_number(value["open_deadline_s"], "open_deadline_s"),
        close_deadline_s=_number(value["close_deadline_s"], "close_deadline_s"),
        test_only=_boolean(value["test_only"], "test_only"),
        protocol_major=_integer(value["protocol_major"], "protocol_major", 1, 1),
        stateful=_boolean(value["stateful"], "stateful"),
        model_bundle_digest=_digest(value["model_bundle_digest"], "model_bundle_digest"),
        processor_digest=_digest(value["processor_digest"], "processor_digest"),
        observation_schema_hash=_digest(
            value["observation_schema_hash"], "observation_schema_hash"
        ),
        action_schema_hash=_digest(value["action_schema_hash"], "action_schema_hash"),
        robot_profile_hash=_digest(value["robot_profile_hash"], "robot_profile_hash"),
        joint_names=_names(value["joint_names"], "joint_names", count=16),
        joint_lower_bounds=_numbers(value["joint_lower_bounds"], "joint_lower_bounds", count=16),
        joint_upper_bounds=_numbers(value["joint_upper_bounds"], "joint_upper_bounds", count=16),
        action_joint_names=_names(value["action_joint_names"], "action_joint_names", count=16),
        camera_names=_names(value["camera_names"], "camera_names", count=3),
        camera_width=_integer(value["camera_width"], "camera_width", 1, 4096),
        camera_height=_integer(value["camera_height"], "camera_height", 1, 4096),
        action_period_ns=_integer(value["action_period_ns"], "action_period_ns", 1, 10_000_000_000),
        chunk_size=_integer(value["chunk_size"], "chunk_size", 1, 4096),
        task_id=_text(value["task_id"], "task_id"),
        task_text=_text(value["task_text"], "task_text"),
        command_space=_text(value["command_space"], "command_space"),
        logging=LogSettings(**_mapping(value.get("logging", {}), "logging")),
    )


def load_policy_config(path: str | Path) -> PolicyServiceConfig:
    source = Path(path)
    try:
        text = source.read_text()
    except OSError as error:
        raise PolicyConfigError("<file>", f"cannot read {source}: {error}") from error
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise PolicyConfigError("<yaml>", str(error)) from error
    return parse_policy_config(document)
