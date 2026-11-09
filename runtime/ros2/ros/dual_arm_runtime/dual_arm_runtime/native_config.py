"""Strict configuration for the diagnostic native runtime.

The file is deliberately small: it composes existing profile/observation/action
documents and the shared controller limits, then adds only runtime choices
(namespace, leases, deadlines, reset seed, bounded observation cache and the
configured motion list).  It imports no backend, MuJoCo, NumPy or SciPy.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.models import ContractError, RobotProfile
from robot_contracts.profiles import (
    load_document,
    load_profile,
    semantic_hash,
    validate_bindings,
)
from robot_run_logging import LogSettings

from dual_arm_runtime.recording_queue import RecordingSettings

MIN_LEASE_DURATION_NS = 100_000_000
MAX_LEASE_DURATION_NS = 10_000_000_000
_ACTION_SOURCES = ("policy", "teacher")
_TERMINATION_MODES = ("finite_chunks", "task")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_POLICY_DEFAULTS = {
    "chunk_size": 60,
    "action_period_ns": 50_000_000,
    "target_count": 5,
    "initial_lead_ns": 250_000_000,
    "max_chunks": 8,
    "total_sim_deadline_ns": 8_000_000_000,
    "total_wall_deadline_s": 45.0,
    "connect_timeout_s": 2.0,
    "open_deadline_s": 2.0,
    "predict_deadline_s": 2.0,
    "close_deadline_s": 2.0,
    "max_message_bytes": 16_777_216,
    "poll_interval_s": 0.02,
    "startup_settle_sim_ns": 0,
    "termination_mode": "finite_chunks",
    "task_stop_margin_s": 1.0,
    "task_sim_budget_ns": 30_000_000_000,
}
_REQUIRED_CAPABILITIES = (
    "continuous_chunks",
    "exact_capture",
    "simulation_reset",
    "lease",
    "stop",
    "command_query",
    "native_events",
    "follower_reference",
)


def _error(location: str, message: str) -> ContractError:
    return ContractError("SCHEMA_MISMATCH", f"{location}: {message}")


def _object(value: object, location: str, allowed: set[str]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise _error(location, "must be an object")
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise _error(location, f"unknown field(s): {', '.join(unknown)}")
    return value


def _required(document: dict[str, Any], name: str, location: str) -> object:
    if name not in document:
        raise _error(f"{location}.{name}", "field is required")
    return document[name]


def _integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise _error(location, "must be an integer")
    if not minimum <= value <= maximum:
        raise _error(location, f"must be within [{minimum}, {maximum}]")
    return value


def _number(value: object, location: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise _error(location, "must be a finite number")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0):
        raise _error(location, "must be a finite positive number")
    return result


def _text(value: object, location: str) -> str:
    if not isinstance(value, str) or not value:
        raise _error(location, "must be a non-empty string")
    return value


def _reject_non_finite(value: object, location: str) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            _reject_non_finite(item, f"{location}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _reject_non_finite(item, f"{location}[{index}]")
    elif isinstance(value, float) and not math.isfinite(value):
        raise _error(location, "must be a finite number")


@dataclass(frozen=True)
class NativeMotion:
    """One configured diagnostic motion with a full joint target and duration."""

    name: str
    duration_ns: int
    positions: tuple[float, ...]


@dataclass(frozen=True)
class NativeReset:
    enabled: bool
    seed: int


@dataclass(frozen=True)
class NativeObservation:
    max_skew_ns: int
    max_age_ns: int
    wall_arrival_timeout_s: float
    capacity: int
    reliable: bool = False


@dataclass(frozen=True)
class NativeControl:
    submit_query_retries: int
    poll_interval_s: float
    stop_timeout_s: float
    terminal_timeout_s: float


@dataclass(frozen=True)
class NativeFollower:
    """The accepted bounded-follower limits, loaded from their single source."""

    target_count: int
    max_target_deviations: tuple[float, ...]
    max_terminal_tail_duration_ns: int
    duration_grid_ns: int


@dataclass(frozen=True)
class NativePolicy:
    """Policy RPC identity, bounds and the finite diagnostic stream budget."""

    endpoint: str
    model_bundle_digest: str
    processor_digest: str
    observation_schema_hash: str
    action_schema_hash: str
    robot_profile_hash: str
    allow_test_policy: bool
    expected_test_only: bool
    task_id: str
    task_text: str
    chunk_size: int
    action_period_ns: int
    target_count: int
    initial_lead_ns: int
    max_chunks: int
    total_sim_deadline_ns: int
    total_wall_deadline_s: float
    connect_timeout_s: float
    open_deadline_s: float
    predict_deadline_s: float
    close_deadline_s: float
    max_message_bytes: int
    poll_interval_s: float
    termination_mode: str = "finite_chunks"
    task_stop_margin_s: float = 1.0
    task_sim_budget_ns: int = 30_000_000_000
    startup_settle_sim_ns: int = 0


@dataclass(frozen=True)
class NativeTeacher:
    """Program-teacher stream identity, independent of policy RPC settings."""

    chunk_size: int
    action_period_ns: int
    target_count: int
    initial_lead_ns: int
    max_chunks: int
    total_sim_deadline_ns: int
    total_wall_deadline_s: float
    prepare_timeout_s: float
    start_timeout_s: float
    chunk_deadline_s: float
    poll_interval_s: float
    termination_mode: str = "finite_chunks"
    task_stop_margin_s: float = 1.0
    task_sim_budget_ns: int = 30_000_000_000


@dataclass(frozen=True)
class NativeRecipe:
    """Values shared by the control client, logger and episode runner."""

    schema_version: int
    robot_profile: str
    observation_schema: str
    action_schema: str
    robot_namespace: str
    lease_duration_ns: int
    renew_interval_ns: int
    command_acceptance_ns: int
    startup_timeout_s: float
    call_timeout_s: float
    command_timeout_s: float
    reset: NativeReset
    observation: NativeObservation
    control: NativeControl
    motions: tuple[NativeMotion, ...]
    action_source: str = "policy"
    follower: NativeFollower | None = None
    policy: NativePolicy | None = None
    teacher: NativeTeacher | None = None
    logging: LogSettings = field(default_factory=LogSettings)
    recording: RecordingSettings = field(default_factory=RecordingSettings)

    @property
    def test_only(self) -> bool:
        return False


@dataclass(frozen=True)
class NativeConfiguration:
    recipe: NativeRecipe
    profile: RobotProfile
    schema_root: Path
    control_document: dict[str, Any]
    _snapshot_json: str = field(repr=False)

    @property
    def snapshot(self) -> dict[str, Any]:
        return json.loads(self._snapshot_json)

    @property
    def required_capabilities(self) -> tuple[str, ...]:
        return _REQUIRED_CAPABILITIES


def _load(path: Path) -> dict[str, Any]:
    try:
        document = load_document(path)
    except (ContractError, OSError, yaml.YAMLError) as error:
        raise ContractError("SCHEMA_MISMATCH", f"{path}: {error}") from error
    _reject_non_finite(document, str(path))
    return document


def _build_logging(document: dict[str, Any], location: str) -> LogSettings:
    allowed = {
        "queue_capacity",
        "max_event_bytes",
        "max_file_bytes",
        "max_total_bytes",
        "close_timeout_s",
        "flush_interval_s",
        "console",
    }
    raw = _object(document.get("logging", {}), f"{location}.logging", allowed)
    values: dict[str, Any] = {}
    for name in ("queue_capacity", "max_event_bytes", "max_file_bytes", "max_total_bytes"):
        if name in raw:
            values[name] = _integer(raw[name], f"{location}.logging.{name}", 1, 2**63 - 1)
    for name in ("close_timeout_s", "flush_interval_s"):
        if name in raw:
            values[name] = _number(raw[name], f"{location}.logging.{name}", positive=True)
    if "console" in raw:
        if not isinstance(raw["console"], bool):
            raise _error(f"{location}.logging.console", "must be a boolean")
        values["console"] = raw["console"]
    try:
        return LogSettings(**values)
    except (TypeError, ValueError) as error:
        raise _error(f"{location}.logging", str(error)) from error


def _build_recording(document: dict[str, Any], location: str) -> RecordingSettings:
    allowed = {"queue_capacity", "max_pending_bytes", "startup_timeout_s", "close_timeout_s"}
    raw = _object(document.get("recording", {}), f"{location}.recording", allowed)
    values: dict[str, Any] = {}
    for name in ("queue_capacity", "max_pending_bytes"):
        if name in raw:
            values[name] = _integer(raw[name], f"{location}.recording.{name}", 1, 2**63 - 1)
    for name in ("startup_timeout_s", "close_timeout_s"):
        if name in raw:
            values[name] = _number(raw[name], f"{location}.recording.{name}", positive=True)
    try:
        return RecordingSettings(**values)
    except (TypeError, ValueError) as error:
        raise _error(f"{location}.recording", str(error)) from error


def _build_motions(
    document: dict[str, Any],
    location: str,
    profile: RobotProfile,
    *,
    allow_empty: bool = False,
) -> tuple[NativeMotion, ...]:
    if allow_empty and "motions" not in document:
        return ()
    raw_motions = _required(document, "motions", location)
    minimum = 0 if allow_empty else 1
    if not isinstance(raw_motions, list) or not minimum <= len(raw_motions) <= 64:
        raise _error(
            f"{location}.motions",
            f"must be an array of {minimum}..64 motions",
        )
    result: list[NativeMotion] = []
    previous = profile.initial_positions
    seen_names: set[str] = set()
    for index, raw_value in enumerate(raw_motions):
        raw = _object(
            raw_value,
            f"{location}.motions[{index}]",
            {"name", "duration_ns", "positions"},
        )
        name = _text(
            _required(raw, "name", f"{location}.motions[{index}]"),
            f"{location}.motions[{index}].name",
        )
        if name in seen_names:
            raise _error(f"{location}.motions[{index}].name", "must be unique")
        seen_names.add(name)
        duration_ns = _integer(
            _required(raw, "duration_ns", f"{location}.motions[{index}]"),
            f"{location}.motions[{index}].duration_ns",
            1,
            3_000_000_000,
        )
        positions = _required(raw, "positions", f"{location}.motions[{index}]")
        if not isinstance(positions, list) or len(positions) != len(profile.joints):
            raise _error(
                f"{location}.motions[{index}].positions",
                f"expected {len(profile.joints)} values for the profile joints",
            )
        values: list[float] = []
        for joint_index, (value, joint) in enumerate(zip(positions, profile.joints, strict=True)):
            number = _number(value, f"{location}.motions[{index}].positions[{joint_index}]")
            if not joint.minimum <= number <= joint.maximum:
                raise _error(
                    f"{location}.motions[{index}].positions[{joint_index}]",
                    f"{joint.name}: {number} outside [{joint.minimum}, {joint.maximum}]",
                )
            values.append(number)
        seconds = duration_ns / 1e9
        for joint_index, (position, start, joint) in enumerate(
            zip(values, previous, profile.joints, strict=True)
        ):
            speed = abs(position - start) / seconds
            if speed > joint.max_velocity + 1e-9:
                raise _error(
                    f"{location}.motions[{index}].positions[{joint_index}]",
                    f"{joint.name}: average speed {speed:.6g} exceeds {joint.max_velocity}",
                )
        result.append(NativeMotion(name, duration_ns, tuple(values)))
        previous = tuple(values)
    return tuple(result)


def _build_follower(
    document: dict[str, Any], location: str, profile: RobotProfile
) -> tuple[NativeFollower | None, dict[str, Any] | None]:
    name = document.get("follower_constraints")
    if name is None:
        return None, None
    path = (Path(location).parent / _text(name, f"{location}.follower_constraints")).resolve()
    if not path.is_file():
        raise _error(f"{location}.follower_constraints", f"file does not exist: {path}")
    raw = _load(path)
    _object(
        raw,
        str(path),
        {
            "joint_names",
            "target_count",
            "max_target_deviations",
            "max_terminal_tail_duration_s",
        },
    )
    names = raw.get("joint_names")
    if not isinstance(names, list) or tuple(names) != profile.joint_names:
        raise _error(f"{path}.joint_names", "must match the robot profile order")
    target_count = _integer(
        _required(raw, "target_count", str(path)), f"{path}.target_count", 2, 128
    )
    deviations = _required(raw, "max_target_deviations", str(path))
    if not isinstance(deviations, list) or len(deviations) != len(profile.joints):
        raise _error(f"{path}.max_target_deviations", "must match the profile joints")
    values = [
        _number(value, f"{path}.max_target_deviations[{index}]", positive=True)
        for index, value in enumerate(deviations)
    ]
    seconds = _number(
        _required(raw, "max_terminal_tail_duration_s", str(path)),
        f"{path}.max_terminal_tail_duration_s",
        positive=True,
    )
    tail_ns = round(seconds * 1_000_000_000)
    if tail_ns <= 0:
        raise _error(f"{path}.max_terminal_tail_duration_s", "must convert to positive ns")
    return (
        NativeFollower(target_count, tuple(values), tail_ns, 2_000_000),
        raw,
    )


def _validate_stream_budget(
    values: dict[str, Any], location: str, *, chunk_size: int, period_ns: int
) -> None:
    """Bound every stream and make a task budget actually coverable in chunks."""
    max_chunks = int(values["max_chunks"])
    if max_chunks > 512:
        raise _error(f"{location}.max_chunks", "must not exceed 512")
    if values["termination_mode"] == "task":
        if values["task_sim_budget_ns"] > 30_000_000_000:
            raise _error(f"{location}.task_sim_budget_ns", "must not exceed 30 s")
        if values["task_stop_margin_s"] * 1e9 >= values["task_sim_budget_ns"]:
            raise _error(f"{location}.task_stop_margin_s", "must leave a positive task budget")
        covered_ns = max_chunks * int(values["target_count"]) * period_ns
        if covered_ns < values["task_sim_budget_ns"]:
            required = values["task_sim_budget_ns"] // (int(values["target_count"]) * period_ns)
            raise _error(
                f"{location}.max_chunks",
                f"task mode needs at least {required} chunks",
            )
    if chunk_size != 60 or period_ns != 50_000_000:
        raise _error(location, "streams are exactly 60 points at 50 ms")


def _build_teacher(
    document: dict[str, Any],
    location: str,
    follower: NativeFollower | None,
) -> NativeTeacher | None:
    if "teacher" not in document:
        return None
    raw = _object(
        _required(document, "teacher", location),
        f"{location}.teacher",
        {
            "chunk_size",
            "action_period_ns",
            "target_count",
            "initial_lead_ns",
            "max_chunks",
            "total_sim_deadline_ns",
            "total_wall_deadline_s",
            "prepare_timeout_s",
            "start_timeout_s",
            "chunk_deadline_s",
            "poll_interval_s",
            "termination_mode",
            "task_stop_margin_s",
            "task_sim_budget_ns",
        },
    )
    if follower is None:
        raise _error(f"{location}.teacher", "teacher action source requires follower_constraints")
    defaults = {
        "chunk_size": 60,
        "action_period_ns": 50_000_000,
        "target_count": 5,
        "initial_lead_ns": 250_000_000,
        "max_chunks": 128,
        "total_sim_deadline_ns": 30_000_000_000,
        "total_wall_deadline_s": 60.0,
        "prepare_timeout_s": 12.0,
        "start_timeout_s": 5.0,
        "chunk_deadline_s": 2.0,
        "poll_interval_s": 0.02,
        "termination_mode": "task",
        "task_stop_margin_s": 1.0,
        "task_sim_budget_ns": 30_000_000_000,
    }
    values: dict[str, Any] = {}
    for name, default in defaults.items():
        if name == "termination_mode":
            value = _text(raw.get(name, default), f"{location}.teacher.{name}")
            if value not in _TERMINATION_MODES:
                raise _error(f"{location}.teacher.{name}", f"must be one of {_TERMINATION_MODES}")
            values[name] = value
        elif name.endswith("_s") or name == "total_wall_deadline_s":
            values[name] = _number(
                raw.get(name, default), f"{location}.teacher.{name}", positive=True
            )
        else:
            values[name] = _integer(
                raw.get(name, default), f"{location}.teacher.{name}", 1, 2**63 - 1
            )
    if values["chunk_size"] != 60:
        raise _error(f"{location}.teacher.chunk_size", "must be 60")
    if values["action_period_ns"] != 50_000_000:
        raise _error(f"{location}.teacher.action_period_ns", "must be 50000000")
    if values["target_count"] != follower.target_count:
        raise _error(f"{location}.teacher.target_count", "must equal the follower target_count")
    if values["target_count"] != 5:
        raise _error(f"{location}.teacher.target_count", "must be 5 for the teacher stream")
    _validate_stream_budget(values, f"{location}.teacher", chunk_size=60, period_ns=50_000_000)
    return NativeTeacher(**values)


def _build_policy(
    document: dict[str, Any],
    location: str,
    profile: RobotProfile,
    observation_document: dict[str, Any],
    action_document: dict[str, Any],
    follower: NativeFollower | None,
) -> NativePolicy | None:
    if "policy" not in document:
        return None
    raw = _object(
        _required(document, "policy", location),
        f"{location}.policy",
        {
            "endpoint",
            "model_bundle_digest",
            "processor_digest",
            "observation_schema_hash",
            "action_schema_hash",
            "robot_profile_hash",
            "allow_test_policy",
            "expected_test_only",
            "task_id",
            "task_text",
            *_POLICY_DEFAULTS,
        },
    )
    if follower is None:
        raise _error(f"{location}.policy", "policy action source requires follower_constraints")
    endpoint = _text(
        _required(raw, "endpoint", f"{location}.policy"),
        f"{location}.policy.endpoint",
    )
    if ":" not in endpoint:
        raise _error(f"{location}.policy.endpoint", "must be host:port")
    digests = {}
    for name in ("model_bundle_digest", "processor_digest"):
        digests[name] = _text(
            _required(raw, name, f"{location}.policy"), f"{location}.policy.{name}"
        )
        if _SHA256.fullmatch(digests[name]) is None:
            raise _error(f"{location}.policy.{name}", "must be a lowercase SHA-256 digest")
    expected = {
        "observation_schema_hash": semantic_hash(observation_document),
        "action_schema_hash": semantic_hash(action_document),
        "robot_profile_hash": profile.digest,
    }
    for name, value in expected.items():
        configured = raw.get(name, value)
        if _text(configured, f"{location}.policy.{name}") != value:
            raise _error(
                f"{location}.policy.{name}",
                "must match the referenced profile/observation/action document",
            )
    for name in ("allow_test_policy", "expected_test_only"):
        if not isinstance(raw.get(name), bool):
            raise _error(f"{location}.policy.{name}", "must be a boolean")
    if raw["expected_test_only"] and not raw["allow_test_policy"]:
        raise _error(
            f"{location}.policy.allow_test_policy",
            "a test-only policy requires explicit allow_test_policy: true",
        )
    values: dict[str, Any] = {}
    for name, default in _POLICY_DEFAULTS.items():
        if name == "termination_mode":
            value = _text(raw.get(name, default), f"{location}.policy.{name}")
            if value not in _TERMINATION_MODES:
                raise _error(f"{location}.policy.{name}", f"must be one of {_TERMINATION_MODES}")
            values[name] = value
        elif name == "startup_settle_sim_ns":
            values[name] = _integer(
                raw.get(name, default), f"{location}.policy.{name}", 0, 5_000_000_000
            )
        elif name.endswith("_s") or name == "total_wall_deadline_s":
            values[name] = _number(
                raw.get(name, default), f"{location}.policy.{name}", positive=True
            )
        else:
            values[name] = _integer(
                raw.get(name, default),
                f"{location}.policy.{name}",
                1,
                2**63 - 1,
            )
    if values["chunk_size"] != 60:
        raise _error(f"{location}.policy.chunk_size", "must be 60")
    if values["action_period_ns"] != 50_000_000:
        raise _error(f"{location}.policy.action_period_ns", "must be 50000000")
    if values["target_count"] != follower.target_count:
        raise _error(f"{location}.policy.target_count", "must equal the follower target_count")
    # The selection window is the follower's 2..128 bound.  _validate_stream_budget
    # below proves the task budget is covered by max_chunks at the chosen count.
    _validate_stream_budget(values, f"{location}.policy", chunk_size=60, period_ns=50_000_000)
    return NativePolicy(
        endpoint=endpoint,
        model_bundle_digest=digests["model_bundle_digest"],
        processor_digest=digests["processor_digest"],
        observation_schema_hash=expected["observation_schema_hash"],
        action_schema_hash=expected["action_schema_hash"],
        robot_profile_hash=expected["robot_profile_hash"],
        allow_test_policy=bool(raw["allow_test_policy"]),
        expected_test_only=bool(raw["expected_test_only"]),
        task_id=_text(
            _required(raw, "task_id", f"{location}.policy"), f"{location}.policy.task_id"
        ),
        task_text=_text(
            _required(raw, "task_text", f"{location}.policy"), f"{location}.policy.task_text"
        ),
        **values,
    )


def load_native_configuration(config_path: Path, project_root: Path) -> NativeConfiguration:
    """Load one native recipe with every referenced document snapshotted."""
    path = config_path if config_path.is_absolute() else project_root / config_path
    path = path.resolve()
    document = _load(path)
    location = str(path)
    allowed = {
        "schema_version",
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "robot_namespace",
        "lease_duration_ns",
        "renew_interval_ns",
        "command_acceptance_ns",
        "startup_timeout_s",
        "call_timeout_s",
        "command_timeout_s",
        "reset",
        "observation",
        "control",
        "action_source",
        "follower_constraints",
        "policy",
        "teacher",
        "motions",
        "logging",
        "recording",
    }
    _object(document, location, allowed)
    if document.get("schema_version") != 1:
        raise _error(f"{location}.schema_version", "must be 1")

    def referenced(name: str) -> Path:
        value = _text(_required(document, name, location), f"{location}.{name}")
        candidate = (path.parent / value).resolve()
        if not candidate.is_file():
            raise _error(f"{location}.{name}", f"file does not exist: {candidate}")
        return candidate

    profile_path = referenced("robot_profile")
    observation_path = referenced("observation_schema")
    action_path = referenced("action_schema")
    control_path = referenced("control_constraints")
    schema_root = project_root / "contracts/schemas"
    profile = load_profile(profile_path, schema_root)
    observation_document = _load(observation_path)
    action_document = _load(action_path)
    control_document = _load(control_path)
    try:
        validate_bindings(profile, observation_document, action_document, schema_root)
    except ContractError as error:
        raise ContractError("SCHEMA_MISMATCH", f"{location}: {error}") from error

    lease_duration_ns = _integer(
        _required(document, "lease_duration_ns", location),
        f"{location}.lease_duration_ns",
        MIN_LEASE_DURATION_NS,
        MAX_LEASE_DURATION_NS,
    )
    renew_interval_ns = _integer(
        _required(document, "renew_interval_ns", location),
        f"{location}.renew_interval_ns",
        1,
        MAX_LEASE_DURATION_NS,
    )
    if renew_interval_ns * 2 >= lease_duration_ns:
        raise _error(
            f"{location}.renew_interval_ns",
            "must be less than half of lease_duration_ns",
        )
    command_acceptance_ns = _integer(
        _required(document, "command_acceptance_ns", location),
        f"{location}.command_acceptance_ns",
        1,
        2**63 - 1,
    )
    startup_timeout_s = _number(
        _required(document, "startup_timeout_s", location),
        f"{location}.startup_timeout_s",
        positive=True,
    )
    call_timeout_s = _number(
        _required(document, "call_timeout_s", location), f"{location}.call_timeout_s", positive=True
    )
    command_timeout_s = _number(
        _required(document, "command_timeout_s", location),
        f"{location}.command_timeout_s",
        positive=True,
    )

    reset_raw = _object(
        _required(document, "reset", location), f"{location}.reset", {"enabled", "seed"}
    )
    if not isinstance(reset_raw.get("enabled"), bool):
        raise _error(f"{location}.reset.enabled", "must be a boolean")
    reset = NativeReset(
        reset_raw["enabled"],
        _integer(
            _required(reset_raw, "seed", f"{location}.reset"),
            f"{location}.reset.seed",
            0,
            2**63 - 1,
        ),
    )
    observation_raw = _object(
        _required(document, "observation", location),
        f"{location}.observation",
        {"max_skew_ns", "max_age_ns", "wall_arrival_timeout_s", "capacity", "reliable"},
    )
    if not isinstance(observation_raw.get("reliable", False), bool):
        raise _error(f"{location}.observation.reliable", "must be a boolean")
    observation = NativeObservation(
        _integer(
            _required(observation_raw, "max_skew_ns", f"{location}.observation"),
            f"{location}.observation.max_skew_ns",
            0,
            2**63 - 1,
        ),
        _integer(
            _required(observation_raw, "max_age_ns", f"{location}.observation"),
            f"{location}.observation.max_age_ns",
            1,
            2**63 - 1,
        ),
        _number(
            _required(observation_raw, "wall_arrival_timeout_s", f"{location}.observation"),
            f"{location}.observation.wall_arrival_timeout_s",
            positive=True,
        ),
        _integer(
            _required(observation_raw, "capacity", f"{location}.observation"),
            f"{location}.observation.capacity",
            1,
            4096,
        ),
        reliable=observation_raw.get("reliable", False),
    )
    control_raw = _object(
        _required(document, "control", location),
        f"{location}.control",
        {"submit_query_retries", "poll_interval_s", "stop_timeout_s", "terminal_timeout_s"},
    )
    control = NativeControl(
        _integer(
            _required(control_raw, "submit_query_retries", f"{location}.control"),
            f"{location}.control.submit_query_retries",
            0,
            8,
        ),
        _number(
            _required(control_raw, "poll_interval_s", f"{location}.control"),
            f"{location}.control.poll_interval_s",
            positive=True,
        ),
        _number(
            _required(control_raw, "stop_timeout_s", f"{location}.control"),
            f"{location}.control.stop_timeout_s",
            positive=True,
        ),
        _number(
            _required(control_raw, "terminal_timeout_s", f"{location}.control"),
            f"{location}.control.terminal_timeout_s",
            positive=True,
        ),
    )
    action_source = _text(
        _required(document, "action_source", location),
        f"{location}.action_source",
    )
    if action_source not in _ACTION_SOURCES:
        raise _error(f"{location}.action_source", f"must be one of {_ACTION_SOURCES}")
    follower, follower_document = _build_follower(document, location, profile)
    policy = _build_policy(
        document, location, profile, observation_document, action_document, follower
    )
    teacher = _build_teacher(document, location, follower)
    if action_source == "policy" and policy is None:
        raise _error(f"{location}.policy", "policy action source requires a policy section")
    if action_source == "teacher" and teacher is None:
        raise _error(f"{location}.teacher", "teacher action source requires a teacher section")
    if action_source != "policy" and policy is not None:
        raise _error(f"{location}.policy", "policy section requires action_source=policy")
    if action_source != "teacher" and teacher is not None:
        raise _error(f"{location}.teacher", "teacher section requires action_source=teacher")
    motions = _build_motions(document, location, profile, allow_empty=True)
    if motions:
        raise _error(
            f"{location}.motions",
            "must be empty; the streaming runtime has no diagnostic motions",
        )
    if control.terminal_timeout_s <= command_timeout_s:
        raise _error(
            f"{location}.control.terminal_timeout_s",
            "must exceed command_timeout_s",
        )
    logging = _build_logging(document, location)
    recording = _build_recording(document, location)
    recipe = NativeRecipe(
        schema_version=1,
        robot_profile=str(profile_path),
        observation_schema=str(observation_path),
        action_schema=str(action_path),
        robot_namespace=_text(
            _required(document, "robot_namespace", location), f"{location}.robot_namespace"
        ),
        lease_duration_ns=lease_duration_ns,
        renew_interval_ns=renew_interval_ns,
        command_acceptance_ns=command_acceptance_ns,
        startup_timeout_s=startup_timeout_s,
        call_timeout_s=call_timeout_s,
        command_timeout_s=command_timeout_s,
        reset=reset,
        observation=observation,
        control=control,
        motions=motions,
        action_source=action_source,
        follower=follower,
        policy=policy,
        teacher=teacher,
        logging=logging,
        recording=recording,
    )
    effective_recipe = asdict(recipe)
    effective_recipe["control_constraints"] = control_document
    if follower_document is not None:
        effective_recipe["follower_document"] = follower_document
    effective_recipe["required_capabilities"] = list(_REQUIRED_CAPABILITIES)
    snapshot = {
        "profile": _load(profile_path),
        "profile_hash": profile.digest,
        "observation": observation_document,
        "observation_hash": semantic_hash(observation_document),
        "action": action_document,
        "action_hash": semantic_hash(action_document),
        "action_source": action_source,
        "policy_test_only": None if policy is None else policy.expected_test_only,
        "recipe": effective_recipe,
    }
    return NativeConfiguration(
        recipe=recipe,
        profile=profile,
        schema_root=schema_root,
        control_document=control_document,
        _snapshot_json=json.dumps(snapshot, sort_keys=True, separators=(",", ":"), allow_nan=False),
    )
