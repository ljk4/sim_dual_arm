"""Strict configuration for the ROS 2 control runtime."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.models import ContractError, RobotProfile
from robot_contracts.profiles import load_profile, semantic_hash, validate_bindings
from robot_contracts.project_paths import project_path
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    Sha256HexStr,
    StrictFloat,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    as_tuple,
    contract_error,
    read_unique_yaml,
)
from robot_run_logging import LogSettings

from dual_arm_runtime.recording_queue import RecordingSettings
from dual_arm_runtime.runtime_limits import (
    INT64_MAX,
    MAX_CHUNKS,
    MAX_TASK_BUDGET_NS,
    STREAM_CHUNK_SIZE,
    STREAM_PERIOD_NS,
)
from dual_arm_runtime.simulation.options import ResetConfig, TeacherConfig, task_enabled
from dual_arm_runtime.simulation.options import required_capabilities as simulation_capabilities

MIN_LEASE_DURATION_NS = 100_000_000
MAX_LEASE_DURATION_NS = 10_000_000_000
UINT64_MAX = 2**64 - 1
REQUIRED_CAPABILITIES = (
    "continuous_chunks",
    "exact_capture",
    "lease",
    "stop",
    "command_query",
    "control_events",
    "follower_reference",
    "bounded_target_follower",
    "execution_feedback.gripper_set",
)
_VECTOR = Annotated[
    tuple[StrictFloat, ...], BeforeValidator(as_tuple), Field(min_length=1, max_length=64)
]
_POSITIVE_VECTOR = Annotated[
    tuple[PositiveFloat, ...], BeforeValidator(as_tuple), Field(min_length=1, max_length=64)
]


def _error(location: str, message: str) -> ContractError:
    return ContractError("SCHEMA_MISMATCH", f"{location}: {message}")


class ObservationConfig(StrictMappingModel):
    """Observation timing and buffering.

    ``max_skew_ns`` and ``max_age_ns`` default to the observation profile's
    values when the recipe omits them; an explicit recipe value overrides.
    """

    max_skew_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    max_age_ns: Annotated[PositiveInt, Field(le=INT64_MAX)]
    wall_arrival_timeout_s: PositiveFloat
    capacity: Annotated[PositiveInt, Field(le=4096)]
    reliable: bool = False


class ControlConfig(StrictMappingModel):
    submit_query_retries: Annotated[int, Field(strict=True, ge=0, le=8)]
    poll_interval_s: PositiveFloat
    stop_timeout_s: PositiveFloat
    terminal_timeout_s: PositiveFloat


class FollowerConfig(StrictMappingModel):
    target_count: Annotated[int, Field(strict=True, ge=2, le=128)]
    max_target_deviations: _POSITIVE_VECTOR
    max_terminal_tail_duration_ns: Annotated[PositiveInt, Field(le=3_000_000_000)]
    duration_grid_ns: Annotated[PositiveInt, Field(le=3_000_000_000)]


class FollowerConstraints(StrictMappingModel):
    joint_names: Annotated[
        tuple[NonEmptyStr, ...], BeforeValidator(as_tuple), Field(min_length=1, max_length=64)
    ]
    target_count: Annotated[int, Field(strict=True, ge=2, le=128)]
    max_target_deviations: _POSITIVE_VECTOR
    max_terminal_tail_duration_s: PositiveFloat

    @property
    def max_terminal_tail_duration_ns(self) -> int:
        return round(self.max_terminal_tail_duration_s * 1_000_000_000)


class PolicyConfig(StrictMappingModel):
    """Policy RPC identity, bounds and the finite diagnostic stream budget.

    Model and processor digests are not configured here: the runtime learns
    them from the Policy Service handshake and never loads a model itself.
    """

    endpoint: Annotated[NonEmptyStr, Field(pattern=r"^[^:\s]+:\d+$")]
    observation_schema_hash: Sha256HexStr
    action_schema_hash: Sha256HexStr
    robot_profile_hash: Sha256HexStr
    allow_test_policy: bool
    expected_test_only: bool
    task_id: NonEmptyStr
    task_text: NonEmptyStr
    chunk_size: Annotated[int, Field(strict=True, ge=STREAM_CHUNK_SIZE, le=STREAM_CHUNK_SIZE)] = (
        STREAM_CHUNK_SIZE
    )
    action_period_ns: Annotated[
        int, Field(strict=True, ge=STREAM_PERIOD_NS, le=STREAM_PERIOD_NS)
    ] = STREAM_PERIOD_NS
    initial_lead_ns: PositiveInt = 250_000_000
    max_chunks: Annotated[PositiveInt, Field(le=MAX_CHUNKS)] = 8
    total_sim_deadline_ns: PositiveInt = 8_000_000_000
    total_wall_deadline_s: PositiveFloat = 45.0
    connect_timeout_s: PositiveFloat = 2.0
    open_deadline_s: PositiveFloat = 2.0
    predict_deadline_s: PositiveFloat = 2.0
    close_deadline_s: PositiveFloat = 2.0
    max_message_bytes: PositiveInt = 16_777_216
    poll_interval_s: PositiveFloat = 0.02
    termination_mode: Literal["finite_chunks", "task"] = "finite_chunks"
    task_stop_margin_s: PositiveFloat = 1.0
    task_sim_budget_ns: Annotated[PositiveInt, Field(le=MAX_TASK_BUDGET_NS)] = 30_000_000_000
    startup_settle_sim_ns: Annotated[NonNegativeInt, Field(le=15_000_000_000)] = 0
    warmup_on_settle: bool = False
    async_inference: bool = True
    prefetch_lead_ns: Annotated[NonNegativeInt, Field(le=1_000_000_000)] = 0
    action_execution_latency_ns: Annotated[NonNegativeInt, Field(le=500_000_000)] = 0

    @model_validator(mode="after")
    def _policy(self) -> PolicyConfig:
        if self.expected_test_only and not self.allow_test_policy:
            raise contract_error(
                "allow_test_policy", "a test-only policy requires allow_test_policy: true"
            )
        return self


class RuntimeRecipe(StrictMappingModel):
    robot_profile: NonEmptyStr
    observation_schema: NonEmptyStr
    action_schema: NonEmptyStr
    control_constraints: NonEmptyStr
    robot_namespace: NonEmptyStr
    lease_duration_ns: Annotated[
        int, Field(strict=True, ge=MIN_LEASE_DURATION_NS, le=MAX_LEASE_DURATION_NS)
    ]
    renew_interval_ns: Annotated[PositiveInt, Field(le=MAX_LEASE_DURATION_NS)]
    command_acceptance_ns: Annotated[PositiveInt, Field(le=INT64_MAX)]
    startup_timeout_s: PositiveFloat
    call_timeout_s: PositiveFloat
    command_timeout_s: PositiveFloat
    reset: ResetConfig = ResetConfig()
    observation: ObservationConfig
    control: ControlConfig
    action_source: Annotated[str, Field(pattern=r"^(policy|teacher)$")]
    # Explicit capability identifier for a scenario Teacher wire extension.
    # It is required for teacher mode and absent for policy runs.
    teacher_api: NonEmptyStr | None = None
    follower_config: NonEmptyStr | None = Field(default=None, exclude=True)
    policy: PolicyConfig | None = None
    teacher: TeacherConfig | None = None
    follower: FollowerConfig | None = Field(default=None, exclude=True)
    logging: LogSettings = LogSettings()
    recording: RecordingSettings = RecordingSettings()

    @model_validator(mode="before")
    @classmethod
    def _settings(cls, data: object) -> object:
        if not isinstance(data, dict):
            return data
        data = dict(data)
        for name, factory in (("logging", LogSettings), ("recording", RecordingSettings)):
            raw = data.get(name, {})
            if isinstance(raw, dict):
                try:
                    data[name] = factory(**raw)
                except (TypeError, ValueError) as error:
                    raise contract_error(name, str(error)) from error
        return data

    @model_validator(mode="after")
    def _recipe(self) -> RuntimeRecipe:
        if self.renew_interval_ns * 2 >= self.lease_duration_ns:
            raise contract_error("renew_interval_ns", "must be less than half of lease_duration_ns")
        if self.control.terminal_timeout_s <= self.command_timeout_s:
            raise contract_error("control.terminal_timeout_s", "must exceed command_timeout_s")
        if self.action_source == "policy" and self.policy is None:
            raise contract_error("policy", "policy section requires a policy block")
        if self.action_source == "teacher" and self.teacher is None:
            raise contract_error("teacher", "teacher section requires a teacher block")
        if self.action_source == "teacher" and self.teacher_api is None:
            raise contract_error(
                "teacher_api",
                "teacher action_source requires an explicit extension capability identifier",
            )
        if self.action_source != "teacher" and self.teacher_api is not None:
            raise contract_error("teacher_api", "teacher_api requires action_source=teacher")
        if self.action_source != "policy" and self.policy is not None:
            raise contract_error("policy", "policy section requires action_source=policy")
        if self.action_source != "teacher" and self.teacher is not None:
            raise contract_error("teacher", "teacher section requires action_source=teacher")
        if (self.policy is not None or self.teacher is not None) and self.follower is None:
            raise contract_error(
                "follower_config", "policy/teacher action sources require follower config"
            )
        for name, stream in (("policy", self.policy), ("teacher", self.teacher)):
            if stream is None:
                continue
            if stream.termination_mode == "task":
                if stream.task_stop_margin_s * 1_000_000_000 >= stream.task_sim_budget_ns:
                    raise contract_error(
                        f"{name}.task_stop_margin_s", "must leave a positive task budget"
                    )
                assert self.follower is not None
                covered = stream.max_chunks * self.follower.target_count * stream.action_period_ns
                if covered < stream.task_sim_budget_ns:
                    required = stream.task_sim_budget_ns // (
                        self.follower.target_count * stream.action_period_ns
                    )
                    raise contract_error(
                        f"{name}.max_chunks", f"task mode needs at least {required} chunks"
                    )
                # The wall watchdog must leave room for RPC, reset and bounded
                # stop after a full simulation budget at real-time speed.
                wall_floor_s = stream.task_sim_budget_ns / 1e9 + 20.0
                if stream.total_wall_deadline_s < wall_floor_s:
                    raise contract_error(
                        f"{name}.total_wall_deadline_s",
                        f"must be at least {wall_floor_s:.1f} s for this task budget",
                    )
        return self

    @property
    def test_only(self) -> bool:
        return False

    @property
    def simulation_capabilities(self) -> tuple[str, ...]:
        return simulation_capabilities(
            reset_enabled=self.reset.enabled,
            task_enabled=task_enabled(self),
            teacher_api=self.teacher_api,
        )

    @property
    def required_capabilities(self) -> tuple[str, ...]:
        reliable = ("reliable_sensor_streams",) if self.observation.reliable else ()
        return (*REQUIRED_CAPABILITIES, *reliable, *self.simulation_capabilities)


class RuntimeConfiguration(StrictMappingModel):
    recipe: RuntimeRecipe
    profile: RobotProfile
    schema_root: Path
    control_document: dict[str, Any]
    snapshot_json: str = Field(exclude=True, repr=False)
    selection: dict[str, Any] | None = None
    configuration_source: str = ""

    snapshot = property(lambda self: json.loads(self.snapshot_json))

    @property
    def required_capabilities(self) -> tuple[str, ...]:
        return self.recipe.required_capabilities


def _load(path: Path) -> dict[str, Any]:
    try:
        document = read_unique_yaml(path)
    except UniqueKeyError as error:
        raise _error(str(path), f"{error.field}: {error.message}") from error
    if not isinstance(document, dict):
        raise _error(str(path), "expected an object")
    return document


def _referenced(document: dict[str, Any], name: str, base_dir: Path, label: str) -> Path:
    value = document.get(name)
    if not isinstance(value, str) or not value:
        raise _error(f"{label}.{name}", "field is required")
    candidate = (base_dir / value).resolve()
    if not candidate.is_file():
        raise _error(f"{label}.{name}", f"file does not exist: {candidate}")
    return candidate


def _load_follower(document: dict[str, Any], base_dir: Path, label: str, profile: RobotProfile):
    name = document.get("follower_config")
    if name is None:
        return None, None
    if not isinstance(name, str) or not name:
        raise _error(f"{label}.follower_config", "field is required")
    path = (base_dir / name).resolve()
    if not path.is_file():
        raise _error(f"{label}.follower_config", f"file does not exist: {path}")
    raw = _load(path)
    try:
        constraints = FollowerConstraints(**raw)
    except StrictMappingError as error:
        raise _error(str(path), f"{error.field}: {error.message}") from error
    if constraints.joint_names != profile.joint_names:
        raise _error(f"{path}.joint_names", "must match the robot profile order")
    if len(constraints.max_target_deviations) != len(profile.joints):
        raise _error(f"{path}.max_target_deviations", "must match the profile joints")
    if constraints.max_terminal_tail_duration_ns <= 0:
        raise _error(f"{path}.max_terminal_tail_duration_s", "must convert to positive ns")
    return (
        FollowerConfig(
            target_count=constraints.target_count,
            max_target_deviations=constraints.max_target_deviations,
            max_terminal_tail_duration_ns=constraints.max_terminal_tail_duration_ns,
            duration_grid_ns=2_000_000,
        ),
        raw,
    )


def load_runtime_configuration(
    config_path: Path,
    project_root: Path,
    *,
    seed: int | None = None,
    policy_endpoint: str | None = None,
) -> RuntimeConfiguration:
    path = project_path(config_path, project_root).resolve()
    document = _load(path)
    if seed is not None:
        reset = document.get("reset")
        if not isinstance(reset, dict):
            raise _error(f"{path}.reset", "field is required")
        reset["seed"] = seed
    return load_runtime_configuration_document(
        document,
        source=str(path),
        base_dir=path.parent,
        project_root=project_root,
        policy_endpoint=policy_endpoint,
    )


def load_runtime_configuration_document(
    document: dict[str, Any],
    *,
    source: str,
    base_dir: Path,
    project_root: Path,
    policy_endpoint: str | None = None,
    selection: dict[str, Any] | None = None,
) -> RuntimeConfiguration:
    """Validate one already-resolved in-memory recipe.

    ``base_dir`` resolves the recipe's own relative references; a scenario
    loader has already replaced profile/schema paths with their absolute
    binding paths.  No file is written and no prepared directory is required.
    """
    path = Path(source) if source.startswith("/") else base_dir / source
    label = str(path)
    profile_path = _referenced(document, "robot_profile", base_dir, label)
    observation_path = _referenced(document, "observation_schema", base_dir, label)
    action_path = _referenced(document, "action_schema", base_dir, label)
    control_path = _referenced(document, "control_constraints", base_dir, label)
    schema_root = project_path("shared/common/interfaces/schemas", project_root)
    profile = load_profile(profile_path, schema_root)
    observation_document = _load(observation_path)
    action_document = _load(action_path)
    control_document = _load(control_path)
    try:
        validate_bindings(profile, observation_document, action_document, schema_root)
    except ContractError as error:
        raise _error(label, str(error)) from error
    # The observation profile is the single default source for timing bounds.
    # A recipe only overrides them when it explicitly declares the same keys.
    observation = document.get("observation")
    if not isinstance(observation, dict):
        raise _error(f"{label}.observation", "field is required")
    observation = dict(observation)
    for name in ("max_skew_ns", "max_age_ns"):
        observation.setdefault(name, observation_document[name])
    document = {**document, "observation": observation}
    follower, follower_document = _load_follower(document, base_dir, label, profile)
    observation_hash = semantic_hash(observation_document)
    action_hash = semantic_hash(action_document)
    policy = document.get("policy")
    if policy_endpoint is not None and not isinstance(policy, dict):
        raise _error(label, "--policy-endpoint requires a Policy recipe")
    if isinstance(policy, dict):
        policy = {**policy}
        if policy_endpoint is not None:
            policy["endpoint"] = policy_endpoint
        policy.setdefault("observation_schema_hash", observation_hash)
        policy.setdefault("action_schema_hash", action_hash)
        policy.setdefault("robot_profile_hash", profile.digest)
        document = {**document, "policy": policy}
    document = {
        **document,
        "robot_profile": str(profile_path),
        "observation_schema": str(observation_path),
        "action_schema": str(action_path),
        "control_constraints": str(control_path),
        "follower": follower,
    }
    try:
        recipe = RuntimeRecipe(**document)
    except StrictMappingError as error:
        raise _error(label, f"{error.field}: {error.message}") from error
    effective = recipe.model_dump(
        mode="json",
        exclude={"logging", "recording", "follower", "control_constraints", "follower_config"},
    )
    effective["control_constraints"] = control_document
    if follower_document is not None:
        effective["follower_document"] = follower_document
    effective["follower"] = None if follower is None else follower.model_dump(mode="json")
    effective["required_capabilities"] = list(recipe.required_capabilities)
    effective["logging"] = dict(vars(recipe.logging))
    effective["recording"] = dict(vars(recipe.recording))
    snapshot = {
        "profile": _load(profile_path),
        "profile_hash": profile.digest,
        "observation": observation_document,
        "observation_hash": observation_hash,
        "action": action_document,
        "action_hash": action_hash,
        "action_source": recipe.action_source,
        "policy_test_only": None if recipe.policy is None else recipe.policy.expected_test_only,
        "recipe": effective,
        "selection": selection,
        "configuration_source": source,
    }
    return RuntimeConfiguration(
        recipe=recipe,
        profile=profile,
        schema_root=schema_root,
        control_document=control_document,
        snapshot_json=json.dumps(snapshot, sort_keys=True, separators=(",", ":"), allow_nan=False),
        selection=selection,
        configuration_source=source,
    )
