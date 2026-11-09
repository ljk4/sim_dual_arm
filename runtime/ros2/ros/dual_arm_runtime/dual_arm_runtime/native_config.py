"""Strict configuration for the diagnostic native runtime."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Annotated, Any

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.models import ContractError, RobotProfile
from robot_contracts.profiles import load_profile, semantic_hash, validate_bindings
from robot_contracts.validation import (
    NonEmptyStr,
    PositiveFloat,
    PositiveInt,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    as_tuple,
    contract_error,
    read_unique_yaml,
)
from robot_run_logging import LogSettings

from dual_arm_runtime.recording_queue import RecordingSettings

from .native_config_sections import (
    INT64_MAX,
    MAX_LEASE_DURATION_NS,
    MIN_LEASE_DURATION_NS,
    REQUIRED_CAPABILITIES,
    FollowerConstraints,
    NativeControl,
    NativeFollower,
    NativeMotion,
    NativeObservation,
    NativePolicy,
    NativeReset,
    NativeTeacher,
)


def _error(location: str, message: str) -> ContractError:
    return ContractError("SCHEMA_MISMATCH", f"{location}: {message}")


class NativeRecipe(StrictMappingModel):
    schema_version: Annotated[int, Field(strict=True, ge=1, le=1)]
    robot_profile: NonEmptyStr
    observation_schema: NonEmptyStr
    action_schema: NonEmptyStr
    control_constraints: NonEmptyStr
    robot_namespace: NonEmptyStr
    lease_duration_ns: Annotated[int, Field(strict=True, ge=MIN_LEASE_DURATION_NS,
                                            le=MAX_LEASE_DURATION_NS)]
    renew_interval_ns: Annotated[PositiveInt, Field(le=MAX_LEASE_DURATION_NS)]
    command_acceptance_ns: Annotated[PositiveInt, Field(le=INT64_MAX)]
    startup_timeout_s: PositiveFloat
    call_timeout_s: PositiveFloat
    command_timeout_s: PositiveFloat
    reset: NativeReset
    observation: NativeObservation
    control: NativeControl
    motions: Annotated[tuple[NativeMotion, ...], BeforeValidator(as_tuple),
                       Field(max_length=64)] = ()
    action_source: Annotated[str, Field(pattern=r"^(policy|teacher)$")]
    follower_constraints: NonEmptyStr | None = Field(default=None, exclude=True)
    policy: NativePolicy | None = None
    teacher: NativeTeacher | None = None
    follower: NativeFollower | None = Field(default=None, exclude=True)
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
    def _recipe(self) -> NativeRecipe:
        if self.renew_interval_ns * 2 >= self.lease_duration_ns:
            raise contract_error("renew_interval_ns",
                                 "must be less than half of lease_duration_ns")
        if self.motions:
            raise contract_error("motions",
                                 "must be empty; the streaming runtime has no motions")
        if self.control.terminal_timeout_s <= self.command_timeout_s:
            raise contract_error("control.terminal_timeout_s",
                                 "must exceed command_timeout_s")
        if self.action_source == "policy" and self.policy is None:
            raise contract_error("policy", "policy section requires a policy block")
        if self.action_source == "teacher" and self.teacher is None:
            raise contract_error("teacher", "teacher section requires a teacher block")
        if self.action_source != "policy" and self.policy is not None:
            raise contract_error("policy", "policy section requires action_source=policy")
        if self.action_source != "teacher" and self.teacher is not None:
            raise contract_error("teacher", "teacher section requires action_source=teacher")
        if (self.policy is not None or self.teacher is not None) and self.follower is None:
            raise contract_error("follower_constraints",
                                 "policy/teacher action sources require follower constraints")
        for name, stream in (("policy", self.policy), ("teacher", self.teacher)):
            if stream is None:
                continue
            if self.follower is not None and stream.target_count != self.follower.target_count:
                raise contract_error(f"{name}.target_count",
                                     "must equal the follower target_count")
            if stream.termination_mode == "task":
                if stream.task_stop_margin_s * 1_000_000_000 >= stream.task_sim_budget_ns:
                    raise contract_error(f"{name}.task_stop_margin_s",
                                         "must leave a positive task budget")
                covered = stream.max_chunks * stream.target_count * stream.action_period_ns
                if covered < stream.task_sim_budget_ns:
                    required = stream.task_sim_budget_ns // (
                        stream.target_count * stream.action_period_ns)
                    raise contract_error(f"{name}.max_chunks",
                                         f"task mode needs at least {required} chunks")
        if self.teacher is not None and self.teacher.target_count != 5:
            raise contract_error("teacher.target_count", "must be 5 for the teacher stream")
        return self

    @property
    def test_only(self) -> bool:
        return False


class NativeConfiguration(StrictMappingModel):
    recipe: NativeRecipe
    profile: RobotProfile
    schema_root: Path
    control_document: dict[str, Any]
    snapshot_json: str = Field(exclude=True, repr=False)

    snapshot = property(lambda self: json.loads(self.snapshot_json))
    required_capabilities = property(lambda self: REQUIRED_CAPABILITIES)


def _load(path: Path) -> dict[str, Any]:
    try:
        document = read_unique_yaml(path)
    except UniqueKeyError as error:
        raise _error(str(path), f"{error.field}: {error.message}") from error
    if not isinstance(document, dict):
        raise _error(str(path), "expected an object")
    return document


def _referenced(document: dict[str, Any], name: str, base: Path) -> Path:
    value = document.get(name)
    if not isinstance(value, str) or not value:
        raise _error(f"{base}.{name}", "field is required")
    candidate = (base.parent / value).resolve()
    if not candidate.is_file():
        raise _error(f"{base}.{name}", f"file does not exist: {candidate}")
    return candidate


def _load_follower(document: dict[str, Any], base: Path, profile: RobotProfile):
    name = document.get("follower_constraints")
    if name is None:
        return None, None
    if not isinstance(name, str) or not name:
        raise _error(f"{base}.follower_constraints", "field is required")
    path = (base.parent / name).resolve()
    if not path.is_file():
        raise _error(f"{base}.follower_constraints", f"file does not exist: {path}")
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
        NativeFollower(
            target_count=constraints.target_count,
            max_target_deviations=constraints.max_target_deviations,
            max_terminal_tail_duration_ns=constraints.max_terminal_tail_duration_ns,
            duration_grid_ns=2_000_000,
        ),
        raw,
    )


def load_native_configuration(config_path: Path, project_root: Path) -> NativeConfiguration:
    path = (config_path if config_path.is_absolute() else project_root / config_path).resolve()
    document = _load(path)
    profile_path = _referenced(document, "robot_profile", path)
    observation_path = _referenced(document, "observation_schema", path)
    action_path = _referenced(document, "action_schema", path)
    control_path = _referenced(document, "control_constraints", path)
    schema_root = project_root / "contracts/schemas"
    profile = load_profile(profile_path, schema_root)
    observation_document = _load(observation_path)
    action_document = _load(action_path)
    control_document = _load(control_path)
    try:
        validate_bindings(profile, observation_document, action_document, schema_root)
    except ContractError as error:
        raise _error(str(path), str(error)) from error
    follower, follower_document = _load_follower(document, path, profile)
    observation_hash = semantic_hash(observation_document)
    action_hash = semantic_hash(action_document)
    policy = document.get("policy")
    if isinstance(policy, dict):
        policy = {**policy}
        policy.setdefault("observation_schema_hash", observation_hash)
        policy.setdefault("action_schema_hash", action_hash)
        policy.setdefault("robot_profile_hash", profile.digest)
        document = {**document, "policy": policy}
    document = {**document, "robot_profile": str(profile_path),
                "observation_schema": str(observation_path),
                "action_schema": str(action_path),
                "control_constraints": str(control_path), "follower": follower}
    try:
        recipe = NativeRecipe(**document)
    except StrictMappingError as error:
        raise _error(str(path), f"{error.field}: {error.message}") from error
    effective = recipe.model_dump(
        mode="json",
        exclude={"logging", "recording", "follower", "control_constraints",
                 "follower_constraints"})
    effective["control_constraints"] = control_document
    if follower_document is not None:
        effective["follower_document"] = follower_document
    effective["follower"] = None if follower is None else follower.model_dump(mode="json")
    effective["required_capabilities"] = list(REQUIRED_CAPABILITIES)
    effective["logging"] = dict(vars(recipe.logging))
    effective["recording"] = dict(vars(recipe.recording))
    snapshot = {"profile": _load(profile_path), "profile_hash": profile.digest,
                "observation": observation_document, "observation_hash": observation_hash,
                "action": action_document, "action_hash": action_hash,
                "action_source": recipe.action_source,
                "policy_test_only": None if recipe.policy is None
                else recipe.policy.expected_test_only, "recipe": effective}
    return NativeConfiguration(
        recipe=recipe, profile=profile, schema_root=schema_root,
        control_document=control_document,
        snapshot_json=json.dumps(snapshot, sort_keys=True, separators=(",", ":"),
                                 allow_nan=False),
    )
