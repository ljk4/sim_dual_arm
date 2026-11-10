from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Annotated

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.profiles import semantic_hash
from robot_contracts.validation import (
    NonEmptyStr,
    PositiveFloat,
    PositiveInt,
    StrictFloat,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    as_tuple,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

PHASES = ("pregrasp", "grasp", "close", "lift", "transport", "lower", "open", "retreat", "stable")
HARD_MAX_CHUNKS = 4096
_SHA256 = r"^[0-9a-f]{64}$"
Vec3 = Annotated[tuple[StrictFloat, StrictFloat, StrictFloat], BeforeValidator(as_tuple)]
StackTaskConfigError = StrictMappingError

class ObjectsSection(StrictMappingModel):
    source: NonEmptyStr
    target: NonEmptyStr
    source_half_extents_m: Vec3
    target_half_extents_m: Vec3

    @model_validator(mode="after")
    def _cubes(self) -> ObjectsSection:
        if self.source == self.target:
            raise contract_error("source", "must differ from target")
        for name in ("source_half_extents_m", "target_half_extents_m"):
            values = getattr(self, name)
            if any(not 1e-4 <= value <= 1.0 for value in values):
                raise contract_error(name, "must be in 1e-4..1.0")
            if values[0] != values[1] or values[0] != values[2]:
                raise contract_error(name, "must describe a cube")
        return self

class SuccessThresholds(StrictMappingModel):
    """All SI thresholds used by the independent evaluator."""

    support_xy_margin_m: Annotated[StrictFloat, Field(ge=0.0, le=0.02)]
    support_height_tolerance_m: Annotated[PositiveFloat, Field(gt=1e-5, le=0.02)]
    max_tilt_rad: Annotated[PositiveFloat, Field(gt=1e-6, le=math.pi / 2.0)]
    max_linear_speed_m_s: Annotated[PositiveFloat, Field(gt=1e-6, le=1.0)]
    max_angular_speed_rad_s: Annotated[PositiveFloat, Field(gt=1e-6, le=10.0)]
    min_support_normal_force_n: Annotated[PositiveFloat, Field(gt=1e-6, le=100.0)]
    min_table_normal_force_n: Annotated[PositiveFloat, Field(gt=1e-6, le=100.0)]
    min_vertical_normal_component: Annotated[PositiveFloat, Field(gt=0.0, le=1.0)]
    min_gripper_opening_m: Annotated[PositiveFloat, Field(gt=1e-6, le=0.08)]
    min_tcp_clearance_m: Annotated[PositiveFloat, Field(gt=1e-6, le=1.0)]
    stable_duration_s: Annotated[PositiveFloat, Field(ge=2.0, le=10.0)]
    max_task_sim_duration_s: Annotated[PositiveFloat, Field(gt=3.0, le=30.0)]
    max_task_wall_duration_s: Annotated[PositiveFloat, Field(gt=1.0, le=300.0)]

    @model_validator(mode="after")
    def _duration(self) -> SuccessThresholds:
        if self.max_task_sim_duration_s <= self.stable_duration_s:
            raise contract_error("max_task_sim_duration_s", "must exceed stable_duration_s")
        return self

class TeacherPlanConfig(StrictMappingModel):
    enabled: bool
    kinematics_config_path: Annotated[
        Path,
        BeforeValidator(lambda value: Path(value) if isinstance(value, str) else value),
        Field(validation_alias="kinematics_config"),
    ]
    chunk_size: Annotated[int, Field(strict=True, ge=60, le=60)]
    action_period_ns: Annotated[int, Field(strict=True, ge=50_000_000, le=50_000_000)]
    ik_wall_timeout_s: Annotated[PositiveFloat, Field(gt=0.1, le=60.0)]
    plan_wall_timeout_s: Annotated[PositiveFloat, Field(gt=0.1, le=60.0)]
    max_plan_duration_s: Annotated[PositiveFloat, Field(gt=1.0, le=29.9)]
    max_joint_velocity_fraction: Annotated[PositiveFloat, Field(gt=0.05, le=1.0)]
    max_joint_acceleration_fraction: Annotated[PositiveFloat, Field(gt=0.05, le=1.0)]
    gripper_velocity_m_s: Annotated[PositiveFloat, Field(gt=1e-4, le=0.04)]
    gripper_acceleration_m_s2: Annotated[PositiveFloat, Field(gt=1e-4, le=1.0)]
    stable_hold_s: Annotated[PositiveFloat, Field(ge=2.0, le=10.0)]
    max_chunk_requests: Annotated[PositiveInt, Field(ge=120, le=4096)]
    chunk_cache_entries: Annotated[PositiveInt, Field(ge=120, le=4096)]
    phase_min_duration_s: dict[str, Annotated[PositiveFloat, Field(ge=0.05, le=15.0)]]
    position_tolerance_m: Annotated[PositiveFloat, Field(gt=1e-5, le=0.05)]
    orientation_tolerance_rad: Annotated[PositiveFloat, Field(gt=1e-5, le=0.5)]
    lift_height_m: Annotated[PositiveFloat, Field(gt=0.01, le=0.5)]
    release_gripper_opening_m: Annotated[PositiveFloat, Field(gt=0.04, le=0.08)]
    retreat_tcp_distance_m: Annotated[PositiveFloat, Field(gt=0.02, le=0.5)]
    collision_sample_step_rad: Annotated[PositiveFloat, Field(gt=0.002, le=0.1)]
    collision_max_samples_per_segment: Annotated[PositiveInt, Field(ge=2, le=2000)]
    phase_joint_tolerance_rad: Annotated[PositiveFloat, Field(gt=1e-5, le=0.5)]
    phase_velocity_tolerance_rad_s: Annotated[PositiveFloat, Field(gt=1e-5, le=2.0)]
    gripper_position_tolerance_m: Annotated[PositiveFloat, Field(gt=1e-5, le=0.05)]
    gripper_velocity_tolerance_m_s: Annotated[PositiveFloat, Field(gt=1e-5, le=0.5)]
    contact_force_min_n: Annotated[PositiveFloat, Field(gt=1e-4, le=100.0)]
    finger_speed_max_m_s: Annotated[PositiveFloat, Field(gt=1e-5, le=1.0)]
    gate_stable_duration_s: Annotated[PositiveFloat, Field(ge=0.02, le=2.0)]
    phase_gate_extra_timeout_s: Annotated[PositiveFloat, Field(gt=0.1, le=20.0)]

    @model_validator(mode="after")
    def _teacher(self) -> TeacherPlanConfig:
        if set(self.phase_min_duration_s) != set(PHASES):
            raise contract_error("phase_min_duration_s", f"must define {PHASES}")
        if self.ik_wall_timeout_s > self.plan_wall_timeout_s:
            raise contract_error("ik_wall_timeout_s", "must not exceed plan_wall_timeout_s")
        if not self.kinematics_config_path.is_file():
            raise contract_error("kinematics_config", "must reference an existing file")
        return self

class CapacityConfig(StrictMappingModel):
    expected_chunks: Annotated[PositiveInt, Field(ge=120, le=HARD_MAX_CHUNKS)]
    expected_references: Annotated[PositiveInt, Field(ge=120, le=HARD_MAX_CHUNKS)]
    expected_events: Annotated[PositiveInt, Field(ge=360, le=4 * HARD_MAX_CHUNKS)]
    max_episode_chunks: Annotated[PositiveInt, Field(ge=120, le=HARD_MAX_CHUNKS)]
    max_teacher_requests: Annotated[PositiveInt, Field(ge=120, le=HARD_MAX_CHUNKS)]
    rpc_cache_entries: Annotated[PositiveInt, Field(ge=120, le=4096)]
    backend_chunk_history: Annotated[PositiveInt, Field(ge=120, le=HARD_MAX_CHUNKS)]

class StackTaskConfig(StrictMappingModel):
    source: Path = Path("")
    file_sha256: Annotated[str, Field(pattern=_SHA256)] = "0" * 64
    config_digest: Annotated[str, Field(pattern=_SHA256)] = "0" * 64
    schema_version: Annotated[int, Field(strict=True, ge=1, le=1)]
    objects: ObjectsSection
    success: SuccessThresholds
    teacher: TeacherPlanConfig
    capacity: CapacityConfig

    @model_validator(mode="after")
    def _task(self) -> StackTaskConfig:
        success, teacher, capacity = self.success, self.teacher, self.capacity
        checks = (
            (teacher.phase_min_duration_s["stable"] + 1e-12 >= success.stable_duration_s,
             "teacher.phase_min_duration_s.stable", "must cover success.stable_duration_s"),
            (teacher.stable_hold_s + 1e-12 >= success.stable_duration_s,
             "teacher.stable_hold_s", "must cover stable_duration_s"),
            (teacher.max_plan_duration_s > success.stable_duration_s + 1.0,
             "teacher.max_plan_duration_s", "must exceed stable_duration_s + 1 s"),
            (capacity.max_episode_chunks >= capacity.expected_chunks,
             "capacity.max_episode_chunks", "must cover expected_chunks"),
            (capacity.expected_references <= capacity.max_episode_chunks,
             "capacity.expected_references", "must not exceed max_episode_chunks"),
            (capacity.expected_events >= 3 * capacity.expected_chunks,
             "capacity.expected_events", "must cover three events per chunk"),
            (capacity.rpc_cache_entries >= capacity.max_teacher_requests,
             "capacity.rpc_cache_entries", "must cover max_teacher_requests"),
            (capacity.backend_chunk_history >= capacity.max_episode_chunks,
             "capacity.backend_chunk_history", "must cover max_episode_chunks"),
            (capacity.max_teacher_requests >= capacity.max_episode_chunks,
             "capacity.max_teacher_requests", "must cover max_episode_chunks"),
        )
        for ok, field, message in checks:
            if not ok:
                raise contract_error(field, message)
        return self

    source_object = property(lambda self: self.objects.source)
    target_object = property(lambda self: self.objects.target)
    source_half_extents_m = property(lambda self: self.objects.source_half_extents_m)
    target_half_extents_m = property(lambda self: self.objects.target_half_extents_m)


def load_task_config(path: Path | str) -> StackTaskConfig:
    """Load and validate the task YAML before any MuJoCo or ROS object exists."""
    source_path = Path(path).resolve()
    try:
        raw_bytes, document = source_path.read_bytes(), read_unique_yaml(source_path)
    except OSError as error:
        raise StackTaskConfigError(source_path, "<file>", str(error)) from error
    except UniqueKeyError as error:
        raise StackTaskConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise StackTaskConfigError(source_path, "<root>", "must be a mapping")
    original_document, teacher = document, document.get("teacher")
    if isinstance(teacher, dict):
        candidate = Path(str(teacher.get("kinematics_config", "")))
        if not candidate.is_absolute():
            candidate = (source_path.parent / candidate).resolve()
        if not candidate.is_file():
            raise StackTaskConfigError(source_path, "teacher.kinematics_config",
                                       "must reference an existing file")
        document = {**document, "teacher": {**teacher, "kinematics_config": candidate}}
    return validate_config_model(
        StackTaskConfig,
        {**document, "source": source_path,
         "file_sha256": hashlib.sha256(raw_bytes).hexdigest(),
         "config_digest": semantic_hash(original_document)},
        source_path, StackTaskConfigError,
    )
