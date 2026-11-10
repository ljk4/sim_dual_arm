"""Strict configuration for the independent stack task and program teacher."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Annotated, Any

from pydantic import Field, model_validator
from robot_contracts.profiles import semantic_hash
from robot_contracts.validation import (
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from .task_config_sections import (
    PHASES,
    CapacityConfig,
    ObjectsSection,
    SuccessThresholds,
    TeacherPlanConfig,
)

__all__ = [
    "PHASES", "CapacityConfig", "ObjectsSection", "StackTaskConfig",
    "StackTaskConfigError", "SuccessThresholds", "TeacherPlanConfig", "load_task_config",
    "validate_scene_geometry",
]

_SHA256 = r"^[0-9a-f]{64}$"
StackTaskConfigError = StrictMappingError


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
        if teacher.phase_min_duration_s["stable"] + 1e-12 < success.stable_duration_s:
            raise contract_error("teacher.phase_min_duration_s.stable",
                                 "must cover success.stable_duration_s")
        if teacher.stable_hold_s + 1e-12 < success.stable_duration_s:
            raise contract_error("teacher.stable_hold_s", "must cover stable_duration_s")
        if teacher.max_plan_duration_s <= success.stable_duration_s + 1.0:
            raise contract_error("teacher.max_plan_duration_s",
                                 "must exceed stable_duration_s + 1 s")
        if capacity.max_episode_chunks < capacity.expected_chunks:
            raise contract_error("capacity.max_episode_chunks", "must cover expected_chunks")
        if capacity.expected_references > capacity.max_episode_chunks:
            raise contract_error("capacity.expected_references",
                                 "must not exceed max_episode_chunks")
        if capacity.expected_events < 3 * capacity.expected_chunks:
            raise contract_error("capacity.expected_events", "must cover three events per chunk")
        if capacity.rpc_cache_entries < capacity.max_teacher_requests:
            raise contract_error("capacity.rpc_cache_entries", "must cover max_teacher_requests")
        if capacity.backend_chunk_history < capacity.max_episode_chunks:
            raise contract_error("capacity.backend_chunk_history",
                                 "must cover max_episode_chunks")
        if capacity.max_teacher_requests < capacity.max_episode_chunks:
            raise contract_error("capacity.max_teacher_requests", "must cover max_episode_chunks")
        return self

    source_object = property(lambda self: self.objects.source)
    target_object = property(lambda self: self.objects.target)
    source_half_extents_m = property(lambda self: self.objects.source_half_extents_m)
    target_half_extents_m = property(lambda self: self.objects.target_half_extents_m)


def load_task_config(path: Path | str) -> StackTaskConfig:
    """Load and validate the task YAML before any MuJoCo or ROS object exists."""
    source_path = Path(path).resolve()
    try:
        raw_bytes = source_path.read_bytes()
        document = read_unique_yaml(source_path)
    except OSError as error:
        raise StackTaskConfigError(source_path, "<file>", str(error)) from error
    except UniqueKeyError as error:
        raise StackTaskConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise StackTaskConfigError(source_path, "<root>", "must be a mapping")
    original_document = document
    teacher = document.get("teacher")
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


def validate_scene_geometry(config: StackTaskConfig, scene_config: Any) -> None:
    """Require the task names/half extents to match the frozen scene exactly."""
    objects = {obj.name: obj for obj in scene_config.objects}
    for name, expected in (
        (config.source_object, config.source_half_extents_m),
        (config.target_object, config.target_half_extents_m),
    ):
        obj = objects.get(name)
        if obj is None:
            raise StackTaskConfigError(config.source, "objects",
                                       f"scene has no configured object {name!r}")
        actual = tuple(float(value) for value in obj.half_extents)
        if any(abs(a - b) > 1e-12 for a, b in zip(actual, expected, strict=True)):
            raise StackTaskConfigError(config.source, "objects",
                                       f"{name!r} half extents do not match task")
