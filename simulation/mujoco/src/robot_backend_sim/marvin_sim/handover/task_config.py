"""Strict task recipe for the Marvin handover lifecycle, gates and Teacher.

Path goals and grasp geometry stay in the referenced P02B planning recipe; this
file adds the behaviour phases, measured gate thresholds, final-state
evaluation, execution budget and Teacher clock.  Contact force, hold duration
and relative-pose tolerances are reused from the planning diagnostics.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Annotated

from pydantic import Field, model_validator
from robot_contracts.profiles import semantic_hash
from robot_contracts.validation import (
    PositiveFloat,
    PositiveInt,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from robot_backend_sim.marvin_sim.handover.config import (
    HandoverPlanConfig,
    load_handover_plan_config,
)

__all__ = [
    "BEHAVIOUR_PHASES",
    "CapacityConfig",
    "GatesConfig",
    "HandoverTaskConfig",
    "HandoverTaskConfigError",
    "SuccessConfig",
    "TeacherConfig",
    "load_handover_task_config",
]

BEHAVIOUR_PHASES = (
    "left_approach",
    "left_grasp",
    "left_lift",
    "handover_approach",
    "dual_hold",
    "left_release",
    "left_retreat",
    "right_transport",
    "right_place",
    "right_release",
    "stable",
)
HARD_MAX_CHUNKS = 4096
HandoverTaskConfigError = StrictMappingError
_SHA256 = r"^[0-9a-f]{64}$"


class SuccessConfig(StrictMappingModel):
    stable_duration_s: Annotated[PositiveFloat, Field(ge=0.2, le=10.0)]
    max_task_sim_duration_s: Annotated[PositiveFloat, Field(gt=1.0, le=3600.0)]
    max_task_wall_duration_s: Annotated[PositiveFloat, Field(gt=1.0, le=3600.0)]
    box_bottom_tolerance_m: Annotated[PositiveFloat, Field(gt=1e-5, le=0.02)]
    min_box_support_normal_force_n: Annotated[PositiveFloat, Field(gt=1e-4, le=100.0)]
    max_linear_speed_m_s: Annotated[PositiveFloat, Field(gt=1e-6, le=1.0)]
    max_angular_speed_rad_s: Annotated[PositiveFloat, Field(gt=1e-6, le=10.0)]
    max_position_drift_m: Annotated[PositiveFloat, Field(gt=1e-5, le=0.05)]
    max_orientation_drift_rad: Annotated[PositiveFloat, Field(gt=1e-5, le=0.5)]

    @model_validator(mode="after")
    def _budget(self) -> SuccessConfig:
        if self.max_task_sim_duration_s <= self.stable_duration_s:
            raise contract_error(
                "max_task_sim_duration_s", "must exceed stable_duration_s"
            )
        if self.max_task_wall_duration_s <= self.max_task_sim_duration_s:
            raise contract_error(
                "max_task_wall_duration_s", "must exceed the simulation budget"
            )
        return self


class GatesConfig(StrictMappingModel):
    stable_duration_s: Annotated[PositiveFloat, Field(ge=0.1, le=5.0)]
    phase_extra_timeout_s: Annotated[PositiveFloat, Field(gt=0.5, le=30.0)]
    arm_position_tolerance_rad: Annotated[PositiveFloat, Field(gt=1e-5, le=0.2)]
    arm_velocity_tolerance_rad_s: Annotated[PositiveFloat, Field(gt=1e-5, le=2.0)]
    gripper_position_tolerance_m: Annotated[PositiveFloat, Field(gt=1e-5, le=0.05)]
    gripper_velocity_tolerance_m_s: Annotated[PositiveFloat, Field(gt=1e-5, le=0.5)]
    release_opening_m: Annotated[PositiveFloat, Field(gt=0.04, le=0.0814)]
    release_velocity_max_m_s: Annotated[PositiveFloat, Field(gt=1e-5, le=0.5)]
    hold_loss_tolerance_s: Annotated[PositiveFloat, Field(gt=0.0, le=2.0)]
    lift_height_m: Annotated[PositiveFloat, Field(gt=0.01, le=0.5)]
    forbidden_contact_force_n: Annotated[PositiveFloat, Field(gt=1e-4, le=100.0)]


class TeacherConfig(StrictMappingModel):
    enabled: bool
    plan_wall_timeout_s: Annotated[PositiveFloat, Field(gt=0.5, le=60.0)]
    chunk_size: Annotated[int, Field(strict=True, ge=60, le=60)]
    action_period_ns: Annotated[int, Field(strict=True, ge=50_000_000, le=50_000_000)]
    max_plan_duration_s: Annotated[PositiveFloat, Field(gt=1.0, le=300.0)]
    max_joint_velocity_fraction: Annotated[PositiveFloat, Field(gt=0.05, le=1.0)]
    max_joint_acceleration_fraction: Annotated[PositiveFloat, Field(gt=0.05, le=1.0)]
    gripper_velocity_m_s: Annotated[PositiveFloat, Field(gt=1e-4, le=0.5)]
    gripper_acceleration_m_s2: Annotated[PositiveFloat, Field(gt=1e-4, le=5.0)]
    phase_min_duration_s: Annotated[PositiveFloat, Field(ge=0.1, le=5.0)]
    left_close_opening_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.0814)]
    right_close_opening_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.0814)]
    chunk_cache_entries: Annotated[PositiveInt, Field(ge=64, le=HARD_MAX_CHUNKS)]


class CapacityConfig(StrictMappingModel):
    expected_chunks: Annotated[PositiveInt, Field(ge=16, le=HARD_MAX_CHUNKS)]
    expected_references: Annotated[PositiveInt, Field(ge=16, le=HARD_MAX_CHUNKS)]
    expected_events: Annotated[PositiveInt, Field(ge=64, le=4 * HARD_MAX_CHUNKS)]
    max_episode_chunks: Annotated[PositiveInt, Field(ge=16, le=HARD_MAX_CHUNKS)]
    max_teacher_requests: Annotated[PositiveInt, Field(ge=16, le=HARD_MAX_CHUNKS)]
    rpc_cache_entries: Annotated[PositiveInt, Field(ge=16, le=HARD_MAX_CHUNKS)]
    backend_chunk_history: Annotated[PositiveInt, Field(ge=16, le=HARD_MAX_CHUNKS)]


class HandoverTaskConfig(StrictMappingModel):
    source: Path = Path("")
    config_digest: Annotated[str, Field(pattern=_SHA256)] = "0" * 64
    planning: HandoverPlanConfig
    success: SuccessConfig
    gates: GatesConfig
    teacher: TeacherConfig
    capacity: CapacityConfig

    @model_validator(mode="after")
    def _task(self) -> HandoverTaskConfig:
        success, gates, teacher, capacity = (
            self.success,
            self.gates,
            self.teacher,
            self.capacity,
        )
        # The plan budget only constrains a real Teacher plan.  An inference
        # recipe disables the Teacher and uses the short evaluation budget
        # directly, so it must not inherit a plan-duration requirement.
        if teacher.enabled and teacher.max_plan_duration_s >= success.max_task_sim_duration_s:
            raise contract_error(
                "teacher.max_plan_duration_s",
                "must leave gate and stop margin inside max_task_sim_duration_s",
            )
        if gates.stable_duration_s >= success.stable_duration_s:
            raise contract_error(
                "gates.stable_duration_s",
                "must be shorter than the final stable window",
            )
        for side in ("left", "right"):
            opening = getattr(teacher, f"{side}_close_opening_m")
            if opening <= 0.0:
                raise contract_error(f"teacher.{side}_close_opening_m", "must be positive")
        checks = (
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
            (gates.release_opening_m > self.planning.grasp.closed_opening_m,
             "gates.release_opening_m", "must exceed the closed grasp opening"),
            (math.isfinite(teacher.max_plan_duration_s), "teacher", "must be finite"),
        )
        for ok, field, message in checks:
            if not ok:
                raise contract_error(field, message)
        return self

    @property
    def planning_config(self) -> HandoverPlanConfig:
        return self.planning


def load_handover_task_config(path: Path | str) -> HandoverTaskConfig:
    """Load the task recipe and its referenced planning recipe."""
    source_path = Path(path).resolve()
    try:
        document = read_unique_yaml(source_path)
    except OSError as error:
        raise HandoverTaskConfigError(source_path, "<file>", str(error)) from error
    except UniqueKeyError as error:
        raise HandoverTaskConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise HandoverTaskConfigError(source_path, "<root>", "must be a mapping")
    declared = document.get("planning_config")
    if not isinstance(declared, str) or not declared:
        raise HandoverTaskConfigError(
            source_path, "planning_config", "must name the P02B planning recipe"
        )
    planning_path = Path(declared)
    if not planning_path.is_absolute():
        planning_path = (source_path.parent / planning_path).resolve()
    if not planning_path.is_file():
        raise HandoverTaskConfigError(
            source_path, "planning_config", f"file does not exist: {planning_path}"
        )
    planning = load_handover_plan_config(planning_path)
    digest = semantic_hash(
        {"task": document, "planning": planning.model_dump(mode="json")}
    )
    return validate_config_model(
        HandoverTaskConfig,
        {
            **{key: value for key, value in document.items() if key != "planning_config"},
            "planning": planning.model_dump(mode="python"),
            "source": source_path,
            "config_digest": digest,
        },
        source_path,
        HandoverTaskConfigError,
    )
