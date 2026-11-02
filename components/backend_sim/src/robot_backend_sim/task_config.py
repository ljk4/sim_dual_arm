"""Strict configuration for the independent stack task and program teacher.

This module has no MuJoCo, ROS or mink dependency.  The YAML document is the
single hand-editable source for the success thresholds, the teacher's phase
timing and the bounded capacity of an episode.  Geometry (4 cm source and
6 cm target half extents) is cross-checked against the frozen scene, not
duplicated as physical truth in code.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

from robot_contracts.models import ContractError
from robot_contracts.profiles import load_document, semantic_hash

__all__ = [
    "PHASES",
    "CapacityConfig",
    "StackTaskConfig",
    "StackTaskConfigError",
    "SuccessThresholds",
    "TeacherPhaseConfig",
    "TeacherPlanConfig",
    "load_task_config",
    "validate_scene_geometry",
]

PHASES = (
    "pregrasp",
    "grasp",
    "close",
    "lift",
    "transport",
    "lower",
    "open",
    "retreat",
    "stable",
)
_TEACHER_KEYS = (
    "enabled",
    "kinematics_config",
    "chunk_size",
    "action_period_ns",
    "ik_wall_timeout_s",
    "plan_wall_timeout_s",
    "max_plan_duration_s",
    "max_joint_velocity_fraction",
    "max_joint_acceleration_fraction",
    "gripper_velocity_m_s",
    "gripper_acceleration_m_s2",
    "stable_hold_s",
    "max_chunk_requests",
    "chunk_cache_entries",
    "phase_min_duration_s",
    "position_tolerance_m",
    "orientation_tolerance_rad",
    "lift_height_m",
    "release_gripper_opening_m",
    "retreat_tcp_distance_m",
    "collision_sample_step_rad",
    "collision_max_samples_per_segment",
    "phase_joint_tolerance_rad",
    "phase_velocity_tolerance_rad_s",
    "gripper_position_tolerance_m",
    "gripper_velocity_tolerance_m_s",
    "contact_force_min_n",
    "finger_speed_max_m_s",
    "gate_stable_duration_s",
    "phase_gate_extra_timeout_s",
)
_CAPACITY_KEYS = (
    "expected_chunks",
    "expected_references",
    "expected_events",
    "max_episode_chunks",
    "max_teacher_requests",
    "rpc_cache_entries",
    "backend_chunk_history",
)
_SUCCESS_KEYS = (
    "support_xy_margin_m",
    "support_height_tolerance_m",
    "max_tilt_rad",
    "max_linear_speed_m_s",
    "max_angular_speed_rad_s",
    "min_support_normal_force_n",
    "min_table_normal_force_n",
    "min_vertical_normal_component",
    "min_gripper_opening_m",
    "min_tcp_clearance_m",
    "stable_duration_s",
    "max_task_sim_duration_s",
    "max_task_wall_duration_s",
)
_OBJECT_KEYS = ("source", "target", "source_half_extents_m", "target_half_extents_m")
_HARD_MAX_CHUNKS = 4096
_HARD_MAX_CACHE = 4096


class StackTaskConfigError(ValueError):
    """A task configuration problem with source path and field attached."""

    def __init__(self, source: str | Path, field: str, message: str) -> None:
        self.source = str(source)
        self.field = field
        self.message = message
        super().__init__(f"{self.source}: {field}: {message}")


@dataclass(frozen=True, slots=True)
class SuccessThresholds:
    """All SI thresholds used by the independent evaluator.

    ``support_height_tolerance_m`` is the allowed error around the exact
    geometric height ``target_z + target_half_z + source_half_z``.
    ``min_vertical_normal_component`` rejects side contacts even when their
    scalar normal force is large: it is the world-Z component of the contact
    normal (cos of the maximum allowed normal tilt).
    """

    support_xy_margin_m: float
    support_height_tolerance_m: float
    max_tilt_rad: float
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    min_support_normal_force_n: float
    min_table_normal_force_n: float
    min_vertical_normal_component: float
    min_gripper_opening_m: float
    min_tcp_clearance_m: float
    stable_duration_s: float
    max_task_sim_duration_s: float
    max_task_wall_duration_s: float


@dataclass(frozen=True, slots=True)
class TeacherPhaseConfig:
    min_duration_s: float


@dataclass(frozen=True, slots=True)
class TeacherPlanConfig:
    enabled: bool
    kinematics_config_path: Path
    chunk_size: int
    action_period_ns: int
    ik_wall_timeout_s: float
    plan_wall_timeout_s: float
    max_plan_duration_s: float
    max_joint_velocity_fraction: float
    max_joint_acceleration_fraction: float
    gripper_velocity_m_s: float
    gripper_acceleration_m_s2: float
    stable_hold_s: float
    max_chunk_requests: int
    chunk_cache_entries: int
    phase_min_duration_s: MappingProxyType[str, float]
    position_tolerance_m: float
    orientation_tolerance_rad: float
    lift_height_m: float
    release_gripper_opening_m: float
    retreat_tcp_distance_m: float
    collision_sample_step_rad: float
    collision_max_samples_per_segment: int
    phase_joint_tolerance_rad: float
    phase_velocity_tolerance_rad_s: float
    gripper_position_tolerance_m: float
    gripper_velocity_tolerance_m_s: float
    contact_force_min_n: float
    finger_speed_max_m_s: float
    gate_stable_duration_s: float
    phase_gate_extra_timeout_s: float


@dataclass(frozen=True, slots=True)
class CapacityConfig:
    expected_chunks: int
    expected_references: int
    expected_events: int
    max_episode_chunks: int
    max_teacher_requests: int
    rpc_cache_entries: int
    backend_chunk_history: int


@dataclass(frozen=True, slots=True)
class StackTaskConfig:
    source: Path
    file_sha256: str
    config_digest: str
    schema_version: int
    source_object: str
    target_object: str
    source_half_extents_m: tuple[float, float, float]
    target_half_extents_m: tuple[float, float, float]
    success: SuccessThresholds
    teacher: TeacherPlanConfig
    capacity: CapacityConfig


def _field(parent: str, child: str) -> str:
    return f"{parent}.{child}" if parent else child


def _mapping(value: object, source: str, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise StackTaskConfigError(source, field, "must be a mapping")
    for key in value:
        if not isinstance(key, str):
            raise StackTaskConfigError(source, field, "keys must be strings")
    return value


def _check_keys(
    document: dict[str, Any], source: str, field: str, required: tuple[str, ...]
) -> None:
    for key in document:
        if key not in required:
            raise StackTaskConfigError(source, _field(field, key), "unknown key")
    for key in required:
        if key not in document:
            raise StackTaskConfigError(source, _field(field, key), "missing required key")


def _number(
    value: object,
    source: str,
    field: str,
    *,
    minimum: float,
    maximum: float,
    strict_minimum: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise StackTaskConfigError(source, field, "must be a number; booleans are not accepted")
    try:
        result = float(value)
    except OverflowError as error:
        raise StackTaskConfigError(source, field, "must be finite") from error
    if not math.isfinite(result):
        raise StackTaskConfigError(source, field, "must be finite")
    if strict_minimum and result <= minimum:
        raise StackTaskConfigError(source, field, f"must be greater than {minimum}")
    if not strict_minimum and result < minimum:
        raise StackTaskConfigError(source, field, f"must be at least {minimum}")
    if result > maximum:
        raise StackTaskConfigError(source, field, f"must be at most {maximum}")
    return result


def _integer(value: object, source: str, field: str, *, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise StackTaskConfigError(source, field, "must be an integer; booleans are not accepted")
    if not minimum <= value <= maximum:
        raise StackTaskConfigError(source, field, f"must be in {minimum}..{maximum}")
    return int(value)


def _boolean(value: object, source: str, field: str) -> bool:
    if not isinstance(value, bool):
        raise StackTaskConfigError(source, field, "must be a boolean")
    return value


def _text(value: object, source: str, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise StackTaskConfigError(source, field, "must be a non-empty string")
    return value


def _vector3(value: object, source: str, field: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise StackTaskConfigError(source, field, "must be a list of three numbers")
    return tuple(
        _number(item, source, f"{field}[{index}]", minimum=1e-4, maximum=1.0)
        for index, item in enumerate(value)
    )  # type: ignore[return-value]


def load_task_config(path: Path | str) -> StackTaskConfig:
    """Load and validate the task YAML before any MuJoCo or ROS object exists."""
    source_path = Path(path).resolve()
    source = str(source_path)
    try:
        raw_bytes = source_path.read_bytes()
    except OSError as error:
        raise StackTaskConfigError(
            source, "<file>", f"cannot read configuration: {error}"
        ) from error
    try:
        document = load_document(source_path)
    except ContractError as error:
        raise StackTaskConfigError(source, "<yaml>", str(error)) from error
    root = _mapping(document, source, "<root>")
    _check_keys(
        root,
        source,
        "",
        ("schema_version", "objects", "success", "teacher", "capacity"),
    )
    version = _integer(root["schema_version"], source, "schema_version", minimum=1, maximum=1)

    objects = _mapping(root["objects"], source, "objects")
    _check_keys(objects, source, "objects", _OBJECT_KEYS)
    source_object = _text(objects["source"], source, "objects.source")
    target_object = _text(objects["target"], source, "objects.target")
    if source_object == target_object:
        raise StackTaskConfigError(source, "objects", "source and target must differ")
    source_half = _vector3(
        objects["source_half_extents_m"], source, "objects.source_half_extents_m"
    )
    target_half = _vector3(
        objects["target_half_extents_m"], source, "objects.target_half_extents_m"
    )
    if source_half[2] != source_half[0] or source_half[0] != source_half[1]:
        raise StackTaskConfigError(
            source, "objects.source_half_extents_m", "configured source must be a cube"
        )
    if target_half[2] != target_half[0] or target_half[0] != target_half[1]:
        raise StackTaskConfigError(
            source, "objects.target_half_extents_m", "configured target must be a cube"
        )

    success_doc = _mapping(root["success"], source, "success")
    _check_keys(success_doc, source, "success", _SUCCESS_KEYS)
    success = SuccessThresholds(
        support_xy_margin_m=_number(
            success_doc["support_xy_margin_m"],
            source,
            "success.support_xy_margin_m",
            minimum=0.0,
            maximum=0.02,
        ),
        support_height_tolerance_m=_number(
            success_doc["support_height_tolerance_m"],
            source,
            "success.support_height_tolerance_m",
            minimum=1e-5,
            maximum=0.02,
            strict_minimum=True,
        ),
        max_tilt_rad=_number(
            success_doc["max_tilt_rad"],
            source,
            "success.max_tilt_rad",
            minimum=1e-6,
            maximum=math.pi / 2.0,
            strict_minimum=True,
        ),
        max_linear_speed_m_s=_number(
            success_doc["max_linear_speed_m_s"],
            source,
            "success.max_linear_speed_m_s",
            minimum=1e-6,
            maximum=1.0,
            strict_minimum=True,
        ),
        max_angular_speed_rad_s=_number(
            success_doc["max_angular_speed_rad_s"],
            source,
            "success.max_angular_speed_rad_s",
            minimum=1e-6,
            maximum=10.0,
            strict_minimum=True,
        ),
        min_support_normal_force_n=_number(
            success_doc["min_support_normal_force_n"],
            source,
            "success.min_support_normal_force_n",
            minimum=1e-6,
            maximum=100.0,
            strict_minimum=True,
        ),
        min_table_normal_force_n=_number(
            success_doc["min_table_normal_force_n"],
            source,
            "success.min_table_normal_force_n",
            minimum=1e-6,
            maximum=100.0,
            strict_minimum=True,
        ),
        min_vertical_normal_component=_number(
            success_doc["min_vertical_normal_component"],
            source,
            "success.min_vertical_normal_component",
            minimum=0.0,
            maximum=1.0,
            strict_minimum=True,
        ),
        min_gripper_opening_m=_number(
            success_doc["min_gripper_opening_m"],
            source,
            "success.min_gripper_opening_m",
            minimum=1e-6,
            maximum=0.08,
            strict_minimum=True,
        ),
        min_tcp_clearance_m=_number(
            success_doc["min_tcp_clearance_m"],
            source,
            "success.min_tcp_clearance_m",
            minimum=1e-6,
            maximum=1.0,
            strict_minimum=True,
        ),
        stable_duration_s=_number(
            success_doc["stable_duration_s"],
            source,
            "success.stable_duration_s",
            minimum=2.0,
            maximum=10.0,
        ),
        max_task_sim_duration_s=_number(
            success_doc["max_task_sim_duration_s"],
            source,
            "success.max_task_sim_duration_s",
            minimum=3.0,
            maximum=30.0,
            strict_minimum=True,
        ),
        max_task_wall_duration_s=_number(
            success_doc["max_task_wall_duration_s"],
            source,
            "success.max_task_wall_duration_s",
            minimum=1.0,
            maximum=300.0,
            strict_minimum=True,
        ),
    )
    if success.max_task_sim_duration_s <= success.stable_duration_s:
        raise StackTaskConfigError(
            source,
            "success.max_task_sim_duration_s",
            "must exceed stable_duration_s",
        )

    teacher_doc = _mapping(root["teacher"], source, "teacher")
    _check_keys(teacher_doc, source, "teacher", _TEACHER_KEYS)
    enabled = _boolean(teacher_doc["enabled"], source, "teacher.enabled")
    kinematics_path = Path(
        _text(teacher_doc["kinematics_config"], source, "teacher.kinematics_config")
    )
    if not kinematics_path.is_absolute():
        kinematics_path = (source_path.parent / kinematics_path).resolve()
    if not kinematics_path.is_file():
        raise StackTaskConfigError(
            source,
            "teacher.kinematics_config",
            f"must reference an existing file: {kinematics_path}",
        )
    phase_doc = _mapping(
        teacher_doc["phase_min_duration_s"], source, "teacher.phase_min_duration_s"
    )
    _check_keys(phase_doc, source, "teacher.phase_min_duration_s", PHASES)
    phase_min: dict[str, float] = {}
    for phase in PHASES:
        phase_min[phase] = _number(
            phase_doc[phase],
            source,
            f"teacher.phase_min_duration_s.{phase}",
            minimum=0.05,
            maximum=15.0,
        )
    if phase_min["stable"] + 1e-12 < success.stable_duration_s:
        raise StackTaskConfigError(
            source,
            "teacher.phase_min_duration_s.stable",
            "must cover success.stable_duration_s",
        )
    stable_hold = _number(
        teacher_doc["stable_hold_s"], source, "teacher.stable_hold_s", minimum=2.0, maximum=10.0
    )
    if stable_hold + 1e-12 < success.stable_duration_s:
        raise StackTaskConfigError(
            source,
            "teacher.stable_hold_s",
            "must cover success.stable_duration_s",
        )
    teacher = TeacherPlanConfig(
        enabled=enabled,
        kinematics_config_path=kinematics_path,
        chunk_size=_integer(
            teacher_doc["chunk_size"], source, "teacher.chunk_size", minimum=60, maximum=60
        ),
        action_period_ns=_integer(
            teacher_doc["action_period_ns"],
            source,
            "teacher.action_period_ns",
            minimum=50_000_000,
            maximum=50_000_000,
        ),
        ik_wall_timeout_s=_number(
            teacher_doc["ik_wall_timeout_s"],
            source,
            "teacher.ik_wall_timeout_s",
            minimum=0.1,
            maximum=60.0,
            strict_minimum=True,
        ),
        plan_wall_timeout_s=_number(
            teacher_doc["plan_wall_timeout_s"],
            source,
            "teacher.plan_wall_timeout_s",
            minimum=0.1,
            maximum=60.0,
            strict_minimum=True,
        ),
        max_plan_duration_s=_number(
            teacher_doc["max_plan_duration_s"],
            source,
            "teacher.max_plan_duration_s",
            minimum=success.stable_duration_s + 1.0,
            maximum=29.9,
            strict_minimum=True,
        ),
        max_joint_velocity_fraction=_number(
            teacher_doc["max_joint_velocity_fraction"],
            source,
            "teacher.max_joint_velocity_fraction",
            minimum=0.05,
            maximum=1.0,
            strict_minimum=True,
        ),
        max_joint_acceleration_fraction=_number(
            teacher_doc["max_joint_acceleration_fraction"],
            source,
            "teacher.max_joint_acceleration_fraction",
            minimum=0.05,
            maximum=1.0,
            strict_minimum=True,
        ),
        gripper_velocity_m_s=_number(
            teacher_doc["gripper_velocity_m_s"],
            source,
            "teacher.gripper_velocity_m_s",
            minimum=1e-4,
            maximum=0.04,
            strict_minimum=True,
        ),
        gripper_acceleration_m_s2=_number(
            teacher_doc["gripper_acceleration_m_s2"],
            source,
            "teacher.gripper_acceleration_m_s2",
            minimum=1e-4,
            maximum=1.0,
            strict_minimum=True,
        ),
        stable_hold_s=stable_hold,
        max_chunk_requests=_integer(
            teacher_doc["max_chunk_requests"],
            source,
            "teacher.max_chunk_requests",
            minimum=120,
            maximum=_HARD_MAX_CACHE,
        ),
        chunk_cache_entries=_integer(
            teacher_doc["chunk_cache_entries"],
            source,
            "teacher.chunk_cache_entries",
            minimum=120,
            maximum=_HARD_MAX_CACHE,
        ),
        phase_min_duration_s=MappingProxyType(phase_min),
        position_tolerance_m=_number(
            teacher_doc["position_tolerance_m"],
            source,
            "teacher.position_tolerance_m",
            minimum=1e-5,
            maximum=0.05,
            strict_minimum=True,
        ),
        orientation_tolerance_rad=_number(
            teacher_doc["orientation_tolerance_rad"],
            source,
            "teacher.orientation_tolerance_rad",
            minimum=1e-5,
            maximum=0.5,
            strict_minimum=True,
        ),
        lift_height_m=_number(
            teacher_doc["lift_height_m"],
            source,
            "teacher.lift_height_m",
            minimum=0.01,
            maximum=0.5,
            strict_minimum=True,
        ),
        release_gripper_opening_m=_number(
            teacher_doc["release_gripper_opening_m"],
            source,
            "teacher.release_gripper_opening_m",
            minimum=0.04,
            maximum=0.08,
            strict_minimum=True,
        ),
        retreat_tcp_distance_m=_number(
            teacher_doc["retreat_tcp_distance_m"],
            source,
            "teacher.retreat_tcp_distance_m",
            minimum=0.02,
            maximum=0.5,
            strict_minimum=True,
        ),
        collision_sample_step_rad=_number(
            teacher_doc["collision_sample_step_rad"],
            source,
            "teacher.collision_sample_step_rad",
            minimum=0.002,
            maximum=0.1,
            strict_minimum=True,
        ),
        collision_max_samples_per_segment=_integer(
            teacher_doc["collision_max_samples_per_segment"],
            source,
            "teacher.collision_max_samples_per_segment",
            minimum=2,
            maximum=2000,
        ),
        phase_joint_tolerance_rad=_number(
            teacher_doc["phase_joint_tolerance_rad"], source,
            "teacher.phase_joint_tolerance_rad", minimum=1e-5, maximum=0.5,
            strict_minimum=True,
        ),
        phase_velocity_tolerance_rad_s=_number(
            teacher_doc["phase_velocity_tolerance_rad_s"], source,
            "teacher.phase_velocity_tolerance_rad_s", minimum=1e-5, maximum=2.0,
            strict_minimum=True,
        ),
        gripper_position_tolerance_m=_number(
            teacher_doc["gripper_position_tolerance_m"], source,
            "teacher.gripper_position_tolerance_m", minimum=1e-5, maximum=0.05,
            strict_minimum=True,
        ),
        gripper_velocity_tolerance_m_s=_number(
            teacher_doc["gripper_velocity_tolerance_m_s"], source,
            "teacher.gripper_velocity_tolerance_m_s", minimum=1e-5, maximum=0.5,
            strict_minimum=True,
        ),
        contact_force_min_n=_number(
            teacher_doc["contact_force_min_n"], source,
            "teacher.contact_force_min_n", minimum=1e-4, maximum=100.0,
            strict_minimum=True,
        ),
        finger_speed_max_m_s=_number(
            teacher_doc["finger_speed_max_m_s"], source,
            "teacher.finger_speed_max_m_s", minimum=1e-5, maximum=1.0,
            strict_minimum=True,
        ),
        gate_stable_duration_s=_number(
            teacher_doc["gate_stable_duration_s"], source,
            "teacher.gate_stable_duration_s", minimum=0.02, maximum=2.0,
        ),
        phase_gate_extra_timeout_s=_number(
            teacher_doc["phase_gate_extra_timeout_s"], source,
            "teacher.phase_gate_extra_timeout_s", minimum=0.1, maximum=20.0,
            strict_minimum=True,
        ),
    )
    if teacher.ik_wall_timeout_s > teacher.plan_wall_timeout_s:
        raise StackTaskConfigError(
            source,
            "teacher.ik_wall_timeout_s",
            "must not exceed teacher.plan_wall_timeout_s",
        )

    capacity_doc = _mapping(root["capacity"], source, "capacity")
    _check_keys(capacity_doc, source, "capacity", _CAPACITY_KEYS)
    capacity = CapacityConfig(
        expected_chunks=_integer(
            capacity_doc["expected_chunks"],
            source,
            "capacity.expected_chunks",
            minimum=120,
            maximum=_HARD_MAX_CHUNKS,
        ),
        expected_references=_integer(
            capacity_doc["expected_references"],
            source,
            "capacity.expected_references",
            minimum=120,
            maximum=_HARD_MAX_CHUNKS,
        ),
        expected_events=_integer(
            capacity_doc["expected_events"],
            source,
            "capacity.expected_events",
            minimum=360,
            maximum=4 * _HARD_MAX_CHUNKS,
        ),
        max_episode_chunks=_integer(
            capacity_doc["max_episode_chunks"],
            source,
            "capacity.max_episode_chunks",
            minimum=120,
            maximum=_HARD_MAX_CHUNKS,
        ),
        max_teacher_requests=_integer(
            capacity_doc["max_teacher_requests"],
            source,
            "capacity.max_teacher_requests",
            minimum=120,
            maximum=_HARD_MAX_CHUNKS,
        ),
        rpc_cache_entries=_integer(
            capacity_doc["rpc_cache_entries"],
            source,
            "capacity.rpc_cache_entries",
            minimum=120,
            maximum=_HARD_MAX_CACHE,
        ),
        backend_chunk_history=_integer(
            capacity_doc["backend_chunk_history"],
            source,
            "capacity.backend_chunk_history",
            minimum=120,
            maximum=_HARD_MAX_CHUNKS,
        ),
    )
    if capacity.max_episode_chunks < capacity.expected_chunks:
        raise StackTaskConfigError(
            source, "capacity.max_episode_chunks", "must cover expected_chunks"
        )
    if capacity.expected_references > capacity.max_episode_chunks:
        raise StackTaskConfigError(
            source, "capacity.expected_references", "must not exceed max_episode_chunks"
        )
    if capacity.expected_events < 3 * capacity.expected_chunks:
        raise StackTaskConfigError(
            source, "capacity.expected_events", "must cover three events per expected chunk"
        )
    if capacity.rpc_cache_entries < capacity.max_teacher_requests:
        raise StackTaskConfigError(
            source, "capacity.rpc_cache_entries", "must cover max_teacher_requests"
        )
    if capacity.backend_chunk_history < capacity.max_episode_chunks:
        raise StackTaskConfigError(
            source, "capacity.backend_chunk_history", "must cover max_episode_chunks"
        )
    if capacity.max_teacher_requests < capacity.max_episode_chunks:
        raise StackTaskConfigError(
            source, "capacity.max_teacher_requests", "must cover max_episode_chunks"
        )

    return StackTaskConfig(
        source=source_path,
        file_sha256=hashlib.sha256(raw_bytes).hexdigest(),
        config_digest=semantic_hash(document),
        schema_version=version,
        source_object=source_object,
        target_object=target_object,
        source_half_extents_m=source_half,
        target_half_extents_m=target_half,
        success=success,
        teacher=teacher,
        capacity=capacity,
    )


def validate_scene_geometry(config: StackTaskConfig, scene_config: object) -> None:
    """Require the task names/half extents to match the frozen scene exactly."""
    objects = {obj.name: obj for obj in scene_config.objects}  # type: ignore[attr-defined]
    for name, expected in (
        (config.source_object, config.source_half_extents_m),
        (config.target_object, config.target_half_extents_m),
    ):
        obj = objects.get(name)
        if obj is None:
            raise StackTaskConfigError(
                config.source, "objects", f"scene has no configured object {name!r}"
            )
        actual = tuple(float(value) for value in obj.half_extents)
        if any(abs(a - b) > 1e-12 for a, b in zip(actual, expected, strict=True)):
            raise StackTaskConfigError(
                config.source,
                "objects",
                f"{name!r} half extents {actual} do not match task {expected}",
            )
