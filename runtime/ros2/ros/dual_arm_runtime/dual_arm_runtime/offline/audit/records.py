"""Decoded immutable recording facts used by the audit rules."""
from dataclasses import dataclass

from robot_contracts.models import RunContext


@dataclass(frozen=True, slots=True)
class BackendInfo:
    context: RunContext
    backend_kind: str
    test_only: bool
    joint_names: tuple[str, ...]
    camera_names: tuple[str, ...]
    capabilities: tuple[str, ...]

@dataclass(frozen=True, slots=True)
class State:
    context: RunContext
    state_seq: int
    stamp_ns: int
    joints_stamp_ns: int
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]

@dataclass(frozen=True, slots=True)
class CameraFrame:
    context: RunContext
    frame_seq: int
    stamp_ns: int
    width: int
    height: int
    encoding: str
    data: bytes
    step: int
    frame_id: str
    info_width: int
    info_height: int
    info_frame_id: str
    calibration_digest: str

@dataclass(frozen=True, slots=True)
class Observation:
    context: RunContext
    observation_id: int
    state_seq: int
    camera_names: tuple[str, ...]
    frame_seqs: tuple[int, ...]
    reference_time_ns: int


@dataclass(frozen=True, slots=True)
class TaskFact:
    phase: str
    failure_reason: str
    task_type: str
    task_epoch: int
    state_seq: int
    sample_time_ns: int
    task_config_digest: str
    plan_digest: str
    metrics: dict
