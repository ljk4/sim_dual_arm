"""Explicit composition facade for the 500 Hz motion state machine.

``MotionController`` owns the ``StreamSchedule``, ``BrakingController``,
``GripperContactTracker`` and ``Snapshotter`` components and delegates the
single-threaded transition engine to :class:`MotionRuntime`.
"""

from __future__ import annotations

from collections.abc import Sequence

from robot_contracts.action_chunks import ChunkAnchor
from robot_trajectory.follower import PreparedFollowerPlan

from robot_backend_sim.config.motion_config import ControlConfig
from robot_backend_sim.config.stream_config import StreamConfig
from robot_backend_sim.motion.motion_runtime import MotionRuntime
from robot_backend_sim.motion.motion_types import (
    FailedReason,
    MotionResult,
    MotionSnapshot,
    Phase,
    StopReason,
)
from robot_backend_sim.motion.stream_schedule import ChunkEvent, ChunkSubmission, StreamSchedule
from robot_backend_sim.physics.robot_io import RobotIO
from robot_backend_sim.physics.scene_container import CompiledScene

__all__ = [
    "ChunkEvent", "ChunkSubmission", "FailedReason", "MotionController",
    "MotionResult", "MotionSnapshot", "Phase", "StopReason",
]


class MotionController:
    """One motion owner bound to the creating thread and one robot I/O."""

    def __init__(
        self,
        scene: CompiledScene,
        io: RobotIO,
        config: ControlConfig,
        initial_reference: Sequence[float] | None = None,
        *,
        stream_config: StreamConfig | None = None,
    ) -> None:
        self._runtime = MotionRuntime(
            scene, io, config, initial_reference, stream_config=stream_config
        )
        self._braking = self._runtime._braking
        self._gripper = self._runtime._gripper

    @property
    def _schedule(self) -> StreamSchedule | None:
        return self._runtime._schedule

    @property
    def phase(self) -> Phase:
        return self._runtime.phase

    @property
    def hold_positions(self) -> tuple[float, ...]:
        return self._runtime.hold_positions

    @property
    def result(self) -> MotionResult | None:
        return self._runtime.result

    @property
    def terminal_snapshot(self) -> MotionSnapshot | None:
        return self._runtime.terminal_snapshot

    def chunk_anchor(self, start_sim_time_ns: int | None = None) -> ChunkAnchor:
        return self._runtime.chunk_anchor(start_sim_time_ns)

    def start_stream(self, first: PreparedFollowerPlan, wall_time_ns: int) -> MotionSnapshot:
        return self._runtime.start_stream(first, wall_time_ns)

    def queue_chunk(
        self, successor: PreparedFollowerPlan, wall_time_ns: int
    ) -> ChunkSubmission:
        return self._runtime.queue_chunk(successor, wall_time_ns)

    def step(self, wall_time_ns: int) -> MotionSnapshot:
        return self._runtime.step(wall_time_ns)

    def poll(self, wall_time_ns: int) -> MotionSnapshot:
        return self._runtime.poll(wall_time_ns)

    def request_stop(self, reason: str, wall_time_ns: int) -> MotionSnapshot:
        return self._runtime.request_stop(reason, wall_time_ns)
