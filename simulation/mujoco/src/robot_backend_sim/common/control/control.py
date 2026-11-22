"""Single-owner control session over an already owned scene and I/O.

``ControlSession`` composes an existing :class:`~robot_backend_sim.common.physics.scene_container.CompiledScene`,
the selected :class:`~robot_backend_sim.common.physics.robot_io.RobotIO` and the one
``MotionController`` physical loop.  It adds explicit leases, immutable command
identity, bounded idempotent command/chunk history, aggregate status and
episode-boundary reset.  It never creates a thread, reads a clock/file/network,
fits a curve or performs a second ``mj_step``.
"""

from __future__ import annotations

import threading
import uuid
from numbers import Integral

from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.control import (
    INT64_MAX,
    check_generation,
    check_lease_context,
    validate_context,
    validate_lease_duration,
    validate_owner,
    validate_seed,
    validate_wall,
)
from robot_contracts.models import ContractError, RunContext
from robot_trajectory.follower import PreparedFollowerPlan

from robot_backend_sim.common.config.motion_config import ControlConfig
from robot_backend_sim.common.config.stream_config import StreamConfig
from robot_backend_sim.common.control.control_events import CommandJournal
from robot_backend_sim.common.control.control_state import (
    COMMAND_ACCEPTED as _COMMAND_ACCEPTED,
    COMMAND_ACTIVE_PHASES,
    COMMAND_STARTED as _COMMAND_STARTED,
    COMMAND_STOPPING as _COMMAND_STOPPING,
    MAX_DEFERRED_EVENTS,
    ChunkHistory,
    ChunkRecord,
    ChunkStatus,
    CommandHistory,
    CommandRecord,
    ControlEvent,
    ControlReply,
    ControlStatus,
    DeferredEvents,
    LeaseBook,
    PreparedControlCommand,
    _status_from_chunk,
    _status_from_record,
)
from robot_backend_sim.common.motion.motion_controller import MotionController, MotionSnapshot, Phase
from robot_backend_sim.common.physics.robot_io import RobotIO, gripper_indices
from robot_backend_sim.common.physics.scene_container import CompiledScene

__all__ = ["MAX_DEFERRED_EVENTS", "ControlSession", "TIME_DOMAIN_ID"]

TIME_DOMAIN_ID = "mujoco-sim"

_MOTION_ACTIVE = frozenset({Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING, Phase.STOPPING})
_MOTION_RESETTABLE = frozenset({Phase.IDLE, Phase.COMPLETED, Phase.STOPPED, Phase.FAILED})


def _invalid(reason: str, detail: str) -> ContractError:
    return ContractError(reason, detail)


def _reason_text(value: object) -> str:
    if not isinstance(value, str) or not value:
        raise _invalid("INVALID_REASON", "reason must be a non-empty string")
    if len(value) > 128:
        raise _invalid("INVALID_REASON", "reason must be at most 128 characters")
    return value


class ControlSession:
    """One controller bound to the creating thread and one physical scene owner."""

    def __init__(
        self,
        scene: CompiledScene,
        io: RobotIO,
        control_config: ControlConfig,
        *,
        stream_config: StreamConfig | None = None,
        max_history: int = 256,
        max_chunk_history: int = 128,
    ) -> None:
        self._owner_thread = threading.current_thread()
        if not isinstance(scene, CompiledScene):
            raise _invalid("INVALID_CONTROLLER", "scene must be a CompiledScene")
        if not isinstance(io, RobotIO):
            raise _invalid("INVALID_CONTROLLER", "io must implement the RobotIO contract")
        if not isinstance(control_config, ControlConfig):
            raise _invalid("INVALID_CONTROLLER", "control_config must be a ControlConfig")
        if stream_config is not None and not isinstance(stream_config, StreamConfig):
            raise _invalid("INVALID_STREAM_CONFIG", "stream_config must be a StreamConfig")
        io.require_scene(scene)
        self._scene = scene
        self._io = io
        self._control_config = control_config
        self._gripper_indices = tuple(gripper_indices(io.joint_names).values())
        self._stream_config = stream_config
        self._commands = CommandHistory(max_history)
        self._chunks = ChunkHistory(max_chunk_history)
        self._journal = CommandJournal(self._chunks)
        self._leases = LeaseBook()
        self._motion = MotionController(
            scene,
            io,
            control_config,
            initial_reference=io.profile.initial_positions,
            stream_config=stream_config,
        )
        self._context = RunContext(
            io.profile.robot_id,
            io.profile.digest,
            f"control-{uuid.uuid4().hex}",
            TIME_DOMAIN_ID,
            0,
        )
        self._current_command_id: str | None = None
        self._highest_sequence = 0
        self._last_wall_ns: int | None = None
        self._deferred = DeferredEvents()

    # ------------------------------------------------------------- public API

    @property
    def context(self) -> RunContext:
        self._check_thread()
        return self._context

    @property
    def phase(self) -> Phase:
        self._check_thread()
        return self._motion.phase

    def hold_reference(self) -> tuple[float, ...]:
        self._check_thread()
        return self._motion.hold_positions

    def lease_held(self, context: RunContext, lease_id: str, wall_ns: int) -> bool:
        self._check_thread()
        try:
            self._require_lease(context, lease_id)
        except ContractError:
            return False
        lease = self._leases.current
        return lease is not None and int(wall_ns) < lease.expires_at_ns

    def is_quiescent(
        self,
        *,
        max_arm_speed_rad_s: float = 0.05,
        max_gripper_speed_m_s: float = 0.01,
    ) -> bool:
        self._check_thread()
        if self._motion.phase in _MOTION_ACTIVE:
            return False
        measurement = self._io.measure()
        for index, velocity in enumerate(measurement.velocities):
            limit = (
                max_gripper_speed_m_s
                if index in self._gripper_indices
                else max_arm_speed_rad_s
            )
            if abs(float(velocity)) > limit:
                return False
        return True

    def current_status(self) -> ControlStatus | None:
        self._check_thread()
        record = self._current_record()
        return None if record is None else _status_from_record(record)

    def prepared_command(
        self, context: RunContext, command_id: str, chunk_seq: int | None = None
    ) -> PreparedControlCommand | None:
        self._check_thread()
        validate_context(context)
        check_generation(context, self._context)
        if not isinstance(command_id, str) or not command_id or len(command_id) > 128:
            return None
        record = self._commands.get(command_id)
        if record is None:
            return None
        if record.command.context != context:
            raise _invalid("STALE_CONTEXT", "command identity")
        if chunk_seq is not None:
            if isinstance(chunk_seq, bool) or not isinstance(chunk_seq, int) or chunk_seq < 1:
                raise _invalid("INVALID_CHUNK_SEQUENCE", "chunk_seq must be positive")
            chunk = self._chunks.get(command_id, chunk_seq)
            return None if chunk is None else chunk.packet
        return record.packet

    def acquire(
        self,
        context: RunContext,
        owner: str,
        duration_ns: int,
        wall_ns: int,
    ) -> str:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, events = self._settle_expiry(wall)
        self._deferred.defer(events)
        validate_context(context)
        check_generation(context, self._context)
        if not context.episode_id:
            raise _invalid("INVALID_CONTEXT", "episode_id must be non-empty")
        if context.control_epoch < 1:
            raise _invalid("INVALID_CONTEXT", "control_epoch must be positive")
        validate_owner(owner)
        validate_lease_duration(duration_ns)
        if self._leases.current is not None:
            raise _invalid("CONTROL_BUSY", "a control lease already exists")
        record = self._current_record()
        if record is not None and record.phase in COMMAND_ACTIVE_PHASES:
            raise _invalid("CONTROL_BUSY", "an active or stopping command still owns control")
        if self._motion.phase in _MOTION_ACTIVE:
            raise _invalid("CONTROL_BUSY", "motion is active or stopping")
        lease = self._leases.acquire(context, owner, duration_ns, wall)
        self._context = context
        # Sequence order belongs to this lease. Old immutable history remains
        # queryable by its original context, as in the existing Robot API.
        self._highest_sequence = 0
        return lease.lease_id

    def renew(
        self,
        context: RunContext,
        lease_id: str,
        duration_ns: int,
        wall_ns: int,
    ) -> None:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, events = self._settle_expiry(wall)
        self._deferred.defer(events)
        self._require_lease(context, lease_id)
        validate_lease_duration(duration_ns)
        self._leases.renew(lease_id, duration_ns, wall)

    def release(self, context: RunContext, lease_id: str, wall_ns: int) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        expiry_snapshot, expiry_events = self._settle_expiry(wall)
        self._deferred.defer(expiry_events)
        events: list[ControlEvent] = []
        self._require_lease(context, lease_id)
        self._leases.release(lease_id)
        record = self._current_record()
        if (
            record is not None
            and record.command.context == context
            and record.phase
            in (
                _COMMAND_ACCEPTED,
                _COMMAND_STARTED,
            )
        ):
            snapshot, stop_events = self._request_stop("CONTROL_RELEASED", wall)
            events.extend(stop_events)
        else:
            snapshot = expiry_snapshot or self._motion.poll(wall)
            self._sync_stream(snapshot, wall, events)
        return self._reply(snapshot, events, accepted=True, reason="CONTROL_RELEASED")

    def submit(self, packet: PreparedControlCommand, wall_ns: int) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, expiry_events = self._settle_expiry(wall)
        self._deferred.defer(expiry_events)
        events: list[ControlEvent] = []
        if not isinstance(packet, PreparedControlCommand):
            raise _invalid("INVALID_PREPARED_COMMAND", "packet must be a PreparedControlCommand")
        command = packet.command
        check_generation(command.context, self._context)
        existing = self._commands.get(command.command_id)
        if existing is not None:
            self._journal.check_conflict(existing, packet)
            self._require_lease(command.context, command.lease_id)
            assert existing.latest_snapshot is not None
            return self._reply(
                existing.latest_snapshot, events, accepted=True, reason=existing.phase
            )
        self._require_lease(command.context, command.lease_id)
        if self._current_sim_ns() > command.accept_before_ns:
            raise _invalid("COMMAND_EXPIRED", "robot time is after accept_before_ns")
        if command.command_seq <= self._highest_sequence:
            raise _invalid("COMMAND_OUT_OF_ORDER", "command_seq must strictly increase")
        record = self._current_record()
        if (record is not None and record.phase in COMMAND_ACTIVE_PHASES) or (
            self._motion.phase in _MOTION_ACTIVE
        ):
            raise _invalid("CONTROL_BUSY", "a previous command or motion is still active")
        prepared = packet.prepared
        if prepared.seq != 1 or prepared.anchor.predecessor_seq != 0:
            raise _invalid("INVALID_CHUNK_ORDER", "first follower must start the stream")
        if not self._commands.has_capacity() or not self._chunks.has_capacity():
            raise _invalid("HISTORY_FULL", "command/chunk history has no evictable record")
        snapshot = self._motion.start_stream(prepared, wall)
        accepted_sim = snapshot.measurement_sim_time_ns
        new_record = CommandRecord(
            packet=packet, phase=_COMMAND_ACCEPTED, reason="", stop_reason=None,
            accepted_sim_ns=accepted_sim, accepted_wall_ns=wall,
            motion_seq=snapshot.motion_seq, stream_id=snapshot.stream_id,
            latest_snapshot=snapshot,
        )
        self._commands.add(new_record)
        self._current_command_id = command.command_id
        self._highest_sequence = command.command_seq
        self._journal.record_chunk(new_record, prepared, snapshot, wall)
        events.append(self._journal.emit(new_record, _COMMAND_ACCEPTED, "", accepted_sim))
        self._sync_stream(snapshot, wall, events)
        return self._reply(snapshot, events, accepted=True, reason=_COMMAND_ACCEPTED)

    def queue_chunk(self, packet: PreparedControlCommand, wall_ns: int) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, expiry_events = self._settle_expiry(wall)
        self._deferred.defer(expiry_events)
        events: list[ControlEvent] = []
        if not isinstance(packet, PreparedControlCommand):
            raise _invalid("INVALID_PREPARED_COMMAND", "packet must be a PreparedControlCommand")
        command = packet.command
        check_generation(command.context, self._context)
        record = self._current_record()
        if record is None or record.command.command_id != command.command_id:
            raise _invalid("STALE_STREAM", "no current command with this id")
        if (
            command.context != record.command.context
            or command.lease_id != record.command.lease_id
            or command.command_seq != record.command.command_seq
        ):
            raise _invalid(
                "COMMAND_ID_CONFLICT",
                "successor context/lease/command_seq differs from the stream",
            )
        self._require_lease(command.context, command.lease_id)
        prepared = packet.prepared
        existing = self._chunks.get(command.command_id, prepared.seq)
        if existing is not None:
            if existing.packet.fingerprint != packet.fingerprint:
                raise _invalid(
                    "CHUNK_ID_CONFLICT",
                    "same command/chunk sequence has a different complete payload",
                )
            assert existing.latest_snapshot is not None
            return self._reply(
                existing.latest_snapshot, events,
                accepted=existing.phase in ("ACCEPTED", "STARTED", "RETIRED"),
                reason=existing.reason or existing.phase,
            )
        if prepared.seq <= record.max_chunk_seq:
            raise _invalid("CHUNK_RESULT_EXPIRED", "an older chunk was retired or evicted")
        if prepared.seq != record.max_chunk_seq + 1:
            raise _invalid("INVALID_CHUNK_ORDER", "chunk sequence is not the next successor")
        if record.terminal_status is not None:
            raise _invalid("STALE_STREAM", "the overall command is already terminal")
        if record.phase not in (_COMMAND_ACCEPTED, _COMMAND_STARTED):
            assert record.latest_snapshot is not None
            return self._reply(
                record.latest_snapshot, events, accepted=False,
                reason=record.reason or record.phase,
            )
        if not self._chunks.has_capacity():
            raise _invalid("CHUNK_HISTORY_FULL", "chunk history has no evictable record")
        if self._current_sim_ns() > command.accept_before_ns and not self._candidate_would_latch(
            prepared
        ):
            raise _invalid("COMMAND_EXPIRED", "robot time is after accept_before_ns")
        submission = self._motion.queue_chunk(prepared, wall)
        if submission.accepted:
            self._chunks.add(ChunkRecord(
                packet=packet, phase="ACCEPTED", reason="PENDING",
                accepted_sim_ns=submission.snapshot.measurement_sim_time_ns,
                accepted_wall_ns=wall, latest_snapshot=submission.snapshot,
            ))
            record.max_chunk_seq = prepared.seq
        self._sync_stream(submission.snapshot, wall, events)
        return self._reply(
            submission.snapshot, events,
            accepted=submission.accepted, reason=submission.reason,
        )

    def stop(
        self,
        context: RunContext,
        lease_id: str,
        reason: str,
        wall_ns: int,
    ) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        expiry_snapshot, expiry_events = self._settle_expiry(wall)
        self._deferred.defer(expiry_events)
        events: list[ControlEvent] = []
        self._require_lease(context, lease_id)
        reason = _reason_text(reason)
        record = self._current_record()
        if record is None or record.command.context != context:
            if self._motion.phase is Phase.IDLE:
                snapshot = expiry_snapshot or self._motion.poll(wall)
                self._sync_stream(snapshot, wall, events)
                return self._reply(
                    snapshot,
                    events,
                    accepted=False,
                    reason="NO_ACTIVE_COMMAND",
                )
            snapshot, stop_events = self._request_stop(reason, wall)
            events.extend(stop_events)
            return self._reply(snapshot, events, accepted=True, reason=reason)
        snapshot, stop_events = self._request_stop(reason, wall)
        events.extend(stop_events)
        return self._reply(snapshot, events, accepted=True, reason=reason)

    def step(self, wall_ns: int) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, expiry_events = self._settle_expiry(wall)
        self._deferred.defer(expiry_events)
        events: list[ControlEvent] = []
        snapshot = self._motion.step(wall)
        self._sync_stream(snapshot, wall, events)
        return self._reply(snapshot, events)

    def poll(self, wall_ns: int) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, expiry_events = self._settle_expiry(wall)
        self._deferred.defer(expiry_events)
        events: list[ControlEvent] = []
        snapshot = self._motion.poll(wall)
        self._sync_stream(snapshot, wall, events)
        return self._reply(snapshot, events)

    def query_command(self, context: RunContext, command_id: str) -> ControlStatus:
        self._check_thread()
        validate_context(context)
        check_generation(context, self._context)
        if not isinstance(command_id, str) or not command_id or len(command_id) > 128:
            raise _invalid("RESULT_EXPIRED", "unknown command id")
        record = self._commands.get(command_id)
        if record is None:
            raise _invalid("RESULT_EXPIRED", command_id)
        if record.command.context != context:
            raise _invalid("STALE_CONTEXT", "command identity")
        return _status_from_record(record)

    def query_chunk(
        self,
        context: RunContext,
        command_id: str,
        chunk_seq: int,
    ) -> ChunkStatus:
        self._check_thread()
        validate_context(context)
        check_generation(context, self._context)
        if not isinstance(command_id, str) or not command_id or len(command_id) > 128:
            raise _invalid("RESULT_EXPIRED", "unknown command id")
        if isinstance(chunk_seq, bool) or not isinstance(chunk_seq, Integral) or int(chunk_seq) < 1:
            raise _invalid("CHUNK_RESULT_EXPIRED", "chunk_seq must be positive")
        record = self._commands.get(command_id)
        if record is None:
            raise _invalid("RESULT_EXPIRED", command_id)
        if record.command.context != context:
            raise _invalid("STALE_CONTEXT", "command identity")
        chunk = self._chunks.get(command_id, int(chunk_seq))
        if chunk is None:
            raise _invalid("CHUNK_RESULT_EXPIRED", f"chunk {chunk_seq}")
        return _status_from_chunk(chunk)

    def chunk_anchor(
        self,
        context: RunContext,
        lease_id: str,
        wall_ns: int,
        start_sim_time_ns: int | None = None,
    ) -> ChunkAnchor:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, events = self._settle_expiry(wall)
        self._deferred.defer(events)
        self._require_lease(context, lease_id)
        return self._motion.chunk_anchor(start_sim_time_ns)

    def system_stop(self, reason: str, wall_ns: int) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        reason = _reason_text(reason)
        events: list[ControlEvent] = []
        self._leases.clear()
        snapshot, stop_events = self._request_stop(reason, wall)
        events.extend(stop_events)
        return self._reply(snapshot, events, accepted=True, reason=reason)

    def reset(
        self,
        context: RunContext,
        lease_id: str,
        seed: int,
        wall_ns: int,
    ) -> RunContext:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, events = self._settle_expiry(wall)
        self._deferred.defer(events)
        self._require_lease(context, lease_id)
        validated_seed = validate_seed(seed)
        if self._motion.phase not in _MOTION_RESETTABLE:
            raise _invalid(
                "RESET_NOT_ALLOWED",
                "reset requires an idle or safely terminal motion",
            )
        if self._context.clock_epoch >= INT64_MAX:
            raise _invalid("INVALID_CONTEXT", "clock_epoch cannot increase")
        self._scene.reset(validated_seed)
        self._motion = MotionController(
            self._scene,
            self._io,
            self._control_config,
            initial_reference=self._io.profile.initial_positions,
            stream_config=self._stream_config,
        )
        self._leases.clear()
        self._commands.clear()
        self._chunks.clear()
        self._current_command_id = None
        self._highest_sequence = 0
        self._context = self._context.model_copy(update={
            "clock_epoch": self._context.clock_epoch + 1,
            "episode_id": "",
            "control_epoch": 0,
        })
        return self._context

    # ------------------------------------------------------------- internals

    def _check_thread(self) -> None:
        if threading.current_thread() is not self._owner_thread:
            raise _invalid("WRONG_THREAD", "ControlSession must be used on its creating thread")

    def _begin_call(self, wall_ns: object) -> int:
        wall = validate_wall(wall_ns)
        if self._last_wall_ns is not None and wall < self._last_wall_ns:
            raise _invalid("WALL_TIME_REGRESSION", "wall_time_ns must not move backwards")
        self._last_wall_ns = wall
        return wall

    def _current_sim_ns(self) -> int:
        return round(float(self._scene.data.time) * 1_000_000_000)

    def _candidate_would_latch(self, prepared: PreparedFollowerPlan) -> bool:
        config = self._stream_config
        if config is None:
            return False
        now = self._current_sim_ns()
        if now > prepared.start_backend_time_ns - config.min_lead_ns:
            return True
        observed = prepared.window.observation_time_ns
        if observed > now:
            return True
        return now - observed > config.max_observation_age_ns

    def _require_lease(self, context: RunContext, lease_id: str) -> None:
        check_generation(context, self._context)
        lease = self._leases.current
        if lease is None:
            raise _invalid("LEASE_EXPIRED", "no control lease")
        check_lease_context(context, lease.context)
        if lease_id != lease.lease_id:
            raise _invalid("LEASE_EXPIRED", "control lease does not match")

    def _sync_stream(
        self, snapshot: MotionSnapshot, wall: int, events: list[ControlEvent]
    ) -> None:
        record = self._current_record()
        if record is not None:
            self._journal.sync_stream(
                record, snapshot, wall, self._motion.result, events
            )

    def _current_record(self) -> CommandRecord | None:
        if self._current_command_id is None:
            return None
        return self._commands.get(self._current_command_id)

    def _settle_expiry(self, wall: int) -> tuple[MotionSnapshot | None, list[ControlEvent]]:
        expired = self._leases.expire(wall)
        if expired is None:
            return None, []
        record = self._current_record()
        if record is not None and record.phase in (_COMMAND_ACCEPTED, _COMMAND_STARTED):
            return self._request_stop("LEASE_EXPIRED", wall)
        if record is None and self._motion.phase in _MOTION_ACTIVE:
            return self._request_stop("LEASE_EXPIRED", wall)
        return None, []

    def _request_stop(
        self, reason: str, wall: int
    ) -> tuple[MotionSnapshot, list[ControlEvent]]:
        events: list[ControlEvent] = []
        record = self._current_record()
        if self._motion.phase not in _MOTION_ACTIVE:
            snapshot = self._motion.poll(wall)
            self._sync_stream(snapshot, wall, events)
            return snapshot, events
        snapshot = self._motion.request_stop(reason, wall)
        if (
            record is not None
            and record.terminal_status is None
            and record.phase in (_COMMAND_ACCEPTED, _COMMAND_STARTED)
            and snapshot.phase is Phase.STOPPING
        ):
            record.phase = _COMMAND_STOPPING
            record.reason = reason
            record.stop_reason = reason
            record.latest_snapshot = snapshot
            events.append(self._journal.emit(
                record, _COMMAND_STOPPING, reason, snapshot.measurement_sim_time_ns
            ))
        self._sync_stream(snapshot, wall, events)
        return snapshot, events

    def _reply(
        self,
        snapshot: MotionSnapshot,
        events: list[ControlEvent],
        *,
        accepted: bool | None = None,
        reason: str = "",
    ) -> ControlReply:
        record = self._current_record()
        status = None if record is None else _status_from_record(record)
        delivered = self._deferred.drain() + tuple(events)
        return ControlReply(snapshot, status, delivered, accepted, reason)
