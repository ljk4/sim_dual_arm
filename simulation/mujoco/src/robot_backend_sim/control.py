"""Single-owner control session over an already owned scene and I/O.

``ControlSession`` composes an existing :class:`~robot_backend_sim.scene.Scene`,
the accepted :class:`~robot_backend_sim.panda_io.PandaIO` and the one
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
from robot_contracts.trajectory import PreparedFollowerPlan

from .control_state import (
    COMMAND_ACTIVE_PHASES,
    MAX_DEFERRED_EVENTS,
    MAX_EVENT_SEQUENCE,
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
from .motion import (
    MotionController,
    MotionSnapshot,
    Phase,
)
from .motion_config import PandaControlConfig
from .panda_io import PandaIO
from .scene import Scene
from .stream_config import StreamConfig

__all__ = ["MAX_DEFERRED_EVENTS", "ControlSession", "TIME_DOMAIN_ID"]

TIME_DOMAIN_ID = "mujoco-sim"

_MOTION_ACTIVE = frozenset({Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING, Phase.STOPPING})
_MOTION_RESETTABLE = frozenset({Phase.IDLE, Phase.COMPLETED, Phase.STOPPED, Phase.FAILED})
_COMMAND_STARTED = "STARTED"
_COMMAND_ACCEPTED = "ACCEPTED"
_COMMAND_STOPPING = "STOPPING"
_COMMAND_COMPLETED = "COMPLETED"
_COMMAND_CANCELED = "CANCELED"
_COMMAND_FAILED = "FAILED"


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
        scene: Scene,
        io: PandaIO,
        control_config: PandaControlConfig,
        *,
        stream_config: StreamConfig | None = None,
        max_history: int = 256,
        max_chunk_history: int = 128,
    ) -> None:
        self._owner_thread = threading.current_thread()
        if not isinstance(scene, Scene):
            raise _invalid("INVALID_CONTROLLER", "scene must be a Scene")
        if not isinstance(io, PandaIO):
            raise _invalid("INVALID_CONTROLLER", "io must be a PandaIO")
        if not isinstance(control_config, PandaControlConfig):
            raise _invalid("INVALID_CONTROLLER", "control_config must be a PandaControlConfig")
        if stream_config is not None and not isinstance(stream_config, StreamConfig):
            raise _invalid("INVALID_STREAM_CONFIG", "stream_config must be a StreamConfig")
        io.require_scene(scene)
        self._scene = scene
        self._io = io
        self._control_config = control_config
        self._stream_config = stream_config
        self._commands = CommandHistory(max_history)
        self._chunks = ChunkHistory(max_chunk_history)
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
        self._event_sequence = 0
        self._last_wall_ns: int | None = None
        self._deferred_events = DeferredEvents()

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
        """Read-only last hold positions for a planner's MOTION compilation.

        A caller must not substitute the current measured positions: doing so
        would silently re-fit a new request from a state the motion was never
        holding.  The physics owner rechecks these exact values at admission.
        """
        self._check_thread()
        return self._motion.hold_positions

    def lease_held(self, context: RunContext, lease_id: str, wall_ns: int) -> bool:
        """Read-only lease check for task prepare/start; never renews or moves."""
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
        """Require no active motion and measured joints close to rest.

        Prepare is read-only, but a snapshot taken while a command is running
        would silently plan from a moving scene.  Both the controller phase and
        the measured velocities must be quiescent.
        """
        self._check_thread()
        if self._motion.phase in _MOTION_ACTIVE:
            return False
        measurement = self._io.measure()
        for index, velocity in enumerate(measurement.velocities):
            limit = max_gripper_speed_m_s if index in (7, 15) else max_arm_speed_rad_s
            if abs(float(velocity)) > limit:
                return False
        return True

    def current_status(self) -> ControlStatus | None:
        """Return a fresh immutable aggregate status, or ``None`` without history.

        A terminal record returns its already frozen ``terminal_status``; later
        live measurements never rewrite it.
        """
        self._check_thread()
        record = self._current_record()
        return None if record is None else _status_from_record(record)

    def prepared_command(
        self, context: RunContext, command_id: str, chunk_seq: int | None = None
    ) -> PreparedControlCommand | None:
        """Return the immutable original packet from the bounded history.

        The mutable ``CommandRecord`` never leaves the owner thread.  ``None``
        means the command was never accepted or has been evicted; callers must
        not invent a replacement packet in that case.
        """
        self._check_thread()
        self._query_generation(context)
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
        self._defer_events(events)
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
        self._defer_events(events)
        self._require_lease(context, lease_id)
        validate_lease_duration(duration_ns)
        self._leases.renew(lease_id, duration_ns, wall)

    def release(self, context: RunContext, lease_id: str, wall_ns: int) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        expiry_snapshot, expiry_events = self._settle_expiry(wall)
        self._defer_events(expiry_events)
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
        self._defer_events(expiry_events)
        events: list[ControlEvent] = []
        if not isinstance(packet, PreparedControlCommand):
            raise _invalid("INVALID_PREPARED_COMMAND", "packet must be a PreparedControlCommand")
        command = packet.command
        check_generation(command.context, self._context)
        existing = self._commands.get(command.command_id)
        if existing is not None:
            self._check_command_conflict(existing, packet)
            self._require_lease(command.context, command.lease_id)
            assert existing.latest_snapshot is not None
            return self._reply(
                existing.latest_snapshot,
                events,
                accepted=True,
                reason=existing.phase,
            )
        self._require_lease(command.context, command.lease_id)
        if self._current_sim_ns() > command.accept_before_ns:
            raise _invalid("COMMAND_EXPIRED", "robot time is after accept_before_ns")
        if command.command_seq <= self._highest_sequence:
            raise _invalid("COMMAND_OUT_OF_ORDER", "command_seq must strictly increase")
        record = self._current_record()
        if record is not None and record.phase in COMMAND_ACTIVE_PHASES:
            raise _invalid("CONTROL_BUSY", "a previous command is still active")
        if self._motion.phase in _MOTION_ACTIVE:
            raise _invalid("CONTROL_BUSY", "motion is active or stopping")
        if packet.kind != "FOLLOWER":
            raise _invalid(
                "INVALID_PREPARED_COMMAND",
                "submit requires a PreparedFollowerPlan",
            )
        prepared = packet.prepared
        if prepared.seq != 1 or prepared.anchor.predecessor_seq != 0:
            raise _invalid("INVALID_CHUNK_ORDER", "first follower must start the stream")
        if not self._commands.has_capacity():
            raise _invalid("HISTORY_FULL", "command history has no evictable terminal record")
        if not self._chunks.has_capacity():
            raise _invalid(
                "CHUNK_HISTORY_FULL",
                "chunk history has no evictable terminal record",
            )
        snapshot = self._motion.start_stream(prepared, wall)
        accepted_sim = snapshot.measurement_sim_time_ns
        new_record = CommandRecord(
            packet=packet,
            phase=_COMMAND_ACCEPTED,
            reason="",
            stop_reason=None,
            accepted_sim_ns=accepted_sim,
            accepted_wall_ns=wall,
            motion_seq=snapshot.motion_seq,
            stream_id=snapshot.stream_id,
            latest_snapshot=snapshot,
        )
        self._commands.add(new_record)
        self._current_command_id = command.command_id
        self._highest_sequence = command.command_seq
        self._record_chunk(new_record, prepared, snapshot, wall)
        events.append(self._emit(new_record, _COMMAND_ACCEPTED, "", accepted_sim))
        self._sync_stream(snapshot, wall, events)
        return self._reply(snapshot, events, accepted=True, reason=_COMMAND_ACCEPTED)

    def queue_chunk(self, packet: PreparedControlCommand, wall_ns: int) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, expiry_events = self._settle_expiry(wall)
        self._defer_events(expiry_events)
        events: list[ControlEvent] = []
        if not isinstance(packet, PreparedControlCommand):
            raise _invalid("INVALID_PREPARED_COMMAND", "packet must be a PreparedControlCommand")
        if packet.kind != "FOLLOWER":
            raise _invalid(
                "INVALID_PREPARED_COMMAND",
                "queue_chunk requires a PreparedFollowerPlan",
            )
        command = packet.command
        check_generation(command.context, self._context)
        record = self._current_record()
        if record is None or record.command.command_id != command.command_id:
            raise _invalid("STALE_STREAM", "no current command with this id")
        self._require_lease(command.context, command.lease_id)
        if packet.kind != record.kind:
            raise _invalid(
                "STREAM_MODE_MISMATCH",
                "a stream cannot change its prepared block kind",
            )
        if (
            command.context != record.command.context
            or command.lease_id != record.command.lease_id
            or command.command_seq != record.command.command_seq
        ):
            raise _invalid(
                "COMMAND_ID_CONFLICT",
                "successor context/lease/command_seq differs from the stream",
            )
        prepared = packet.prepared
        existing = self._chunks.get(command.command_id, prepared.seq)
        if existing is not None:
            self._require_lease(command.context, command.lease_id)
            if existing.packet.fingerprint != packet.fingerprint:
                raise _invalid(
                    "CHUNK_ID_CONFLICT",
                    "same command/chunk sequence has a different complete payload",
                )
            assert existing.latest_snapshot is not None
            accepted = existing.phase in ("ACCEPTED", "STARTED", "RETIRED")
            return self._reply(
                existing.latest_snapshot,
                events,
                accepted=accepted,
                reason=existing.reason or existing.phase,
            )
        self._require_lease(command.context, command.lease_id)
        if prepared.seq <= record.max_chunk_seq:
            raise _invalid(
                "CHUNK_RESULT_EXPIRED",
                "an older chunk was already retired or evicted",
            )
        if prepared.seq != record.max_chunk_seq + 1:
            raise _invalid("INVALID_CHUNK_ORDER", "chunk sequence is not the next successor")
        if record.terminal_status is not None:
            raise _invalid("STALE_STREAM", "the overall command is already terminal")
        if record.phase not in (_COMMAND_ACCEPTED, _COMMAND_STARTED):
            assert record.latest_snapshot is not None
            return self._reply(
                record.latest_snapshot,
                events,
                accepted=False,
                reason=record.reason or record.phase,
            )
        if not self._chunks.has_capacity():
            raise _invalid(
                "CHUNK_HISTORY_FULL",
                "chunk history has no evictable terminal record",
            )
        if self._current_sim_ns() > command.accept_before_ns and not self._candidate_would_latch(
            prepared
        ):
            raise _invalid("COMMAND_EXPIRED", "robot time is after accept_before_ns")
        submission = self._motion.queue_chunk(prepared, wall)
        if submission.accepted:
            chunk_record = ChunkRecord(
                packet=packet,
                phase="ACCEPTED",
                reason="PENDING",
                accepted_sim_ns=submission.snapshot.measurement_sim_time_ns,
                accepted_wall_ns=wall,
                latest_snapshot=submission.snapshot,
            )
            self._chunks.add(chunk_record)
            record.max_chunk_seq = prepared.seq
        self._sync_stream(submission.snapshot, wall, events)
        return self._reply(
            submission.snapshot,
            events,
            accepted=submission.accepted,
            reason=submission.reason,
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
        self._defer_events(expiry_events)
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
        self._defer_events(expiry_events)
        events: list[ControlEvent] = []
        snapshot = self._motion.step(wall)
        self._sync_stream(snapshot, wall, events)
        return self._reply(snapshot, events)

    def poll(self, wall_ns: int) -> ControlReply:
        self._check_thread()
        wall = self._begin_call(wall_ns)
        _snapshot, expiry_events = self._settle_expiry(wall)
        self._defer_events(expiry_events)
        events: list[ControlEvent] = []
        snapshot = self._motion.poll(wall)
        self._sync_stream(snapshot, wall, events)
        return self._reply(snapshot, events)

    def query_command(self, context: RunContext, command_id: str) -> ControlStatus:
        self._check_thread()
        self._query_generation(context)
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
        self._query_generation(context)
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
        self._defer_events(events)
        self._require_lease(context, lease_id)
        return self._motion.chunk_anchor(start_sim_time_ns)

    def system_stop(self, reason: str, wall_ns: int) -> ControlReply:
        """Brake on behalf of the process, bypassing an expired lease.

        This is deliberately not a ROS entry point.  It is used for shutdown,
        logger/queue/renderer faults and other software failures.  The real
        ``MotionController`` brake and terminal rules still decide the physical
        outcome; a CANCELED terminal result does not make the software failure
        disappear (the worker keeps its own nonzero fault state).
        """
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
        self._defer_events(events)
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
        if now > prepared.start_sim_time_ns - config.min_lead_ns:
            return True
        observed = prepared.window.observation_time_ns
        if observed > now:
            return True
        return now - observed > config.max_observation_age_ns

    def _defer_events(self, events: list[ControlEvent]) -> None:
        self._deferred_events.defer(events)

    def _query_generation(self, context: object) -> None:
        validate_context(context)
        check_generation(context, self._context)

    def _require_lease(self, context: RunContext, lease_id: str) -> None:
        check_generation(context, self._context)
        lease = self._leases.current
        if lease is None:
            raise _invalid("LEASE_EXPIRED", "no control lease")
        check_lease_context(context, lease.context)
        if lease_id != lease.lease_id:
            raise _invalid("LEASE_EXPIRED", "control lease does not match")

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
            events.append(
                self._emit(record, _COMMAND_STOPPING, reason, snapshot.measurement_sim_time_ns)
            )
        self._sync_stream(snapshot, wall, events)
        return snapshot, events

    def _sync_stream(
        self,
        snapshot: MotionSnapshot,
        wall: int,
        events: list[ControlEvent],
    ) -> None:
        record = self._current_record()
        if record is None:
            return
        record.latest_snapshot = snapshot
        if record.motion_seq is None:
            record.motion_seq = snapshot.motion_seq
        if record.stream_id is None and snapshot.stream_id is not None:
            record.stream_id = snapshot.stream_id
        self._translate_chunk_events(record, snapshot, wall, events)
        if record.terminal_status is not None:
            return
        phase = snapshot.phase
        if phase is Phase.COMPLETED:
            self._finish_record(record, snapshot, wall, _COMMAND_COMPLETED, events)
            return
        if phase is Phase.FAILED:
            self._finish_record(record, snapshot, wall, _COMMAND_FAILED, events)
            return
        if phase is Phase.STOPPED:
            self._finish_record(record, snapshot, wall, _COMMAND_CANCELED, events)
            return
        if phase is Phase.STOPPING:
            if record.phase in (_COMMAND_ACCEPTED, _COMMAND_STARTED):
                reason = snapshot.reason or record.reason or "STOP_REQUESTED"
                record.phase = _COMMAND_STOPPING
                record.reason = reason
                record.stop_reason = reason
                events.append(
                    self._emit(
                        record,
                        _COMMAND_STOPPING,
                        reason,
                        snapshot.measurement_sim_time_ns,
                    )
                )
            return
        if record.phase == _COMMAND_ACCEPTED and snapshot.started_sim_ns is not None:
            record.phase = _COMMAND_STARTED
            record.started_sim_ns = snapshot.started_sim_ns
            record.started_wall_ns = snapshot.started_wall_ns
            record.reason = ""
            events.append(
                self._emit(
                    record,
                    _COMMAND_STARTED,
                    "",
                    snapshot.started_sim_ns,
                )
            )

    def _translate_chunk_events(
        self,
        record: CommandRecord,
        snapshot: MotionSnapshot,
        wall: int,
        events: list[ControlEvent],
    ) -> None:
        for control_event in sorted(snapshot.chunk_events, key=lambda item: item.event_seq):
            if control_event.event_seq <= record.last_control_event_seq:
                continue
            record.last_control_event_seq = control_event.event_seq
            chunk = self._chunks.get(record.command.command_id, control_event.chunk_seq)
            if chunk is not None:
                chunk.latest_snapshot = snapshot
                if control_event.phase == "STARTED":
                    chunk.phase = "STARTED"
                    chunk.started_sim_ns = control_event.sim_time_ns
                    chunk.started_wall_ns = wall
                    chunk.reason = control_event.reason
                elif control_event.phase == "RETIRED":
                    chunk.phase = "RETIRED"
                    chunk.finished_sim_ns = control_event.sim_time_ns
                    chunk.finished_wall_ns = wall
                    chunk.reason = control_event.reason
                elif control_event.phase == "CANCELED":
                    chunk.phase = "CANCELED"
                    chunk.finished_sim_ns = control_event.sim_time_ns
                    chunk.finished_wall_ns = wall
                    chunk.reason = control_event.reason
                elif control_event.phase == "REJECTED":
                    chunk.phase = "REJECTED"
                    chunk.finished_sim_ns = control_event.sim_time_ns
                    chunk.finished_wall_ns = wall
                    chunk.reason = control_event.reason
                elif control_event.phase == "ACCEPTED":
                    chunk.reason = control_event.reason or chunk.reason
            self._event_sequence += 1
            if self._event_sequence > MAX_EVENT_SEQUENCE:
                raise _invalid("EVENT_SEQUENCE_EXHAUSTED", "controller event sequence")
            events.append(
                ControlEvent(
                    event_seq=self._event_sequence,
                    context=record.command.context,
                    command_id=record.command.command_id,
                    command_seq=record.command.command_seq,
                    observation_id=control_event.observation_id,
                    chunk_seq=control_event.chunk_seq,
                    phase=control_event.phase,
                    reason=control_event.reason,
                    sim_time_ns=control_event.sim_time_ns,
                )
            )

    def _finish_record(
        self,
        record: CommandRecord,
        snapshot: MotionSnapshot,
        wall: int,
        phase: str,
        events: list[ControlEvent],
    ) -> None:
        result = self._motion.result
        if phase == _COMMAND_COMPLETED:
            reason = _COMMAND_COMPLETED
            stop_reason = None
        elif phase == _COMMAND_CANCELED:
            reason = (
                (result.stop_reason if result is not None else None)
                or record.stop_reason
                or snapshot.reason
                or "STOPPED"
            )
            stop_reason = reason
        else:
            reason = (
                (result.reason if result is not None else None)
                or snapshot.reason
                or _COMMAND_FAILED
            )
            stop_reason = (result.stop_reason if result is not None else None) or record.stop_reason
        record.phase = phase
        record.reason = reason
        record.stop_reason = stop_reason
        record.finished_sim_ns = snapshot.measurement_sim_time_ns
        record.finished_wall_ns = wall
        record.terminal_status = _status_from_record(record)
        events.append(self._emit(record, phase, reason, snapshot.measurement_sim_time_ns))
        self._close_chunks(record, snapshot, wall, phase, reason, events)

    def _close_chunks(
        self,
        record: CommandRecord,
        snapshot: MotionSnapshot,
        wall: int,
        phase: str,
        reason: str,
        events: list[ControlEvent],
    ) -> None:
        for chunk in self._chunks.values():
            if chunk.command.command_id != record.command.command_id:
                continue
            if chunk.phase not in ("ACCEPTED", "STARTED"):
                continue
            chunk.phase = "RETIRED" if phase == _COMMAND_COMPLETED else "CANCELED"
            chunk.reason = _COMMAND_COMPLETED if phase == _COMMAND_COMPLETED else reason
            chunk.finished_sim_ns = snapshot.measurement_sim_time_ns
            chunk.finished_wall_ns = wall
            chunk.latest_snapshot = snapshot
            # A forced stream stop also closes the active block. Its immutable
            # status and its public event must describe the same terminal fact.
            events.append(
                self._emit(
                    record,
                    chunk.phase,
                    chunk.reason,
                    snapshot.measurement_sim_time_ns,
                    chunk_seq=chunk.chunk.seq,
                    observation_id=chunk.command.observation_id,
                )
            )

    def _record_chunk(
        self,
        record: CommandRecord,
        prepared: PreparedFollowerPlan,
        snapshot: MotionSnapshot,
        wall: int,
    ) -> None:
        chunk_record = ChunkRecord(
            packet=record.packet,
            phase="ACCEPTED",
            reason="",
            accepted_sim_ns=snapshot.measurement_sim_time_ns,
            accepted_wall_ns=wall,
            latest_snapshot=snapshot,
        )
        self._chunks.add(chunk_record)
        record.max_chunk_seq = prepared.seq

    def _emit(
        self,
        record: CommandRecord,
        phase: str,
        reason: str,
        sim_time_ns: int,
        *,
        chunk_seq: int | None = None,
        observation_id: int | None = None,
    ) -> ControlEvent:
        self._event_sequence += 1
        if self._event_sequence > MAX_EVENT_SEQUENCE:
            raise _invalid("EVENT_SEQUENCE_EXHAUSTED", "controller event sequence")
        return ControlEvent(
            event_seq=self._event_sequence,
            context=record.command.context,
            command_id=record.command.command_id,
            command_seq=record.command.command_seq,
            observation_id=(
                record.command.observation_id if observation_id is None else observation_id
            ),
            chunk_seq=chunk_seq,
            phase=phase,
            reason=reason,
            sim_time_ns=sim_time_ns,
        )

    def _check_command_conflict(
        self, record: CommandRecord, packet: PreparedControlCommand
    ) -> None:
        command = packet.command
        original = record.command
        if (
            command.command_seq != original.command_seq
            or command.context != original.context
            or command.lease_id != original.lease_id
        ):
            raise _invalid(
                "COMMAND_ID_CONFLICT",
                "same command_id has different sequence/context/lease",
            )
        if record.packet.fingerprint != packet.fingerprint:
            raise _invalid(
                "COMMAND_ID_CONFLICT",
                "same command_id has a different source or plan",
            )

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
        delivered = self._deferred_events.drain() + tuple(events)
        return ControlReply(snapshot, status, delivered, accepted, reason)
