"""Command/chunk status journal and event sequence for one control session."""

from __future__ import annotations

from robot_contracts.models import ContractError
from robot_trajectory.follower import PreparedFollowerPlan

from robot_backend_sim.common.control.control_state import (
    COMMAND_ACCEPTED as _COMMAND_ACCEPTED,
    COMMAND_CANCELED as _COMMAND_CANCELED,
    COMMAND_COMPLETED as _COMMAND_COMPLETED,
    COMMAND_FAILED as _COMMAND_FAILED,
    COMMAND_STARTED as _COMMAND_STARTED,
    COMMAND_STOPPING as _COMMAND_STOPPING,
    MAX_EVENT_SEQUENCE,
    ChunkHistory,
    ChunkRecord,
    CommandRecord,
    ControlEvent,
    PreparedControlCommand,
    _status_from_record,
)
from robot_backend_sim.common.motion.motion_types import MotionSnapshot, Phase

__all__ = ["CommandJournal"]


def _invalid(reason: str, detail: str) -> ContractError:
    return ContractError(reason, detail)


class CommandJournal:
    def __init__(self, chunks: ChunkHistory) -> None:
        self._chunks = chunks
        self._event_sequence = 0

    def next_event_sequence(self) -> int:
        self._event_sequence += 1
        if self._event_sequence > MAX_EVENT_SEQUENCE:
            raise _invalid("EVENT_SEQUENCE_EXHAUSTED", "controller event sequence")
        return self._event_sequence

    def sync_stream(
        self, record: CommandRecord, snapshot: MotionSnapshot, wall: int,
        motion_result: object, events: list[ControlEvent],
    ) -> None:
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
            self.finish_record(record, snapshot, wall, _COMMAND_COMPLETED, events, motion_result)
            return
        if phase is Phase.FAILED:
            self.finish_record(record, snapshot, wall, _COMMAND_FAILED, events, motion_result)
            return
        if phase is Phase.STOPPED:
            self.finish_record(record, snapshot, wall, _COMMAND_CANCELED, events, motion_result)
            return
        if phase is Phase.STOPPING:
            if record.phase in (_COMMAND_ACCEPTED, _COMMAND_STARTED):
                reason = snapshot.reason or record.reason or "STOP_REQUESTED"
                record.phase = _COMMAND_STOPPING
                record.reason = reason
                record.stop_reason = reason
                events.append(
                    self.emit(
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
                self.emit(
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
                elif control_event.phase in {"RETIRED", "CANCELED", "REJECTED"}:
                    chunk.phase = control_event.phase
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

    def finish_record(
        self, record: CommandRecord, snapshot: MotionSnapshot, wall: int,
        phase: str, events: list[ControlEvent], result: object,
    ) -> None:
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
        events.append(self.emit(record, phase, reason, snapshot.measurement_sim_time_ns))
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
                self.emit(
                    record,
                    chunk.phase,
                    chunk.reason,
                    snapshot.measurement_sim_time_ns,
                    chunk_seq=chunk.chunk.seq,
                    observation_id=chunk.command.observation_id,
                )
            )

    def record_chunk(
        self, record: CommandRecord, prepared: PreparedFollowerPlan,
        snapshot: MotionSnapshot, wall: int,
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

    def emit(
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

    def check_conflict(self, record: CommandRecord, packet: PreparedControlCommand) -> None:
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
