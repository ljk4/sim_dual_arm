"""The single physics-owner thread; the only place live ``MjData`` is touched."""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from concurrent.futures import Future, wait
from concurrent.futures import TimeoutError as FutureTimeout
from dataclasses import dataclass
from typing import Protocol

from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_contracts.stream_request import REQUEST_STREAM_NEXT, StreamRequest
from robot_contracts.trajectory.follower_reference import FollowerReference

from robot_backend_sim.common.config.backend_config import BackendConfig
from robot_backend_sim.common.config.motion_config import ControlConfig
from robot_backend_sim.common.config.scene_loader import SceneConfigLike
from robot_backend_sim.common.config.stream_config import StreamConfig
from robot_backend_sim.common.control.control import ControlSession
from robot_backend_sim.common.control.control_state import (
    ChunkStatus,
    ControlEvent,
    ControlReply,
    ControlStatus,
    PreparedControlCommand,
)
from robot_backend_sim.common.control.feedback_mailbox import FeedbackMailbox
from robot_backend_sim.common.follower.follower_protocol import follower_reference_from_chunk_status
from robot_backend_sim.common.motion.motion_controller import Phase
from robot_backend_sim.common.physics.robot_adapters import build_io_for, build_scene_for
from robot_backend_sim.common.physics.scene_container import CompiledScene
from robot_backend_sim.common.render.render_worker import Capture, RenderedCapture, RendererThread
from robot_backend_sim.common.render.rendering import capture_snapshot
from robot_backend_sim.common.tasks.base import CAPTURE_HISTORY_LIMIT, SimTaskPlugin

__all__ = ["BackendClosed", "SubmitOutcome", "WorkerFault", "WorkerSummary"]

_MAX_OPERATION_CYCLE = 16
_ACTIVE_PHASES = frozenset({Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING, Phase.STOPPING})
_TERMINAL_PHASES = frozenset({Phase.COMPLETED, Phase.STOPPED, Phase.FAILED})
_OWNER_IDLE_SLEEP_S = 0.0005


class BackendClosed(RuntimeError):
    """The worker is closing and no new operation can be admitted."""


@dataclass(frozen=True, slots=True)
class WorkerFault:
    """A software fault that must fail the process, even if motion CANCELs."""

    reason: str
    detail: str


@dataclass(frozen=True, slots=True)
class SubmitOutcome:
    """Successful admission result for one SubmitFollower service call."""

    reply: ControlReply
    execution: ControlStatus | None
    chunk: ChunkStatus | None
    reference: FollowerReference | None
    idempotent: bool


@dataclass(frozen=True, slots=True)
class WorkerSummary:
    """Counters and terminal facts for ``status.json``."""

    closed: bool
    physics_stopped: bool
    reason: str
    fault: WorkerFault | None
    phase: str
    context: RunContext | None
    accepted_commands: int
    events_emitted: int
    captures: int
    dropped_captures: int
    dropped_snapshots: int
    dropped_outputs: int
    stale_outputs: int
    rendered: int
    renderer_closed: bool
    planner_stopped: bool


class _LoggerSink(Protocol):
    def emit(self, event: str, **kwargs: object) -> bool: ...

    def check_health(self) -> None: ...


@dataclass(slots=True)
class _Operation:
    name: str
    payload: tuple[object, ...]
    future: Future


class _Owner:
    def __init__(
        self,
        config: BackendConfig,
        scene_config: SceneConfigLike,
        profile: RobotProfile,
        control_config: ControlConfig,
        stream_config: StreamConfig,
        task_plugin: SimTaskPlugin | None,
        task_kinematics: object | None,
        log: _LoggerSink | None,
    ) -> None:
        self._config, self._scene_config, self._profile = config, scene_config, profile
        self._control_config, self._stream_config = control_config, stream_config
        self._task_plugin, self._task_kinematics = task_plugin, task_kinematics
        self._last_task_observation = None
        self._reset_seed, self._log = config.seed, log
        self._queue: queue.Queue[_Operation] = queue.Queue(maxsize=config.owner_queue_capacity)
        self._ready, self._done, self._closing = Future(), threading.Event(), threading.Event()
        self._close_reason, self.final_phase, self._close_deadline_ns = "SHUTDOWN", None, 0
        self._fault_lock, self._fault, self._system_stop_requested = threading.Lock(), None, False
        self._renderer = self._health_check = self._controller = None
        self._scene = self._owner_context = None
        self._io = None
        self._renderer_lock = threading.Lock()
        self._captures: queue.Queue[Capture] = queue.Queue(maxsize=config.capture_queue_capacity)
        self._feedback = FeedbackMailbox(config.feedback_queue_capacity)
        self._events: queue.Queue[ControlEvent] = queue.Queue(maxsize=config.event_queue_capacity)
        self._follower_references: queue.Queue[FollowerReference] = queue.Queue(
            maxsize=config.follower_reference_queue_capacity
        )
        self._dropped_captures = self._stale_outputs = 0
        self._capture_seq = self._capture_count = 0
        self._capture_times: deque[tuple[int, int]] = deque(maxlen=CAPTURE_HISTORY_LIMIT)
        self._next_capture_sim_ns = self._accepted_commands = self._events_emitted = 0
        self._feedback_key: tuple[str, str] | None = None
        self._thread = threading.Thread(target=self._run, name="backend-physics", daemon=True)

    # ------------------------------------------------------------- lifecycle
    def start(self) -> None:
        self._thread.start()

    def wait_ready(
        self, timeout_s: float
    ) -> tuple[CompiledScene, RobotProfile, SceneConfigLike]:
        done, _pending = wait((self._ready,), timeout=timeout_s)
        if not done:
            raise TimeoutError("physics owner did not become ready")
        error = self._ready.exception()
        if error is not None:
            raise error
        return self._ready.result()

    def attach_renderer(self, renderer: RendererThread) -> None:
        with self._renderer_lock:
            self._renderer = renderer

    def set_health_check(self, callback: object) -> None:
        self._health_check = callback

    done = property(lambda self: self._done.is_set())

    captures = property(lambda self: self._captures)

    feedback = property(lambda self: self._feedback)

    events = property(lambda self: self._events)

    follower_references = property(lambda self: self._follower_references)

    dropped_captures = property(lambda self: self._dropped_captures)

    stale_outputs = property(lambda self: self._stale_outputs)

    @property
    def fault(self) -> WorkerFault | None:
        with self._fault_lock:
            return self._fault

    context = property(lambda self: self._owner_context)

    task_plugin = property(lambda self: self._task_plugin)

    task_kinematics = property(lambda self: self._task_kinematics)

    capture_seq = property(lambda self: self._capture_seq)

    model = property(lambda self: None if self._scene is None else self._scene.model)

    def latch_fault(self, reason: str, detail: str = "") -> WorkerFault:
        with self._fault_lock:
            if self._fault is None:
                self._fault = WorkerFault(reason, detail or reason)
            return self._fault

    def request_system_stop(self, reason: str, deadline_ns: int) -> None:
        self._close_reason = reason
        self._close_deadline_ns = deadline_ns
        self._closing.set()

    def enqueue(self, name: str, payload: tuple[object, ...]) -> Future:
        """Admit one fixed operation or fail immediately; never block the caller."""
        if self._closing.is_set():
            raise BackendClosed("backend is closing")
        future: Future = Future()
        try:
            self._queue.put_nowait(_Operation(name, payload, future))
        except queue.Full as error:
            raise ContractError("BACKEND_BUSY", "owner operation queue is full") from error
        return future

    def submit(self, name: str, payload: tuple[object, ...], timeout_s: float) -> object:
        future = self.enqueue(name, payload)
        try:
            return future.result(timeout=timeout_s)
        except FutureTimeout as error:
            future.cancel()
            raise TimeoutError(f"owner operation {name!r} timed out") from error

    def close(self, reason: str, timeout_s: float) -> None:
        self.request_system_stop(reason, time.monotonic_ns() + round(timeout_s * 1e9))
        deadline = time.monotonic() + max(0.0, timeout_s)
        self._done.wait(timeout=max(0.0, timeout_s))
        self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        self._drain_pending("backend closed")

    # --------------------------------------------------------------- thread
    def _run(self) -> None:
        controller: ControlSession | None = None
        try:
            scene = build_scene_for(self._scene_config)
            scene.reset(self._config.seed)
            self._reset_seed = self._config.seed
            io = build_io_for(
                scene, self._profile,
                velocity_feedforward=self._config.velocity_feedforward,
            )
            self._io = io
            controller = ControlSession(
                scene, io, self._control_config, stream_config=self._stream_config,
                max_history=self._config.command_history_capacity,
                max_chunk_history=self._config.chunk_history_capacity,
            )
            self._scene = scene
            self._controller = controller
            if self._task_plugin is not None:
                self._task_plugin.bind_scene(scene)
            self._owner_context = controller.context
            self._next_capture_sim_ns = self._sim_ns() + round(
                1_000_000_000 / self._config.capture_hz
            )
            self._ready.set_result((scene, self._profile, self._scene_config))
        except BaseException as error:  # noqa: BLE001 - startup must reach main
            self._ready.set_exception(error)
            self._done.set()
            self._drain_pending("physics owner failed to start")
            return

        assert controller is not None
        step_ns = round(float(self._scene.model.opt.timestep) * 1_000_000_000)  # type: ignore[union-attr]
        if step_ns <= 0:
            self.latch_fault("MODEL_MISMATCH", "scene timestep is not a positive nanosecond grid")
            self._done.set()
            return
        period_ns = step_ns / self._config.realtime_speed
        next_step_ns = time.monotonic_ns()
        last_health_ns = 0
        try:
            while True:
                if self._closing.is_set():
                    if not self._system_stop_requested:
                        self._drain_pending(self._close_reason or "backend closing")
                        self._system_stop(self._close_reason or "shutdown")
                    if self._braking_complete(controller, time.monotonic_ns()):
                        break
                else:
                    self._process_queue(controller)
                now = time.monotonic_ns()
                if now - last_health_ns >= 100_000_000:
                    last_health_ns = now
                    self._check_health(controller, now)
                if now >= next_step_ns:
                    self._step_and_capture(controller, now)
                    next_step_ns += round(period_ns)
                    if now - next_step_ns > self._config.max_catchup_steps * period_ns:
                        next_step_ns = now
                    continue
                timeout = min(
                    _OWNER_IDLE_SLEEP_S, max(0.0, (next_step_ns - now) / 1_000_000_000)
                )
                try:
                    operation = self._queue.get(timeout=timeout)
                except queue.Empty:
                    continue
                self._execute(controller, operation)
            self._owner_context = controller.context
        finally:
            self.final_phase = controller.phase.name
            self._done.set()
            self._drain_pending(self._close_reason or "backend closed")

    def _process_queue(self, controller: ControlSession) -> None:
        for _ in range(_MAX_OPERATION_CYCLE):
            try:
                operation = self._queue.get_nowait()
            except queue.Empty:
                return
            self._execute(controller, operation)

    def _execute(self, controller: ControlSession, operation: _Operation) -> None:
        if not operation.future.set_running_or_notify_cancel():
            return
        try:
            operation.future.set_result(
                self._dispatch(controller, operation.name, operation.payload)
            )
        except BaseException as error:  # noqa: BLE001 - every caller gets the error
            operation.future.set_exception(error)

    def _dispatch(
        self, controller: ControlSession, name: str, payload: tuple[object, ...]
    ) -> object:
        now = time.monotonic_ns()
        if name == "acquire":
            context, owner, duration_ns = payload
            lease = controller.acquire(context, owner, duration_ns, now)  # type: ignore[arg-type]
            self._owner_context = controller.context
            return lease
        if name == "renew":
            context, lease_id, duration_ns = payload
            controller.renew(context, lease_id, duration_ns, now)  # type: ignore[arg-type]
            return None
        if name == "release":
            context, lease_id = payload
            reply = controller.release(context, lease_id, now)  # type: ignore[arg-type]
            self._record_reply(controller, reply)
            if self._task_plugin is not None:
                self._task_plugin.fail_control("CONTROL_RELEASED")
            return reply
        if name == "stop":
            context, lease_id, reason = payload
            reply = controller.stop(context, lease_id, reason, now)  # type: ignore[arg-type]
            self._record_reply(controller, reply)
            if self._task_plugin is not None:
                self._task_plugin.fail_control(f"CONTROL_STOPPED:{reason}")
            return reply
        if name == "lookup":
            context, command_id, chunk_seq = payload
            packet = controller.prepared_command(context, command_id, chunk_seq)  # type: ignore[arg-type]
            if packet is None:
                return None
            status = controller.query_command(context, command_id)  # type: ignore[arg-type]
            return packet, status
        if name == "admit":
            request, packet = payload
            return self._admit(controller, request, packet)  # type: ignore[arg-type]
        if name == "anchor":
            context, lease_id, start_sim_time_ns = payload
            return controller.chunk_anchor(context, lease_id, now, start_sim_time_ns)  # type: ignore[arg-type]
        if name == "query_command":
            context, command_id = payload
            return controller.query_command(context, command_id)  # type: ignore[arg-type]
        if name == "query_chunk":
            context, command_id, chunk_seq = payload
            return controller.query_chunk(context, command_id, chunk_seq)  # type: ignore[arg-type]
        if name.startswith("task_"):
            return self._dispatch_task(controller, name, payload, now)
        if name == "reset":
            context, lease_id, seed = payload
            if self._task_plugin is not None and self._task_plugin.state.phase == "RUNNING":
                raise ContractError(
                    "TASK_BUSY", "ResetSimulation is rejected while a task is running"
                )
            new_context = controller.reset(context, lease_id, seed, now)  # type: ignore[arg-type]
            if self._task_plugin is not None:
                self._task_plugin.reset("EPISODE_RESET")
            self._reset_seed = int(seed)
            self._last_task_observation = None
            self._owner_context = new_context
            self._capture_seq = 0
            self._capture_times.clear()
            self._next_capture_sim_ns = self._sim_ns() + round(
                1_000_000_000 / self._config.capture_hz
            )
            self._clear_queues()
            return new_context
        if name == "identity":
            return controller.context
        if name == "latest_status":
            return controller.current_status()
        if name == "system_stop":
            (reason,) = payload
            self._system_stop_requested = True
            return self._record_reply(controller, controller.system_stop(reason, now))
        raise ContractError("UNKNOWN_OPERATION", name)

    def _dispatch_task(
        self, controller: ControlSession, name: str, payload: tuple[object, ...], now: int
    ) -> object:
        plugin = self._task_plugin
        if plugin is None or self._scene is None:
            raise ContractError("TASK_DISABLED", "no task plugin is configured")
        if name == "task_snapshot":
            context, lease_id, request_id, content_digest, episode_id, seed = payload
            return plugin.prepare_snapshot(
                scene=self._scene, controller=controller, context=context,  # type: ignore[arg-type]
                lease_id=lease_id, request_id=request_id,  # type: ignore[arg-type]
                content_digest=content_digest, episode_id=episode_id,  # type: ignore[arg-type]
                seed=seed, capture_seq=self._capture_seq,  # type: ignore[arg-type]
                sim_time_ns=self._sim_ns(), now=now, reset_seed=self._reset_seed,
            )
        if name == "task_commit":
            snapshot, plan, request_id, content_digest, context, lease_id = payload
            return plugin.commit_prepare(
                scene=self._scene, controller=controller, snapshot=snapshot,  # type: ignore[arg-type]
                plan=plan, request_id=request_id, content_digest=content_digest,  # type: ignore[arg-type]
                context=context, lease_id=lease_id, now=now,  # type: ignore[arg-type]
            )
        if name == "task_start":
            (
                context, lease_id, request_id, content_digest, plan_digest, task_epoch, episode_id,
            ) = payload
            status = controller.current_status()
            return plugin.start(
                controller=controller, context=context, lease_id=lease_id,  # type: ignore[arg-type]
                request_id=request_id, content_digest=content_digest,  # type: ignore[arg-type]
                plan_digest=plan_digest, task_epoch=task_epoch,  # type: ignore[arg-type]
                episode_id=episode_id, sim_time_ns=self._sim_ns(),  # type: ignore[arg-type]
                wall_ns=now, command_id="" if status is None else status.command_id,
            )
        if name == "task_chunk_view":
            context, task_epoch, capture_seq, observation_time_ns = payload
            return plugin.chunk_view(
                context=context, task_epoch=task_epoch,  # type: ignore[arg-type]
                capture_seq=capture_seq, observation_time_ns=observation_time_ns,  # type: ignore[arg-type]
                latest_capture_seq=self._capture_seq, capture_times=self._capture_times,
                current_sim_time_ns=self._sim_ns(),
            )
        if name == "task_state":
            context, task_epoch = payload
            state = plugin.state
            if state.context is not None and context != state.context:
                raise ContractError("STALE_CONTEXT", "task state context differs")
            if task_epoch not in (0, state.task_epoch):
                raise ContractError("STALE_TASK_CONTEXT", "task state epoch differs")
            return state
        raise ContractError("UNKNOWN_OPERATION", name)

    def _admit(
        self,
        controller: ControlSession,
        request: StreamRequest,
        packet: PreparedControlCommand,
    ) -> ControlReply:
        if not isinstance(request, StreamRequest):
            raise ContractError("INVALID_STREAM_REQUEST", "request must be a StreamRequest")
        if not isinstance(packet, PreparedControlCommand):
            raise ContractError(
                "INVALID_PREPARED_COMMAND", "packet must be a PreparedControlCommand"
            )
        if packet.request is not None and packet.request != request:
            raise ContractError(
                "REQUEST_PACKET_MISMATCH",
                "compiled packet does not retain this complete request",
            )
        chunk_seq = None if request.anchor is None else request.anchor.predecessor_seq + 1
        original = controller.prepared_command(
            request.command.context, request.command.command_id, chunk_seq
        )
        reply = (
            controller.submit(packet, time.monotonic_ns())
            if request.kind != REQUEST_STREAM_NEXT
            else controller.queue_chunk(packet, time.monotonic_ns())
        )
        self._record_reply(controller, reply)
        self._accepted_commands += int(original is None and request.kind != REQUEST_STREAM_NEXT)
        status = reply.status
        if (
            original is None and reply.accepted and status is not None
            and packet.request is not None
        ):
            prepared = packet.prepared
            chunk = controller.query_chunk(
                packet.command.context, packet.command.command_id, prepared.seq
            )
            reference = follower_reference_from_chunk_status(
                packet.request, packet, chunk, motion_seq=status.motion_seq or 0
            )
            self._offer(self._follower_references, reference, "follower_reference")
        return reply

    def _record_reply(self, controller: ControlSession, reply: ControlReply) -> ControlReply:
        for event in reply.events:
            self._offer(self._events, event, "event")
            self._events_emitted += 1
        status = reply.status
        if status is not None:
            key = (status.command_id, status.phase)
            if key != self._feedback_key:
                if self._task_plugin is not None and status.phase in ("STOPPING", "FAILED"):
                    # Freeze the first physical stop before runtime cleanup can
                    # replace it with CONTROL_STOPPED:RUNTIME_SHUTDOWN.
                    self._task_plugin.fail_control(status.stop_reason or status.reason)
                self._feedback_key = key
                self._offer(self._feedback, status, "feedback_critical")
        return reply

    def _offer(self, queue_: queue.Queue | FeedbackMailbox, item: object, label: str) -> None:
        if isinstance(queue_, FeedbackMailbox):
            try:
                queue_.put_nowait(item, critical=label == "feedback_critical")
            except queue.Full:
                fault = self.latch_fault("QUEUE_FULL", "feedback transition queue is full")
                self._begin_fault_stop(fault)
            return
        try:
            queue_.put_nowait(item)
            return
        except queue.Full:
            pass
        if label in ("event", "follower_reference"):
            fault = self.latch_fault(
                "QUEUE_FULL", f"{label} queue is full; refusing to drop a control fact"
            )
            self._begin_fault_stop(fault)
            return
        try:
            queue_.get_nowait()
        except queue.Empty:  # pragma: no cover - single producer/consumer
            pass
        try:
            queue_.put_nowait(item)
        except queue.Full:  # pragma: no cover - bounded race
            pass

    def _step_and_capture(self, controller: ControlSession, now: int) -> None:
        try:
            reply = controller.step(now)
        except BaseException as error:  # noqa: BLE001 - a step failure is fatal
            self.latch_fault("PHYSICS_STEP_FAILED", f"{type(error).__name__}: {error}")
            self._begin_fault_stop(self.fault)
            return
        self._record_reply(controller, reply)
        if self._task_plugin is not None:
            try:
                observation = self._task_plugin.capture_observation(self._scene)
                self._task_plugin.tick(observation, now)
                self._last_task_observation = observation
            except BaseException as error:  # noqa: BLE001 - evaluation failure is fatal
                self.latch_fault("TASK_EVALUATION_FAILED", f"{type(error).__name__}: {error}")
                self._begin_fault_stop(self.fault)
                return
        sim_ns = reply.snapshot.measurement_sim_time_ns
        if sim_ns < self._next_capture_sim_ns:
            return
        status = controller.current_status()
        if status is not None and status.phase not in ("COMPLETED", "CANCELED", "FAILED"):
            self._offer(self._feedback, status, "feedback")
        period_ns = round(1_000_000_000 / self._config.capture_hz)
        self._next_capture_sim_ns = max(self._next_capture_sim_ns + period_ns, sim_ns)
        try:
            self._capture(controller, reply, now)
        except BaseException as error:  # noqa: BLE001 - capture failure is fatal
            self.latch_fault("CAPTURE_FAILED", f"{type(error).__name__}: {error}")
            self._begin_fault_stop(self.fault)

    def _capture(self, controller: ControlSession, reply: ControlReply, now: int) -> None:
        renderer = self._renderer
        if renderer is None or renderer.failure is not None:
            self._dropped_captures += 1
            return
        snapshot = capture_snapshot(self._scene.model, self._scene.data)  # type: ignore[union-attr]
        context = controller.context
        status = controller.current_status()
        self._capture_seq += 1
        self._capture_count += 1
        self._capture_times.append((self._capture_seq, reply.snapshot.measurement_sim_time_ns))
        capture = Capture(
            context=context, capture_seq=self._capture_seq,
            sim_time_ns=reply.snapshot.measurement_sim_time_ns,
            joint_names=reply.snapshot.joint_names,
            positions=reply.snapshot.measured_positions,
            velocities=reply.snapshot.measured_velocities,
            phase=reply.snapshot.phase.value,
            command_id=None if status is None else status.command_id, snapshot=snapshot,
        )
        self._update_task_feedback(capture)
        if not renderer.submit(capture):
            self._dropped_captures += 1
            return
        try:
            self._captures.put_nowait(capture)
        except queue.Full:
            self._dropped_captures += 1
            try:
                self._captures.get_nowait()
            except queue.Empty:  # pragma: no cover - single consumer
                pass
            try:
                self._captures.put_nowait(capture)
            except queue.Full:  # pragma: no cover - bounded race
                pass

    def _update_task_feedback(self, capture: Capture) -> None:
        if self._task_plugin is None or self._last_task_observation is None:
            return
        try:
            self._task_plugin.on_capture(capture, self._last_task_observation)
        except BaseException as error:  # noqa: BLE001 - gate evidence is required
            self.latch_fault("TASK_GATE_FAILED", f"{type(error).__name__}: {error}")
            self._begin_fault_stop(self.fault)

    def _check_health(self, controller: ControlSession, now: int) -> None:
        if self._health_check is None:
            return
        try:
            fault = self._health_check(now)  # type: ignore[operator]
        except BaseException as error:  # noqa: BLE001 - health check itself is critical
            self.latch_fault("HEALTH_CHECK_FAILED", f"{type(error).__name__}: {error}")
            fault = self.fault
        if fault is not None:
            self.latch_fault(fault.reason, fault.detail)
            self._begin_fault_stop(fault)
        if self._fault is not None and not self._system_stop_requested:
            self._begin_fault_stop(self._fault)

    def _begin_fault_stop(self, fault: WorkerFault | None) -> None:
        if fault is None or self._system_stop_requested or self._controller is None:
            return
        self._system_stop_requested = True
        self._safe_system_stop(self._controller, fault.reason)

    def _system_stop(self, reason: str) -> None:
        self._system_stop_requested = True
        if self._task_plugin is not None:
            self._task_plugin.fail_control(reason)
        if self._controller is not None:
            self._safe_system_stop(self._controller, reason)

    def _safe_system_stop(self, controller: ControlSession, reason: str) -> None:
        try:
            self._record_reply(controller, controller.system_stop(reason, time.monotonic_ns()))
        except BaseException as error:  # noqa: BLE001 - braking failure is another fault
            self.latch_fault("SYSTEM_STOP_FAILED", f"{type(error).__name__}: {error}")

    def _braking_complete(self, controller: ControlSession, now: int) -> bool:
        if now >= self._close_deadline_ns:
            return True
        phase = controller.phase
        if phase not in _ACTIVE_PHASES:
            return True
        current = controller.current_status()
        if current is not None and current.phase in ("COMPLETED", "CANCELED", "FAILED"):
            return True
        return phase in _TERMINAL_PHASES

    def _sim_ns(self) -> int:
        assert self._scene is not None
        return round(float(self._scene.data.time) * 1_000_000_000)

    def _clear_queues(self) -> None:
        for queue_ in (self._captures,):
            while True:
                try:
                    queue_.get_nowait()
                except queue.Empty:
                    break

    def _drain_pending(self, reason: str) -> None:
        while True:
            try:
                operation = self._queue.get_nowait()
            except queue.Empty:
                return
            if not operation.future.done():
                operation.future.set_exception(BackendClosed(reason))

    def drain_outputs(
        self, renderer: RendererThread, limit: int = 64
    ) -> tuple[RenderedCapture, ...]:
        """Drop outputs from an older generation before the main thread sees them."""
        current = self._owner_context
        kept: list[RenderedCapture] = []
        for item in renderer.drain(limit):
            if current is None or item.capture.context != current:
                self._stale_outputs += 1
                continue
            kept.append(item)
        return tuple(kept)
