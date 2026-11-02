"""The single physics-owner thread for the native backend.

All live ``MjData`` access and every ``mj_step`` happen here.  Callers can only
enqueue fixed operations with a bounded queue and wait on their own ``Future``.
The separate planner, renderer and public worker live in
:mod:`robot_backend_sim.worker` and :mod:`robot_backend_sim.render_worker`.
"""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from concurrent.futures import Future, wait
from concurrent.futures import TimeoutError as FutureTimeout
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from robot_contracts.follower_reference import FollowerReference
from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_contracts.native import (
    NATIVE_STREAM_NEXT,
    AcceptedReference,
    NativeRequest,
)

from .backend_config import BackendConfig
from .control import NativeController
from .control_state import (
    ChunkStatus,
    ControlEvent,
    ControlReply,
    ControlStatus,
    PreparedControlCommand,
)
from .feedback_mailbox import FeedbackMailbox
from .follower_protocol import follower_reference_from_chunk_status
from .kinematics import extract_object_pose
from .motion import Phase
from .motion_config import PandaControlConfig
from .panda_io import PandaIO
from .protocol import (
    accepted_reference_from_chunk_status,
    accepted_reference_from_status,
)
from .render_worker import (
    Capture,
    RenderedCapture,
    RendererThread,
)
from .rendering import capture_snapshot
from .scene import Scene, build_scene
from .scene_config import SceneConfig
from .stack_task import (
    StackChunkView,
    StackPrepareResult,
    StackPrepareSnapshot,
    StackTaskError,
    StackTaskRuntime,
    capture_task_observation,
    make_task_binding,
)
from .stream_config import StreamConfig
from .task_config import StackTaskConfig
from .teacher import TeacherFeedback, TeacherProgress

__all__ = [
    "BackendClosed",
    "PlanningFacts",
    "SubmitOutcome",
    "WorkerFault",
    "WorkerSummary",
]

_MAX_OWNER_WAIT_S = 60.0
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
class PlanningFacts:
    """Immutable inputs the planner may use; never a live scene/controller."""

    context: RunContext
    phase: Phase
    hold_reference: tuple[float, ...]
    motion_seq: int


@dataclass(frozen=True, slots=True)
class SubmitOutcome:
    """Successful admission result for one SubmitNative service call."""

    reply: ControlReply
    execution: ControlStatus | None
    chunk: ChunkStatus | None
    reference: AcceptedReference | FollowerReference | None
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
    """The only thread allowed to read/write the live scene."""

    def __init__(
        self,
        config: BackendConfig,
        assets_root: Path,
        summary_path: Path,
        source_path: Path,
        scene_config: SceneConfig,
        profile: RobotProfile,
        control_config: PandaControlConfig,
        stream_config: StreamConfig,
        task_config: StackTaskConfig | None,
        log: _LoggerSink | None,
    ) -> None:
        self._config = config
        self._assets_root = assets_root
        self._summary_path = summary_path
        self._source_path = source_path
        self._scene_config = scene_config
        self._profile = profile
        self._control_config = control_config
        self._stream_config = stream_config
        self._task_config = task_config
        self._stack: StackTaskRuntime | None = None
        self._task_evaluation_fault: WorkerFault | None = None
        self._last_task_observation: object | None = None
        self._prepared_command_id = ""
        self._teacher_progress: TeacherProgress | None = None
        self._teacher_feedback: dict[int, tuple[TeacherFeedback, object]] = {}
        self._reset_seed = config.seed
        self._log = log
        self._queue: queue.Queue[_Operation] = queue.Queue(maxsize=config.owner_queue_capacity)
        self._ready: Future = Future()
        self._done = threading.Event()
        self._closing = threading.Event()
        self._close_reason = "SHUTDOWN"
        self.final_phase: str | None = None
        self._close_deadline_ns = 0
        self._fault_lock = threading.Lock()
        self._fault: WorkerFault | None = None
        self._system_stop_requested = False
        self._renderer: RendererThread | None = None
        self._renderer_lock = threading.Lock()
        self._health_check = None
        self._controller: NativeController | None = None
        self._scene: Scene | None = None
        self._owner_context: RunContext | None = None
        self._captures: queue.Queue[Capture] = queue.Queue(maxsize=config.capture_queue_capacity)
        self._feedback = FeedbackMailbox(config.feedback_queue_capacity)
        self._events: queue.Queue[ControlEvent] = queue.Queue(maxsize=config.event_queue_capacity)
        self._accepted: queue.Queue[AcceptedReference] = queue.Queue(
            maxsize=config.accepted_reference_queue_capacity
        )
        self._follower_references: queue.Queue[FollowerReference] = queue.Queue(
            maxsize=config.follower_reference_queue_capacity
        )
        self._dropped_captures = 0
        self._stale_outputs = 0
        self._capture_seq = 0
        self._capture_count = 0
        self._capture_times: deque[tuple[int, int]] = deque(maxlen=2048)
        self._next_capture_sim_ns = 0
        self._accepted_commands = 0
        self._events_emitted = 0
        self._feedback_key: tuple[str, str] | None = None
        self._thread = threading.Thread(target=self._run, name="native-physics", daemon=True)

    # ------------------------------------------------------------- lifecycle
    def start(self) -> None:
        self._thread.start()

    def wait_ready(self, timeout_s: float) -> tuple[Scene, RobotProfile, SceneConfig]:
        done, _pending = wait((self._ready,), timeout=timeout_s)
        if not done:
            raise TimeoutError("physics owner did not become ready")
        error = self._ready.exception()
        if error is not None:
            raise error
        scene, profile, scene_config = self._ready.result()
        return scene, profile, scene_config

    def attach_renderer(self, renderer: RendererThread) -> None:
        with self._renderer_lock:
            self._renderer = renderer

    def set_health_check(self, callback: object) -> None:
        self._health_check = callback

    @property
    def done(self) -> bool:
        return self._done.is_set()

    @property
    def captures(self) -> queue.Queue[Capture]:
        return self._captures

    @property
    def feedback(self) -> FeedbackMailbox:
        return self._feedback

    @property
    def events(self) -> queue.Queue[ControlEvent]:
        return self._events

    @property
    def accepted(self) -> queue.Queue[object]:
        return self._accepted

    @property
    def follower_references(self) -> queue.Queue[FollowerReference]:
        return self._follower_references

    @property
    def dropped_captures(self) -> int:
        return self._dropped_captures

    @property
    def stale_outputs(self) -> int:
        return self._stale_outputs

    @property
    def fault(self) -> WorkerFault | None:
        with self._fault_lock:
            return self._fault

    def context(self) -> RunContext | None:
        return self._owner_context

    @property
    def stack_task(self) -> StackTaskRuntime | None:
        return self._stack

    @property
    def model(self) -> object | None:
        return None if self._scene is None else self._scene.model

    @property
    def task_config(self) -> StackTaskConfig | None:
        return self._task_config

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
        controller: NativeController | None = None
        try:
            scene = build_scene(
                self._scene_config,
                assets_root=self._assets_root,
                summary_path=self._summary_path,
                source_path=self._source_path,
            )
            scene.reset(self._config.seed)
            self._reset_seed = self._config.seed
            io = PandaIO(
                scene, self._profile, velocity_feedforward=self._config.velocity_feedforward
            )
            controller = NativeController(
                scene,
                io,
                self._control_config,
                stream_config=self._stream_config,
                max_history=self._config.command_history_capacity,
                max_chunk_history=self._config.chunk_history_capacity,
            )
            self._scene = scene
            self._controller = controller
            if self._task_config is not None:
                binding = make_task_binding(
                    scene.model,
                    self._task_config,
                    table_top_z_m=scene.config.table.top_z,
                )
                self._stack = StackTaskRuntime(self._task_config, binding)
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
                        # Refuse every operation admitted before the close flag
                        # became visible; they must not start a new motion while
                        # the process is braking.
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
                        # Wall lateness is bounded; no physical step is skipped,
                        # but the next pacing target restarts from now.
                        next_step_ns = now
                    continue
                timeout = min(
                    _OWNER_IDLE_SLEEP_S,
                    max(0.0, (next_step_ns - now) / 1_000_000_000),
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

    def _process_queue(self, controller: NativeController) -> None:
        for _ in range(_MAX_OPERATION_CYCLE):
            try:
                operation = self._queue.get_nowait()
            except queue.Empty:
                return
            self._execute(controller, operation)

    def _execute(self, controller: NativeController, operation: _Operation) -> None:
        if not operation.future.set_running_or_notify_cancel():
            return
        try:
            result = self._dispatch(controller, operation.name, operation.payload)
            operation.future.set_result(result)
        except BaseException as error:  # noqa: BLE001 - every caller gets the error
            operation.future.set_exception(error)

    def _dispatch(
        self, controller: NativeController, name: str, payload: tuple[object, ...]
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
            if self._stack is not None:
                self._stack.fail_control("CONTROL_RELEASED")
            return self._record_reply(controller, reply)
        if name == "stop":
            context, lease_id, reason = payload
            reply = controller.stop(context, lease_id, reason, now)  # type: ignore[arg-type]
            if self._stack is not None:
                self._stack.fail_control(f"CONTROL_STOPPED:{reason}")
            return self._record_reply(controller, reply)
        if name == "planning_facts":
            return PlanningFacts(
                context=controller.context,
                phase=controller.phase,
                hold_reference=controller.hold_reference(),
                motion_seq=0,
            )
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
            return controller.chunk_anchor(
                context,
                lease_id,
                now,
                start_sim_time_ns,  # type: ignore[arg-type]
            )
        if name == "query_command":
            context, command_id = payload
            return controller.query_command(context, command_id)  # type: ignore[arg-type]
        if name == "query_chunk":
            context, command_id, chunk_seq = payload
            return controller.query_chunk(context, command_id, chunk_seq)  # type: ignore[arg-type]
        if name == "stack_snapshot":
            (
                context,
                lease_id,
                request_id,
                content_digest,
                episode_id,
                seed,
            ) = payload
            return self._stack_snapshot(
                controller,
                context,  # type: ignore[arg-type]
                lease_id,  # type: ignore[arg-type]
                request_id,  # type: ignore[arg-type]
                content_digest,  # type: ignore[arg-type]
                episode_id,  # type: ignore[arg-type]
                seed,  # type: ignore[arg-type]
                now,
            )
        if name == "stack_commit":
            (
                context,
                lease_id,
                request_id,
                content_digest,
                plan,
                plan_digest,
                duration_ns,
                prepare_kind,
                snapshot_epoch,
                snapshot_command_id,
                snapshot_source_qpos,
                snapshot_target_qpos,
            ) = payload
            return self._stack_commit(
                controller,
                context,  # type: ignore[arg-type]
                lease_id,  # type: ignore[arg-type]
                request_id,  # type: ignore[arg-type]
                content_digest,  # type: ignore[arg-type]
                plan,
                plan_digest,  # type: ignore[arg-type]
                duration_ns,  # type: ignore[arg-type]
                prepare_kind,  # type: ignore[arg-type]
                snapshot_epoch,  # type: ignore[arg-type]
                snapshot_command_id,  # type: ignore[arg-type]
                snapshot_source_qpos,  # type: ignore[arg-type]
                snapshot_target_qpos,  # type: ignore[arg-type]
                now,
            )
        if name == "stack_start":
            (
                context,
                lease_id,
                request_id,
                content_digest,
                plan_digest,
                task_epoch,
                episode_id,
            ) = payload
            return self._stack_start(
                controller,
                context,  # type: ignore[arg-type]
                lease_id,  # type: ignore[arg-type]
                request_id,  # type: ignore[arg-type]
                content_digest,  # type: ignore[arg-type]
                plan_digest,  # type: ignore[arg-type]
                task_epoch,  # type: ignore[arg-type]
                episode_id,  # type: ignore[arg-type]
                now,
            )
        if name == "stack_chunk_view":
            context, task_epoch, capture_seq, observation_time_ns = payload
            return self._stack_chunk_view(
                context,  # type: ignore[arg-type]
                task_epoch,  # type: ignore[arg-type]
                capture_seq,  # type: ignore[arg-type]
                observation_time_ns,  # type: ignore[arg-type]
            )
        if name == "stack_state":
            context, task_epoch = payload
            return self._stack_state(context, task_epoch)  # type: ignore[arg-type]
        if name == "reset":
            context, lease_id, seed = payload
            if self._stack is not None and self._stack.state.state == "RUNNING":
                raise StackTaskError(
                    "TASK_BUSY",
                    "ResetSimulation is rejected while a stack task is running",
                )
            new_context = controller.reset(context, lease_id, seed, now)  # type: ignore[arg-type]
            if self._stack is not None:
                self._stack.reset("EPISODE_RESET")
            self._reset_seed = int(seed)
            self._teacher_progress = None
            self._teacher_feedback.clear()
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
            return self._record_reply(
                controller,
                controller.system_stop(reason, now),  # type: ignore[arg-type]
            )
        raise ContractError("UNKNOWN_OPERATION", name)

    def _require_stack(self) -> StackTaskRuntime:
        if self._stack is None or self._task_config is None:
            raise StackTaskError("TASK_DISABLED", "stack task is not configured")
        return self._stack

    def _stack_snapshot(
        self,
        controller: NativeController,
        context: RunContext,
        lease_id: str,
        request_id: str,
        content_digest: str,
        episode_id: str,
        seed: int,
        now: int,
    ) -> object:
        runtime = self._require_stack()
        if not isinstance(request_id, str) or not request_id:
            raise StackTaskError("INVALID_REQUEST_ID", "request_id must be non-empty")
        if not isinstance(content_digest, str) or not content_digest:
            raise StackTaskError("INVALID_REQUEST_ID", "content digest must be non-empty")
        if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
            raise StackTaskError("INVALID_SEED", "seed must be a non-negative integer")
        if not controller.lease_held(context, lease_id, now):
            raise StackTaskError("LEASE_EXPIRED", "prepare does not own the current lease")
        if context != controller.context or episode_id != context.episode_id:
            raise StackTaskError("STALE_CONTEXT", "prepare context/episode differs")
        if runtime.state.state == "RUNNING":
            raise StackTaskError("TASK_BUSY", "a stack task is already running")
        if not controller.is_quiescent():
            raise StackTaskError("TASK_NOT_QUIESCENT", "prepare requires a still robot")
        if seed != self._reset_seed:
            raise StackTaskError(
                "SEED_MISMATCH",
                f"prepare seed {seed} differs from the last Scene.reset seed {self._reset_seed}",
            )
        cached = runtime.lookup_prepare(request_id, content_digest)
        if cached is not None:
            fact = cached.fact
            return StackPrepareResult(
                task_epoch=fact.state.task_epoch,
                config_digest=fact.state.task_config_digest,
                plan_digest=fact.plan_digest,
                key_pose_count=fact.key_pose_count,
                collision_segment_count=fact.collision_segment_count,
                ik_max_position_error_m=fact.ik_max_position_error_m,
                ik_max_orientation_error_rad=fact.ik_max_orientation_error_rad,
                planned_duration_s=fact.plan_duration_s,
                idempotent=True,
                state=fact.state,
                prepare_kind=getattr(fact, "prepare_kind", "teacher"),
            )
        runtime.require_prepare_slot()
        assert self._scene is not None
        scene = self._scene
        binding = runtime.binding
        qpos = tuple(float(value) for value in scene.data.qpos)
        source = extract_object_pose(
            qpos, binding.source_qpos_address, self._task_config.source_object
        )
        target = extract_object_pose(
            qpos, binding.target_qpos_address, self._task_config.target_object
        )
        indices = scene.indices
        left = tuple(qpos[address] for address in indices.arm_joint_qpos["left"])
        right = tuple(qpos[address] for address in indices.arm_joint_qpos["right"])
        left_gripper = float(sum(qpos[address] for address in indices.finger_joint_qpos["left"]))
        right_gripper = float(sum(qpos[address] for address in indices.finger_joint_qpos["right"]))
        return StackPrepareSnapshot(
            context=context,
            task_epoch=runtime.task_epoch + 1,
            sim_time_ns=self._sim_ns(),
            capture_seq=self._capture_seq,
            qpos=qpos,
            source=source,
            target=target,
            left_positions=left,
            right_positions=right,
            left_gripper_opening_m=left_gripper,
            right_gripper_opening_m=right_gripper,
            table_top_z_m=float(scene.config.table.top_z),
            seed=seed,
            controller_command_id=(
                ""
                if controller.current_status() is None
                else controller.current_status().command_id
            ),
        )

    def _stack_commit(
        self,
        controller: NativeController,
        context: RunContext,
        lease_id: str,
        request_id: str,
        content_digest: str,
        plan: object,
        plan_digest: str,
        duration_ns: int,
        prepare_kind: str,
        snapshot_epoch: int,
        snapshot_command_id: str,
        snapshot_source_qpos: tuple[float, ...],
        snapshot_target_qpos: tuple[float, ...],
        now: int,
    ) -> StackPrepareResult:
        runtime = self._require_stack()
        if not controller.lease_held(context, lease_id, now):
            raise StackTaskError("LEASE_EXPIRED", "prepare lease expired before commit")
        if context != controller.context:
            raise StackTaskError("STALE_CONTEXT", "prepare context changed before commit")
        if runtime.state.state != "IDLE" or runtime.task_epoch != snapshot_epoch - 1:
            raise StackTaskError("STALE_TASK_PREPARE", "task generation changed before commit")
        status = controller.current_status()
        current_command_id = "" if status is None else status.command_id
        if current_command_id != snapshot_command_id:
            raise StackTaskError(
                "STALE_TASK_PREPARE", "another motion ran while task planning was pending"
            )
        if not controller.is_quiescent():
            raise StackTaskError("TASK_NOT_QUIESCENT", "robot moved before prepare commit")
        assert self._scene is not None
        scene = self._scene
        binding = runtime.binding
        current_source = tuple(
            float(value)
            for value in scene.data.qpos[
                binding.source_qpos_address : binding.source_qpos_address + 7
            ]
        )
        current_target = tuple(
            float(value)
            for value in scene.data.qpos[
                binding.target_qpos_address : binding.target_qpos_address + 7
            ]
        )

        def moved(current: tuple[float, ...], original: tuple[float, ...]) -> bool:
            if any(abs(a - b) > 1e-4 for a, b in zip(current[:3], original[:3], strict=True)):
                return True
            dot = abs(sum(a * b for a, b in zip(current[3:7], original[3:7], strict=True)))
            return abs(dot - 1.0) > 1e-6

        if moved(current_source, snapshot_source_qpos) or moved(
            current_target, snapshot_target_qpos
        ):
            raise StackTaskError(
                "STALE_TASK_PREPARE", "scene objects moved between snapshot and commit"
            )
        ik_results = getattr(plan, "ik_results", ())
        collision_results = getattr(plan, "collision_results", ())
        key_poses = getattr(plan, "key_poses", ())
        max_position = 0.0
        max_orientation = 0.0
        for result in ik_results:
            max_position = max(max_position, float(result.position_error_m))
            max_orientation = max(max_orientation, float(result.orientation_error_rad))
        self._teacher_progress = None
        self._teacher_feedback.clear()
        self._last_task_observation = None
        outcome = runtime.commit_prepare(
            request_id=request_id,
            content_digest=content_digest,
            context=context,
            plan=plan,
            plan_digest=plan_digest,
            plan_duration_ns=duration_ns,
            key_pose_count=len(key_poses),
            collision_segment_count=len(collision_results),
            ik_max_position_error_m=max_position,
            ik_max_orientation_error_rad=max_orientation,
            prepare_kind=prepare_kind,
        )
        self._prepared_command_id = snapshot_command_id
        return StackPrepareResult(
            task_epoch=outcome.fact.state.task_epoch,
            config_digest=outcome.fact.state.task_config_digest,
            plan_digest=plan_digest,
            key_pose_count=len(key_poses),
            collision_segment_count=len(collision_results),
            ik_max_position_error_m=max_position,
            ik_max_orientation_error_rad=max_orientation,
            planned_duration_s=duration_ns / 1e9,
            idempotent=outcome.idempotent,
            prepare_kind=prepare_kind,
            state=outcome.fact.state,
        )

    def _stack_start(
        self,
        controller: NativeController,
        context: RunContext,
        lease_id: str,
        request_id: str,
        content_digest: str,
        plan_digest: str,
        task_epoch: int,
        episode_id: str,
        now: int,
    ) -> object:
        runtime = self._require_stack()
        if not controller.lease_held(context, lease_id, now):
            raise StackTaskError("LEASE_EXPIRED", "start does not own the current lease")
        if context != controller.context or episode_id != context.episode_id:
            raise StackTaskError("STALE_CONTEXT", "start context/episode differs")
        status = controller.current_status()
        command_id = "" if status is None else status.command_id
        if runtime.state.state == "PREPARED" and command_id != self._prepared_command_id:
            raise StackTaskError("STALE_TASK_PREPARE", "robot command changed after prepare")
        state, idempotent = runtime.start(
            request_id=request_id,
            content_digest=content_digest,
            context=context,
            plan_digest=plan_digest,
            task_epoch=task_epoch,
            sim_time_ns=self._sim_ns(),
            wall_ns=now,
        )
        if (
            idempotent is False
            and self._task_config is not None
            and self._config.task_mode == "teacher"
            and self._task_config.teacher.enabled
            and runtime.plan is not None
            and hasattr(runtime.plan, "segments")
        ):
            self._teacher_progress = TeacherProgress(runtime.plan, self._task_config)  # type: ignore[arg-type]
            self._teacher_feedback.clear()
        return state, idempotent

    def _stack_chunk_view(
        self,
        context: RunContext,
        task_epoch: int,
        capture_seq: int,
        observation_time_ns: int,
    ) -> object:
        runtime = self._require_stack()
        state = runtime.state
        if state.state != "RUNNING":
            raise StackTaskError("TASK_NOT_RUNNING", "teacher chunks require a running task")
        if state.context != context or task_epoch != state.task_epoch:
            raise StackTaskError("STALE_TASK_CONTEXT", "teacher chunk task epoch differs")
        if isinstance(capture_seq, bool) or not isinstance(capture_seq, int) or capture_seq < 1:
            raise StackTaskError("INVALID_CAPTURE", "capture_seq must be positive")
        if capture_seq > self._capture_seq:
            raise StackTaskError("CAPTURE_NOT_RETAINED", "capture_seq is not retained yet")
        current_sim = self._sim_ns()
        if observation_time_ns > current_sim:
            raise StackTaskError(
                "TEACHER_FUTURE_OBSERVATION",
                "observation time is later than the current simulation time",
            )
        retained_time = next(
            (sim_ns for seq, sim_ns in reversed(self._capture_times) if seq == capture_seq),
            None,
        )
        if retained_time is None:
            raise StackTaskError("CAPTURE_NOT_RETAINED", "capture_seq is no longer retained")
        if observation_time_ns != retained_time:
            raise StackTaskError(
                "TEACHER_OBSERVATION_MISMATCH",
                "observation time does not belong to the retained capture",
            )
        if observation_time_ns < state.task_start_sim_time_ns:
            raise StackTaskError("TEACHER_BEFORE_START", "observation precedes task start")
        plan = runtime.plan
        if plan is None:
            raise StackTaskError("TASK_NOT_PREPARED", "task has no immutable plan")
        feedback = self._teacher_feedback.get(capture_seq)
        if feedback is None or self._teacher_progress is None:
            raise StackTaskError(
                "CAPTURE_FEEDBACK_MISSING",
                "no immutable teacher feedback exists for this capture",
            )
        return StackChunkView(
            plan=plan,
            task_epoch=state.task_epoch,
            task_start_sim_time_ns=state.task_start_sim_time_ns,
            current_sim_time_ns=current_sim,
            latest_capture_seq=self._capture_seq,
            state=state,
            feedback=feedback[0],
            progress=feedback[1],
        )

    def _stack_state(self, context: RunContext, task_epoch: int) -> object:
        runtime = self._require_stack()
        state = runtime.state
        if state.context is not None and context != state.context:
            raise StackTaskError("STALE_CONTEXT", "task state context differs")
        if task_epoch not in (0, state.task_epoch):
            raise StackTaskError("STALE_TASK_CONTEXT", "task state epoch differs")
        return state

    def _admit(
        self,
        controller: NativeController,
        request: NativeRequest,
        packet: PreparedControlCommand,
    ) -> ControlReply:
        if not isinstance(request, NativeRequest):
            raise ContractError("INVALID_NATIVE_REQUEST", "request must be a NativeRequest")
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
        if packet.kind == "MOTION" and original is None:
            if packet.prepared.initial_reference != controller.hold_reference():  # type: ignore[union-attr]
                raise ContractError(
                    "INITIAL_REFERENCE_MISMATCH",
                    "planner did not use the owner's current last-hold reference",
                )
        reply = (
            controller.submit(packet, time.monotonic_ns())
            if request.kind != NATIVE_STREAM_NEXT
            else controller.queue_chunk(packet, time.monotonic_ns())
        )
        self._record_reply(controller, reply)
        self._accepted_commands += int(original is None and request.kind != NATIVE_STREAM_NEXT)
        status = reply.status
        if (
            original is None
            and reply.accepted
            and status is not None
            and packet.request is not None
        ):
            if packet.kind in ("CHUNK", "FOLLOWER"):
                prepared = packet.prepared
                chunk = controller.query_chunk(
                    packet.command.context,
                    packet.command.command_id,
                    prepared.seq,  # type: ignore[union-attr]
                )
                if packet.kind == "FOLLOWER":
                    reference = follower_reference_from_chunk_status(
                        packet.request, packet, chunk, motion_seq=status.motion_seq or 0
                    )
                    self._offer(self._follower_references, reference, "follower_reference")
                else:
                    reference = accepted_reference_from_chunk_status(
                        packet.request, packet, chunk, motion_seq=status.motion_seq or 0
                    )
                    self._offer(self._accepted, reference, "accepted_reference")
            else:
                reference = accepted_reference_from_status(packet.request, packet, status)
                self._offer(self._accepted, reference, "accepted_reference")
        return reply

    def _record_reply(self, controller: NativeController, reply: ControlReply) -> ControlReply:
        for event in reply.events:
            self._offer(self._events, event, "event")
            self._events_emitted += 1
        status = reply.status
        if status is not None:
            key = (status.command_id, status.phase)
            if key != self._feedback_key:
                # A phase transition or terminal fact must reach the node even
                # if the high-frequency feedback queue is under pressure.
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
        if label in ("event", "accepted_reference", "follower_reference"):
            # Control transitions and accepted references must never disappear
            # silently.  Latch a hard fault and stop the backend instead of
            # dropping the fact; the node reports the nonzero run failure.
            fault = self.latch_fault(
                "QUEUE_FULL", f"{label} queue is full; refusing to drop a control fact"
            )
            self._begin_fault_stop(fault)
            return
        # Periodic feedback is intentionally lossy at its bound.
        try:
            queue_.get_nowait()
        except queue.Empty:  # pragma: no cover - single producer/consumer
            pass
        try:
            queue_.put_nowait(item)
        except queue.Full:  # pragma: no cover - bounded race
            pass

    def _step_and_capture(self, controller: NativeController, now: int) -> None:
        try:
            reply = controller.step(now)
        except BaseException as error:  # noqa: BLE001 - a step failure is fatal
            self.latch_fault("PHYSICS_STEP_FAILED", f"{type(error).__name__}: {error}")
            self._begin_fault_stop(self.fault)
            return
        self._record_reply(controller, reply)
        if self._stack is not None:
            try:
                observation = capture_task_observation(
                    self._scene,
                    self._stack.binding,
                    self._task_config,  # type: ignore[arg-type]
                )
                self._stack.tick(observation, now)
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

    def _capture(self, controller: NativeController, reply: ControlReply, now: int) -> None:
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
            context=context,
            capture_seq=self._capture_seq,
            sim_time_ns=reply.snapshot.measurement_sim_time_ns,
            joint_names=reply.snapshot.joint_names,
            positions=reply.snapshot.measured_positions,
            velocities=reply.snapshot.measured_velocities,
            phase=reply.snapshot.phase.value,
            command_id=None if status is None else status.command_id,
            snapshot=snapshot,
        )
        self._update_teacher_feedback(capture)
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

    def _update_teacher_feedback(self, capture: Capture) -> None:
        if (
            self._teacher_progress is None
            or self._stack is None
            or self._task_config is None
            or self._last_task_observation is None
        ):
            return
        try:
            feedback = self._teacher_progress.feedback_for(
                capture,
                self._last_task_observation,  # type: ignore[arg-type]
            )
            fact = self._teacher_progress.update(feedback, self._stack.state.state)
        except BaseException as error:  # noqa: BLE001 - gate evidence is required
            self.latch_fault("TASK_GATE_FAILED", f"{type(error).__name__}: {error}")
            self._begin_fault_stop(self.fault)
            return
        if len(self._teacher_feedback) >= 2048:
            self._teacher_feedback.clear()
        self._teacher_feedback[capture.capture_seq] = (feedback, fact)

    def _check_health(self, controller: NativeController, now: int) -> None:
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
        if self._stack is not None:
            self._stack.fail_control(reason)
        if self._controller is not None:
            self._safe_system_stop(self._controller, reason)

    def _safe_system_stop(self, controller: NativeController, reason: str) -> None:
        try:
            reply = controller.system_stop(reason, time.monotonic_ns())
            self._record_reply(controller, reply)
        except BaseException as error:  # noqa: BLE001 - braking failure is another fault
            self.latch_fault("SYSTEM_STOP_FAILED", f"{type(error).__name__}: {error}")

    def _braking_complete(self, controller: NativeController, now: int) -> bool:
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
        # Old control facts keep their original context and must still be
        # delivered. Only replaceable capture payloads are invalidated here.
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
