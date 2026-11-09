"""Native worker orchestration: bounded planner and public worker surface.

The single live-physics owner lives in :mod:`robot_backend_sim.worker_owner`; this
module keeps the one-compile planner thread and the lifecycle/output helpers the
ROS node uses.  Every caller receives an explicit ``Future``.
"""

from __future__ import annotations

import hashlib
import json
import queue
import threading
import time
from concurrent.futures import Future

from robot_contracts.follower_reference import FollowerReference
from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_contracts.native import NativeRequest
from robot_contracts.profiles import load_profile

from .backend_config import BackendConfig
from .control_state import (
    ChunkStatus,
    ControlEvent,
    ControlReply,
    ControlStatus,
    PreparedControlCommand,
)
from .follower_config import PandaFollowerConfig, load_follower_config
from .follower_protocol import (
    follower_reference_from_chunk_status,
    follower_reference_from_status,
    prepare_follower_request,
)
from .kinematics_config import KinematicsCheckConfig, load_kinematics_config
from .motion_config import PandaControlConfig, load_control_config
from .render_worker import (
    Capture,
    RenderCloseSummary,
    RenderedCapture,
    RendererFactory,
    RendererThread,
)
from .scene_config import load_scene_config
from .stack_task import (
    EvaluationOnlyPlan,
    StackChunkView,
    StackPrepareResult,
    StackPrepareSnapshot,
    StackTaskState,
)
from .stream_config import load_stream_config
from .task_config import StackTaskConfig, load_task_config, validate_scene_geometry
from .teacher import TeacherChunk, TeacherSnapshot, build_teacher_plan, sample_teacher_chunk
from .worker_owner import (
    BackendClosed,
    SubmitOutcome,
    WorkerFault,
    WorkerSummary,
    _LoggerSink,
    _Owner,
)

__all__ = [
    "BackendClosed",
    "NativeWorker",
    "SubmitOutcome",
    "WorkerFault",
    "WorkerSummary",
]


class _Planner:
    """One planner thread; at most one compile is in flight and queued work is bounded."""

    def __init__(
        self,
        owner: _Owner,
        config: BackendConfig,
        profile: RobotProfile,
        control_config: PandaControlConfig,
        log: _LoggerSink | None,
        follower_config: PandaFollowerConfig | None = None,
        duration_grid_ns: int | None = 2_000_000,
    ) -> None:
        self._owner = owner
        self._config = config
        self._profile = profile
        self._control_config = control_config
        self._log = log
        self._follower_config: PandaFollowerConfig | None = follower_config
        self._duration_grid_ns = duration_grid_ns
        self._queue: queue.Queue[tuple[NativeRequest, Future]] = queue.Queue(
            maxsize=config.planner_queue_capacity
        )
        self._lock = threading.Lock()
        self._busy_since_ns: int | None = None
        self._active_future: Future | None = None
        self._closing = threading.Event()
        self._thread = threading.Thread(target=self._run, name="native-planner", daemon=True)

    def start(self) -> None:
        self._thread.start()

    @property
    def busy_since_ns(self) -> int | None:
        with self._lock:
            return self._busy_since_ns

    def submit(self, request: NativeRequest, future: Future) -> None:
        if self._closing.is_set():
            future.set_exception(BackendClosed("planner is closing"))
            return
        try:
            self._queue.put_nowait((request, future))
        except queue.Full as error:
            future.set_exception(
                ContractError("BACKEND_BUSY", "planner queue is full; retry after querying")
            )
            raise error

    def close(self, timeout_s: float) -> bool:
        self._closing.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=max(0.0, timeout_s))
        with self._lock:
            active = self._active_future
        # Resolve the in-flight future even if a planner call cannot be
        # interrupted; the caller must never wait on a planner that outlives the
        # close budget.
        if active is not None and not active.done():
            active.set_exception(BackendClosed("planner close budget expired"))
        while True:
            try:
                _request, future = self._queue.get_nowait()
            except queue.Empty:
                break
            if not future.done():
                future.set_exception(BackendClosed("planner closed"))
        return not self._thread.is_alive()

    def _run(self) -> None:
        while not self._closing.is_set():
            try:
                request, future = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue
            if not future.set_running_or_notify_cancel():
                continue
            with self._lock:
                self._busy_since_ns = time.monotonic_ns()
                self._active_future = future
            try:
                result = self._compile_and_admit(request)
                if not future.done():
                    future.set_result(result)
            except BaseException as error:  # noqa: BLE001 - caller owns the outcome
                if not future.done():
                    future.set_exception(error)
            finally:
                with self._lock:
                    self._busy_since_ns = None
                    self._active_future = None

    def _compile_and_admit(self, request: NativeRequest) -> SubmitOutcome:
        if self._follower_config is None:
            raise ContractError(
                "FOLLOWER_DISABLED",
                "follower protocol is not enabled by this backend configuration",
            )
        context = request.command.context
        lookup = self._owner.submit(
            "lookup",
            (
                context,
                request.command.command_id,
                None if request.anchor is None else request.anchor.predecessor_seq + 1,
            ),
            self._config.request_timeout_s,
        )
        if lookup is not None:
            packet, status = lookup  # type: ignore[misc]
            if not isinstance(packet, PreparedControlCommand):
                raise ContractError("RESULT_EXPIRED", "stored packet is not immutable")
            if packet.request is not None:
                if packet.request != request:
                    raise ContractError(
                        "COMMAND_ID_CONFLICT",
                        "same command_id has a different complete request",
                    )
            elif packet.command != request.command:
                raise ContractError(
                    "COMMAND_ID_CONFLICT", "same command_id has a different identity"
                )
            if packet.kind != "FOLLOWER":
                raise ContractError(
                    "COMMAND_ID_CONFLICT",
                    "same command_id was not accepted through the follower entry",
                )
            # Idempotent replies still pass through the authoritative lease
            # and generation checks; a history lookup is not control authority.
            reply = self._owner.submit("admit", (request, packet), self._config.request_timeout_s)
            status = reply.status
            if status is None or status.snapshot is None:
                raise ContractError("RESULT_EXPIRED", "stored command has no immutable snapshot")
            chunk = self._owner.submit(
                "query_chunk",
                (context, request.command.command_id, packet.prepared.seq),
                self._config.request_timeout_s,
            )
            reference = None
            if packet.request is not None:
                if isinstance(chunk, ChunkStatus):
                    reference = follower_reference_from_chunk_status(
                        packet.request,
                        packet,
                        chunk,
                        motion_seq=status.motion_seq or 0,
                    )
                else:
                    reference = follower_reference_from_status(packet.request, packet, status)
            return SubmitOutcome(
                reply=reply,
                execution=status,
                chunk=chunk if isinstance(chunk, ChunkStatus) else None,
                reference=reference,
                idempotent=True,
            )
        packet = prepare_follower_request(
            request,
            self._profile,
            self._control_config,
            self._follower_config,
            duration_grid_ns=self._duration_grid_ns,
        )
        reply = self._owner.submit("admit", (request, packet), self._config.request_timeout_s)
        if not isinstance(reply, ControlReply):
            raise ContractError("INVALID_ADMISSION", "owner returned an invalid reply")
        status = reply.status
        chunk = None
        reference = None
        if status is not None and reply.accepted:
            chunk = self._owner.submit(
                "query_chunk",
                (context, request.command.command_id, packet.prepared.seq),
                self._config.request_timeout_s,
            )
        if reply.accepted and status is not None and packet.request is not None:
            if isinstance(chunk, ChunkStatus):
                reference = follower_reference_from_chunk_status(
                    packet.request,
                    packet,
                    chunk,
                    motion_seq=status.motion_seq or 0,
                )
            else:
                reference = follower_reference_from_status(packet.request, packet, status)
        return SubmitOutcome(
            reply=reply,
            execution=status,
            chunk=chunk if isinstance(chunk, ChunkStatus) else None,
            reference=reference,
            idempotent=False,
        )


def _failed_future(error: BaseException) -> Future:
    future: Future = Future()
    future.set_exception(error)
    return future


def _request_digest(payload: dict[str, object]) -> str:
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


class _TaskPlanner:
    """One bounded prepare/teacher thread, separate from live physics.

    It owns no MjData.  Prepare first copies an immutable snapshot on the owner
    thread, then runs mink IK against the read-only model on this thread.  The
    owner commits only the finished immutable plan.
    """

    def __init__(
        self,
        owner: _Owner,
        config: BackendConfig,
        task_config: StackTaskConfig,
        profile: RobotProfile,
        control_config: PandaControlConfig,
        kinematics_config: KinematicsCheckConfig | None,
    ) -> None:
        self._owner = owner
        self._config = config
        self._task_config = task_config
        self._profile = profile
        self._control_config = control_config
        self._kinematics_config = kinematics_config
        self._queue: queue.Queue[tuple[str, dict[str, object], Future]] = queue.Queue(
            maxsize=max(2, config.planner_queue_capacity)
        )
        self._lock = threading.Lock()
        self._busy_since_ns: int | None = None
        self._active_future: Future | None = None
        self._closing = threading.Event()
        self._chunk_cache: dict[str, tuple[str, TeacherChunk]] = {}
        self._thread = threading.Thread(target=self._run, name="stack-task-planner", daemon=True)

    def start(self) -> None:
        self._thread.start()

    @property
    def busy_since_ns(self) -> int | None:
        with self._lock:
            return self._busy_since_ns

    def submit(self, kind: str, payload: dict[str, object], future: Future) -> None:
        if kind not in ("prepare", "chunk"):
            future.set_exception(ContractError("INVALID_TASK_OPERATION", kind))
            return
        if self._closing.is_set():
            future.set_exception(BackendClosed("task planner is closing"))
            return
        try:
            self._queue.put_nowait((kind, payload, future))
        except queue.Full:
            future.set_exception(
                ContractError("TASK_BUSY", "stack-task planner queue is full; retry")
            )

    def clear_cache(self) -> None:
        with self._lock:
            self._chunk_cache.clear()

    def close(self, timeout_s: float) -> bool:
        self._closing.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=max(0.0, timeout_s))
        with self._lock:
            active = self._active_future
        if active is not None and not active.done():
            active.set_exception(BackendClosed("task planner close budget expired"))
        while True:
            try:
                _kind, _payload, future = self._queue.get_nowait()
            except queue.Empty:
                break
            if not future.done():
                future.set_exception(BackendClosed("task planner closed"))
        return not self._thread.is_alive()

    def _run(self) -> None:
        while not self._closing.is_set():
            try:
                kind, payload, future = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue
            if not future.set_running_or_notify_cancel():
                continue
            with self._lock:
                self._busy_since_ns = time.monotonic_ns()
                self._active_future = future
            try:
                result = self._prepare(payload) if kind == "prepare" else self._chunk(payload)
                if not future.done():
                    future.set_result(result)
            except BaseException as error:  # noqa: BLE001 - caller owns the outcome
                if not future.done():
                    future.set_exception(error)
            finally:
                with self._lock:
                    self._busy_since_ns = None
                    self._active_future = None

    def _prepare(self, payload: dict[str, object]) -> StackPrepareResult:
        context = payload["context"]
        episode_id = payload["episode_id"]
        request_id = payload["request_id"]
        lease_id = payload["lease_id"]
        seed = payload["seed"]
        assert isinstance(context, RunContext)
        assert isinstance(episode_id, str)
        assert isinstance(request_id, str)
        assert isinstance(lease_id, str)
        assert isinstance(seed, int)
        content_digest = _request_digest(
            {
                "kind": "prepare",
                "context": context.__dict__,
                "episode_id": episode_id,
                "lease_id": lease_id,
                "seed": seed,
                "task_config_digest": self._task_config.config_digest,
            }
        )
        result = self._owner.submit(
            "stack_snapshot",
            (context, lease_id, request_id, content_digest, episode_id, seed),
            self._config.request_timeout_s,
        )
        if isinstance(result, StackPrepareResult):
            return result
        if not isinstance(result, StackPrepareSnapshot):
            raise ContractError("INVALID_TASK_SNAPSHOT", "owner returned no task snapshot")
        model = self._owner.model
        binding = self._owner.stack_task.binding
        teacher_enabled = (
            self._task_config.teacher.enabled
            and self._config.task_mode == "teacher"
            and self._kinematics_config is not None
        )
        if not teacher_enabled:
            plan = EvaluationOnlyPlan(
                plan_digest=_request_digest(
                    {
                        "kind": "evaluation_only",
                        "task_config_digest": self._task_config.config_digest,
                        "context": result.context.__dict__,
                        "task_epoch": result.task_epoch,
                        "source": result.source.center,
                        "target": result.target.center,
                    }
                ),
                duration_ns=round(self._task_config.success.max_task_sim_duration_s * 1e9),
            )
            binding = self._owner.stack_task.binding
            committed = self._owner.submit(
                "stack_commit",
                (
                    result.context,
                    lease_id,
                    request_id,
                    content_digest,
                    plan,
                    plan.plan_digest,
                    plan.duration_ns,
                    "evaluation_only",
                    result.task_epoch,
                    result.controller_command_id,
                    tuple(
                        result.qpos[binding.source_qpos_address : binding.source_qpos_address + 7]
                    ),
                    tuple(
                        result.qpos[binding.target_qpos_address : binding.target_qpos_address + 7]
                    ),
                ),
                self._config.request_timeout_s,
            )
            if not isinstance(committed, StackPrepareResult):
                raise ContractError("INVALID_TASK_COMMIT", "owner returned no prepare result")
            self.clear_cache()
            return committed
        if model is None or self._kinematics_config is None:
            raise ContractError("INVALID_TASK_SNAPSHOT", "scene model is unavailable")
        snapshot = TeacherSnapshot(
            context=result.context,
            task_epoch=result.task_epoch,
            sim_time_ns=result.sim_time_ns,
            qpos=result.qpos,
            source=result.source,
            target=result.target,
            source_half_extents_m=self._task_config.source_half_extents_m,
            target_half_extents_m=self._task_config.target_half_extents_m,
            left_positions=result.left_positions,
            right_positions=result.right_positions,
            left_gripper_opening_m=result.left_gripper_opening_m,
            right_gripper_opening_m=result.right_gripper_opening_m,
            table_top_z_m=result.table_top_z_m,
        )
        plan = build_teacher_plan(
            model,
            snapshot,
            self._profile,
            self._control_config,
            self._kinematics_config,
            self._task_config,
            deadline=time.monotonic() + self._task_config.teacher.plan_wall_timeout_s,
        )
        committed = self._owner.submit(
            "stack_commit",
            (
                result.context,
                lease_id,
                request_id,
                content_digest,
                plan,
                plan.plan_digest,
                plan.duration_ns,
                "teacher",
                result.task_epoch,
                result.controller_command_id,
                tuple(result.qpos[binding.source_qpos_address : binding.source_qpos_address + 7]),
                tuple(result.qpos[binding.target_qpos_address : binding.target_qpos_address + 7]),
            ),
            self._config.request_timeout_s,
        )
        if not isinstance(committed, StackPrepareResult):
            raise ContractError("INVALID_TASK_COMMIT", "owner returned no prepare result")
        self.clear_cache()
        return committed

    def _chunk(self, payload: dict[str, object]) -> TeacherChunk:
        context = payload["context"]
        episode_id = payload["episode_id"]
        task_epoch = payload["task_epoch"]
        request_id = payload["request_id"]
        capture_seq = payload["capture_seq"]
        observation_time_ns = payload["observation_time_ns"]
        assert isinstance(context, RunContext)
        assert isinstance(episode_id, str)
        assert isinstance(task_epoch, int)
        assert isinstance(request_id, str)
        assert isinstance(capture_seq, int)
        assert isinstance(observation_time_ns, int)
        cached = self._chunk_cache.get(request_id)
        if cached is not None:
            chunk = cached[1]
            if (
                chunk.task_epoch == task_epoch
                and chunk.capture_seq == capture_seq
                and chunk.observation_time_ns == observation_time_ns
            ):
                content_digest = _request_digest(
                    {
                        "kind": "chunk",
                        "context": context.__dict__,
                        "episode_id": episode_id,
                        "task_epoch": task_epoch,
                        "request_id": request_id,
                        "capture_seq": capture_seq,
                        "observation_time_ns": observation_time_ns,
                        "plan_digest": chunk.plan_digest,
                    }
                )
                if content_digest != cached[0]:
                    raise ContractError(
                        "REQUEST_ID_CONFLICT",
                        "same teacher request_id has different content",
                    )
                return chunk
        view = self._owner.submit(
            "stack_chunk_view",
            (context, task_epoch, capture_seq, observation_time_ns),
            self._config.request_timeout_s,
        )
        if not isinstance(view, StackChunkView):
            raise ContractError("INVALID_TASK_CHUNK", "owner returned no chunk view")
        content_digest = _request_digest(
            {
                "kind": "chunk",
                "context": context.__dict__,
                "episode_id": episode_id,
                "task_epoch": task_epoch,
                "request_id": request_id,
                "capture_seq": capture_seq,
                "observation_time_ns": observation_time_ns,
                "plan_digest": view.plan.plan_digest,
            }
        )
        if cached is not None:
            if cached[0] != content_digest:
                raise ContractError(
                    "REQUEST_ID_CONFLICT",
                    "same teacher request_id has different content",
                )
            return cached[1]
        if len(self._chunk_cache) >= self._task_config.teacher.chunk_cache_entries:
            raise ContractError(
                "TEACHER_CACHE_FULL",
                "teacher idempotency cache is full; refusing to drop an audited chunk",
            )
        chunk = sample_teacher_chunk(
            view.plan,
            gate=view.progress,
            feedback=view.feedback,
            request_id=request_id,
            task_epoch=view.task_epoch,
            capture_seq=capture_seq,
            observation_time_ns=observation_time_ns,
            chunk_size=self._task_config.teacher.chunk_size,
            period_ns=self._task_config.teacher.action_period_ns,
            profile=self._profile,
            control_config=self._control_config,
        )
        self._chunk_cache[request_id] = (content_digest, chunk)
        return chunk


class NativeWorker:
    """Public orchestrator used by the ROS node and by native tests."""

    def __init__(
        self,
        config: BackendConfig,
        *,
        log: _LoggerSink | None = None,
        renderer_factory: RendererFactory | None = None,
    ) -> None:
        self.config = config
        self._log = log
        self._renderer_factory = renderer_factory
        self._fault_lock = threading.Lock()
        self._fault: WorkerFault | None = None
        self._closed = False
        scene_config = load_scene_config(config.scene_path)
        control_config = load_control_config(config.control_path)
        stream_config = load_stream_config(config.stream_path)
        profile = load_profile(config.profile_path, config.schemas_path)
        follower_config = (
            None if config.follower_path is None else load_follower_config(config.follower_path)
        )
        task_config = None if config.task_path is None else load_task_config(config.task_path)
        if task_config is not None:
            validate_scene_geometry(task_config, scene_config)
        kinematics_config = None
        if task_config is not None and task_config.teacher.enabled:
            kinematics_config = load_kinematics_config(
                task_config.teacher.kinematics_config_path, scene_config
            )
        if task_config is not None:
            self._check_task_capacity(config, task_config)
        duration_grid_ns = round(scene_config.physics.timestep * 1_000_000_000)
        self.profile = profile
        self.scene_config = scene_config
        self.follower_config = follower_config
        self.task_config = task_config
        self.task_kinematics_config = kinematics_config
        self.follower_duration_grid_ns = duration_grid_ns
        self._owner = _Owner(
            config,
            scene_config,
            profile,
            control_config,
            stream_config,
            task_config,
            log,
        )
        self._planner = _Planner(
            self._owner,
            config,
            profile,
            control_config,
            log,
            follower_config=follower_config,
            duration_grid_ns=duration_grid_ns,
        )
        self._task_planner: _TaskPlanner | None = (
            None
            if task_config is None
            else _TaskPlanner(
                self._owner,
                config,
                task_config,
                profile,
                control_config,
                kinematics_config,
            )
        )
        self._renderer: RendererThread | None = None
        self._summary: WorkerSummary | None = None
        self._started = False

    @staticmethod
    def _check_task_capacity(config: BackendConfig, task_config: StackTaskConfig) -> None:
        capacity = task_config.capacity
        checks = (
            ("backend chunk history", config.chunk_history_capacity, capacity.max_episode_chunks),
            ("command history", config.command_history_capacity, capacity.max_episode_chunks),
            ("event queue", config.event_queue_capacity, capacity.expected_events),
            (
                "accepted reference queue",
                config.accepted_reference_queue_capacity,
                capacity.expected_references,
            ),
        )
        for name, configured, required in checks:
            if configured < required:
                raise ContractError(
                    "CAPACITY_MISMATCH",
                    f"{name} {configured} is below task capacity {required}",
                )

    # ------------------------------------------------------------- lifecycle
    def start(self) -> None:
        if self._started:
            raise RuntimeError("worker is already started")
        self._started = True
        self._owner.start()
        try:
            scene, profile, scene_config = self._owner.wait_ready(self.config.request_timeout_s)
        except BaseException:
            self._owner.close("START_FAILED", self.config.shutdown_timeout_s)
            raise
        segmentation_geoms = None
        if self.task_config is not None and self.config.render_quality_enabled:
            import mujoco

            geom_type = int(mujoco.mjtObj.mjOBJ_GEOM)
            segmentation_geoms = {}
            for name, geom_name in (
                ("source", f"{self.task_config.source_object}_geom"),
                ("target", f"{self.task_config.target_object}_geom"),
                ("table", "table_top"),
            ):
                geom_id = int(
                    mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
                )
                if geom_id >= 0:
                    segmentation_geoms[name] = (geom_id, geom_type)
        renderer = RendererThread(
            scene.model,
            profile.camera_names,
            width=scene_config.cameras[0].width,
            height=scene_config.cameras[0].height,
            snapshot_capacity=self.config.render_snapshot_capacity,
            output_capacity=self.config.render_output_capacity,
            health_timeout_s=self.config.renderer_health_timeout_s,
            renderer_factory=self._renderer_factory,
            segmentation_geoms=segmentation_geoms,
        )
        self._renderer = renderer
        renderer.start()
        try:
            renderer.wait_ready(self.config.request_timeout_s)
        except BaseException:
            renderer.close(self.config.shutdown_timeout_s)
            self._owner.close("RENDER_START_FAILED", self.config.shutdown_timeout_s)
            raise
        self._owner.attach_renderer(renderer)
        self._owner.set_health_check(self._monitor_health)
        self._planner.start()
        if self._task_planner is not None:
            self._task_planner.start()
        self._emit("backend.worker.start", message="native backend worker started")

    def _monitor_health(self, now: int) -> WorkerFault | None:
        renderer = self._renderer
        if renderer is not None:
            failure = renderer.check_health(now)
            if failure is not None:
                return WorkerFault(failure.reason, failure.detail)
        busy = self._planner.busy_since_ns
        if busy is not None and now - busy > round(
            self.config.planner_health_timeout_s * 1_000_000_000
        ):
            return WorkerFault(
                "PLANNER_STALLED",
                f"planner compile exceeded {self.config.planner_health_timeout_s:g}s",
            )
        task_busy = self._task_planner.busy_since_ns if self._task_planner is not None else None
        if task_busy is not None and now - task_busy > round(
            self.config.planner_health_timeout_s * 1_000_000_000
        ):
            return WorkerFault(
                "TASK_PLANNER_STALLED",
                f"stack-task planner exceeded {self.config.planner_health_timeout_s:g}s",
            )
        if self._log is not None:
            try:
                self._log.check_health()
            except Exception as error:  # noqa: BLE001 - logger health is a hard fault
                return WorkerFault("LOGGER_FAILURE", f"{type(error).__name__}: {error}")
        return None

    @property
    def current_context(self) -> RunContext | None:
        """Latest immutable identity, readable without touching live MjData."""
        return self._owner.context()

    # -------------------------------------------------------------- requests
    def request(self, name: str, payload: tuple[object, ...] = ()) -> Future:
        """Return the owner's own Future; the async ROS callback awaits it."""
        return self._owner.enqueue(name, payload)

    def acquire(self, context: object, owner: str, duration_ns: int) -> Future:
        return self.request("acquire", (context, owner, duration_ns))

    def renew(self, context: object, lease_id: str, duration_ns: int) -> Future:
        return self.request("renew", (context, lease_id, duration_ns))

    def release(self, context: object, lease_id: str) -> Future:
        return self.request("release", (context, lease_id))

    def stop(self, context: object, lease_id: str, reason: str) -> Future:
        return self.request("stop", (context, lease_id, reason))

    def get_anchor(self, context: object, lease_id: str, start_sim_time_ns: int | None) -> Future:
        return self.request("anchor", (context, lease_id, start_sim_time_ns))

    def query_command(self, context: object, command_id: str) -> Future:
        return self.request("query_command", (context, command_id))

    def query_chunk(self, context: object, command_id: str, chunk_seq: int) -> Future:
        return self.request("query_chunk", (context, command_id, chunk_seq))

    def reset(self, context: object, lease_id: str, seed: int) -> Future:
        future = self.request("reset", (context, lease_id, seed))
        future.add_done_callback(self._after_task_reset)
        return future

    def _after_task_reset(self, future: Future) -> None:
        if future.cancelled() or future.exception() is not None:
            return
        self.clear_task_cache()

    def identity(self, timeout_s: float | None = None) -> object:
        return self._owner.submit(
            "identity", (), self.config.request_timeout_s if timeout_s is None else timeout_s
        )

    def latest_status(self, timeout_s: float | None = None) -> ControlStatus | None:
        value = self._owner.submit(
            "latest_status", (), self.config.request_timeout_s if timeout_s is None else timeout_s
        )
        return value if isinstance(value, ControlStatus) else None

    def system_stop(self, reason: str, timeout_s: float | None = None) -> Future:
        return self.request(
            "system_stop",
            (reason,),
        )

    def submit_follower(self, request: NativeRequest) -> Future:
        if self.follower_config is None:
            future: Future = Future()
            future.set_exception(
                ContractError(
                    "FOLLOWER_DISABLED",
                    "follower protocol is not enabled by this backend configuration",
                )
            )
            return future
        return self._submit(request)

    def prepare_stack_task(
        self,
        context: RunContext,
        episode_id: str,
        request_id: str,
        lease_id: str,
        seed: int,
    ) -> Future:
        if self._task_planner is None:
            return _failed_future(ContractError("TASK_DISABLED", "stack task is not configured"))
        if self._closed:
            return _failed_future(BackendClosed("backend is closing"))
        future: Future = Future()
        self._task_planner.submit(
            "prepare",
            {
                "context": context,
                "episode_id": episode_id,
                "request_id": request_id,
                "lease_id": lease_id,
                "seed": seed,
            },
            future,
        )
        return future

    def start_stack_task(
        self,
        context: RunContext,
        episode_id: str,
        request_id: str,
        lease_id: str,
        task_epoch: int,
        plan_digest: str,
    ) -> Future:
        if self._task_planner is None or self.task_config is None:
            return _failed_future(ContractError("TASK_DISABLED", "stack task is not configured"))
        content_digest = _request_digest(
            {
                "kind": "start",
                "context": context.__dict__,
                "episode_id": episode_id,
                "request_id": request_id,
                "lease_id": lease_id,
                "task_epoch": task_epoch,
                "plan_digest": plan_digest,
            }
        )
        return self.request(
            "stack_start",
            (
                context,
                lease_id,
                request_id,
                content_digest,
                plan_digest,
                task_epoch,
                episode_id,
            ),
        )

    def get_teacher_chunk(
        self,
        context: RunContext,
        episode_id: str,
        task_epoch: int,
        request_id: str,
        capture_seq: int,
        observation_time_ns: int,
    ) -> Future:
        if self._task_planner is None or self.task_config is None:
            return _failed_future(ContractError("TASK_DISABLED", "stack task is not configured"))
        if not self.task_config.teacher.enabled or self.config.task_mode != "teacher":
            return _failed_future(
                ContractError("TEACHER_DISABLED", "teacher capability is disabled")
            )
        future: Future = Future()
        self._task_planner.submit(
            "chunk",
            {
                "context": context,
                "episode_id": episode_id,
                "task_epoch": task_epoch,
                "request_id": request_id,
                "capture_seq": capture_seq,
                "observation_time_ns": observation_time_ns,
            },
            future,
        )
        return future

    def get_stack_task_state(self, context: RunContext, task_epoch: int) -> Future:
        if self.task_config is None:
            return _failed_future(ContractError("TASK_DISABLED", "stack task is not configured"))
        return self.request("stack_state", (context, task_epoch))

    @property
    def stack_state(self) -> StackTaskState | None:
        runtime = self._owner.stack_task
        return None if runtime is None else runtime.state

    @property
    def teacher_enabled(self) -> bool:
        return (
            self.task_config is not None
            and self.task_config.teacher.enabled
            and self.config.task_mode == "teacher"
        )

    @property
    def task_enabled(self) -> bool:
        return self.task_config is not None

    def clear_task_cache(self) -> None:
        if self._task_planner is not None:
            self._task_planner.clear_cache()

    def _submit(self, request: NativeRequest) -> Future:
        if self._closed:
            future: Future = Future()
            future.set_exception(BackendClosed("backend is closing"))
            return future
        if not isinstance(request, NativeRequest):
            future: Future = Future()
            future.set_exception(
                ContractError("INVALID_NATIVE_REQUEST", "request must be a NativeRequest")
            )
            return future
        future = Future()
        try:
            self._planner.submit(request, future)
        except queue.Full:
            pass  # the planner already set BACKEND_BUSY on this future
        return future

    # ----------------------------------------------------------------- output
    def captures(self) -> tuple[Capture, ...]:
        items: list[Capture] = []
        while True:
            try:
                items.append(self._owner.captures.get_nowait())
            except queue.Empty:
                break
        return tuple(items)

    def feedback(self, limit: int = 128) -> tuple[ControlStatus, ...]:
        items: list[ControlStatus] = []
        for _ in range(limit):
            try:
                items.append(self._owner.feedback.get_nowait())
            except queue.Empty:
                break
        return tuple(items)

    def native_events(self, limit: int = 256) -> tuple[ControlEvent, ...]:
        items: list[ControlEvent] = []
        for _ in range(limit):
            try:
                items.append(self._owner.events.get_nowait())
            except queue.Empty:
                break
        return tuple(items)

    def follower_references(self, limit: int = 64) -> tuple[FollowerReference, ...]:
        items: list[FollowerReference] = []
        for _ in range(limit):
            try:
                items.append(self._owner.follower_references.get_nowait())
            except queue.Empty:
                break
        return tuple(items)

    def rendered(self, limit: int = 64) -> tuple[RenderedCapture, ...]:
        renderer = self._renderer
        if renderer is None:
            return ()
        return self._owner.drain_outputs(renderer, limit)

    # ------------------------------------------------------------------ fault
    @property
    def fault(self) -> WorkerFault | None:
        with self._fault_lock:
            if self._fault is not None:
                return self._fault
        return self._owner.fault

    def latch_fault(self, reason: str, detail: str = "") -> WorkerFault:
        with self._fault_lock:
            if self._fault is None:
                self._fault = WorkerFault(reason, detail or reason)
            fault = self._fault
        self._owner.latch_fault(reason, detail)
        return fault

    def _emit(self, event: str, **kwargs: object) -> bool:
        if self._log is None:
            return True
        try:
            self._log.emit(event, **kwargs)
            return True
        except Exception as error:  # noqa: BLE001 - logging failure is a hard fault
            self.latch_fault("LOGGER_FAILURE", f"{type(error).__name__}: {error}")
            return False

    # ------------------------------------------------------------------ close
    def close(self, reason: str, timeout_s: float | None = None) -> WorkerSummary:
        if self._summary is not None:
            return self._summary
        budget = self.config.shutdown_timeout_s if timeout_s is None else timeout_s
        deadline_ns = time.monotonic_ns() + round(budget * 1_000_000_000)
        self._closed = True
        self._emit("backend.close.begin", message=reason)
        phase = "UNKNOWN"
        try:
            latest = self.latest_status(min(0.5, budget))
            if latest is not None:
                phase = latest.phase
        except BaseException:  # noqa: BLE001 - close must remain bounded
            pass
        owner_budget = min(budget, max(0.05, budget * 0.6))
        self._owner.close(reason, owner_budget)
        phase = self._owner.final_phase or phase
        physics_stopped = not self._owner._thread.is_alive()
        if not physics_stopped:
            self.latch_fault("PHYSICS_CLOSE_TIMEOUT", "physics owner exceeded close budget")
        renderer_summary: RenderCloseSummary | None = None
        renderer = self._renderer
        if renderer is not None:
            remaining = max(0.0, (deadline_ns - time.monotonic_ns()) / 1_000_000_000)
            renderer_summary = renderer.close(remaining)
            if renderer_summary.failure is not None:
                self.latch_fault(renderer_summary.failure.reason, renderer_summary.failure.detail)
        planner_ok = self._planner.close(max(0.0, (deadline_ns - time.monotonic_ns()) / 1e9))
        task_planner_ok = (
            True
            if self._task_planner is None
            else self._task_planner.close(max(0.0, (deadline_ns - time.monotonic_ns()) / 1e9))
        )
        context = self._owner.context()
        fault = self.fault
        summary = WorkerSummary(
            closed=(
                fault is None
                and physics_stopped
                and planner_ok
                and task_planner_ok
                and (renderer_summary is None or renderer_summary.closed)
            ),
            physics_stopped=physics_stopped,
            reason=reason,
            fault=fault,
            phase=phase,
            context=context,
            accepted_commands=self._owner._accepted_commands,
            events_emitted=self._owner._events_emitted,
            captures=self._owner._capture_count,
            dropped_captures=self._owner.dropped_captures,
            dropped_snapshots=0 if renderer_summary is None else renderer_summary.dropped_snapshots,
            dropped_outputs=0 if renderer_summary is None else renderer_summary.dropped_outputs,
            stale_outputs=self._owner.stale_outputs,
            rendered=0 if renderer_summary is None else renderer_summary.rendered,
            renderer_closed=renderer_summary is None or renderer_summary.closed,
            planner_stopped=planner_ok,
        )
        self._summary = summary
        self._emit(
            "backend.close.end",
            message=reason,
            fields={
                "closed": summary.closed,
                "fault": None if summary.fault is None else summary.fault.reason,
                "captures": summary.captures,
                "rendered": summary.rendered,
                "dropped_captures": summary.dropped_captures,
                "dropped_snapshots": summary.dropped_snapshots,
                "dropped_outputs": summary.dropped_outputs,
            },
        )
        return summary
