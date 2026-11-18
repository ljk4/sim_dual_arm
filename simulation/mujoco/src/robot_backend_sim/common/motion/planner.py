"""Bounded planner threads: follower compilation and task preparation."""

from __future__ import annotations

import hashlib
import json
import queue
import threading
import time
from concurrent.futures import Future
from dataclasses import replace

from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_contracts.stream_request import StreamRequest

from robot_backend_sim.common.config.backend_config import BackendConfig
from robot_backend_sim.common.config.follower_config import FollowerConfig
from robot_backend_sim.common.config.motion_config import ControlConfig
from robot_backend_sim.common.control.control_state import (
    ChunkStatus,
    ControlReply,
    PreparedControlCommand,
)
from robot_backend_sim.common.follower.follower_protocol import (
    follower_reference_from_chunk_status,
    follower_reference_from_status,
    prepare_follower_request,
)
from robot_backend_sim.common.runtime.worker_owner import BackendClosed, SubmitOutcome, _LoggerSink, _Owner
from robot_backend_sim.common.tasks import (
    SimTaskPlugin,
    TaskChunk,
    TaskChunkView,
    TaskPrepareResult,
    TaskPrepareSnapshot,
)

__all__ = ["_Planner", "_TaskPlanner", "_failed_future"]


class _Planner:
    """One planner thread; at most one compile is in flight and queued work is bounded."""

    def __init__(
        self,
        owner: _Owner,
        config: BackendConfig,
        profile: RobotProfile,
        control_config: ControlConfig,
        log: _LoggerSink | None,
        follower_config: FollowerConfig | None = None,
        duration_grid_ns: int | None = 2_000_000,
    ) -> None:
        self._owner = owner
        self._config = config
        self._profile = profile
        self._control_config = control_config
        self._log = log
        self._follower_config: FollowerConfig | None = follower_config
        self._duration_grid_ns = duration_grid_ns
        self._queue: queue.Queue[tuple[StreamRequest, Future]] = queue.Queue(
            maxsize=config.planner_queue_capacity
        )
        self._lock = threading.Lock()
        self._busy_since_ns: int | None = None
        self._active_future: Future | None = None
        self._closing = threading.Event()
        self._thread = threading.Thread(target=self._run, name="backend-planner", daemon=True)

    def start(self) -> None:
        self._thread.start()

    @property
    def busy_since_ns(self) -> int | None:
        with self._lock:
            return self._busy_since_ns

    def submit(self, request: StreamRequest, future: Future) -> None:
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

    @staticmethod
    def _reference(request, packet, status, chunk):
        if packet.request is None:
            return None
        if isinstance(chunk, ChunkStatus):
            return follower_reference_from_chunk_status(
                packet.request,
                packet,
                chunk,
                motion_seq=status.motion_seq or 0,
            )
        return follower_reference_from_status(packet.request, packet, status)

    def _lookup_outcome(
        self, request: StreamRequest, context: object
    ) -> SubmitOutcome | None:
        lookup = self._owner.submit(
            "lookup",
            (
                context,
                request.command.command_id,
                None if request.anchor is None else request.anchor.predecessor_seq + 1,
            ),
            self._config.request_timeout_s,
        )
        if lookup is None:
            return None
        packet, _status = lookup  # type: ignore[misc]
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
        reply = self._owner.submit(
            "admit", (request, packet), self._config.request_timeout_s
        )
        status = reply.status
        if status is None or status.snapshot is None:
            raise ContractError("RESULT_EXPIRED", "stored command has no immutable snapshot")
        chunk = self._owner.submit(
            "query_chunk",
            (context, request.command.command_id, packet.prepared.seq),
            self._config.request_timeout_s,
        )
        return SubmitOutcome(
            reply=reply,
            execution=status,
            chunk=chunk if isinstance(chunk, ChunkStatus) else None,
            reference=self._reference(request, packet, status, chunk),
            idempotent=True,
        )

    def _fresh_outcome(self, request: StreamRequest, context: object) -> SubmitOutcome:
        assert self._follower_config is not None
        packet = prepare_follower_request(
            request,
            self._profile,
            self._control_config,
            self._follower_config,
            duration_grid_ns=self._duration_grid_ns,
        )
        reply = self._owner.submit(
            "admit", (request, packet), self._config.request_timeout_s
        )
        if not isinstance(reply, ControlReply):
            raise ContractError("INVALID_ADMISSION", "owner returned an invalid reply")
        status = reply.status
        chunk = None
        if status is not None and reply.accepted:
            chunk = self._owner.submit(
                "query_chunk",
                (context, request.command.command_id, packet.prepared.seq),
                self._config.request_timeout_s,
            )
        reference = None
        if reply.accepted and status is not None and packet.request is not None:
            reference = self._reference(request, packet, status, chunk)
        return SubmitOutcome(
            reply=reply,
            execution=status,
            chunk=chunk if isinstance(chunk, ChunkStatus) else None,
            reference=reference,
            idempotent=False,
        )

    def _compile_and_admit(self, request: StreamRequest) -> SubmitOutcome:
        if self._follower_config is None:
            raise ContractError(
                "FOLLOWER_DISABLED",
                "follower protocol is not enabled by this backend configuration",
            )
        context = request.command.context
        existing = self._lookup_outcome(request, context)
        if existing is not None:
            return existing
        return self._fresh_outcome(request, context)


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
        plugin: SimTaskPlugin,
        profile: RobotProfile,
        control_config: ControlConfig,
    ) -> None:
        self._owner = owner
        self._config = config
        self._plugin = plugin
        self._profile = profile
        self._control_config = control_config
        self._queue: queue.Queue[tuple[str, dict[str, object], Future]] = queue.Queue(
            maxsize=max(2, config.planner_queue_capacity)
        )
        self._lock = threading.Lock()
        self._busy_since_ns: int | None = None
        self._active_future: Future | None = None
        self._closing = threading.Event()
        self._thread = threading.Thread(target=self._run, name="task-planner", daemon=True)

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
                ContractError("TASK_BUSY", "task planner queue is full; retry")
            )

    def clear_cache(self) -> None:
        self._plugin.clear_chunk_cache()

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

    def _prepare(self, payload: dict[str, object]) -> TaskPrepareResult:
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
                "task_config_digest": self._plugin.config_digest,  # type: ignore[attr-defined]
            }
        )
        snapshot = self._owner.submit(
            "task_snapshot",
            (context, lease_id, request_id, content_digest, episode_id, seed),
            self._config.request_timeout_s,
        )
        if isinstance(snapshot, TaskPrepareResult):
            return snapshot
        if not isinstance(snapshot, TaskPrepareSnapshot):
            raise ContractError("INVALID_TASK_SNAPSHOT", "owner returned no task snapshot")
        plan = self._plugin.build_plan(
            snapshot,
            model=self._owner.model,
            profile=self._profile,
            control_config=self._control_config,
            kinematics_config=self._owner.task_kinematics,
            deadline=time.monotonic() + float(
                getattr(self._plugin, "plan_timeout_s", 30.0)
            ),
        )
        committed = self._owner.submit(
            "task_commit",
            (snapshot, plan, request_id, content_digest, context, lease_id),
            self._config.request_timeout_s,
        )
        if not isinstance(committed, TaskPrepareResult):
            raise ContractError("INVALID_TASK_COMMIT", "owner returned no prepare result")
        self._plugin.clear_chunk_cache()
        return committed

    def _chunk(self, payload: dict[str, object]) -> TaskChunk:
        context = payload["context"]
        task_epoch = payload["task_epoch"]
        request_id = payload["request_id"]
        capture_seq = payload["capture_seq"]
        observation_time_ns = payload["observation_time_ns"]
        handoff_time_ns = int(payload.get("handoff_time_ns", 0))
        assert isinstance(context, RunContext)
        assert isinstance(task_epoch, int)
        assert isinstance(request_id, str)
        assert isinstance(capture_seq, int)
        assert isinstance(observation_time_ns, int)
        view = self._owner.submit(
            "task_chunk_view",
            (context, task_epoch, capture_seq, observation_time_ns),
            self._config.request_timeout_s,
        )
        if not isinstance(view, TaskChunkView):
            raise ContractError("INVALID_TASK_CHUNK", "owner returned no chunk view")
        if handoff_time_ns > 0:
            view = replace(view, handoff_time_ns=handoff_time_ns)
        return self._plugin.sample_chunk(
            view,
            request_id=request_id,
            capture_seq=capture_seq,
            observation_time_ns=observation_time_ns,
            profile=self._profile,
            control_config=self._control_config,
        )
