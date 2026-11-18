"""Worker orchestration: bounded planner and public worker surface.

The single live-physics owner lives in :mod:`robot_backend_sim.common.runtime.worker_owner`; this
module keeps the one-compile planner thread and the lifecycle/output helpers the
ROS node uses.  Every caller receives an explicit ``Future``.
"""

from __future__ import annotations

import queue
import threading
import time
from concurrent.futures import Future

from robot_contracts.models import ContractError, RunContext
from robot_contracts.profiles import load_profile
from robot_contracts.stream_request import StreamRequest
from robot_contracts.trajectory.follower_reference import FollowerReference

from robot_backend_sim.common.config.backend_config import BackendConfig
from robot_backend_sim.common.config.follower_config import load_follower_config
from robot_backend_sim.common.config.motion_config import load_control_config
from robot_backend_sim.common.config.scene_loader import load_scene_config_for
from robot_backend_sim.common.config.stream_config import load_stream_config
from robot_backend_sim.common.control.control_state import ControlEvent, ControlStatus
from robot_backend_sim.common.motion.planner import _failed_future, _Planner, _request_digest, _TaskPlanner
from robot_backend_sim.common.render.render_worker import (
    Capture,
    RenderCloseSummary,
    RenderedCapture,
    RendererFactory,
    RendererThread,
)
from robot_backend_sim.common.runtime.worker_owner import (
    BackendClosed,
    SubmitOutcome,
    WorkerFault,
    WorkerSummary,
    _LoggerSink,
    _Owner,
)
from robot_backend_sim.common.tasks import SimTaskPlugin, create_task_plugin

__all__ = [
    "BackendClosed",
    "Worker",
    "SubmitOutcome",
    "WorkerFault",
    "WorkerSummary",
]


class Worker:
    """Public orchestrator used by the ROS node and by direct tests."""

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
        scene_config = load_scene_config_for(config.scene_path)  # type: ignore[arg-type]
        control_config = load_control_config(config.control_path)
        stream_config = load_stream_config(config.stream_path)
        profile = load_profile(config.profile_path, config.schemas_path)
        follower_config = (
            None if config.follower_path is None else load_follower_config(config.follower_path)
        )
        task_plugin = (
            None if config.task_path is None
            else create_task_plugin(
                config.task_type, task_path=config.task_path,
                task_mode=config.task_mode, scene_config=scene_config,
            )
        )
        kinematics_config = (
            None if task_plugin is None else task_plugin.load_kinematics(scene_config)
        )
        if task_plugin is not None:
            self._check_task_capacity(config, task_plugin)
        duration_grid_ns = round(scene_config.physics.timestep * 1_000_000_000)
        self.profile = profile
        self.scene_config = scene_config
        self.follower_config = follower_config
        self.task_plugin = task_plugin
        self.task_kinematics = kinematics_config
        self.follower_duration_grid_ns = duration_grid_ns
        self._owner = _Owner(
            config,
            scene_config,
            profile,
            control_config,
            stream_config,
            task_plugin,
            kinematics_config,
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
            if task_plugin is None
            else _TaskPlanner(self._owner, config, task_plugin, profile, control_config)
        )
        self._renderer: RendererThread | None = None
        self._summary: WorkerSummary | None = None
        self._started = False

    @staticmethod
    def _check_task_capacity(config: BackendConfig, plugin: SimTaskPlugin) -> None:
        configured = {
            "chunk_history": config.chunk_history_capacity,
            "command_history": config.command_history_capacity,
            "event_queue": config.event_queue_capacity,
            "accepted_reference_queue": config.accepted_reference_queue_capacity,
        }
        for name, required in plugin.capacity_requirements().items():
            if configured[name] < required:
                raise ContractError(
                    "CAPACITY_MISMATCH",
                    f"{name} {configured[name]} is below task capacity {required}",
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
        if self.task_plugin is not None and self.config.render_quality_enabled:
            import mujoco

            geom_type = int(mujoco.mjtObj.mjOBJ_GEOM)
            segmentation_geoms = {}
            for name, geom_name in self.task_plugin.segmentation_geoms(scene.model).items():
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
        self._emit("backend.worker.start", message="backend worker started")

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
                f"task planner exceeded {self.config.planner_health_timeout_s:g}s",
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
        return self._owner.context

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

    def contact_snapshot(self) -> Future:
        """One consistent owner-thread snapshot of contacts and object motion."""
        return self.request("contact_snapshot", (self._owner.capture_seq,))

    def system_stop(self, reason: str, timeout_s: float | None = None) -> Future:
        return self.request(
            "system_stop",
            (reason,),
        )

    def submit_follower(self, request: StreamRequest) -> Future:
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

    def prepare_task(
        self,
        context: RunContext,
        episode_id: str,
        request_id: str,
        lease_id: str,
        seed: int,
        task_type: str = "",
    ) -> Future:
        if self._task_planner is None or self.task_plugin is None:
            return _failed_future(ContractError("TASK_DISABLED", "no task plugin is configured"))
        if task_type and task_type != self.task_plugin.task_type:
            return _failed_future(
                ContractError("TASK_TYPE_MISMATCH", f"task type {task_type!r} is not configured")
            )
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

    def start_task(
        self,
        context: RunContext,
        episode_id: str,
        request_id: str,
        lease_id: str,
        task_epoch: int,
        plan_digest: str,
    ) -> Future:
        if self._task_planner is None:
            return _failed_future(ContractError("TASK_DISABLED", "no task plugin is configured"))
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
            "task_start",
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

    def get_task_chunk(
        self,
        context: RunContext,
        episode_id: str,
        task_epoch: int,
        request_id: str,
        capture_seq: int,
        observation_time_ns: int,
        handoff_time_ns: int = 0,
    ) -> Future:
        if self._task_planner is None:
            return _failed_future(ContractError("TASK_DISABLED", "no task plugin is configured"))
        if not self.teacher_enabled:
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
                "handoff_time_ns": int(handoff_time_ns),
            },
            future,
        )
        return future

    def get_task_state(self, context: RunContext, task_epoch: int) -> Future:
        if self.task_plugin is None:
            return _failed_future(ContractError("TASK_DISABLED", "no task plugin is configured"))
        return self.request("task_state", (context, task_epoch))

    @property
    def task_state(self):
        return None if self.task_plugin is None else self.task_plugin.state

    @property
    def teacher_enabled(self) -> bool:
        return self.task_plugin is not None and self.task_plugin.teacher_enabled

    @property
    def task_enabled(self) -> bool:
        return self.task_plugin is not None

    def fill_task_chunk_response(self, response: object, chunk: object) -> object:
        if self.task_plugin is None:
            raise ContractError("TASK_DISABLED", "no task plugin is configured")
        return self.task_plugin.fill_chunk_response(response, chunk)  # type: ignore[arg-type]

    def clear_task_cache(self) -> None:
        if self._task_planner is not None:
            self._task_planner.clear_cache()

    def _submit(self, request: StreamRequest) -> Future:
        if self._closed:
            future: Future = Future()
            future.set_exception(BackendClosed("backend is closing"))
            return future
        if not isinstance(request, StreamRequest):
            future: Future = Future()
            future.set_exception(
                ContractError("INVALID_STREAM_REQUEST", "request must be a StreamRequest")
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

    def control_events(self, limit: int = 256) -> tuple[ControlEvent, ...]:
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
        context = self._owner.context
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
