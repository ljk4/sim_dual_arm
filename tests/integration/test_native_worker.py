"""Real-physics tests for the native worker's threads, queues and faults.

The renderer is injected in these host tests: it exercises snapshot delivery,
bounds and failure propagation, but it is not GPU/EGL acceptance.  Real EGL and
ROS services are covered by ``test_native_backend_ros.py`` under Docker.
"""

from __future__ import annotations

import sys
import time
from dataclasses import replace
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

ROOT = Path(__file__).resolve().parents[2]
for source in (
    ROOT / "simulation/mujoco/src",
    ROOT / "contracts/python/src",
    ROOT / "libraries/run_logging/src",
):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

import robot_backend_sim.worker as worker_module  # noqa: E402
from robot_backend_sim.backend_config import load_backend_config  # noqa: E402
from robot_backend_sim.worker import BackendClosed, NativeWorker  # noqa: E402
from robot_contracts.models import ContractError, Target, Trajectory  # noqa: E402
from robot_contracts.native import NATIVE_MOTION, NativeRequest  # noqa: E402

CONFIG_PATH = ROOT / "simulation/mujoco/configs/backend/backend.yaml"
STACK_TEACHER_CONFIG = ROOT / "simulation/mujoco/configs/backend/backend_stack_teacher.yaml"
STACK_INFERENCE_CONFIG = ROOT / "simulation/mujoco/configs/backend/backend_stack_inference.yaml"


class FakeRenderer:
    delay_s = 0.0
    fail = False
    rendered = 0

    def __init__(self, model, camera_names, width, height):
        self.camera_names = tuple(camera_names)
        self.width = width
        self.height = height
        type(self).rendered = 0

    def render_rgb(self, snapshot):
        time.sleep(type(self).delay_s)
        if type(self).fail:
            raise RuntimeError("injected renderer failure")
        type(self).rendered += 1
        return {
            name: np.zeros((self.height, self.width, 3), dtype=np.uint8)
            for name in self.camera_names
        }

    def close(self):
        pass


def config_for(**changes):
    return replace(load_backend_config(CONFIG_PATH), **changes)


def new_worker_from(path, renderer=FakeRenderer, **changes):
    FakeRenderer.delay_s = 0.0
    FakeRenderer.fail = False
    worker = NativeWorker(
        replace(load_backend_config(path), **changes),
        renderer_factory=renderer,
    )
    worker.start()
    return worker


def new_worker(renderer=FakeRenderer, **changes):
    return new_worker_from(CONFIG_PATH, renderer=renderer, **changes)


def new_stack_worker(mode="teacher", renderer=FakeRenderer, **changes):
    path = (
        STACK_TEACHER_CONFIG if mode == "teacher" else STACK_INFERENCE_CONFIG
    )
    return new_worker_from(path, renderer=renderer, **changes)


def context_for(worker, *, episode="episode-1", control_epoch=1):
    return replace(worker.current_context, episode_id=episode, control_epoch=control_epoch)


def motion_request(
    worker,
    context,
    lease,
    positions,
    *,
    command_id="command-1",
    duration_ns=1_000_000_000,
):
    trajectory = Trajectory(
        context,
        lease,
        command_id,
        1,
        1,
        time.monotonic_ns() + 5_000_000_000,
        tuple(worker.profile.joint_names),
        (Target(duration_ns, tuple(positions)),),
    )
    return NativeRequest(trajectory, NATIVE_MOTION)


def drain_until(worker, predicate, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = predicate(worker)
        if value:
            return value
        time.sleep(0.01)
    return predicate(worker)


def test_single_owner_and_real_stepping():
    worker = new_worker(realtime_speed=20.0, capture_hz=50.0)
    try:
        controller = worker._owner._controller
        with pytest.raises(ContractError, match="WRONG_THREAD"):
            controller.step(time.monotonic_ns())
        first = drain_until(worker, lambda item: item.captures() or None)
        assert first
        before = first[-1].sim_time_ns
        later = drain_until(
            worker,
            lambda item: next((c for c in item.captures() if c.sim_time_ns > before), None),
        )
        assert later is not None
        assert later.sim_time_ns > before
    finally:
        summary = worker.close("TEST_DONE", 5.0)
    assert summary.closed


def test_motion_moves_measured_state_and_captures_are_coherent():
    worker = new_worker(realtime_speed=4.0, capture_hz=50.0)
    seen = []
    try:
        ctx = context_for(worker)
        lease = worker.acquire(ctx, "test", 10_000_000_000).result(5)
        target = list(worker.profile.initial_positions)
        target[0] = 0.08
        request = motion_request(worker, ctx, lease, target, duration_ns=2_000_000_000)
        outcome = worker.submit_native(request).result(10)
        assert outcome.reply.accepted is True
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            seen.extend(worker.captures())
            worker.rendered()
            status = worker.latest_status()
            if status is not None and status.phase in ("COMPLETED", "CANCELED", "FAILED"):
                break
            time.sleep(0.01)
        positions = [capture.positions[0] for capture in seen]
        assert len(positions) >= 2
        assert max(positions) - min(positions) > 0.01
        for capture in seen:
            assert capture.context == ctx
            assert capture.sim_time_ns == round(capture.snapshot.sim_time * 1e9)
    finally:
        summary = worker.close("TEST_DONE", 5.0)
    assert summary.closed
    assert summary.rendered > 0


def test_idempotent_retry_reuses_original_packet_and_receipt():
    worker = new_worker(realtime_speed=20.0, capture_hz=20.0)
    try:
        ctx = context_for(worker)
        lease = worker.acquire(ctx, "test", 10_000_000_000).result(5)
        target = list(worker.profile.initial_positions)
        target[0] = 0.03
        request = motion_request(worker, ctx, lease, target)
        first = worker.submit_native(request).result(10)
        first_digest = first.reference.reference_digest
        first_sim = first.reference.accepted_sim_time_ns
        first_seq = first.reference.motion_seq
        time.sleep(0.3)
        second = worker.submit_native(request).result(10)
        assert second.idempotent is True
        assert second.reference.reference_digest == first_digest
        assert second.reference.accepted_sim_time_ns == first_sim
        assert second.reference.motion_seq == first_seq
        changed = list(worker.profile.initial_positions)
        changed[0] = 0.04
        conflict = motion_request(worker, ctx, lease, changed)
        with pytest.raises(ContractError, match="COMMAND_ID_CONFLICT"):
            worker.submit_native(conflict).result(10)
    finally:
        worker.close("TEST_DONE", 5.0)


def test_lease_expiry_requests_real_brake():
    worker = new_worker(realtime_speed=1.0, capture_hz=20.0)
    try:
        ctx = context_for(worker)
        lease = worker.acquire(ctx, "test", 200_000_000).result(5)
        target = list(worker.profile.initial_positions)
        target[0] = 0.20
        request = motion_request(worker, ctx, lease, target, duration_ns=3_000_000_000)
        worker.submit_native(request).result(10)

        def terminal(item):
            status = item.latest_status()
            return (
                status
                if status is not None
                and status.phase
                in (
                    "COMPLETED",
                    "CANCELED",
                    "FAILED",
                )
                else None
            )

        status = drain_until(worker, terminal, timeout_s=5.0)
        assert status is not None
        assert status.phase in ("CANCELED", "FAILED")
        assert status.stop_reason == "LEASE_EXPIRED"
    finally:
        summary = worker.close("TEST_DONE", 5.0)
    assert summary.closed


def test_reset_changes_epoch_and_clears_queues():
    worker = new_worker(realtime_speed=20.0, capture_hz=20.0)
    try:
        ctx = context_for(worker, episode="first")
        lease = worker.acquire(ctx, "test", 10_000_000_000).result(5)
        drain_until(worker, lambda item: item.captures() or None)
        new_context = worker.reset(ctx, lease, 42).result(5)
        assert new_context.clock_epoch == ctx.clock_epoch + 1
        assert new_context.episode_id == ""
        assert new_context.control_epoch == 0
        assert worker.current_context.clock_epoch == new_context.clock_epoch
        with pytest.raises(ContractError, match="STALE_CONTEXT"):
            worker.query_command(ctx, "missing").result(5)
        # Captures from the old epoch are filtered before the main thread sees them.
        time.sleep(0.2)
        for capture in worker.captures():
            assert capture.context == new_context
    finally:
        worker.close("TEST_DONE", 5.0)


def test_planner_exception_reaches_caller_and_worker_stays_healthy():
    worker = new_worker(realtime_speed=20.0)
    try:
        ctx = context_for(worker)
        lease = worker.acquire(ctx, "test", 10_000_000_000).result(5)
        target = list(worker.profile.initial_positions)
        target[0] = 0.2
        request = motion_request(worker, ctx, lease, target, duration_ns=100_000_000)
        with pytest.raises(ContractError):
            worker.submit_native(request).result(10)
        assert worker.fault is None
        assert worker.latest_status() is None
    finally:
        summary = worker.close("TEST_DONE", 5.0)
    assert summary.closed


def test_render_failure_is_a_fault_and_stops_motion():
    worker = new_worker(realtime_speed=20.0, capture_hz=50.0)
    FakeRenderer.fail = True
    try:
        ctx = context_for(worker)
        lease = worker.acquire(ctx, "test", 10_000_000_000).result(5)
        target = list(worker.profile.initial_positions)
        target[0] = 0.10
        request = motion_request(worker, ctx, lease, target, duration_ns=2_000_000_000)
        worker.submit_native(request).result(10)
        fault = drain_until(worker, lambda item: item.fault, timeout_s=3.0)
        assert fault is not None
        assert fault.reason == "RENDER_FAILED"
        summary = worker.close("RENDER_FAULT", 5.0)
        assert summary.fault is not None
        assert summary.closed is False
    finally:
        FakeRenderer.fail = False
        worker.close("TEST_DONE", 5.0)


def test_slow_renderer_queues_stay_bounded_and_count_drops():
    worker = new_worker(
        realtime_speed=20.0,
        capture_hz=100.0,
        render_snapshot_capacity=2,
        render_output_capacity=2,
        renderer_health_timeout_s=30.0,
    )
    FakeRenderer.delay_s = 0.02
    try:
        time.sleep(0.6)
        assert worker._owner._captures.qsize() <= worker.config.capture_queue_capacity
        assert worker._renderer._snapshots.qsize() <= worker.config.render_snapshot_capacity
        assert worker._renderer._outputs.qsize() <= worker.config.render_output_capacity
        summary = worker.close("TEST_DONE", 5.0)
        assert summary.closed
        assert summary.dropped_snapshots + summary.dropped_outputs > 0
    finally:
        FakeRenderer.delay_s = 0.0
        worker.close("TEST_DONE", 5.0)


def test_close_resolves_queued_and_planner_futures():
    worker = new_worker(realtime_speed=20.0)
    try:
        queued = worker._owner.enqueue("lookup", (worker.current_context, "never"))
        assert isinstance(queued, object)
    finally:
        worker.close("TEST_DONE", 5.0)
    assert queued.done()
    with pytest.raises(BackendClosed):
        worker.acquire(worker.current_context, "test", 1_000_000_000)
    late_submit = worker.submit_native(
        NativeRequest(
            Trajectory(
                worker.current_context,
                "lease",
                "late",
                1,
                1,
                0,
                tuple(worker.profile.joint_names),
                (Target(1_000_000_000, tuple(worker.profile.initial_positions)),),
            ),
            NATIVE_MOTION,
        )
    )
    with pytest.raises(BackendClosed):
        late_submit.result(2)


def test_logger_failure_latches_a_nonzero_fault():
    class BrokenLogger:
        def emit(self, event, **kwargs):
            raise RuntimeError("injected logger failure")

        def check_health(self):
            raise RuntimeError("injected logger failure")

    worker = NativeWorker(
        config_for(realtime_speed=20.0),
        log=BrokenLogger(),
        renderer_factory=FakeRenderer,
    )
    worker.start()
    try:
        fault = drain_until(worker, lambda item: item.fault, timeout_s=2.0)
        assert fault is not None
        assert fault.reason == "LOGGER_FAILURE"
        summary = worker.close("LOGGER_FAULT", 5.0)
        assert summary.closed is False
    finally:
        worker.close("TEST_DONE", 5.0)


def test_stream_first_next_and_retries_use_chunk_history():
    from robot_contracts.native import NATIVE_STREAM_FIRST, NATIVE_STREAM_NEXT

    worker = new_worker()
    try:
        ctx = context_for(worker)
        lease = worker.acquire(ctx, "stream-test", 9_000_000_000).result(3)
        captures = drain_until(worker, lambda item: item.captures() or None)
        capture = captures[-1]
        anchor = worker.get_anchor(ctx, lease, None).result(3)
        targets = tuple(
            Target((index + 1) * 50_000_000, worker.profile.initial_positions)
            for index in range(60)
        )

        def request(current, kind, observation_id, final):
            return NativeRequest(
                Trajectory(
                    ctx,
                    lease,
                    "stream-1",
                    1,
                    observation_id,
                    current.start_sim_time_ns - 10_000_000,
                    worker.profile.joint_names,
                    targets,
                ),
                kind,
                anchor=current,
                observation_time_ns=capture.sim_time_ns,
                target_count=20,
                settle_at_end=final,
            )

        first = request(anchor, NATIVE_STREAM_FIRST, 1, False)
        outcome = worker.submit_native(first).result(3)
        assert outcome.reply.accepted
        next_anchor = worker.get_anchor(ctx, lease, None).result(3)
        second = request(next_anchor, NATIVE_STREAM_NEXT, 2, True)
        outcome2 = worker.submit_native(second).result(3)
        assert outcome2.reply.accepted
        for original, accepted in ((first, outcome), (second, outcome2)):
            retried = worker.submit_native(original).result(3)
            assert retried.idempotent and retried.reference == accepted.reference
        changed = replace(second, trajectory=replace(second.trajectory, observation_id=3))
        with pytest.raises(ContractError, match="COMMAND_ID_CONFLICT"):
            worker.submit_native(changed).result(3)
        worker.release(ctx, lease).result(3)
        with pytest.raises(ContractError, match="LEASE_EXPIRED"):
            worker.submit_native(second).result(3)
    finally:
        summary = worker.close("TEST_DONE", 5.0)
    assert summary.phase == "STOPPED"


def test_initial_seed_is_applied_and_idle_close_is_reported():
    from robot_backend_sim.rendering import apply_snapshot
    from robot_backend_sim.scene import build_scene

    worker = new_worker(seed=711)
    try:
        capture = drain_until(worker, lambda item: item.captures() or None)[-1]
        expected = build_scene(
            worker.scene_config,
        )
        expected.reset(711)
        clone = mujoco.MjData(expected.model)
        apply_snapshot(expected.model, clone, capture.snapshot)
        for address in expected.indices.object_free_joint_qpos.values():
            np.testing.assert_allclose(
                clone.qpos[address : address + 2],
                expected.data.qpos[address : address + 2],
                atol=1e-12,
            )
    finally:
        summary = worker.close("TEST_DONE", 5.0)
    assert summary.phase == "IDLE"


def test_running_owner_call_cannot_be_cancelled(monkeypatch):
    import threading

    from robot_backend_sim.worker_owner import _Owner

    entered = threading.Event()
    release = threading.Event()
    original = _Owner._dispatch
    dispatched = []

    def dispatch(self, controller, name, payload):
        if name == "identity":
            dispatched.append(name)
            entered.set()
            assert release.wait(3)
        return original(self, controller, name, payload)

    monkeypatch.setattr(_Owner, "_dispatch", dispatch)
    worker = new_worker()
    try:
        running = worker.request("identity")
        assert entered.wait(2)
        assert running.running() and not running.cancel()
        queued = worker.request("identity")
        assert queued.cancel()
        release.set()
        assert running.result(2) == worker.current_context
        time.sleep(0.03)
        assert dispatched == ["identity"]
    finally:
        release.set()
        worker.close("TEST_DONE", 5.0)


def test_stack_task_prepare_start_and_teacher_chunk_are_bounded():
    worker = new_stack_worker()
    try:
        context = context_for(worker, episode="stack-episode")
        lease = worker.acquire(context, "stack-test", 5_000_000_000).result(timeout=5)
        with pytest.raises(ContractError, match="SEED_MISMATCH"):
            worker.prepare_stack_task(
                context, "stack-episode", "bad-seed", lease, 7
            ).result(timeout=5)
        wrong_context = replace(context, clock_epoch=context.clock_epoch + 1)
        with pytest.raises(ContractError):
            worker.reset(wrong_context, lease, 0).result(timeout=5)
        prepared = worker.prepare_stack_task(
            context, "stack-episode", "prepare-1", lease, 0
        ).result(timeout=30)
        assert prepared.task_epoch >= 1
        assert prepared.key_pose_count == 6
        assert prepared.collision_segment_count >= 1
        assert prepared.planned_duration_s < 30.0
        assert prepared.prepare_kind == "teacher"
        # A rejected reset must not clear the prepared plan or its epoch.
        with pytest.raises(ContractError):
            worker.reset(wrong_context, lease, 0).result(timeout=5)
        worker.captures()  # discard pre-start captures retained by the test
        state, idempotent = worker.start_stack_task(
            context,
            "stack-episode",
            "start-1",
            lease,
            prepared.task_epoch,
            prepared.plan_digest,
        ).result(timeout=5)
        assert not idempotent
        assert state.state == "RUNNING"
        started_at = state.task_start_sim_time_ns

        capture = drain_until(
            worker,
            lambda item: next(
                (
                    value
                    for value in item.captures()
                    if value.sim_time_ns >= started_at
                ),
                None,
            ),
            timeout_s=5.0,
        )
        assert capture is not None
        chunk = worker.get_teacher_chunk(
            context,
            "stack-episode",
            state.task_epoch,
            "chunk-1",
            capture.capture_seq,
            capture.sim_time_ns,
        ).result(timeout=15)
        assert len(chunk.positions) == 60
        assert all(len(position) == 16 for position in chunk.positions)
        with pytest.raises(ContractError, match="TEACHER_FUTURE_OBSERVATION"):
            worker.get_teacher_chunk(
                context,
                "stack-episode",
                state.task_epoch,
                "chunk-future",
                capture.capture_seq,
                capture.sim_time_ns + 10_000_000_000,
            ).result(timeout=15)
        with pytest.raises(ContractError, match="TEACHER_OBSERVATION_MISMATCH"):
            worker.get_teacher_chunk(
                context,
                "stack-episode",
                state.task_epoch,
                "chunk-wrong-time",
                capture.capture_seq,
                capture.sim_time_ns - 1,
            ).result(timeout=15)
        with pytest.raises(ContractError, match="CAPTURE_NOT_RETAINED"):
            worker.get_teacher_chunk(
                context,
                "stack-episode",
                state.task_epoch,
                "chunk-missing",
                capture.capture_seq + 10_000,
                capture.sim_time_ns,
            ).result(timeout=15)
        retried = worker.get_teacher_chunk(
            context,
            "stack-episode",
            state.task_epoch,
            "chunk-1",
            capture.capture_seq,
            capture.sim_time_ns,
        ).result(timeout=15)
        assert retried.chunk_digest == chunk.chunk_digest
        with pytest.raises(ContractError, match="REQUEST_ID_CONFLICT"):
            worker.get_teacher_chunk(
                context,
                "different-episode",
                state.task_epoch,
                "chunk-1",
                capture.capture_seq,
                capture.sim_time_ns,
            ).result(timeout=15)
        queried = worker.get_stack_task_state(context, state.task_epoch).result(timeout=5)
        assert queried.state == "RUNNING"
        with pytest.raises(ContractError, match="TASK_BUSY"):
            worker.reset(context, lease, 1).result(timeout=5)
        worker.stop(context, lease, "TEST_STOP").result(timeout=5)
        stopped = worker.get_stack_task_state(context, state.task_epoch).result(timeout=5)
        assert stopped.state == "FAILED"
        assert "CONTROL_STOPPED" in stopped.reason
        new_context = worker.reset(context, lease, 1).result(timeout=5)
        assert new_context is not None
        after = worker.get_stack_task_state(context, 0).result(timeout=5)
        assert after.state == "IDLE"
    finally:
        worker.close("stack-test", 10.0)


def test_inference_mode_does_not_expose_teacher_chunks():
    worker = new_stack_worker(mode="inference")
    try:
        assert worker.task_enabled
        assert not worker.teacher_enabled
        context = context_for(worker, episode="inference-episode")
        lease = worker.acquire(context, "inference-test", 5_000_000_000).result(timeout=5)
        prepared = worker.prepare_stack_task(
            context, "inference-episode", "prepare-1", lease, 0
        ).result(timeout=10)
        assert prepared.prepare_kind == "evaluation_only"
        assert prepared.key_pose_count == 0
        state, _ = worker.start_stack_task(
            context, "inference-episode", "start-1", lease,
            prepared.task_epoch, prepared.plan_digest,
        ).result(timeout=5)
        assert state.state == "RUNNING"
        with pytest.raises(ContractError, match="TEACHER_DISABLED"):
            worker.get_teacher_chunk(
                context, "inference-episode", state.task_epoch, "chunk-1", 1, 0
            ).result(timeout=5)
    finally:
        worker.close("inference-test", 10.0)


def test_prepare_inflight_reset_rejects_the_old_commit(monkeypatch):
    worker = new_stack_worker()
    try:
        context = context_for(worker, episode="race-episode")
        lease = worker.acquire(context, "race-test", 5_000_000_000).result(timeout=5)
        original = worker_module.build_teacher_plan

        def slow_plan(*args, **kwargs):
            time.sleep(0.3)
            return original(*args, **kwargs)

        monkeypatch.setattr(worker_module, "build_teacher_plan", slow_plan)
        future = worker.prepare_stack_task(
            context, "race-episode", "race-prepare", lease, 0
        )
        time.sleep(0.1)
        worker.reset(context, lease, 1).result(timeout=5)
        with pytest.raises(ContractError, match="LEASE_EXPIRED|STALE_TASK_PREPARE"):
            future.result(timeout=10)
    finally:
        worker.close("race-test", 10.0)
