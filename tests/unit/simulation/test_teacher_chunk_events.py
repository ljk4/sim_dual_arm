"""ROS adapter event correlation for Teacher chunks.

``backend.task.teacher_chunk`` must keep the real episode and the teacher
request id as separate correlation fields.  The old code wrote the request id
into ``episode_id``.
"""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
SIM_SRC = ROOT / "simulation/mujoco/src"
CONTRACTS = ROOT / "shared/common/python/contracts/src"
RUN_LOGGING = ROOT / "shared/common/python/run_logging/src"
ROS_SUPPORT = ROOT / "shared/common/python/ros_support/src"
for source in (SIM_SRC, CONTRACTS, RUN_LOGGING, ROS_SUPPORT):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.common.ros.node_services import ServiceAdapter  # noqa: E402
from robot_backend_sim.common.tasks.base import TaskChunk  # noqa: E402


class _RecordingNode:
    def __init__(self, chunk: TaskChunk) -> None:
        self.chunk = chunk
        self.events: list[tuple[str, dict[str, object]]] = []

    async def _await(self, future: object, *, submit: bool = False) -> object:
        del submit
        return future

    def _failure_response(self, response: object, error: BaseException) -> object:
        raise AssertionError(f"unexpected adapter failure: {error!r}")

    def _emit(self, event: str, **kwargs: object) -> None:
        self.events.append((event, kwargs))


def _chunk(request_id: str) -> TaskChunk:
    return TaskChunk(
        request_id=request_id,
        task_epoch=7,
        capture_seq=11,
        observation_time_ns=2_000_000_000,
        source_time_ns=1_900_000_000,
        robot_profile_hash="profile",
        plan_digest="p" * 64,
        chunk_digest="c" * 64,
        phase="dual_hold",
        phases=("dual_hold",),
        phase_codes=(1,),
        time_from_observation_ns=(0,),
        positions=((0.0,) * 16,),
        ik_max_position_error_m=0.0,
        ik_max_orientation_error_rad=0.0,
        gate_state="HOLDING",
        gate_failure="",
        phase_start_sim_time_ns=1_500_000_000,
        phase_wait_s=0.0,
    )


def test_teacher_chunk_event_keeps_episode_and_request_ids_separate():
    request_id = "teacher-request-1"
    episode_id = "episode-real-1"
    chunk = _chunk(request_id)
    node = _RecordingNode(chunk)
    worker = SimpleNamespace(
        get_task_chunk=lambda *args: chunk,
        fill_task_chunk_response=lambda response, value: response,
    )
    adapter = ServiceAdapter(node, worker, SimpleNamespace())
    request = SimpleNamespace(
        context=SimpleNamespace(
            robot_id="robot",
            robot_profile_hash="profile",
            backend_instance_id="backend",
            time_domain_id="sim",
            clock_epoch=0,
            episode_id=episode_id,
            control_epoch=0,
        ),
        episode_id=episode_id,
        task_epoch=7,
        request_id=request_id,
        capture_seq=11,
        observation_time=SimpleNamespace(sec=2, nanosec=0),
    )
    response = SimpleNamespace(success=False, reason="not-set", state=None)

    asyncio.run(adapter.get_task_chunk(request, response))

    assert response.success is True
    event, kwargs = node.events[0]
    assert event == "backend.task.teacher_chunk"
    context = kwargs["context"]
    assert context.episode_id == episode_id
    assert context.request_id == request_id
    assert context.episode_id != context.request_id
    fields = kwargs["fields"]
    assert fields["task_epoch"] == 7
    assert fields["capture_seq"] == 11
