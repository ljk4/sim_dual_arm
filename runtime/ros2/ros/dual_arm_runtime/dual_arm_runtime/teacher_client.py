"""Runtime-side adapter for a scenario Teacher extension.

The core :class:`TaskClient` owns only the optional ``sim_api_msgs`` task
lifecycle.  This module adds the scenario-specific Teacher chunk client.  The
generated extension type is imported only when teacher mode creates this
adapter, so ordinary policy/control never loads ``panda_stack_api_msgs``.

Selection is by explicit capability identifier, never by scenario id.  The
identifier is validated against the extension's own generated
``TeacherCapability.ID``; an old backend advertising the bare ``teacher_chunk``
capability is rejected by the episode capability check.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

from robot_contracts.models import ContractError, RunContext
from robot_ros_support.conversions import context_message, stamp, time_ns

from dual_arm_runtime.run_logging import EventSink
from dual_arm_runtime.task_client import TaskClient

__all__ = ["TeacherChunkFact", "TeacherTaskClient"]


@dataclass(frozen=True, slots=True)
class TeacherChunkFact:
    task_epoch: int
    capture_seq: int
    plan_digest: str
    chunk_digest: str
    observation_time_ns: int
    source_time_ns: int
    robot_profile_hash: str
    phase: str
    times_from_observation_ns: tuple[int, ...]
    positions: tuple[tuple[float, ...], ...]
    phase_codes: tuple[int, ...]
    gate_state: str
    gate_failure: str
    phase_start_sim_time_ns: int
    phase_wait_s: float
    ik_max_position_error_m: float
    ik_max_orientation_error_rad: float


def teacher_service_type(capability: str):
    """Resolve the generated extension service only for the runtime adapter."""
    try:
        from panda_stack_api_msgs.msg import TeacherCapability
        from panda_stack_api_msgs.srv import GetTeacherChunk
    except ImportError as error:
        raise ContractError(
            "TEACHER_EXTENSION_MISSING",
            f"teacher capability {capability!r} requires the built "
            f"panda_stack_api_msgs extension: {error}",
        ) from error
    if capability != str(TeacherCapability.ID):
        raise ContractError(
            "UNSUPPORTED_TEACHER_CAPABILITY",
            f"runtime selected {capability!r}, extension implements "
            f"{str(TeacherCapability.ID)!r}",
        )
    return GetTeacherChunk


class TeacherTaskClient:
    """Add one bounded Teacher chunk client to the task lifecycle client."""

    def __init__(
        self,
        task: TaskClient,
        node: object,
        root: str,
        *,
        capability: str,
        chunk_timeout_s: float,
        call_timeout_s: float,
        poll_interval_s: float,
        log: EventSink | None = None,
        stop_requested: threading.Event | None = None,
    ) -> None:
        from rclpy.callback_groups import ReentrantCallbackGroup

        self.task = task
        self.capability = capability
        self._chunk_timeout_s = chunk_timeout_s
        self._call_timeout_s = call_timeout_s
        self._poll_interval_s = poll_interval_s
        self._log = log
        self._stop = stop_requested or threading.Event()
        self._client = node.create_client(  # type: ignore[attr-defined]
            teacher_service_type(capability),
            f"{root.rstrip('/')}/get_task_chunk",
            callback_group=ReentrantCallbackGroup(),
        )

    def wait_ready(self) -> None:
        self.task.wait_ready()
        if not self._client.wait_for_service(timeout_sec=self._call_timeout_s):
            raise ContractError("NOT_READY", "teacher chunk service")

    def prepare(self, *args: object, **kwargs: object) -> object:
        return self.task.prepare(*args, **kwargs)

    def start(self, *args: object, **kwargs: object) -> int:
        return self.task.start(*args, **kwargs)

    def state(self, *args: object, **kwargs: object) -> object:
        return self.task.state(*args, **kwargs)

    @property
    def frozen_state(self) -> object | None:
        return self.task.frozen_state

    def chunk(
        self, context: RunContext, episode_id: str, task_epoch: int, request_id: str,
        capture_seq: int, observation_time_ns: int, *, robot_profile_hash: str,
        chunk_size: int, period_ns: int,
    ) -> TeacherChunkFact:
        request_type = teacher_service_type(self.capability)
        response = self.task.invoke(
            self._client,
            request_type.Request(
                context=context_message(context), episode_id=episode_id,
                task_epoch=task_epoch, request_id=request_id, capture_seq=capture_seq,
                observation_time=stamp(observation_time_ns),
            ),
            name="chunk",
            timeout_s=self._chunk_timeout_s,
        )
        if int(response.task_epoch) != task_epoch:
            raise ContractError("TEACHER_CONTEXT_MISMATCH", "teacher task epoch changed")
        if int(response.capture_seq) != capture_seq:
            raise ContractError("TEACHER_CAPTURE_MISMATCH", "teacher capture differs")
        if time_ns(response.observation_time) != observation_time_ns:
            raise ContractError("TEACHER_FEEDBACK_MISMATCH", "teacher observation time differs")
        if str(response.robot_profile_hash) != robot_profile_hash:
            raise ContractError("TEACHER_PROFILE_MISMATCH", "teacher profile differs")
        if int(response.point_count) != chunk_size or int(response.joint_count) != 16:
            raise ContractError("TEACHER_CHUNK_SHAPE", "teacher chunk is not 60x16")
        flat = tuple(float(value) for value in response.positions)
        if len(flat) != chunk_size * 16:
            raise ContractError("TEACHER_CHUNK_SHAPE", "teacher positions are not 960-wide")
        times = tuple(int(value) for value in response.time_from_observation_ns)
        if len(times) != chunk_size:
            raise ContractError("TEACHER_CHUNK_CLOCK", "teacher time grid is incomplete")
        for index, value in enumerate(times):
            if value != (index + 1) * period_ns:
                raise ContractError(
                    "TEACHER_CHUNK_CLOCK", "teacher time grid is not a uniform 50 ms"
                )
        positions = tuple(flat[point * 16 : point * 16 + 16] for point in range(chunk_size))
        phase_codes = tuple(int(value) for value in response.phase_codes)
        if len(phase_codes) != chunk_size:
            raise ContractError("TEACHER_CHUNK_SHAPE", "teacher phase codes are incomplete")
        return TeacherChunkFact(
            int(response.task_epoch), int(response.capture_seq), str(response.plan_digest),
            str(response.chunk_digest), time_ns(response.observation_time),
            time_ns(response.source_time), str(response.robot_profile_hash),
            str(response.phase), times, positions, phase_codes, str(response.gate_state),
            str(response.gate_failure), time_ns(response.phase_start_time),
            float(response.phase_wait_s), float(response.ik_max_position_error_m),
            float(response.ik_max_orientation_error_rad),
        )
