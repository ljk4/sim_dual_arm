"""Runtime-side adapter for a scenario Teacher extension.

The core :class:`TaskClient` owns only the optional ``sim_api_msgs`` task
lifecycle.  This module adds the scenario-specific Teacher chunk client.  The
generated extension type is selected by explicit capability identifier
``<package>.teacher_chunk.v<N>`` and imported only when teacher mode creates
this adapter, so ordinary policy/control never loads a scenario extension.
The identifier is validated against the extension's own generated
``TeacherCapability.ID``; an old backend advertising a different capability is
rejected by the episode capability check.
"""

from __future__ import annotations

import importlib
import re
import threading
from dataclasses import dataclass

from robot_contracts.models import ContractError, RunContext
from robot_ros_support.conversions import context_message, stamp, time_ns

from dual_arm_runtime.run_logging import EventSink
from dual_arm_runtime.task_client import TaskClient

__all__ = ["TeacherChunkFact", "TeacherTaskClient", "teacher_service_type"]

_CAPABILITY = re.compile(r"^([a-z][a-z0-9_]*)\.teacher_chunk\.v[1-9][0-9]*$")


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
    """Resolve the generated service type named by one Teacher capability."""
    match = _CAPABILITY.fullmatch(str(capability))
    if match is None:
        raise ContractError(
            "UNSUPPORTED_TEACHER_CAPABILITY",
            f"{capability!r} is not a <package>.teacher_chunk.v<N> capability",
        )
    package = match.group(1)
    try:
        messages = importlib.import_module(f"{package}.msg")
        services = importlib.import_module(f"{package}.srv")
        capability_type = messages.TeacherCapability
        service_type = services.GetTeacherChunk
    except (ImportError, AttributeError) as error:
        raise ContractError(
            "TEACHER_EXTENSION_MISSING",
            f"teacher capability {capability!r} requires the built "
            f"{package} extension: {error}",
        ) from error
    if str(capability_type.ID) != capability:
        raise ContractError(
            "UNSUPPORTED_TEACHER_CAPABILITY",
            f"runtime selected {capability!r}, extension implements "
            f"{str(capability_type.ID)!r}",
        )
    return service_type


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
        chunk_size: int, joint_count: int, period_ns: int, handoff_time_ns: int = 0,
    ) -> TeacherChunkFact:
        request_type = teacher_service_type(self.capability)
        request = request_type.Request(
            context=context_message(context), episode_id=episode_id,
            task_epoch=task_epoch, request_id=request_id, capture_seq=capture_seq,
            observation_time=stamp(observation_time_ns),
        )
        if hasattr(request, "handoff_time"):
            request.handoff_time = stamp(
                handoff_time_ns if handoff_time_ns > 0 else observation_time_ns
            )
        response = self.task.invoke(
            self._client,
            request,
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
        if int(response.point_count) != chunk_size or int(response.joint_count) != joint_count:
            raise ContractError(
                "TEACHER_CHUNK_SHAPE",
                f"teacher chunk is not {chunk_size}x{joint_count}",
            )
        flat = tuple(float(value) for value in response.positions)
        if len(flat) != chunk_size * joint_count:
            raise ContractError(
                "TEACHER_CHUNK_SHAPE",
                f"teacher positions are not {chunk_size * joint_count}-wide",
            )
        times = tuple(int(value) for value in response.time_from_observation_ns)
        if len(times) != chunk_size:
            raise ContractError("TEACHER_CHUNK_CLOCK", "teacher time grid is incomplete")
        for index, value in enumerate(times):
            if value != (index + 1) * period_ns:
                raise ContractError(
                    "TEACHER_CHUNK_CLOCK",
                    f"teacher time grid does not follow the {period_ns} ns period",
                )
        positions = tuple(
            flat[point * joint_count : (point + 1) * joint_count]
            for point in range(chunk_size)
        )
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
