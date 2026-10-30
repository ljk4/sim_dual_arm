"""Bounded, causal observation selection, independent of ROS message classes."""

import threading
from collections import deque
from dataclasses import dataclass

from robot_contracts.models import ContractError, RunContext


@dataclass(frozen=True)
class SampleRef:
    sequence: int
    stamp_ns: int


@dataclass(frozen=True)
class ObservationRef:
    context: RunContext
    observation_id: int
    state: SampleRef
    frames: dict[str, SampleRef]


class ObservationBuffer:
    """Choose camera samples at or before the newest state; never future frames."""

    def __init__(self, cameras: tuple[str, ...], max_skew_ns: int, max_age_ns: int) -> None:
        self._cameras = cameras
        self._max_skew = max_skew_ns
        self._max_age = max_age_ns
        self._context: RunContext | None = None
        self._streams = {name: deque(maxlen=32) for name in ("state", *cameras)}
        self._lock = threading.Lock()
        self._observation_id = 0
        self._last_state_seq = 0
        self.dropped_samples = 0

    def set_context(self, context: RunContext) -> None:
        with self._lock:
            if context != self._context:
                self._context = context
                for stream in self._streams.values():
                    stream.clear()
                self._observation_id, self._last_state_seq = 0, 0

    def add(self, stream: str, context: RunContext, sequence: int, stamp_ns: int) -> bool:
        with self._lock:
            samples = self._streams[stream]
            if context != self._context or stamp_ns < 0 or sequence < 1:
                self.dropped_samples += 1
                return False
            if samples and (sequence <= samples[-1].sequence or stamp_ns < samples[-1].stamp_ns):
                self.dropped_samples += 1
                return False
            samples.append(SampleRef(sequence, stamp_ns))
            return True

    def assemble(self, now_ns: int) -> ObservationRef:
        with self._lock:
            states = self._streams["state"]
            if self._context is None or not states:
                raise ContractError("NOT_READY", "No state in current context")
            state = states[-1]
            if not 0 <= now_ns - state.stamp_ns <= self._max_age:
                raise ContractError("STALE_OBSERVATION")
            frames = {}
            for camera in self._cameras:
                candidates = [
                    sample
                    for sample in self._streams[camera]
                    if 0 <= state.stamp_ns - sample.stamp_ns <= self._max_skew
                ]
                if not candidates:
                    raise ContractError("NOT_READY", f"No causal frame for {camera}")
                frames[camera] = candidates[-1]
            if state.sequence != self._last_state_seq:
                self._observation_id += 1
                self._last_state_seq = state.sequence
            return ObservationRef(self._context, self._observation_id, state, frames)
