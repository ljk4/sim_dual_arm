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
        self._observation: ObservationRef | None = None
        self.dropped_samples = 0

    def set_context(self, context: RunContext) -> None:
        with self._lock:
            if context != self._context:
                self._context = context
                for stream in self._streams.values():
                    stream.clear()
                self._observation_id = 0
                self._observation = None

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

            if self._observation is not None and self._observation.state == state:
                return self._copy(self._observation)

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

            self._observation_id += 1
            self._observation = ObservationRef(self._context, self._observation_id, state, frames)
            return self._copy(self._observation)

    @staticmethod
    def _copy(observation: ObservationRef) -> ObservationRef:
        return ObservationRef(
            observation.context,
            observation.observation_id,
            observation.state,
            dict(observation.frames),
        )


@dataclass(frozen=True)
class CapturedPayload:
    """One immutable message payload retained with its exact capture identity."""

    context: RunContext
    sequence: int
    stamp_ns: int
    wall_arrival_ns: int
    payload: object
    fingerprint: object | None = None


@dataclass(frozen=True)
class ExactObservation:
    """A stable capture reference plus the real state/frame payloads it names.

    The observation ID is allocated once per complete capture identity.  Repeated
    reads return this same value, including the same payload objects, so a caller
    can never select an index and then read an overwritten cache entry.
    """

    context: RunContext
    observation_id: int
    state: CapturedPayload
    frames: dict[str, CapturedPayload]

    @property
    def state_seq(self) -> int:
        return self.state.sequence

    @property
    def stamp_ns(self) -> int:
        return self.state.stamp_ns


@dataclass(frozen=True)
class ExactCaptureStats:
    received: int
    accepted: int
    duplicates: int
    conflicts: int
    evicted: int
    rejected_context: int
    published: int
    dropped_old: int


class ExactCaptureBuffer:
    """Bounded exact-capture pairing for async state and three camera streams.

    A complete capture requires one state payload and every configured camera
    payload with the same ``context``, the same sequence and exactly the same
    stamp.  Messages of one stream are accepted out of order and duplicates are
    idempotent; a same-sequence message with a different identity is a latched
    conflict.  Streams and the published-capture map are both bounded.
    """

    def __init__(
        self,
        cameras: tuple[str, ...],
        *,
        max_age_ns: int,
        wall_arrival_timeout_ns: int,
        capacity: int = 32,
    ) -> None:
        if len(set(cameras)) != len(cameras) or not cameras:
            raise ContractError("CONFIGURATION_ERROR", "exact capture needs unique cameras")
        for name, value in (
            ("max_age_ns", max_age_ns),
            ("wall_arrival_timeout_ns", wall_arrival_timeout_ns),
            ("capacity", capacity),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ContractError("CONFIGURATION_ERROR", f"{name} must be a positive integer")
        self._cameras = tuple(cameras)
        self._max_age = max_age_ns
        self._wall_timeout = wall_arrival_timeout_ns
        self._capacity = capacity
        self._streams: dict[str, list[CapturedPayload]] = {
            name: [] for name in ("state", *self._cameras)
        }
        self._context: RunContext | None = None
        self._observation_id = 0
        self._last_key: tuple[int, int] | None = None
        self._last_observation: ExactObservation | None = None
        self._lock = threading.Lock()
        self.received = 0
        self.accepted = 0
        self.duplicates = 0
        self.conflicts = 0
        self.evicted = 0
        self.rejected_context = 0
        self.published = 0
        self.dropped_old = 0
        self.conflict_reason = ""

    def set_context(self, context: RunContext) -> None:
        """Start a new identity window; old context payloads are dropped."""
        with self._lock:
            if context == self._context:
                return
            self._context = context
            for stream in self._streams.values():
                stream.clear()
            self._observation_id = 0
            self._last_key = None
            self._last_observation = None

    def add(
        self,
        stream: str,
        context: RunContext,
        sequence: int,
        stamp_ns: int,
        payload: object,
        *,
        wall_arrival_ns: int,
        fingerprint: object | None = None,
    ) -> bool:
        """Accept one payload; return False for a duplicate, conflict or rejection.

        ``fingerprint`` lets a transport compare the real serialized identity of a
        same-sequence message.  Without one, equal sequence and stamp count as the
        same sample and never create a second observation.
        """
        with self._lock:
            if stream not in self._streams:
                raise ContractError("CONFIGURATION_ERROR", f"unknown capture stream: {stream}")
            self.received += 1
            if context != self._context:
                self.rejected_context += 1
                return False
            if (
                isinstance(sequence, bool)
                or not isinstance(sequence, int)
                or sequence < 1
                or isinstance(stamp_ns, bool)
                or not isinstance(stamp_ns, int)
                or stamp_ns < 0
                or isinstance(wall_arrival_ns, bool)
                or not isinstance(wall_arrival_ns, int)
                or wall_arrival_ns < 0
            ):
                self._conflict(f"{stream}: invalid sequence/stamp/arrival")
                return False
            samples = self._streams[stream]
            # Identity lives with the real bounded entry, not in a second,
            # shorter global queue that could forget live payloads.
            for existing in samples:
                if existing.sequence != sequence:
                    continue
                if (existing.stamp_ns, existing.fingerprint) != (stamp_ns, fingerprint):
                    self._conflict(f"{stream}: same sequence {sequence} has different identity")
                    return False
                self.duplicates += 1
                return False
            # Insert in sequence order first.  When the cache is full, an
            # arrival older than every retained entry is explicitly dropped and
            # must never evict a newer, still useful sample.
            if samples and len(samples) >= self._capacity and sequence < samples[0].sequence:
                self.dropped_old += 1
                return False
            entry = CapturedPayload(
                context, sequence, stamp_ns, wall_arrival_ns, payload, fingerprint
            )
            index = len(samples)
            for position, existing in enumerate(samples):
                if existing.sequence > sequence:
                    index = position
                    break
            samples.insert(index, entry)
            if len(samples) > self._capacity:
                samples.pop(0)
                self.evicted += 1
            self.accepted += 1
            return True

    def latest_state_stamp(self) -> int | None:
        """Return the newest state stamp, for a simulation-clock freshness base."""
        with self._lock:
            states = self._streams["state"]
            return None if not states else states[-1].stamp_ns

    def assemble(self, now_ns: int, wall_now_ns: int) -> ExactObservation:
        """Return the newest fresh complete capture, or raise ``NOT_READY``."""
        with self._lock:
            if self._context is None:
                raise ContractError("NOT_READY", "No exact capture context")
            states = self._streams["state"]
            if not states:
                raise ContractError("NOT_READY", "No state in current capture context")
            newest_state = states[-1].stamp_ns
            saw_complete = False
            saw_wall_stale = False
            saw_rollback = False
            last = self._last_observation
            for state in reversed(states):
                if last is not None and (
                    state.sequence < last.state.sequence or state.stamp_ns < last.state.stamp_ns
                ):
                    # Once a capture has been published, a late older complete
                    # group must never receive a new observation ID.
                    saw_rollback = True
                    continue
                if (
                    wall_now_ns - state.wall_arrival_ns < 0
                    or wall_now_ns - state.wall_arrival_ns > self._wall_timeout
                ):
                    saw_wall_stale = True
                    continue
                if now_ns - state.stamp_ns < 0 or now_ns - state.stamp_ns > self._max_age:
                    continue
                frames = self._matching_frames(state)
                if frames is None:
                    continue
                components = (state, *frames.values())
                if any(
                    wall_now_ns - item.wall_arrival_ns < 0
                    or wall_now_ns - item.wall_arrival_ns > self._wall_timeout
                    for item in components
                ):
                    saw_wall_stale = True
                    continue
                saw_complete = True
                key = (state.sequence, state.stamp_ns)
                if last is not None and key == self._last_key:
                    return self._copy(last)
                self._observation_id += 1
                observation = ExactObservation(
                    self._context, self._observation_id, state, dict(frames)
                )
                self._last_key = key
                self._last_observation = observation
                self.published += 1
                return self._copy(observation)
            if saw_complete or saw_wall_stale or saw_rollback:
                raise ContractError(
                    "STALE_OBSERVATION",
                    "Exact capture exceeded freshness or would roll back",
                )
            if now_ns - newest_state > self._max_age:
                raise ContractError("STALE_OBSERVATION", "Newest state exceeded max_age")
            raise ContractError("NOT_READY", "No complete state/camera capture")

    def _matching_frames(self, state: CapturedPayload) -> dict[str, CapturedPayload] | None:
        frames: dict[str, CapturedPayload] = {}
        for camera in self._cameras:
            samples = self._streams[camera]
            found = None
            for sample in reversed(samples):
                if sample.sequence == state.sequence and sample.stamp_ns == state.stamp_ns:
                    found = sample
                    break
            if found is None:
                return None
            frames[camera] = found
        return frames

    def _conflict(self, reason: str) -> None:
        self.conflicts += 1
        if not self.conflict_reason:
            self.conflict_reason = reason

    def stats(self) -> ExactCaptureStats:
        with self._lock:
            return ExactCaptureStats(
                self.received,
                self.accepted,
                self.duplicates,
                self.conflicts,
                self.evicted,
                self.rejected_context,
                self.published,
                self.dropped_old,
            )

    @staticmethod
    def _copy(observation: ExactObservation) -> ExactObservation:
        return ExactObservation(
            observation.context,
            observation.observation_id,
            observation.state,
            dict(observation.frames),
        )
