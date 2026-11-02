"""Immutable observation-anchored action windows and chunk hand-off anchors.

This module intentionally uses only the Python standard library.  It maps
policy predictions expressed relative to one observation onto a window that
starts at a pre-arranged hand-off time, and carries the reference state that a
later physical owner will use to continue the curve.  It does not admit late
predictions, read a clock/scene, or execute anything.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from numbers import Integral, Real

from .models import ContractError, Target
from .reference import MAX_HORIZON_NS, MAX_JOINTS, MAX_SEGMENTS, ReferenceSample

UINT64_MAX = 2**64 - 1
INT64_MAX = 2**63 - 1
MAX_TARGET_COUNT = 128
MAX_STREAM_ID_LENGTH = 128


def _invalid_window(location: str, message: str) -> ContractError:
    return ContractError("INVALID_ACTION_WINDOW", f"{location}: {message}")


def _invalid_anchor(location: str, message: str) -> ContractError:
    return ContractError("INVALID_CHUNK_ANCHOR", f"{location}: {message}")


def _integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid_window(location, "must be an integer")
    result = int(value)
    if result < minimum or result > maximum:
        raise _invalid_window(location, f"must be in {minimum}..{maximum}")
    return result


def _anchor_integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid_anchor(location, "must be an integer")
    result = int(value)
    if result < minimum or result > maximum:
        raise _invalid_anchor(location, f"must be in {minimum}..{maximum}")
    return result


def _finite_vector(value: object, location: str) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise _invalid_window(location, "must be an array of real numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid_window(location, "must be an array of real numbers") from error
    if not 1 <= len(items) <= MAX_JOINTS:
        raise _invalid_window(location, f"must contain 1..{MAX_JOINTS} values")
    values: list[float] = []
    for index, item in enumerate(items):
        if isinstance(item, bool) or not isinstance(item, Real):
            raise _invalid_window(f"{location}[{index}]", "must be a finite real number")
        try:
            converted = float(item)
        except (TypeError, ValueError, OverflowError) as error:
            raise _invalid_window(
                f"{location}[{index}]", "cannot be represented as a finite float"
            ) from error
        if not math.isfinite(converted):
            raise _invalid_window(f"{location}[{index}]", "must be finite")
        values.append(converted)
    return tuple(values)


@dataclass(frozen=True)
class ActionWindow:
    """A pre-arranged window of original predictions relative to one hand-off.

    ``source_targets`` are retained in full and use times relative to
    ``observation_time_ns`` (the same convention as policy predictions).
    ``targets`` selects the first ``target_count`` points whose absolute time is
    *strictly* greater than ``handoff_time_ns`` and rewrites only their times to
    be relative to that hand-off.  Positions are never modified.
    """

    observation_id: int
    observation_time_ns: int
    handoff_time_ns: int
    source_targets: tuple[Target, ...]
    target_count: int
    _selected_indices: tuple[int, ...] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        observation_id = _integer(self.observation_id, "observation_id", 1, UINT64_MAX)
        observation_time_ns = _integer(
            self.observation_time_ns, "observation_time_ns", 0, INT64_MAX
        )
        handoff_time_ns = _integer(self.handoff_time_ns, "handoff_time_ns", 0, INT64_MAX)
        target_count = _integer(self.target_count, "target_count", 2, MAX_TARGET_COUNT)
        if handoff_time_ns < observation_time_ns:
            raise _invalid_window(
                "handoff_time_ns",
                "must not be earlier than observation_time_ns",
            )

        if isinstance(self.source_targets, str | bytes):
            raise _invalid_window("source_targets", "must be an array of Target values")
        try:
            source_items = tuple(self.source_targets)
        except TypeError as error:
            raise _invalid_window("source_targets", "must be an array of Target values") from error
        if not 1 <= len(source_items) <= MAX_SEGMENTS:
            raise _invalid_window("source_targets", f"must contain 1..{MAX_SEGMENTS} targets")

        normalized: list[Target] = []
        expected_joints: int | None = None
        previous_relative_ns = 0
        for index, target in enumerate(source_items):
            location = f"source_targets[{index}]"
            if not isinstance(target, Target):
                raise _invalid_window(location, "must be a Target")
            relative_ns = _integer(target.time_ns, f"{location}.time_ns", 1, MAX_HORIZON_NS)
            if relative_ns <= previous_relative_ns:
                raise _invalid_window(
                    f"{location}.time_ns",
                    "must be strictly increasing and after relative zero",
                )
            if observation_time_ns + relative_ns > INT64_MAX:
                raise _invalid_window(
                    f"{location}.time_ns",
                    "absolute observation time plus relative time exceeds int64",
                )
            positions = _finite_vector(target.positions, f"{location}.positions")
            if expected_joints is None:
                expected_joints = len(positions)
            elif len(positions) != expected_joints:
                raise _invalid_window(
                    f"{location}.positions",
                    f"has {len(positions)} values for {expected_joints} joints",
                )
            normalized.append(Target(relative_ns, positions))
            previous_relative_ns = relative_ns

        absolute_times = tuple(observation_time_ns + target.time_ns for target in normalized)
        selected = tuple(
            index
            for index, absolute_ns in enumerate(absolute_times)
            if absolute_ns > handoff_time_ns
        )[:target_count]
        if not selected:
            raise ContractError(
                "CHUNK_EXPIRED",
                "source_targets: every prediction is at or before handoff_time_ns",
            )
        if len(selected) < target_count:
            raise ContractError(
                "INSUFFICIENT_HORIZON",
                "source_targets: only "
                f"{len(selected)} predictions are strictly after handoff_time_ns, "
                f"target_count is {target_count}",
            )

        object.__setattr__(self, "observation_id", observation_id)
        object.__setattr__(self, "observation_time_ns", observation_time_ns)
        object.__setattr__(self, "handoff_time_ns", handoff_time_ns)
        object.__setattr__(self, "source_targets", tuple(normalized))
        object.__setattr__(self, "target_count", target_count)
        object.__setattr__(self, "_selected_indices", selected)

    @property
    def selected_indices(self) -> tuple[int, ...]:
        """Indices into the retained full prediction for the executed window."""
        return self._selected_indices

    @property
    def skipped_overlap_count(self) -> int:
        """Number of original points at or before the hand-off (not executed)."""
        return self._selected_indices[0]

    @property
    def skipped_overlap_indices(self) -> tuple[int, ...]:
        return tuple(range(self._selected_indices[0]))

    @property
    def selected_source_targets(self) -> tuple[Target, ...]:
        """Selected original predictions, still relative to the observation."""
        return tuple(self.source_targets[index] for index in self._selected_indices)

    @property
    def targets(self) -> tuple[Target, ...]:
        """Selected targets with positive times rewritten relative to hand-off."""
        return tuple(
            Target(
                self.observation_time_ns
                + self.source_targets[index].time_ns
                - self.handoff_time_ns,
                self.source_targets[index].positions,
            )
            for index in self._selected_indices
        )

    @property
    def end_time_ns(self) -> int:
        """Time of the last selected target relative to the hand-off."""
        return (
            self.observation_time_ns
            + self.source_targets[self._selected_indices[-1]].time_ns
            - self.handoff_time_ns
        )

    @property
    def absolute_times_ns(self) -> tuple[int, ...]:
        """Original absolute robot times for every retained source prediction."""
        return tuple(self.observation_time_ns + target.time_ns for target in self.source_targets)


@dataclass(frozen=True)
class ChunkAnchor:
    """Reference state at a scheduled hand-off; no identity or scene is read.

    ``reference`` positions/velocities/accelerations are control metadata for
    matching successive chunks, not new policy features.  Consecutive chunks
    match position and velocity; acceleration continuity is not claimed.
    """

    stream_id: str
    predecessor_seq: int
    start_sim_time_ns: int
    reference: ReferenceSample
    robot_profile_hash: str

    def __post_init__(self) -> None:
        if (
            not isinstance(self.stream_id, str)
            or not self.stream_id
            or len(self.stream_id) > MAX_STREAM_ID_LENGTH
        ):
            raise _invalid_anchor(
                "stream_id",
                f"must be a non-empty string of at most {MAX_STREAM_ID_LENGTH} characters",
            )
        predecessor_seq = _anchor_integer(
            self.predecessor_seq, "predecessor_seq", 0, UINT64_MAX - 1
        )
        start_sim_time_ns = _anchor_integer(
            self.start_sim_time_ns, "start_sim_time_ns", 0, INT64_MAX
        )
        if not isinstance(self.reference, ReferenceSample):
            raise _invalid_anchor("reference", "must be a ReferenceSample")
        if not isinstance(self.robot_profile_hash, str) or not self.robot_profile_hash:
            raise _invalid_anchor("robot_profile_hash", "must be a non-empty string")
        object.__setattr__(self, "predecessor_seq", predecessor_seq)
        object.__setattr__(self, "start_sim_time_ns", start_sim_time_ns)
        object.__setattr__(
            self,
            "reference",
            ReferenceSample(
                self.reference.positions,
                self.reference.velocities,
                self.reference.accelerations,
            ),
        )
        object.__setattr__(self, "robot_profile_hash", str(self.robot_profile_hash))

    @property
    def seq(self) -> int:
        """Sequence of the chunk this anchor starts, derived from predecessor."""
        return self.predecessor_seq + 1


__all__ = [
    "INT64_MAX",
    "MAX_STREAM_ID_LENGTH",
    "MAX_TARGET_COUNT",
    "UINT64_MAX",
    "ActionWindow",
    "ChunkAnchor",
]
