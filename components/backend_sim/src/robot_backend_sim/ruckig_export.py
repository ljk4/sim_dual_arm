"""Export a public Ruckig trajectory as explicit shared cubic segments.

Ruckig Community 0.15.3 exposes ``trajectory.profiles[section][axis]`` with
phase start values ``t``, ``p``, ``v``, ``a`` and ``j``.  Each phase is the
second-order polynomial

``p(s) = p0 + v0*s + a0/2*s^2 + j0/6*s^3``

in local seconds.  This adapter merges the per-axis phase boundaries, converts
the result to integer-nanosecond :class:`ReferencePlan` segments, and can
truncate the curve at a fixed hand-off.  At a merged boundary the position and
velocity are taken from the previous exported segment, so the stored curve is
exactly C1 even when two axis phase boundaries round to slightly different
nanoseconds.  Acceleration may step; jerk is not constrained in this round.

Only the explicit coefficient plan leaves this module.  Sampling, verification
and deviation math remain in the standard-library contracts.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robot_contracts.models import ContractError
from robot_contracts.reference import MAX_HORIZON_NS, CubicSegment, ReferencePlan

_NS_PER_S = 1_000_000_000
_MAX_PHASES = 64
_BOUNDARY_ATOL = 1e-9


def _invalid(message: str) -> ContractError:
    return ContractError("INVALID_RUCKIG_EXPORT", message)


def _round_ns(seconds: float, location: str) -> int:
    if not math.isfinite(seconds):
        raise _invalid(f"{location}: duration is not finite")
    scaled = seconds * _NS_PER_S
    if not math.isfinite(scaled) or scaled < 0.0 or scaled > MAX_HORIZON_NS + 0.5:
        raise _invalid(f"{location}: duration is outside the integer-nanosecond range")
    return int(round(scaled))


def _finite(value: object, location: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise _invalid(f"{location}: cannot be converted to float") from error
    if not math.isfinite(result):
        raise _invalid(f"{location}: must be finite")
    return result


@dataclass(frozen=True)
class _Phase:
    start_s: float
    end_s: float
    start_ns: int
    end_ns: int
    p: float
    v: float
    a: float
    j: float


@dataclass(frozen=True)
class RuckigExportedReference:
    """An exported coefficient curve plus its exact profile/truncation metadata."""

    plan: ReferencePlan
    profile_duration_ns: int
    truncated_duration_ns: int
    end_positions: tuple[float, ...]
    end_velocities: tuple[float, ...]
    end_accelerations: tuple[float, ...]


def _phase_state(piece: _Phase, local_s: float) -> tuple[float, float, float, float]:
    j = piece.j
    a = piece.a + j * local_s
    v = piece.v + piece.a * local_s + 0.5 * j * local_s * local_s
    p = (
        piece.p
        + piece.v * local_s
        + 0.5 * piece.a * local_s * local_s
        + j * local_s * local_s * local_s / 6.0
    )
    return p, v, a, j


def _phase_pieces(profile: object, axis: int) -> tuple[list[_Phase], float]:
    try:
        times = tuple(float(value) for value in profile.t)  # type: ignore[attr-defined]
        positions = tuple(float(value) for value in profile.p)  # type: ignore[attr-defined]
        velocities = tuple(float(value) for value in profile.v)  # type: ignore[attr-defined]
        accelerations = tuple(float(value) for value in profile.a)  # type: ignore[attr-defined]
        jerks = tuple(float(value) for value in profile.j)  # type: ignore[attr-defined]
    except (AttributeError, TypeError, ValueError) as error:
        raise _invalid(f"profiles[{axis}]: cannot read public phase arrays") from error
    phase_count = len(times)
    if not 1 <= phase_count <= _MAX_PHASES:
        raise _invalid(f"profiles[{axis}]: phase array length is outside 1..{_MAX_PHASES}")
    # Ruckig exposes one start value per phase and one extra final value for
    # p/v/a.  The phase count is authoritative; trailing values are ignored.
    for name, values in (
        ("p", positions),
        ("v", velocities),
        ("a", accelerations),
        ("j", jerks),
    ):
        if len(values) not in (phase_count, phase_count + 1):
            raise _invalid(
                f"profiles[{axis}].{name}: length {len(values)} does not match "
                f"{phase_count} phases",
            )
        for index, value in enumerate(values):
            if not math.isfinite(value):
                raise _invalid(f"profiles[{axis}].{name}[{index}]: must be finite")
    for index, value in enumerate(times):
        if not math.isfinite(value):
            raise _invalid(f"profiles[{axis}].t[{index}]: must be finite")

    brake = getattr(profile, "brake", None)
    if brake is not None:
        try:
            brake_duration = float(brake.duration)
            brake_times = tuple(float(value) for value in brake.t)
        except (AttributeError, TypeError, ValueError) as error:
            raise _invalid(
                f"profiles[{axis}].brake: cannot read the public brake profile",
            ) from error
        if brake_duration != 0.0 or any(value != 0.0 for value in brake_times):
            raise _invalid(
                f"profiles[{axis}].brake: nonzero brake profiles are not supported "
                "by the second-order main-profile export",
            )

    pieces: list[_Phase] = []
    cumulative_s = 0.0
    for index, duration in enumerate(times):
        if not math.isfinite(duration):
            raise _invalid(f"profiles[{axis}].t[{index}]: must be finite")
        if duration < 0.0:
            raise _invalid(f"profiles[{axis}].t[{index}]: must not be negative")
        if duration == 0.0:
            continue
        start_s = cumulative_s
        end_s = start_s + duration
        pieces.append(
            _Phase(
                start_s=start_s,
                end_s=end_s,
                start_ns=0,
                end_ns=0,
                p=positions[index],
                v=velocities[index],
                a=accelerations[index],
                j=jerks[index],
            )
        )
        cumulative_s = end_s

    for previous, current in zip(pieces[:-1], pieces[1:], strict=False):
        if not math.isclose(
            previous.end_s,
            current.start_s,
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            raise _invalid(f"profiles[{axis}]: phase times contain a gap")
        end_p, end_v, _, _ = _phase_state(previous, previous.end_s - previous.start_s)
        if not math.isclose(end_p, current.p, rel_tol=1e-9, abs_tol=1e-9):
            raise _invalid(f"profiles[{axis}]: position discontinuity at a phase boundary")
        if not math.isclose(end_v, current.v, rel_tol=1e-9, abs_tol=1e-9):
            raise _invalid(f"profiles[{axis}]: velocity discontinuity at a phase boundary")
    return pieces, cumulative_s


def _assign_ns(pieces: list[_Phase], total_ns: int, axis: int) -> list[_Phase]:
    assigned: list[_Phase] = []
    for index, piece in enumerate(pieces):
        start_ns = _round_ns(piece.start_s, f"profiles[{axis}].t[{index}].start")
        end_ns = _round_ns(piece.end_s, f"profiles[{axis}].t[{index}].end")
        start_ns = min(max(start_ns, 0), total_ns)
        end_ns = min(max(end_ns, 0), total_ns)
        if assigned:
            start_ns = max(start_ns, assigned[-1].end_ns)
        else:
            start_ns = 0
        if end_ns <= start_ns:
            continue
        assigned.append(
            _Phase(
                start_s=piece.start_s,
                end_s=piece.end_s,
                start_ns=start_ns,
                end_ns=end_ns,
                p=piece.p,
                v=piece.v,
                a=piece.a,
                j=piece.j,
            )
        )
    return assigned


def _evaluate(piece: _Phase, time_ns: int) -> tuple[float, float, float, float]:
    """Evaluate a Ruckig phase at an integer-nanosecond global time."""
    return _phase_state(piece, time_ns / _NS_PER_S - piece.start_s)


def export_ruckig_reference(
    trajectory: object,
    joint_names: tuple[str, ...],
    *,
    truncate_ns: int | None = None,
) -> RuckigExportedReference:
    """Export one calculated public Ruckig trajectory as an explicit plan.

    ``truncate_ns`` is the fixed hand-off for a non-final chunk.  It must not
    exceed the calculated profile duration.  The caller remains responsible for
    invoking Ruckig exactly once per execution window.
    """
    try:
        names = tuple(joint_names)
    except TypeError as error:
        raise _invalid("joint_names: must be an array of names") from error
    if not 1 <= len(names) <= 64 or len(set(names)) != len(names):
        raise _invalid("joint_names: must be a non-empty unique array")
    if any(not isinstance(name, str) or not name for name in names):
        raise _invalid("joint_names: every name must be a non-empty string")

    try:
        duration_s = _finite(trajectory.duration, "trajectory.duration")  # type: ignore[attr-defined]
        profiles = tuple(trajectory.profiles)  # type: ignore[attr-defined]
    except AttributeError as error:
        raise _invalid("trajectory: must be a calculated Ruckig Trajectory") from error
    if duration_s <= 0.0:
        raise _invalid("trajectory.duration: must be positive")
    if len(profiles) != 1:
        raise _invalid(
            "trajectory.profiles: only one section is supported (no waypoints)",
        )
    section = tuple(profiles[0])
    if len(section) != len(names):
        raise _invalid(
            f"trajectory.profiles[0]: has {len(section)} axes for {len(names)} joints",
        )
    profile_duration_ns = _round_ns(duration_s, "trajectory.duration")
    if profile_duration_ns <= 0 or profile_duration_ns > MAX_HORIZON_NS:
        raise _invalid("trajectory.duration: outside the supported horizon")
    if truncate_ns is None:
        truncate_ns = profile_duration_ns
    if (
        isinstance(truncate_ns, bool)
        or not isinstance(truncate_ns, int)
        or truncate_ns <= 0
        or truncate_ns > profile_duration_ns
    ):
        raise _invalid("truncate_ns: must be a positive integer within the profile duration")

    axis_pieces: list[list[_Phase]] = []
    for axis in range(len(names)):
        pieces, cumulative_s = _phase_pieces(section[axis], axis)
        if not pieces:
            # A truly static axis can be represented as one hold phase at its
            # public initial value.
            public_profile = section[axis]
            try:
                hold_p = _finite(public_profile.p[0], f"profiles[0][{axis}].p[0]")
                hold_v = _finite(public_profile.v[0], f"profiles[0][{axis}].v[0]")
                hold_a = _finite(public_profile.a[0], f"profiles[0][{axis}].a[0]")
                hold_j = _finite(public_profile.j[0], f"profiles[0][{axis}].j[0]")
            except (AttributeError, IndexError, TypeError) as error:
                raise _invalid(f"profiles[0][{axis}]: cannot read the hold state") from error
            pieces = [
                _Phase(
                    start_s=0.0,
                    end_s=duration_s,
                    start_ns=0,
                    end_ns=profile_duration_ns,
                    p=hold_p,
                    v=hold_v,
                    a=hold_a,
                    j=hold_j,
                )
            ]
            cumulative_s = duration_s
        if not math.isclose(
            cumulative_s,
            duration_s,
            rel_tol=1e-9,
            abs_tol=2e-9,
        ):
            raise _invalid(
                f"profiles[0][{axis}]: phase durations do not cover the trajectory",
            )
        assigned = _assign_ns(pieces, profile_duration_ns, axis)
        if not assigned:
            raise _invalid(f"profiles[0][{axis}]: no positive-duration phase")
        if assigned[-1].end_ns < profile_duration_ns:
            last = assigned[-1]
            assigned[-1] = _Phase(
                start_s=last.start_s,
                end_s=duration_s,
                start_ns=last.start_ns,
                end_ns=profile_duration_ns,
                p=last.p,
                v=last.v,
                a=last.a,
                j=last.j,
            )
        if assigned[-1].end_ns != profile_duration_ns:
            raise _invalid(f"profiles[0][{axis}]: exported phases end before the horizon")
        axis_pieces.append(assigned)

    boundaries = {0, truncate_ns}
    for pieces in axis_pieces:
        for piece in pieces:
            if piece.start_ns < truncate_ns:
                boundaries.add(piece.start_ns)
            if piece.end_ns <= truncate_ns:
                boundaries.add(piece.end_ns)
    ordered = sorted(boundaries)

    segments: list[CubicSegment] = []
    previous_end: tuple[tuple[float, ...], tuple[float, ...]] | None = None
    for start_ns, nominal_end_ns in zip(ordered, ordered[1:], strict=False):
        if start_ns >= truncate_ns:
            break
        end_ns = min(nominal_end_ns, truncate_ns)
        if end_ns <= start_ns:
            continue
        c0: list[float] = []
        c1: list[float] = []
        c2: list[float] = []
        c3: list[float] = []
        for axis, pieces in enumerate(axis_pieces):
            active = [
                piece for piece in pieces if piece.start_ns <= start_ns < piece.end_ns
            ]
            if not active:
                raise _invalid(
                    f"profiles[0][{axis}]: no phase covers {start_ns}ns",
                )
            piece = active[-1]
            if end_ns > piece.end_ns:
                raise _invalid(
                    f"profiles[0][{axis}]: interval {start_ns}..{end_ns}ns crosses a phase",
                )
            position, velocity, acceleration, jerk = _evaluate(piece, start_ns)
            if previous_end is not None:
                position, velocity = previous_end[0][axis], previous_end[1][axis]
            c0.append(position)
            c1.append(velocity)
            c2.append(acceleration / 2.0)
            c3.append(jerk / 6.0)
        segment = CubicSegment(
            duration_ns=end_ns - start_ns,
            c0=tuple(c0),
            c1=tuple(c1),
            c2=tuple(c2),
            c3=tuple(c3),
        )
        end_positions, end_velocities, _ = _segment_state_local(segment, segment.duration_s)
        previous_end = (end_positions, end_velocities)
        segments.append(segment)

    if not segments:
        raise _invalid("truncate_ns: produced no exported segments")
    plan = ReferencePlan(names, tuple(segments))
    if plan.duration_ns != truncate_ns:
        raise _invalid(
            f"exported horizon {plan.duration_ns}ns differs from {truncate_ns}ns",
        )
    end = plan.sample_interval(plan.duration_ns)
    if previous_end is None:
        raise _invalid("exported plan has no endpoint")
    for axis in range(len(names)):
        if not math.isclose(
            end.positions[axis],
            previous_end[0][axis],
            rel_tol=1e-12,
            abs_tol=1e-12,
        ) or not math.isclose(
            end.velocities[axis],
            previous_end[1][axis],
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            raise _invalid("exported endpoint differs from the last segment")
    return RuckigExportedReference(
        plan=plan,
        profile_duration_ns=profile_duration_ns,
        truncated_duration_ns=truncate_ns,
        end_positions=end.positions,
        end_velocities=end.velocities,
        end_accelerations=end.accelerations,
    )


def _segment_state_local(
    segment: CubicSegment, local_s: float
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    positions: list[float] = []
    velocities: list[float] = []
    accelerations: list[float] = []
    for c0, c1, c2, c3 in zip(segment.c0, segment.c1, segment.c2, segment.c3, strict=True):
        c3_s = c3 * local_s
        positions.append(((c3_s + c2) * local_s + c1) * local_s + c0)
        velocities.append((3.0 * c3_s + 2.0 * c2) * local_s + c1)
        accelerations.append(6.0 * c3_s + 2.0 * c2)
    return tuple(positions), tuple(velocities), tuple(accelerations)


__all__ = ["RuckigExportedReference", "export_ruckig_reference"]
