"""Cubic plan evaluation with unchanged interval and hold semantics."""
from __future__ import annotations

from robot_contracts.models import ContractError
from robot_contracts.trajectory.spline import ReferencePlan, ReferenceSample

from .spline_math import _invalid, _state_at


def sample_reference_interval(plan: ReferencePlan, elapsed_ns: int) -> ReferenceSample:
    if isinstance(elapsed_ns, bool) or not isinstance(elapsed_ns, int):
        raise _invalid("elapsed_ns", "must be an integer")
    if elapsed_ns < 0 or elapsed_ns > plan.duration_ns:
        raise ContractError("REFERENCE_EXPIRED",
                            f"elapsed_ns {elapsed_ns} outside [0, {plan.duration_ns}]")
    return _sample_within_horizon(plan, elapsed_ns)

def _sample_within_horizon(plan: ReferencePlan, elapsed: int) -> ReferenceSample:
    for index, segment in enumerate(plan.segments):
        if elapsed <= segment.duration_ns:
            return ReferenceSample(
                *_state_at(segment, elapsed / 1_000_000_000, f"segments[{index}]"))
        elapsed -= segment.duration_ns
    raise AssertionError("validated plan did not contain the elapsed time")
