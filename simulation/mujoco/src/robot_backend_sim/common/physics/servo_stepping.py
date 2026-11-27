"""Pure-Python zero-ROS servo command dynamics filter.

Models actuator transport delay (FIFOs), per-step joint rate clamping matching
hardware CSP step limits, and gripper retrigger deadband hysteresis without
importing ``rclpy`` or ROS libraries so both ``MarvinIO`` and future RL
``Env.step()`` loops can share identical step-level actuator dynamics.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass

__all__ = ["ServoDynamicsFilter", "ServoDynamicsParams"]


@dataclass(frozen=True, slots=True)
class ServoDynamicsParams:
    """Typed parameters governing step-level servo command filtering."""

    arm_delay_steps: int = 0
    max_joint_step_rad: float | None = None
    gripper_delay_steps: int = 0
    gripper_deadband_m: float = 0.0

    def __post_init__(self) -> None:
        if self.arm_delay_steps < 0:
            raise ValueError("arm_delay_steps must be non-negative")
        if self.gripper_delay_steps < 0:
            raise ValueError("gripper_delay_steps must be non-negative")
        if self.max_joint_step_rad is not None and (
            not math.isfinite(self.max_joint_step_rad) or self.max_joint_step_rad <= 0.0
        ):
            raise ValueError("max_joint_step_rad must be positive when configured")
        if not math.isfinite(self.gripper_deadband_m) or self.gripper_deadband_m < 0.0:
            raise ValueError("gripper_deadband_m must be non-negative and finite")

    @property
    def is_identity(self) -> bool:
        return (
            self.arm_delay_steps == 0
            and self.max_joint_step_rad is None
            and self.gripper_delay_steps == 0
            and self.gripper_deadband_m <= 0.0
        )


class ServoDynamicsFilter:
    """Stateful step filter for arm/gripper reference commands."""

    def __init__(
        self,
        params: ServoDynamicsParams,
        axis_count: int,
        gripper_axes: frozenset[int],
        *,
        gripper_bounds: tuple[float, float] | None = None,
    ) -> None:
        if axis_count <= 0:
            raise ValueError("axis_count must be positive")
        self._params = params
        self._axis_count = int(axis_count)
        self._gripper_axes = gripper_axes
        self._gripper_bounds = gripper_bounds
        self._arm_queues: dict[int, deque[float]] = {
            axis: deque() for axis in range(self._axis_count) if axis not in gripper_axes
        }
        self._gripper_queues: dict[int, deque[float]] = {
            axis: deque() for axis in range(self._axis_count) if axis in gripper_axes
        }
        self._last_arm_cmd: dict[int, float] = {axis: 0.0 for axis in self._arm_queues}
        self._last_gripper_trigger: dict[int, float] = {
            axis: 0.0 for axis in self._gripper_queues
        }
        self._initialized = False

    def reset(self, initial_positions: Sequence[float]) -> None:
        """Reset delay queues and rate/deadband latches to the initial state."""
        if len(initial_positions) != self._axis_count:
            raise ValueError(
                f"expected {self._axis_count} initial positions, got {len(initial_positions)}"
            )
        arm_depth = self._params.arm_delay_steps
        gripper_depth = self._params.gripper_delay_steps
        for axis, value in enumerate(initial_positions):
            val = float(value)
            if axis in self._gripper_axes:
                queue = self._gripper_queues[axis]
                queue.clear()
                for _ in range(gripper_depth):
                    queue.append(val)
                self._last_gripper_trigger[axis] = val
            else:
                queue = self._arm_queues[axis]
                queue.clear()
                for _ in range(arm_depth):
                    queue.append(val)
                self._last_arm_cmd[axis] = val
        self._initialized = True

    def _is_gripper_endpoint(self, value: float) -> bool:
        if self._gripper_bounds is None:
            return False
        closed_val, open_val = self._gripper_bounds
        return math.isclose(value, closed_val, abs_tol=1e-9) or math.isclose(
            value, open_val, abs_tol=1e-9
        )

    def step(self, positions: Sequence[float]) -> tuple[float, ...]:
        """Filter one 16-DOF public target vector and return filtered targets."""
        if len(positions) != self._axis_count:
            raise ValueError(
                f"expected {self._axis_count} target positions, got {len(positions)}"
            )
        if not self._initialized:
            self.reset(positions)
        if self._params.is_identity:
            out = tuple(float(value) for value in positions)
            for axis, value in enumerate(out):
                if axis in self._gripper_axes:
                    self._last_gripper_trigger[axis] = value
                else:
                    self._last_arm_cmd[axis] = value
            return out

        filtered: list[float] = []
        max_step = self._params.max_joint_step_rad
        deadband = self._params.gripper_deadband_m
        for axis, raw in enumerate(positions):
            want = float(raw)
            if axis in self._gripper_axes:
                last_trig = self._last_gripper_trigger[axis]
                if deadband <= 0.0 or self._is_gripper_endpoint(want) or abs(want - last_trig) >= deadband:
                    triggered = want
                    self._last_gripper_trigger[axis] = want
                else:
                    triggered = last_trig
                queue = self._gripper_queues[axis]
                if self._params.gripper_delay_steps > 0:
                    queue.append(triggered)
                    emitted = queue.popleft()
                else:
                    emitted = triggered
                filtered.append(emitted)
            else:
                queue = self._arm_queues[axis]
                if self._params.arm_delay_steps > 0:
                    queue.append(want)
                    delayed = queue.popleft()
                else:
                    delayed = want
                prev = self._last_arm_cmd[axis]
                if max_step is not None:
                    delta = delayed - prev
                    if abs(delta) > max_step:
                        delayed = prev + math.copysign(max_step, delta)
                self._last_arm_cmd[axis] = delayed
                filtered.append(delayed)
        return tuple(filtered)
