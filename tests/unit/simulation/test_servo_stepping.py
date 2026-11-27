"""Tests for the zero-ROS pure-Python servo stepping filter."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.physics.servo_stepping import (  # noqa: E402
    ServoDynamicsFilter,
    ServoDynamicsParams,
)

GRIPPER_AXES = frozenset({7, 15})


def _vec(arm_val: float, gripper_val: float) -> tuple[float, ...]:
    return (arm_val,) * 7 + (gripper_val,) + (arm_val,) * 7 + (gripper_val,)


def test_servo_stepping_imports_zero_rclpy():
    subprocess.run(
        [
            sys.executable,
            "-c",
            (
                f"import sys; sys.path.insert(0, {str(SIM_SRC)!r}); "
                "from robot_backend_sim.common.physics.servo_stepping import "
                "ServoDynamicsFilter, ServoDynamicsParams; "
                "assert 'rclpy' not in sys.modules"
            ),
        ],
        check=True,
    )
    filt = ServoDynamicsFilter(ServoDynamicsParams(), 16, GRIPPER_AXES)
    out = filt.step(_vec(0.25, 0.09))
    assert out == _vec(0.25, 0.09)


def test_arm_delay_and_joint_step_clamp_and_reset():
    params = ServoDynamicsParams(arm_delay_steps=2, max_joint_step_rad=0.1)
    filt = ServoDynamicsFilter(params, 16, GRIPPER_AXES, gripper_bounds=(0.0, 0.09))
    filt.reset(_vec(0.0, 0.09))

    # First two steps emit delayed initial 0.0 even when target jumps to 0.5
    s1 = filt.step(_vec(0.5, 0.09))
    s2 = filt.step(_vec(0.5, 0.09))
    assert s1[0] == pytest.approx(0.0)
    assert s2[0] == pytest.approx(0.0)

    # Third step sees 0.5 delayed from step 1 -> clamped at +0.1 per step
    s3 = filt.step(_vec(0.5, 0.09))
    s4 = filt.step(_vec(0.5, 0.09))
    assert s3[0] == pytest.approx(0.1)
    assert s4[0] == pytest.approx(0.2)

    # Reset clears queue back to new initial state
    filt.reset(_vec(-0.3, 0.09))
    assert filt.step(_vec(0.4, 0.09))[0] == pytest.approx(-0.3)


def test_gripper_deadband_suppresses_micro_commands_but_allows_endpoints():
    params = ServoDynamicsParams(gripper_delay_steps=1, gripper_deadband_m=0.0003)
    filt = ServoDynamicsFilter(params, 16, GRIPPER_AXES, gripper_bounds=(0.0, 0.09))
    filt.reset(_vec(0.0, 0.05))

    # Below 0.3 mm deadband: stays latched at 0.05
    out1 = filt.step(_vec(0.0, 0.0502))
    out2 = filt.step(_vec(0.0, 0.0502))
    assert out1[7] == pytest.approx(0.05)
    assert out2[7] == pytest.approx(0.05)

    # Exact endpoint 0.0 bypasses deadband even from 0.0001
    filt.reset(_vec(0.0, 0.0001))
    filt.step(_vec(0.0, 0.0))  # pushed into 1-step delay
    out3 = filt.step(_vec(0.0, 0.0))
    assert out3[7] == pytest.approx(0.0)
