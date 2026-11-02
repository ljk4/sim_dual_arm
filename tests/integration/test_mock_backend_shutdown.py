"""Native ROS context shutdown errors must not hide healthy-context failures."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
PROGRAM = r"""
import sys
from pathlib import Path
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as native
from robot_api_msgs.msg import RobotState
import mock_robot_backend.node as backend

mode, root = sys.argv[1], Path(sys.argv[2])
assert Path(backend.__file__).is_relative_to(root / 'tests/doubles/ros/mock_robot_backend')

def spin(executor):
    node = executor.get_nodes()[0]
    if mode == 'healthy_error':
        assert rclpy.ok()
        raise native.RCLError('INDEPENDENT_HEALTHY_CONTEXT_ERROR')
    rclpy.shutdown()
    if mode == 'other_error':
        raise ValueError('INDEPENDENT_UNRELATED_ERROR')
    try:
        node._state.publish(RobotState())
    except native.RCLError:
        print('NATIVE_CONTEXT_PUBLISH_ERROR', flush=True)
        raise
    raise AssertionError('native publisher unexpectedly accepted an invalid context')

backend.SingleThreadedExecutor.spin = spin
sys.argv = ['backend', '--profile', str(root / 'profiles/robots/mock_dual_arm_v1/profile.json'),
            '--schemas', str(root / 'contracts/schemas/v1')]
backend.main()
"""


@pytest.mark.parametrize(
    ("mode", "expected", "marker"),
    [
        ("closed_context", 0, "NATIVE_CONTEXT_PUBLISH_ERROR"),
        ("healthy_error", 1, "INDEPENDENT_HEALTHY_CONTEXT_ERROR"),
        ("other_error", 1, "INDEPENDENT_UNRELATED_ERROR"),
    ],
)
def test_backend_shutdown_only_accepts_native_errors_from_an_invalid_context(
    mode, expected, marker
):
    result = subprocess.run(
        [sys.executable, "-c", PROGRAM, mode, str(ROOT)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        timeout=15,
    )
    assert result.returncode == expected, result.stdout + result.stderr
    assert marker in result.stdout + result.stderr
