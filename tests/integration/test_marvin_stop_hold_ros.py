"""P02B local diagnostic: controlled stop while both hands hold the block."""

from __future__ import annotations

import sys
import uuid
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from marvin_diagnostic_common import (  # noqa: E402
    ROS_IMPORT_ERROR,
    TERMINAL,
    Driver,
    Stack,
    _context,
    _gripper,
    _holding,
    _hold_snapshots,
    _object,
    _plan_for_execution,
    _public_target,
    _snapshot,
    _support_force,
    _write_evidence,
    setup_dual_hold,
)

if ROS_IMPORT_ERROR is not None:  # pragma: no cover
    pytest.skip(f"ROS unavailable: {ROS_IMPORT_ERROR}", allow_module_level=True)

try:
    import rclpy
    from robot_api_msgs.srv import Stop, SubmitFollower
    from robot_ros_support.conversions import context_message
    from robot_ros_support.stream_conversions import stream_command_message
except ImportError as error:  # pragma: no cover
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_stop_under_load(tmp_path):
    scene, planning, plan, targets, segments = _plan_for_execution()
    assert plan.passed, plan.failures
    stack = Stack(tmp_path)
    payload: dict[str, object] = {
        "diagnostic": "stop_under_load",
        "plan": {"passed": plan.passed, "failures": list(plan.failures)},
        "snapshots": [],
        "windows": [],
    }
    snapshots = payload["snapshots"]  # type: ignore[assignment]
    driver = None
    try:
        stack.wait_ready()
        assert stack.info is not None
        driver = Driver(stack, _context(stack.info, "p02b-stop"), "p02b-stop")
        stack.spin(0.5)
        names = tuple(stack.info.joint_names)
        open_width = scene.gripper_open_total
        closed_width = planning.grasp.closed_opening_m
        initial = _snapshot(stack, driver.context)
        initial["label"] = "initial"
        snapshots.append(initial)
        initial_block = _object(initial, "block")["center"]
        expected, table_hold = setup_dual_hold(
            stack, driver, scene, planning, targets, segments, names, sink=snapshots
        )
        waypoints = plan.coordinated_waypoints
        step_duration = max(0.5, segments["dual_hold_lift"].duration_s / max(1, len(waypoints) - 1))
        for index in range(1, len(waypoints)):
            target = _public_target(
                scene, waypoints[index], names, open_width, closed_width, False, False
            )
            window = driver.submit(target, f"dual-waypoint-{index}", step_duration)
            assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED", (
                index, window
            )
        stack.spin(1.0)
        held = _hold_snapshots(
            stack, driver, "dual_hold_before_stop", "block", ("left", "right"),
            minimum_force=planning.diagnostics.contact_force_min_n,
            duration_s=planning.diagnostics.hold_duration_s,
            interval_s=planning.diagnostics.hold_sample_interval_s,
            sink=snapshots,
        )
        lifted_block = _object(held[-1], "block")["center"]
        assert lifted_block[2] - initial_block[2] >= 0.02, lifted_block

        # One small coordinated lift window is stopped while both hands hold.
        # Continue the same coordinated lift by 10% rather than adding an
        # arbitrary joint delta that could lower or twist the block.
        first_waypoint = waypoints[0]
        extra = [
            float(value) + 0.1 * (float(value) - float(first))
            for value, first in zip(waypoints[-1], first_waypoint, strict=True)
        ]
        stop_target = _public_target(
            scene, tuple(extra), names, open_width, closed_width, False, False
        )
        command_id = f"stop-hold-{uuid.uuid4().hex[:8]}"
        request, _hold = driver.request(stop_target, command_id, 1.2)
        response = stack.call(
            "submit_follower",
            SubmitFollower.Request(request=stream_command_message(request)),
            timeout=30.0,
        )
        assert response.accepted, response.reason
        driver.wait_phase(command_id, {"STARTED"}, 20.0)
        before_stop = _snapshot(stack, driver.context)
        before_stop["label"] = "before_stop"
        snapshots.append(before_stop)
        stop_ack = driver.stop("P02B_HOLD_STOP")
        assert stop_ack["success"], stop_ack
        terminal = driver.wait_phase(command_id, TERMINAL, 30.0)
        assert terminal.phase in {"STOPPED", "CANCELED"}, terminal.model_dump()
        assert terminal.stop_reason == "P02B_HOLD_STOP", terminal.model_dump()
        stack.spin(0.4)
        after_stop = _snapshot(stack, driver.context)
        after_stop["label"] = "after_stop"
        snapshots.append(after_stop)
        before_open = {
            side: _gripper(before_stop, side)["opening"] for side in ("left", "right")
        }
        after_open = {
            side: _gripper(after_stop, side)["opening"] for side in ("left", "right")
        }
        for side in ("left", "right"):
            assert abs(after_open[side] - before_open[side]) <= (
                planning.diagnostics.stop_gripper_tolerance_m
            ), (side, before_open[side], after_open[side])
            assert _holding(
                after_stop, side, "block", planning.diagnostics.contact_force_min_n
            ), f"{side} lost the block after stop"
        assert _support_force(after_stop, "block", "table") == 0.0
        payload["result"] = {
            "command_id": command_id,
            "stop_ack": stop_ack,
            "terminal_phase": terminal.phase,
            "stop_reason": terminal.stop_reason,
            "before_openings": before_open,
            "after_openings": after_open,
            "before_block_z": _object(before_stop, "block")["center"][2],
            "after_block_z": _object(after_stop, "block")["center"][2],
            "feedback": [
                item.model_dump(mode="json")
                for item in [
                    driver.query(command_id)
                ]
            ],
        }
        release = driver.submit(
            _public_target(
                scene, waypoints[-1], names, open_width, closed_width, True, True
            ),
            "dual-release",
            0.5,
        )
        assert release["accepted"], release["reason"]
        assert release.get("terminal", {}).get("phase") == "COMPLETED", release
        payload["windows"] = driver.windows
        driver.close()
        status = stack.close()
        assert status == 0, stack.output()
        _write_evidence("completed", stack, payload)
    except BaseException:
        if driver is not None:
            payload["windows"] = driver.windows
        try:
            stack.close()
        finally:
            _write_evidence("failed", stack, payload)
        raise
