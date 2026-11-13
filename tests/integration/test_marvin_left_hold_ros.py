"""P02B local diagnostic: left independent grasp, lift and hold."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from marvin_diagnostic_common import (  # noqa: E402
    ROS_IMPORT_ERROR,
    Driver,
    Stack,
    _context,
    _finger_contacts,
    _gripper,
    _holding,
    _hold_snapshots,
    _object,
    _plan_for_execution,
    _public_target,
    _assert_relative_stable,
    _snapshot,
    _support_force,
    _write_evidence,
)

if ROS_IMPORT_ERROR is not None:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {ROS_IMPORT_ERROR}", allow_module_level=True)

try:
    import rclpy
except ImportError as error:  # pragma: no cover
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_left_independent_hold(tmp_path):
    scene, planning, plan, targets, segments = _plan_for_execution()
    stack = Stack(tmp_path)
    payload: dict[str, object] = {
        "diagnostic": "left_hold",
        "plan": {"passed": plan.passed, "failures": list(plan.failures)},
        "snapshots": [],
        "windows": [],
    }
    snapshots = payload["snapshots"]  # type: ignore[assignment]
    driver = None
    try:
        stack.wait_ready()
        assert stack.info is not None
        driver = Driver(stack, _context(stack.info, "p02b-left"), "p02b-left")
        stack.spin(0.5)
        names = tuple(stack.info.joint_names)
        open_width = scene.gripper_open_total
        closed_width = planning.grasp.closed_opening_m

        def target(name: str, *, left_open: bool, right_open: bool = True):
            return _public_target(
                scene, targets[name].qpos, names, open_width, closed_width,
                left_open, right_open,
            )

        initial = _snapshot(stack, driver.context)
        initial["label"] = "initial"
        snapshots.append(initial)
        initial_block = _object(initial, "block")["center"]
        assert _support_force(initial, "block", "table") > 0.0
        for name, segment, label in (
            ("left_ready", "left_approach", "left-ready"),
            ("left_pregrasp", "left_pregrasp_path", "left-pregrasp"),
            ("left_grasp", "left_grasp_path", "left-grasp"),
        ):
            window = driver.submit(
                target(name, left_open=True), label, segments[segment].duration_s
            )
            assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED", (
                label, window
            )
        before = _snapshot(stack, driver.context)
        before["label"] = "before_close"
        snapshots.append(before)
        assert not _finger_contacts(before, "left", "block")
        assert _object(before, "block")["center"] == pytest.approx(initial_block, abs=2e-3)
        window = driver.submit(
            target("left_grasp", left_open=False), "left-close", 0.5
        )
        assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED"
        after = _snapshot(stack, driver.context)
        after["label"] = "after_close"
        snapshots.append(after)
        assert _holding(after, "left", "block", planning.diagnostics.contact_force_min_n)
        assert abs(_object(after, "block")["center"][2] - initial_block[2]) <= 2e-3

        lifted = _hold_snapshots(
            stack, driver, "left_table_hold", "block", ("left",),
            minimum_force=planning.diagnostics.contact_force_min_n,
            duration_s=planning.diagnostics.hold_duration_s,
            interval_s=planning.diagnostics.hold_sample_interval_s,
            require_off_table=False,
            sink=snapshots,
        )
        lift = driver.submit(
            target("left_lift", left_open=False), "left-lift", segments["left_lift"].duration_s
        )
        assert lift["accepted"] and lift.get("terminal", {}).get("phase") == "COMPLETED", lift
        stack.spin(1.0)
        held = _hold_snapshots(
            stack, driver, "left_lifted_hold", "block", ("left",),
            minimum_force=planning.diagnostics.contact_force_min_n,
            duration_s=planning.diagnostics.hold_duration_s,
            interval_s=planning.diagnostics.hold_sample_interval_s,
            sink=snapshots,
        )
        lifted_block = _object(held[-1], "block")["center"]
        assert lifted_block[2] - initial_block[2] >= 0.05, lifted_block
        _assert_relative_stable(
            held, "left", "block",
            position_tolerance_m=planning.diagnostics.relative_position_tolerance_m,
            orientation_tolerance_rad=planning.diagnostics.relative_orientation_tolerance_rad,
            object_max_linear_speed=planning.diagnostics.max_object_linear_speed_m_s,
            object_max_angular_speed=planning.diagnostics.max_object_angular_speed_rad_s,
        )
        payload["result"] = {
            "initial_z": initial_block[2],
            "lifted_z": lifted_block[2],
            "rise_m": lifted_block[2] - initial_block[2],
            "left_opening": _gripper(held[-1], "left")["opening"],
            "table_hold": len(lifted),
            "lifted_hold": len(held),
        }
        driver.submit(target("left_grasp", left_open=True), "left-release", 0.5)
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
