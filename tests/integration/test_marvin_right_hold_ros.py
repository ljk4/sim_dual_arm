"""P02B local diagnostic: right independent grasp, lift and hold."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from marvin_diagnostic_common import (  # noqa: E402
    ROS_IMPORT_ERROR,
    Driver,
    Stack,
    _assert_relative_stable,
    _context,
    _finger_contacts,
    _gripper,
    _hold_snapshots,
    _holding,
    _object,
    _plan_for_execution,
    _public_target,
    _snapshot,
    _support_force,
    _write_evidence,
)

if ROS_IMPORT_ERROR is not None:  # pragma: no cover
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


def test_right_independent_hold(tmp_path):
    scene, planning, plan, targets, segments = _plan_for_execution()
    stack = Stack(tmp_path)
    payload: dict[str, object] = {
        "diagnostic": "right_hold",
        "plan": {"passed": plan.passed, "failures": list(plan.failures)},
        "snapshots": [],
        "windows": [],
    }
    snapshots = payload["snapshots"]  # type: ignore[assignment]
    driver = None
    try:
        stack.wait_ready()
        assert stack.info is not None
        driver = Driver(stack, _context(stack.info, "p02b-right"), "p02b-right")
        stack.spin(0.5)
        names = tuple(stack.info.joint_names)
        open_width = scene.gripper_open_total
        closed_width = planning.grasp.closed_opening_m

        def target(name: str, *, right_open: bool):
            return _public_target(
                scene, targets[name].qpos, names, open_width, closed_width, True, right_open
            )

        initial = _snapshot(stack, driver.context)
        initial["label"] = "initial"
        snapshots.append(initial)
        initial_block = _object(initial, "block")["center"]
        assert _support_force(initial, "block", "table") > 0.0
        for name, segment, label in (
            ("right_wait_initial", "right_initial_wait", "right-wait"),
            ("right_initial_pregrasp", "right_initial_pregrasp_path", "right-pregrasp"),
            ("right_center_initial", "right_initial_grasp_path", "right-grasp"),
        ):
            window = driver.submit(
                target(name, right_open=True), label, segments[segment].duration_s
            )
            assert (
                window["accepted"]
                and window.get("terminal", {}).get("phase") == "COMPLETED"
            ), (label, window)
        before = _snapshot(stack, driver.context)
        before["label"] = "before_close"
        snapshots.append(before)
        assert not _finger_contacts(before, "right", "block")
        window = driver.submit(
            target("right_center_initial", right_open=False), "right-close", 0.5
        )
        assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED"
        after = _snapshot(stack, driver.context)
        after["label"] = "after_close"
        snapshots.append(after)
        assert _holding(after, "right", "block", planning.diagnostics.contact_force_min_n)
        lift = driver.submit(
            target("right_center_lift", right_open=False),
            "right-lift",
            segments["right_initial_lift"].duration_s,
        )
        assert lift["accepted"] and lift.get("terminal", {}).get("phase") == "COMPLETED", lift
        stack.spin(1.5)
        held = _hold_snapshots(
            stack, driver, "right_lifted_hold", "block", ("right",),
            minimum_force=planning.diagnostics.contact_force_min_n,
            duration_s=planning.diagnostics.hold_duration_s,
            interval_s=planning.diagnostics.hold_sample_interval_s,
            sink=snapshots,
        )
        lifted_block = _object(held[-1], "block")["center"]
        assert lifted_block[2] - initial_block[2] >= 0.03, lifted_block
        assert _finger_contacts(held[-1], "left", "block") == []
        _assert_relative_stable(
            held, "right", "block",
            position_tolerance_m=planning.diagnostics.relative_position_tolerance_m,
            orientation_tolerance_rad=planning.diagnostics.relative_orientation_tolerance_rad,
            object_max_linear_speed=planning.diagnostics.max_object_linear_speed_m_s,
            object_max_angular_speed=planning.diagnostics.max_object_angular_speed_rad_s,
        )
        payload["result"] = {
            "initial_z": initial_block[2],
            "lifted_z": lifted_block[2],
            "rise_m": lifted_block[2] - initial_block[2],
            "right_opening": _gripper(held[-1], "right")["opening"],
            "lifted_hold": len(held),
        }
        driver.submit(target("right_center_lift", right_open=True), "right-release", 0.5)
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
