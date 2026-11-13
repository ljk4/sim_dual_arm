"""P02B local diagnostic: dual hold and the coordinated airborne lift.

The task module plans and validates each commanded lift step from a measured
grasp.  Owner snapshots provide the independent measured relative-pose curve.
"""

from __future__ import annotations

import sys
from dataclasses import asdict
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_backend_sim.tasks.handover.cohold import (  # noqa: E402
    CoholdReference,
    plan_cohold_step,
)
from marvin_diagnostic_common import (  # noqa: E402
    PROFILE,
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
    _quaternion_matrix,
    _relative_pose,
    _rotation_angle,
    _snapshot,
    _support_force,
    _write_evidence,
)

if ROS_IMPORT_ERROR is not None:  # pragma: no cover
    pytest.skip(f"ROS unavailable: {ROS_IMPORT_ERROR}", allow_module_level=True)

try:
    import rclpy
    import numpy as np
except ImportError as error:  # pragma: no cover
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _planned_curve(plan) -> list[dict[str, float]]:
    return [
        {
            "fraction": item.fraction,
            "left_position_error_m": item.left_position_error_m,
            "left_orientation_error_rad": item.left_orientation_error_rad,
            "right_position_error_m": item.right_position_error_m,
            "right_orientation_error_rad": item.right_orientation_error_rad,
        }
        for item in plan.coordination
    ]


def _measured_errors(snapshot, expected) -> dict[str, float]:
    result = {}
    for side in ("left", "right"):
        position, rotation = _relative_pose(snapshot, side, "block")
        result[f"{side}_position_error_m"] = float(
            np.linalg.norm(position - expected[side][0])
        )
        result[f"{side}_orientation_error_rad"] = _rotation_angle(
            rotation, expected[side][1]
        )
    return result


def test_dual_hold_and_coordinated_lift(tmp_path):
    scene, planning, plan, targets, segments = _plan_for_execution()
    assert plan.passed, plan.failures
    assert len(plan.coordinated_waypoints) >= 2
    stack = Stack(tmp_path)
    payload: dict[str, object] = {
        "diagnostic": "dual_hold",
        "plan_coordination": {
            "summary": plan.coordination_summary,
            "curve": _planned_curve(plan),
            "waypoint_count": len(plan.coordinated_waypoints),
        },
        "snapshots": [],
        "windows": [],
        "measured_curve": [],
        "planned_steps": [],
    }
    snapshots = payload["snapshots"]  # type: ignore[assignment]
    measured_curve = payload["measured_curve"]  # type: ignore[assignment]
    driver = None
    try:
        stack.wait_ready()
        assert stack.info is not None
        driver = Driver(stack, _context(stack.info, "p02b-dual"), "p02b-dual")
        stack.spin(0.5)
        names = tuple(stack.info.joint_names)
        open_width = scene.gripper_open_total
        closed_width = planning.grasp.closed_opening_m

        def target(name: str, *, left_open: bool, right_open: bool):
            return _public_target(
                scene, targets[name].qpos, names, open_width, closed_width,
                left_open, right_open,
            )

        initial = _snapshot(stack, driver.context)
        initial["label"] = "initial"
        snapshots.append(initial)
        initial_block = _object(initial, "block")["center"]
        for name, segment, label in (
            ("left_ready", "left_cohold_approach", "cohold-left-ready"),
            ("left_cohold_pregrasp", "left_cohold_pregrasp_path", "cohold-left-pregrasp"),
            ("left_cohold_grasp", "left_cohold_grasp_path", "cohold-left-grasp"),
        ):
            window = driver.submit(
                target(name, left_open=True, right_open=True),
                label,
                segments[segment].duration_s,
            )
            assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED", (
                label, window
            )
        window = driver.submit(
            target("left_cohold_grasp", left_open=False, right_open=True),
            "cohold-left-close",
            0.5,
        )
        assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED"
        for name, segment, label in (
            ("right_cohold_wait", "right_cohold_wait_path", "cohold-right-wait"),
            ("right_cohold_pregrasp", "right_cohold_pregrasp_path", "cohold-right-pregrasp"),
            ("right_cohold_grasp", "right_cohold_grasp_path", "cohold-right-grasp"),
        ):
            window = driver.submit(
                target(name, left_open=False, right_open=True),
                label,
                segments[segment].duration_s,
            )
            assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED", (
                label, window
            )
        before = _snapshot(stack, driver.context)
        before["label"] = "dual_before_close"
        snapshots.append(before)
        assert _holding(before, "left", "block", planning.diagnostics.contact_force_min_n)
        assert not _finger_contacts(before, "right", "block")
        window = driver.submit(
            target("right_cohold_grasp", left_open=False, right_open=False),
            "cohold-right-close",
            0.5,
        )
        assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED"
        table_hold = _hold_snapshots(
            stack, driver, "dual_table_hold", "block", ("left", "right"),
            minimum_force=planning.diagnostics.contact_force_min_n,
            duration_s=planning.diagnostics.hold_duration_s,
            interval_s=planning.diagnostics.hold_sample_interval_s,
            require_off_table=False,
            sink=snapshots,
        )
        expected = {
            side: _relative_pose(table_hold[0], side, "block") for side in ("left", "right")
        }
        for snapshot in table_hold:
            entry = {"label": "dual_table_hold", **_measured_errors(snapshot, expected)}
            measured_curve.append(entry)

        waypoints = plan.coordinated_waypoints
        if len(waypoints) < 2:
            raise AssertionError("coordinated lift has fewer than two waypoints")
        reference = CoholdReference(
            block_center=tuple(_object(table_hold[0], "block")["center"]),
            block_rotation=tuple(
                tuple(float(value) for value in row)
                for row in _quaternion_matrix(tuple(_object(table_hold[0], "block")["quaternion"]))
            ),
            tcp_positions={
                side: tuple(_gripper(table_hold[0], side)["tcp_position"])
                for side in ("left", "right")
            },
            tcp_rotations={
                side: tuple(
                    tuple(float(value) for value in row)
                    for row in _quaternion_matrix(
                        tuple(_gripper(table_hold[0], side)["tcp_quaternion"])
                    )
                )
                for side in ("left", "right")
            },
        )
        model_joint_names = tuple(
            name for chain in scene.robot_geometry.chains for name in chain.joint_names
        )
        profile_arm_joints = tuple(joint for joint in PROFILE.joints if joint.unit == "rad")
        public_to_model = tuple(zip(profile_arm_joints, model_joint_names, strict=True))
        velocities = {name: float(joint.max_velocity) for joint, name in public_to_model}
        step_rise = planning.grasp.cohold_lift_height_m / (len(waypoints) - 1)
        previous_command = tuple(waypoints[0])
        for index in range(1, len(waypoints)):
            state = stack.latest_state()
            assert state is not None
            measured = dict(zip(names, state.joints.position, strict=True))
            step = plan_cohold_step(
                scene, planning, reference,
                measured_joints={
                    model_name: float(measured[joint.name])
                    for joint, model_name in public_to_model
                },
                previous_command=previous_command,
                start_rise_m=step_rise * (index - 1),
                end_rise_m=step_rise * index,
                max_velocities=velocities,
                first_step=index == 1,
            )
            payload["planned_steps"].append({"index": index, **asdict(step)})
            assert step.passed, (
                index, step.failures,
                step.clearance.failure_reason, step.clearance.failure_message,
            )
            window = driver.submit(
                _public_target(scene, step.qpos, names, open_width, closed_width, False, False),
                f"dual-feedback-{index}", max(0.5, step.clearance.duration_s),
            )
            assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED", (
                index, window
            )
            snapshot = _snapshot(stack, driver.context)
            snapshot["label"] = f"dual_waypoint_{index}"
            snapshots.append(snapshot)
            measured_curve.append(
                {"label": snapshot["label"], **_measured_errors(snapshot, expected)}
            )
            previous_command = step.qpos
        stack.spin(1.0)
        held = _hold_snapshots(
            stack, driver, "dual_hold", "block", ("left", "right"),
            minimum_force=planning.diagnostics.contact_force_min_n,
            duration_s=planning.diagnostics.hold_duration_s,
            interval_s=planning.diagnostics.hold_sample_interval_s,
            sink=snapshots,
        )
        for snapshot in held:
            measured_curve.append(
                {"label": "dual_hold", **_measured_errors(snapshot, expected)}
            )
        lifted_block = _object(held[-1], "block")["center"]
        assert lifted_block[2] - initial_block[2] >= 0.02, lifted_block
        assert _support_force(held[-1], "block", "table") == 0.0
        # Full process and both sides, against the table baseline.  The
        # planned waypoint path is checked separately; this check is the
        # physical relative-pose budget over the table grasp, lift and hold.
        process_max = {
            "left_position": max(item["left_position_error_m"] for item in measured_curve),
            "left_orientation": max(
                item["left_orientation_error_rad"] for item in measured_curve
            ),
            "right_position": max(
                item["right_position_error_m"] for item in measured_curve
            ),
            "right_orientation": max(
                item["right_orientation_error_rad"] for item in measured_curve
            ),
        }
        assert process_max["left_position"] <= planning.diagnostics.relative_position_tolerance_m, measured_curve
        assert process_max["right_position"] <= planning.diagnostics.relative_position_tolerance_m, measured_curve
        assert process_max["left_orientation"] <= planning.diagnostics.relative_orientation_tolerance_rad, measured_curve
        assert process_max["right_orientation"] <= planning.diagnostics.relative_orientation_tolerance_rad, measured_curve
        for snapshot in snapshots:
            label = snapshot.get("label", "")
            if label.startswith("dual_waypoint") or label == "dual_hold":
                obj = _object(snapshot, "block")
                assert max(abs(value) for value in obj["linear_velocity"]) <= (
                    planning.diagnostics.max_object_linear_speed_m_s
                ), (label, obj["linear_velocity"])
                assert max(abs(value) for value in obj["angular_velocity"]) <= (
                    planning.diagnostics.max_object_angular_speed_rad_s
                ), (label, obj["angular_velocity"])
        payload["result"] = {
            "initial_z": initial_block[2],
            "lifted_z": lifted_block[2],
            "rise_m": lifted_block[2] - initial_block[2],
            "left_opening": _gripper(held[-1], "left")["opening"],
            "right_opening": _gripper(held[-1], "right")["opening"],
            "waypoints": len(waypoints),
            "table_hold": len(table_hold),
            "lifted_hold": len(held),
            "process_max": process_max,
        }
        release = driver.submit(
            target("right_cohold_lift", left_open=True, right_open=True),
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
