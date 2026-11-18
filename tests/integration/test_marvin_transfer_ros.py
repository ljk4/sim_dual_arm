"""P02B continuous Robot API diagnostic: airborne transfer and box placement."""

from __future__ import annotations

import sys
from dataclasses import asdict
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
    _hold_snapshots,
    _holding,
    _object,
    _plan_for_execution,
    _public_target,
    _quaternion_matrix,
    _rotation_angle,
    _snapshot,
    _support_force,
    _write_evidence,
)

if ROS_IMPORT_ERROR is not None:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {ROS_IMPORT_ERROR}", allow_module_level=True)

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from robot_backend_sim.marvin_sim.handover.plan import _placement_containment  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_continuous_airborne_transfer_and_box_placement(tmp_path):
    scene, planning, plan, targets, segments = _plan_for_execution()
    diagnostics = planning.diagnostics
    layout = plan.layout
    object_name = layout.block_name
    stack = Stack(tmp_path)
    payload: dict[str, object] = {
        "diagnostic": "continuous_transfer",
        "plan": {
            "passed": plan.passed,
            "failures": list(plan.failures),
            "layout": asdict(layout),
            "segments": [
                {
                    "name": segment.name,
                    "start_target": segment.start_target,
                    "end_target": segment.end_target,
                    "duration_s": segment.duration_s,
                    "passed": segment.passed,
                }
                for segment in plan.segments
            ],
        },
        "snapshots": [],
        "windows": [],
        "placement_checks": [],
    }
    snapshots = payload["snapshots"]
    driver = None
    status = "failed"
    try:
        stack.wait_ready()
        assert stack.info is not None
        driver = Driver(stack, _context(stack.info, "p02b-transfer"), "p02b-transfer")
        stack.spin(0.5)
        names = tuple(stack.info.joint_names)

        def capture(label):
            snapshot = _snapshot(stack, driver.context)
            snapshot["label"] = label
            snapshots.append(snapshot)
            return snapshot

        def command(name, label, *, left_open, right_open, segment=None):
            target = _public_target(
                scene, targets[name].qpos, names,
                scene.gripper_open_total, planning.grasp.closed_opening_m,
                left_open, right_open,
            )
            duration = segments[segment].duration_s if segment is not None else 0.5
            window = driver.submit(target, label, duration)
            capture(label)
            assert window["accepted"] and window.get("terminal", {}).get("phase") == (
                "COMPLETED"
            ), (label, window)

        def hold(label, sides, *, require_off_table=True):
            held = _hold_snapshots(
                stack, driver, label, object_name, sides,
                minimum_force=diagnostics.contact_force_min_n,
                duration_s=diagnostics.hold_duration_s,
                interval_s=diagnostics.hold_sample_interval_s,
                require_off_table=require_off_table,
                sink=snapshots,
            )
            for side in sides:
                _assert_relative_stable(
                    held, side, object_name,
                    position_tolerance_m=diagnostics.relative_position_tolerance_m,
                    orientation_tolerance_rad=diagnostics.relative_orientation_tolerance_rad,
                    object_max_linear_speed=diagnostics.max_object_linear_speed_m_s,
                    object_max_angular_speed=diagnostics.max_object_angular_speed_rad_s,
                )
            return held

        initial = capture("initial")
        initial_block = _object(initial, object_name)
        assert _support_force(initial, object_name, "table") > 0.0
        for name, segment in (
            ("left_ready", "left_approach"),
            ("left_pregrasp", "left_pregrasp_path"),
            ("left_grasp", "left_grasp_path"),
        ):
            command(name, segment, left_open=True, right_open=True, segment=segment)
        command("left_grasp", "left-close", left_open=False, right_open=True)
        hold("left_table_hold", ("left",), require_off_table=False)
        command(
            "left_lift", "left-lift", left_open=False, right_open=True,
            segment="left_lift",
        )
        stack.spin(1.0)
        left_lifted = hold("left_lifted_hold", ("left",))
        command(
            "left_handover", "left-handover", left_open=False, right_open=True,
            segment="left_handover",
        )
        stack.spin(1.0)
        hold("left_handover_hold", ("left",))
        for name, segment in (
            ("right_handover_wait", "right_handover_wait_path"),
            ("right_handover_pregrasp", "right_handover_pregrasp_path"),
            ("right_handover_grasp", "right_handover_grasp_path"),
        ):
            command(name, segment, left_open=False, right_open=True, segment=segment)
        before_close = capture("before_right_close")
        assert _holding(
            before_close, "left", object_name, diagnostics.contact_force_min_n
        )
        assert not _finger_contacts(before_close, "right", object_name)
        command(
            "right_handover_grasp", "right-close", left_open=False, right_open=False,
        )
        stack.spin(1.0)
        dual_held = hold("airborne_dual_hold", ("left", "right"))
        command(
            "right_handover_grasp", "left-open", left_open=True, right_open=False,
        )
        command(
            "left_retreat", "left-retreat", left_open=True, right_open=False,
            segment="left_retreat",
        )
        stack.spin(1.0)
        right_held = hold("right_after_transfer_hold", ("right",))
        for snapshot in right_held:
            assert not _finger_contacts(snapshot, "left", object_name), snapshot
        for name, segment in (
            ("right_preplace", "right_transport"),
            ("right_place", "right_place"),
        ):
            command(name, segment, left_open=True, right_open=False, segment=segment)
        command("right_place", "right-open", left_open=True, right_open=True)
        stack.spin(1.0)
        placed = hold("released_in_box", ())
        first_block = _object(placed[0], object_name)
        first_center = np.asarray(first_block["center"])
        first_rotation = _quaternion_matrix(tuple(first_block["quaternion"]))
        half = np.asarray(layout.block_half_extents)
        corners = np.asarray(
            [[sx * half[0], sy * half[1], sz * half[2]]
             for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]
        )
        for snapshot in placed:
            block = _object(snapshot, object_name)
            center = np.asarray(block["center"])
            rotation = _quaternion_matrix(tuple(block["quaternion"]))
            vertices = corners @ rotation.T + center
            contained, message = _placement_containment(
                layout, center, rotation, layout.block_half_extents,
            )
            bottom_gap = float(vertices[:, 2].min() - layout.box_inner_bottom_z)
            support = _support_force(snapshot, object_name, layout.box_name)
            payload["placement_checks"].append({
                "sim_time_ns": snapshot["sim_time_ns"],
                "vertices_world": vertices.tolist(),
                "contained": contained,
                "containment_message": message,
                "bottom_gap_m": bottom_gap,
                "box_support_force_n": support,
            })
            assert contained, (message, snapshot)
            assert support > 0.0, snapshot
            assert abs(bottom_gap) <= diagnostics.relative_position_tolerance_m, snapshot
            for side in ("left", "right"):
                assert not _finger_contacts(snapshot, side, object_name), snapshot
            assert abs(center - first_center).max() <= (
                diagnostics.relative_position_tolerance_m
            ), snapshot
            assert _rotation_angle(first_rotation, rotation) <= (
                diagnostics.relative_orientation_tolerance_rad
            ), snapshot
            assert max(abs(value) for value in block["linear_velocity"]) <= (
                diagnostics.max_object_linear_speed_m_s
            ), snapshot
            assert max(abs(value) for value in block["angular_velocity"]) <= (
                diagnostics.max_object_angular_speed_rad_s
            ), snapshot
        payload["result"] = {
            "initial_block_center": initial_block["center"],
            "left_lifted_block_center": _object(left_lifted[-1], object_name)["center"],
            "airborne_dual_hold_samples": len(dual_held),
            "right_after_transfer_hold_samples": len(right_held),
            "placement_samples": len(placed),
            "final_block_center": _object(placed[-1], object_name)["center"],
        }
        driver.close()
        status = "completed"
    except BaseException as error:
        payload["failure"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        if driver is not None:
            payload["windows"] = driver.windows
        try:
            exit_code = stack.close()
            payload["backend_exit_code"] = exit_code
            if status == "completed":
                assert exit_code == 0, stack.output()
        except BaseException:
            status = "failed"
            raise
        finally:
            _write_evidence(status, stack, payload)
