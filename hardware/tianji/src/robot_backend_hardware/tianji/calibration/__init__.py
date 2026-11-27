"""TianJi real-to-sim calibration and system-identification solvers."""
from __future__ import annotations

import importlib
from typing import Any

_EXPORTS: dict[str, tuple[str, str]] = {
    "UrdfArmChain": ("robot_backend_hardware.tianji.calibration.fk_reconcile", "UrdfArmChain"),
    "arm_joint_names": ("robot_backend_hardware.tianji.calibration.fk_reconcile", "arm_joint_names"),
    "reconcile_samples": ("robot_backend_hardware.tianji.calibration.fk_reconcile", "reconcile_samples"),
    "CaliperResult": ("robot_backend_hardware.tianji.calibration.gripper_caliper", "CaliperResult"),
    "compute_caliper_calibration": (
        "robot_backend_hardware.tianji.calibration.gripper_caliper",
        "compute_caliper_calibration",
    ),
    "probe_gripper_touch": (
        "robot_backend_hardware.tianji.calibration.gripper_caliper",
        "probe_gripper_touch",
    ),
    "CharucoBoardSpec": ("robot_backend_hardware.tianji.calibration.handeye_charuco", "CharucoBoardSpec"),
    "board_corner_points": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "board_corner_points",
    ),
    "create_charuco_board": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "create_charuco_board",
    ),
    "detect_charuco_pose": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "detect_charuco_pose",
    ),
    "generate_board_image": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "generate_board_image",
    ),
    "mujoco_camera_pose": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "mujoco_camera_pose",
    ),
    "opencv_to_mujoco_rotation": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "opencv_to_mujoco_rotation",
    ),
    "rotation_matrix_to_wxyz_quat": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "rotation_matrix_to_wxyz_quat",
    ),
    "solve_captures_document": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "solve_captures_document",
    ),
    "solve_eye_in_hand": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "solve_eye_in_hand",
    ),
    "solve_eye_to_hand_from_board": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "solve_eye_to_hand_from_board",
    ),
    "solve_table_board_pose_from_touches": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "solve_table_board_pose_from_touches",
    ),
    "solve_tcp_offset_from_pivot_touches": (
        "robot_backend_hardware.tianji.calibration.handeye_charuco",
        "solve_tcp_offset_from_pivot_touches",
    ),
    "SysIdResult": ("robot_backend_hardware.tianji.calibration.sysid", "SysIdResult"),
    "fit_servo_dynamics": ("robot_backend_hardware.tianji.calibration.sysid", "fit_servo_dynamics"),
}

__all__ = list(_EXPORTS)


def __getattr__(name: str) -> Any:
    target = _EXPORTS.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attr_name = target
    value = getattr(importlib.import_module(module_name), attr_name)
    globals()[name] = value
    return value
