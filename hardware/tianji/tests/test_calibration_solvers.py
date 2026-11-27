"""Offline tests for FK reconciliation, gripper caliper, ChArUco hand-eye, and Sys-ID."""
from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from robot_backend_hardware.common.gripper import GripperCommand, GripperState, TwoFingerGripper
from robot_backend_hardware.tianji.arms import _quat_to_euler_xyz
from robot_backend_hardware.tianji.calibration.fk_reconcile import (
    UrdfArmChain,
    arm_joint_names,
    reconcile_samples,
)
from robot_backend_hardware.tianji.calibration.gripper_caliper import (
    compute_caliper_calibration,
    probe_gripper_touch,
)
from robot_backend_hardware.tianji.calibration.handeye_charuco import (
    CharucoBoardSpec,
    board_corner_points,
    create_charuco_board,
    detect_charuco_pose,
    mujoco_camera_pose,
    opencv_to_mujoco_rotation,
    solve_eye_in_hand,
    solve_eye_to_hand_from_board,
    solve_table_board_pose_from_touches,
    solve_tcp_offset_from_pivot_touches,
)
from robot_backend_hardware.tianji.calibration.sysid import fit_servo_dynamics


REPO_ROOT = Path(__file__).resolve().parents[3]
URDF_PATH = REPO_ROOT / "shared/robots/marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf"


def _matrix_to_apex_pose(T: np.ndarray) -> dict[str, float]:
    R = T[:3, :3]
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    rx, ry, rz = _quat_to_euler_xyz(qx, qy, qz, qw)
    return {
        "x": float(T[0, 3]),
        "y": float(T[1, 3]),
        "z": float(T[2, 3]),
        "rx": math.degrees(rx),
        "ry": math.degrees(ry),
        "rz": math.degrees(rz),
    }


def _wxyz_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def test_urdf_fk_and_reconcile_passes_and_flags_mismatch() -> None:
    chain = UrdfArmChain(URDF_PATH)
    for side, link in (("left", "Arm_L7_Link"), ("right", "Arm_R7_Link")):
        names = arm_joint_names(side)
        samples = []
        for degs in (
            [0.0, -45.0, 15.0, -70.0, 20.0, 10.0, -15.0],
            [25.0, -50.0, -20.0, -85.0, -15.0, 25.0, 30.0],
        ):
            T = chain.forward_kinematics(link, {n: math.radians(d) for n, d in zip(names, degs)})
            samples.append({"joints_deg": degs, "apex_pose": _matrix_to_apex_pose(T)})

        ok_res = reconcile_samples(chain, side, samples, target_link=link)
        assert ok_res["passed"] is True
        assert ok_res["max_pos_err_mm"] < 1e-6
        assert ok_res["max_rot_err_deg"] < 1e-6

        bad_samples = [
            {
                "joints_deg": samples[0]["joints_deg"],
                "apex_pose": {**samples[0]["apex_pose"], "z": samples[0]["apex_pose"]["z"] + 0.002},
            }
        ]
        bad_res = reconcile_samples(chain, side, bad_samples, target_link=link)
        assert bad_res["passed"] is False
        assert bad_res["max_pos_err_mm"] == pytest.approx(2.0, abs=1e-3)


class _FakeCaliperGripper(TwoFingerGripper):
    def __init__(self, touch_mm: float) -> None:
        super().__init__(name="fake_90c", max_stroke_mm=90.0)
        self._touch_mm = touch_mm
        self._pos_mm = 90.0
        self._holding = False
        self.commands: list[GripperCommand] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def is_healthy(self) -> bool:
        return True

    def get(self) -> GripperState:
        return GripperState(
            position_mm=self._pos_mm,
            speed=0.0,
            current=1.0 if self._holding else 0.0,
            is_holding=self._holding,
            is_moving=False,
            valid=True,
        )

    def set(self, cmd: GripperCommand) -> None:
        self.commands.append(cmd)
        if cmd.position_mm <= self._touch_mm:
            self._pos_mm = self._touch_mm
            self._holding = True
        else:
            self._pos_mm = cmd.position_mm
            self._holding = False


def test_gripper_caliper_math_and_safe_probe() -> None:
    fake = _FakeCaliperGripper(touch_mm=4.4)
    measured_empty = probe_gripper_touch(fake, speed_pct=10.0, force_pct=15.0, reopen_mm=85.0)
    assert measured_empty == pytest.approx(4.4)
    assert fake.commands[-1].position_mm == pytest.approx(85.0)

    with pytest.raises(ValueError, match="speed_pct"):
        probe_gripper_touch(fake, speed_pct=50.0, force_pct=15.0)
    with pytest.raises(ValueError, match="force_pct"):
        probe_gripper_touch(fake, speed_pct=10.0, force_pct=60.0)

    cal = compute_caliper_calibration(
        "right",
        measured_empty,
        {"red_cube": 44.6, "blue_cube": 54.4},
    )
    assert cal.pad_inward_offset_m == pytest.approx(0.0022)
    assert cal.measured_widths_m["red_cube"] == pytest.approx(0.0402)
    assert cal.measured_widths_m["blue_cube"] == pytest.approx(0.0500)


def _se3(rvec: tuple[float, float, float], tvec: tuple[float, float, float]) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T


def _render_charuco_view(
    spec: CharucoBoardSpec,
    K: np.ndarray,
    width: int,
    height: int,
    T_cam_board: np.ndarray,
) -> np.ndarray:
    board = create_charuco_board(spec)
    board_px_w, board_px_h = 1050, 750
    board_img = board.generateImage((board_px_w, board_px_h), marginSize=0)
    px_per_m = board_px_w / (spec.squares_x * spec.square_length_m)

    R_cb = T_cam_board[:3, :3]
    t_cb = T_cam_board[:3, 3]
    # 3x3 planar homography mapping board meter coordinates (X_b, Y_b, 1) -> pixel (u, v, 1)
    H_px_to_board = np.diag([1.0 / px_per_m, 1.0 / px_per_m, 1.0])
    H_board_to_cam_px = K @ np.column_stack([R_cb[:, 0], R_cb[:, 1], t_cb])
    H = H_board_to_cam_px @ H_px_to_board
    warped = cv2.warpPerspective(board_img, H, (width, height), borderValue=220)
    return cv2.cvtColor(warped, cv2.COLOR_GRAY2RGB)


def test_charuco_handeye_tcp_and_table_solvers() -> None:
    spec = CharucoBoardSpec()
    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)

    # 1. Detect synthetic ChArUco board pose
    T_cb_gt = _se3((0.18, -0.12, 0.25), (-0.11, -0.07, 0.45))
    img = _render_charuco_view(spec, K, 640, 480, T_cb_gt)
    T_cb_est, rms_px, n_corners = detect_charuco_pose(img, K, None, spec)
    assert n_corners >= 12
    assert rms_px < 0.8
    assert np.linalg.norm(T_cb_est[:3, 3] - T_cb_gt[:3, 3]) * 1000.0 < 1.5

    # 2. Pivot TCP calibration
    tcp_gt = np.array([0.0, -0.236, 0.002], dtype=np.float64)
    world_pivot_gt = np.array([0.52, -0.18, 0.81], dtype=np.float64)
    pivot_poses = []
    for rvec in (
        (0.0, 0.0, 0.0),
        (0.35, -0.20, 0.15),
        (-0.30, 0.25, -0.25),
        (0.20, 0.35, -0.40),
        (-0.25, -0.30, 0.35),
    ):
        R_i, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64))
        t_i = world_pivot_gt - R_i @ tcp_gt
        T_i = np.eye(4, dtype=np.float64)
        T_i[:3, :3] = R_i
        T_i[:3, 3] = t_i
        pivot_poses.append(T_i)
    tcp_est, pivot_est, rms_m = solve_tcp_offset_from_pivot_touches(pivot_poses)
    assert rms_m < 1e-6
    assert np.allclose(tcp_est, tcp_gt, atol=1e-5)
    assert np.allclose(pivot_est, world_pivot_gt, atol=1e-5)

    # 3. Table ChArUco pose from 4 touched corners
    T_base_board_gt = _se3((0.0, 0.0, 0.22), (0.45, -0.15, 0.783))
    corners = board_corner_points(spec)
    corner_pts = [corners[0], corners[5], corners[18], corners[23]]
    touch_poses = []
    for pt_b in corner_pts:
        p_world = (T_base_board_gt[:3, :3] @ pt_b) + T_base_board_gt[:3, 3]
        T_flange = _se3((0.1, -0.2, 0.3), tuple((p_world - _se3((0.1, -0.2, 0.3), (0, 0, 0))[:3, :3] @ tcp_gt).tolist()))
        touch_poses.append(T_flange)
    T_bb_est, top_z, fit_rms = solve_table_board_pose_from_touches(
        touch_poses, corner_pts, tcp_gt, board_thickness_m=0.003
    )
    assert fit_rms < 1e-6
    assert top_z == pytest.approx(0.780, abs=1e-6)
    assert np.allclose(T_bb_est, T_base_board_gt, atol=1e-5)

    # 4. Eye-to-hand top camera
    T_base_cam_gt = _se3((2.1, 0.05, -1.55), (0.04, 0.0, 1.46))
    T_cb_top = np.linalg.inv(T_base_cam_gt) @ T_base_board_gt
    T_base_cam_est, spread_mm = solve_eye_to_hand_from_board(T_base_board_gt, [T_cb_top, T_cb_top])
    assert spread_mm < 1e-6
    assert np.allclose(T_base_cam_est, T_base_cam_gt, atol=1e-5)

    # 5. Eye-in-hand wrist camera across varied 6-DOF gripper poses
    T_gc_gt = _se3((-0.15, 0.02, 0.04), (0.05, -0.02, -0.08))
    T_bg_list = [
        _se3(rv, tv)
        for rv, tv in (
            ((0.4, -0.2, 0.1), (0.35, -0.25, 1.15)),
            ((-0.5, 0.4, -0.3), (0.48, -0.10, 1.10)),
            ((0.1, 0.6, -0.6), (0.40, -0.30, 1.22)),
            ((-0.3, -0.5, 0.7), (0.55, -0.18, 1.08)),
            ((0.7, 0.1, -0.4), (0.32, -0.05, 1.18)),
            ((-0.2, 0.7, 0.5), (0.52, -0.22, 1.25)),
        )
    ]
    T_cb_wrist = [np.linalg.inv(T_bg @ T_gc_gt) @ T_base_board_gt for T_bg in T_bg_list]
    T_gc_est, wrist_spread_mm = solve_eye_in_hand(T_bg_list, T_cb_wrist)
    assert wrist_spread_mm < 1.0
    assert np.linalg.norm(T_gc_est[:3, 3] - T_gc_gt[:3, 3]) * 1000.0 < 1.0
    mj = mujoco_camera_pose(T_gc_est)
    R_mj_expected = opencv_to_mujoco_rotation(T_gc_est[:3, :3])
    R_mj_roundtrip = _wxyz_to_matrix(*mj["quat_wxyz"])
    assert np.allclose(R_mj_roundtrip, R_mj_expected, atol=1e-5)


def test_sysid_identifies_delay_step_clamp_and_gripper_deadband() -> None:
    dt = 0.02
    n = 150
    t = np.arange(n, dtype=np.float64) * dt
    q_cmd = np.zeros((n, 7), dtype=np.float64)
    # Sine wave with a brief fast step in joint 0 + sine in joint 5 & 6
    q_cmd[:, 0] = 0.20 * np.sin(2.0 * math.pi * 0.4 * t)
    q_cmd[60:, 0] += 0.30
    q_cmd[:, 5] = 0.10 * np.sin(2.0 * math.pi * 0.5 * t)

    # Simulate exact 2-step delay + 0.04 rad/step clamp
    q_fdb = np.zeros_like(q_cmd)
    queue = [q_cmd[0].copy() for _ in range(2)]
    cur = q_cmd[0].copy()
    for i in range(n):
        delayed = queue.pop(0)
        queue.append(q_cmd[i].copy())
        delta = np.clip(delayed - cur, -0.04, 0.04)
        cur = cur + delta
        q_fdb[i] = cur

    # Gripper with 3-step delay + 0.0004 m deadband
    g_cmd = np.full(n, 0.060, dtype=np.float64)
    g_cmd[20:40] = 0.0602  # within 0.0004 deadband -> should stay at 0.060
    g_cmd[40:65] = 0.0608  # 0.6/0.8 mm move -> clears deadband
    g_cmd[65:105] = 0.055  # 5.8 mm move -> should move after 3 steps
    g_cmd[105:] = 0.075    # 20 mm move -> should move after 3 steps
    g_fdb = np.full(n, 0.060, dtype=np.float64)
    g_queue = [0.060, 0.060, 0.060]
    latched = 0.060
    for i in range(n):
        g_queue.append(float(g_cmd[i]))
        delayed_g = g_queue.pop(0)
        if abs(delayed_g - latched) >= 0.0004:
            latched = delayed_g
        g_fdb[i] = latched

    res = fit_servo_dynamics(
        t,
        q_cmd,
        q_fdb,
        control_dt_s=dt,
        gripper_timestamps_s=t,
        g_cmd_m=g_cmd,
        g_fdb_m=g_fdb,
    )
    assert res.arm_delay_steps == 2
    assert res.gripper_delay_steps == 3
    assert 0.0002 <= res.gripper_deadband_m <= 0.0006
    assert res.max_joint_step_rad == pytest.approx(0.04, abs=0.008)
    assert "arm_delay_steps: 2" in res.servo_dynamics_yaml
