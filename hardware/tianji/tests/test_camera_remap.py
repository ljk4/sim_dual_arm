"""Tests for camera undistortion and MuJoCo pinhole FOV remapping."""
from __future__ import annotations

import math
import os
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from robot_backend_hardware.common.camera.capture import CameraConfig, FFmpegV4L2Stream
from robot_backend_hardware.common.camera.remap import (
    PinholeRemapConfig,
    PinholeRemapper,
    build_pinhole_matrix,
)
from robot_backend_hardware.tianji.calibration.handeye_charuco import (
    CharucoBoardSpec,
    detect_charuco_pose,
)


def test_pinhole_matrix_matches_mujoco_convention() -> None:
    K = build_pinhole_matrix(320, 240, 55.0)
    expected_fy = 240.0 / (2.0 * math.tan(math.radians(55.0) / 2.0))
    assert K.shape == (3, 3)
    assert K[0, 0] == pytest.approx(expected_fy)
    assert K[1, 1] == pytest.approx(expected_fy)
    assert K[0, 2] == pytest.approx(160.0)
    assert K[1, 2] == pytest.approx(120.0)


def test_undistort_and_fov_remap_restores_ideal_pinhole() -> None:
    w_src, h_src = 640, 480
    K_src = np.array(
        [
            [610.0, 0.0, 326.0],
            [0.0, 612.0, 234.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    dist = np.array([-0.15, 0.04, 0.001, -0.001, 0.0], dtype=np.float64)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard((7, 5), 0.030, 0.022, dictionary)
    board_px_w, board_px_h = 1400, 1000
    board_img = board.generateImage((board_px_w, board_px_h), marginSize=0)
    board_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)

    # Known rigid board pose in front of camera centered slightly off-axis
    rvec = np.array([[0.15], [-0.10], [0.05]], dtype=np.float64)
    tvec = np.array([[-0.105], [-0.075], [0.42]], dtype=np.float64)
    R_cb, _ = cv2.Rodrigues(rvec)

    # Render exact distorted 640x480 view by tracing rays through (K_src, dist) onto board z=0 plane
    uu, vv = np.meshgrid(np.arange(w_src, dtype=np.float64), np.arange(h_src, dtype=np.float64))
    pts_dist = np.stack([uu.ravel(), vv.ravel()], axis=1).reshape(-1, 1, 2)
    pts_norm = cv2.undistortPoints(pts_dist, K_src, dist).reshape(-1, 2)
    rays_cam = np.column_stack([pts_norm, np.ones(len(pts_norm), dtype=np.float64)])
    rays_board = (R_cb.T @ rays_cam.T).T
    origin_board = -(R_cb.T @ tvec).reshape(3)
    lam = -origin_board[2] / rays_board[:, 2]
    hit_board = origin_board[None, :] + lam[:, None] * rays_board
    px_per_m = board_px_w / (7 * 0.030)
    map_x = (hit_board[:, 0] * px_per_m).reshape(h_src, w_src).astype(np.float32)
    map_y = (hit_board[:, 1] * px_per_m).reshape(h_src, w_src).astype(np.float32)
    distorted_640 = cv2.remap(
        board_rgb,
        map_x,
        map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(220, 220, 220),
    )

    # Verify exact analytical undistortion + pinhole resampling math (< 0.05 px)
    remapper = PinholeRemapper(
        K_src,
        dist,
        target_width=320,
        target_height=240,
        target_fovy_deg=55.0,
    )
    map_x_32, map_y_32 = cv2.initUndistortRectifyMap(
        K_src,
        dist,
        np.eye(3, dtype=np.float64),
        remapper.target_camera_matrix,
        (320, 240),
        cv2.CV_32FC1,
    )
    pts_3d = np.array([[-0.08, -0.05, 0.42], [0.06, 0.04, 0.42], [0.0, 0.0, 0.42]])
    uv_sim, _ = cv2.projectPoints(pts_3d, np.zeros(3), np.zeros(3), remapper.target_camera_matrix, np.zeros(5))
    uv_dist, _ = cv2.projectPoints(pts_3d, np.zeros(3), np.zeros(3), K_src, dist)
    for (us, vs), (ud, vd) in zip(uv_sim.reshape(-1, 2), uv_dist.reshape(-1, 2), strict=True):
        # Bilinear sample of the float map at (us, vs) matches exact forward-projected distorted pixel
        u0, v0 = int(us), int(vs)
        du, dv = float(us - u0), float(vs - v0)
        ux = (
            (1 - dv) * ((1 - du) * map_x_32[v0, u0] + du * map_x_32[v0, u0 + 1])
            + dv * ((1 - du) * map_x_32[v0 + 1, u0] + du * map_x_32[v0 + 1, u0 + 1])
        )
        vy = (
            (1 - dv) * ((1 - du) * map_y_32[v0, u0] + du * map_y_32[v0, u0 + 1])
            + dv * ((1 - du) * map_y_32[v0 + 1, u0] + du * map_y_32[v0 + 1, u0 + 1])
        )
        assert math.hypot(ux - ud, vy - vd) < 0.05

    remapped = remapper.remap(distorted_640)
    assert remapped.shape == (240, 320, 3)

    det = cv2.aruco.CharucoDetector(board)
    cc, ci, _, _ = det.detectBoard(cv2.cvtColor(remapped, cv2.COLOR_RGB2GRAY))
    assert cc is not None and ci is not None and len(ci) >= 12

    obj_pts, img_pts = board.matchImagePoints(cc, ci)
    ideal_img_pts, _ = cv2.projectPoints(
        obj_pts, rvec, tvec, remapper.target_camera_matrix, np.zeros(5)
    )
    err_px = np.linalg.norm(img_pts.reshape(-1, 2) - ideal_img_pts.reshape(-1, 2), axis=1)
    assert float(np.mean(err_px)) < 0.75

    T_cb_est, _, _ = detect_charuco_pose(
        remapped, remapper.target_camera_matrix, None, CharucoBoardSpec()
    )
    delta_mm = np.abs(T_cb_est[:3, 3] - tvec.reshape(3)) * 1000.0
    assert float(delta_mm[0]) < 1.5 and float(delta_mm[1]) < 1.5
    assert float(np.linalg.norm(delta_mm)) < 5.0

    intr = remapper.describe_intrinsics(
        {
            "width": 640,
            "height": 480,
            "camera_matrix": K_src.tolist(),
            "distortion_coefficients": dist.tolist(),
        }
    )
    assert intr["width"] == 320
    assert intr["height"] == 240
    assert intr["calibration_status"] == "pinhole_remapped"
    assert intr["distortion_coefficients"] == [0.0, 0.0, 0.0, 0.0, 0.0]


def test_ffmpeg_stream_applies_pinhole_remap() -> None:
    cfg = CameraConfig(
        name="wrist_test",
        backend="ffmpeg_v4l2",
        width=4,
        height=4,
        intrinsics={
            "camera_matrix": [[4.0, 0.0, 2.0], [0.0, 4.0, 2.0], [0.0, 0.0, 1.0]],
            "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
        },
        remap=PinholeRemapConfig(
            enabled=True,
            target_width=2,
            target_height=2,
            target_fovy_deg=60.0,
        ),
    )
    stream = FFmpegV4L2Stream(cfg)
    stream._running = True
    read_fd, write_fd = os.pipe()
    os.write(write_fd, bytes([128] * (4 * 4 * 3)))
    os.close(write_fd)
    with os.fdopen(read_fd, "rb", buffering=0) as pipe:
        stream._read_frames(SimpleNamespace(stdout=pipe, poll=lambda: None))

    frame = stream.get_frame()
    assert frame is not None
    assert frame.shape == (2, 2, 3)
    intr = stream.get_intrinsics()
    assert intr["width"] == 2 and intr["height"] == 2
    assert intr["calibration_status"] == "pinhole_remapped"
