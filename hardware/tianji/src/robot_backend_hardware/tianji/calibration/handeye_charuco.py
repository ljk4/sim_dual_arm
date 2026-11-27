"""ChArUco board hand-eye, table-plane, and TCP calibration solvers."""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np

from robot_backend_hardware.tianji.calibration.fk_reconcile import (
    DEFAULT_URDF_PATH,
    UrdfArmChain,
    arm_joint_names,
)


DEFAULT_OUTPUT_PATH = Path("/data/_system/debug/tianji_calibration/handeye_result.json")

_ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}


@dataclass(frozen=True)
class CharucoBoardSpec:
    """Physical geometry and dictionary of a printed ChArUco calibration board."""

    squares_x: int = 7
    squares_y: int = 5
    square_length_m: float = 0.030
    marker_length_m: float = 0.022
    dictionary_name: str = "DICT_5X5_100"
    board_thickness_m: float = 0.003


def create_charuco_board(spec: CharucoBoardSpec) -> cv2.aruco.CharucoBoard:
    if spec.dictionary_name not in _ARUCO_DICTS:
        raise ValueError(f"Unsupported ArUco dictionary: {spec.dictionary_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(_ARUCO_DICTS[spec.dictionary_name])
    return cv2.aruco.CharucoBoard(
        (int(spec.squares_x), int(spec.squares_y)),
        float(spec.square_length_m),
        float(spec.marker_length_m),
        dictionary,
    )


def board_corner_points(spec: CharucoBoardSpec) -> np.ndarray:
    """Return all inner ChArUco corner 3D coordinates (N, 3) in board frame."""
    board = create_charuco_board(spec)
    return np.asarray(board.getChessboardCorners(), dtype=np.float64).reshape(-1, 3)


def generate_board_image(
    spec: CharucoBoardSpec,
    output_path: str | Path,
    *,
    pixels_per_meter: int = 4000,
    margin_px: int = 40,
) -> Path:
    """Render a printable ChArUco board PNG at exact square/marker aspect ratio."""
    board = create_charuco_board(spec)
    width_px = max(200, int(round(spec.squares_x * spec.square_length_m * pixels_per_meter)))
    height_px = max(200, int(round(spec.squares_y * spec.square_length_m * pixels_per_meter)))
    img = board.generateImage((width_px, height_px), marginSize=margin_px)
    dest = Path(output_path)
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(dest), img):
        raise OSError(f"Failed to write ChArUco image to {dest}")
    return dest


def opencv_to_mujoco_rotation(R_parent_cam_cv: np.ndarray) -> np.ndarray:
    """Convert OpenCV camera (+X right, +Y down, +Z forward) to MuJoCo (+X right, +Y up, -Z forward)."""
    R_cv = np.asarray(R_parent_cam_cv, dtype=np.float64)
    if R_cv.shape != (3, 3):
        raise ValueError(f"Expected 3x3 rotation matrix, got {R_cv.shape}")
    return R_cv @ np.diag([1.0, -1.0, -1.0])


def rotation_matrix_to_wxyz_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 proper rotation matrix into a normalized (w, x, y, z) MuJoCo quaternion."""
    m = np.asarray(R, dtype=np.float64)
    if m.shape != (3, 3):
        raise ValueError(f"Expected 3x3 rotation matrix, got {m.shape}")
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    q /= np.linalg.norm(q)
    if q[0] < 0.0:
        q = -q
    return tuple(float(v) for v in q.tolist())  # type: ignore[return-value]


def mujoco_camera_pose(T_parent_cam_cv: np.ndarray) -> dict[str, list[float]]:
    """Convert T_{parent <- cam_cv} into MuJoCo YAML pos + wxyz quaternion."""
    T = np.asarray(T_parent_cam_cv, dtype=np.float64)
    R_mj = opencv_to_mujoco_rotation(T[:3, :3])
    return {
        "pos": [round(float(v), 6) for v in T[:3, 3].tolist()],
        "quat_wxyz": [round(float(v), 6) for v in rotation_matrix_to_wxyz_quat(R_mj)],
    }


def _project_so3(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(R)
    S = np.eye(3, dtype=np.float64)
    if np.linalg.det(U @ Vt) < 0.0:
        S[2, 2] = -1.0
    return U @ S @ Vt


def _average_transforms(transforms: Sequence[np.ndarray]) -> tuple[np.ndarray, float]:
    """Average SE(3) transforms cleanly and return (mean_T, pos_rms_spread_mm)."""
    if not transforms:
        raise ValueError("Cannot average empty transform sequence")
    mats = [np.asarray(T, dtype=np.float64) for T in transforms]
    t_mean = np.mean([T[:3, 3] for T in mats], axis=0)
    R_sum = np.sum([T[:3, :3] for T in mats], axis=0)
    R_mean = _project_so3(R_sum)
    T_mean = np.eye(4, dtype=np.float64)
    T_mean[:3, :3] = R_mean
    T_mean[:3, 3] = t_mean
    spread_mm = float(np.sqrt(np.mean([np.sum((T[:3, 3] - t_mean) ** 2) for T in mats])) * 1000.0)
    return T_mean, spread_mm


def detect_charuco_pose(
    image_rgb: np.ndarray,
    camera_matrix: np.ndarray | Sequence[Sequence[float]],
    dist_coeffs: np.ndarray | Sequence[float] | None,
    spec: CharucoBoardSpec,
    *,
    min_corners: int = 6,
) -> tuple[np.ndarray, float, int]:
    """Detect a ChArUco board in an RGB/grayscale image and return (T_cam_board, reproj_rms_px, n_corners)."""
    board = create_charuco_board(spec)
    detector = cv2.aruco.CharucoDetector(board)
    gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY) if image_rgb.ndim == 3 else image_rgb
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
    if charuco_corners is None or charuco_ids is None or len(charuco_ids) < min_corners:
        count = 0 if charuco_ids is None else len(charuco_ids)
        raise ValueError(f"Detected only {count} ChArUco corners (need >= {min_corners})")

    obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
    K = np.asarray(camera_matrix, dtype=np.float64)
    dist = np.zeros(5, dtype=np.float64) if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("cv2.solvePnP failed on ChArUco corners")

    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, K, dist)
    rms_px = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - img_points.reshape(-1, 2)) ** 2, axis=1))))

    R, _ = cv2.Rodrigues(rvec)
    T_cam_board = np.eye(4, dtype=np.float64)
    T_cam_board[:3, :3] = R
    T_cam_board[:3, 3] = tvec.reshape(3)
    return T_cam_board, rms_px, int(len(charuco_ids))


def solve_tcp_offset_from_pivot_touches(
    flange_poses_base: Sequence[np.ndarray],
) -> tuple[np.ndarray, np.ndarray, float]:
    """Solve flange-local TCP offset by pivoting around a fixed world contact point.

    For each pose i: R_i @ p_tcp_flange + t_i = p_world_pivot  =>  [R_i, -I] x = -t_i.
    Returns (p_tcp_flange_m, p_world_pivot_m, rms_residual_m).
    """
    if len(flange_poses_base) < 4:
        raise ValueError(f"Pivot TCP solve needs >= 4 diverse orientations, got {len(flange_poses_base)}")
    rows_A: list[np.ndarray] = []
    rows_b: list[np.ndarray] = []
    for T in flange_poses_base:
        mat = np.asarray(T, dtype=np.float64)
        rows_A.append(np.hstack([mat[:3, :3], -np.eye(3, dtype=np.float64)]))
        rows_b.append(-mat[:3, 3])
    A = np.vstack(rows_A)
    b = np.concatenate(rows_b)
    sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    p_tcp_flange = sol[:3]
    p_world = sol[3:6]
    residual = A @ sol - b
    rms_m = float(np.sqrt(np.mean(np.sum(residual.reshape(-1, 3) ** 2, axis=1))))
    return p_tcp_flange, p_world, rms_m


def solve_table_board_pose_from_touches(
    touch_flange_poses_base: Sequence[np.ndarray],
    board_points_m: Sequence[Sequence[float]],
    tcp_offset_flange_m: Sequence[float] | np.ndarray,
    *,
    board_thickness_m: float = 0.003,
) -> tuple[np.ndarray, float, float]:
    """Kabsch rigid alignment from robot touch poses to ChArUco board points.

    Computes world touch point p_base_i = R_i @ tcp_offset + t_i and aligns to p_board_i,
    returning (T_base_board, table_top_z_m, rms_fit_error_m).
    """
    if len(touch_flange_poses_base) < 3 or len(touch_flange_poses_base) != len(board_points_m):
        raise ValueError("Need >= 3 matching touch poses and board points")
    tcp = np.asarray(tcp_offset_flange_m, dtype=np.float64).reshape(3)
    pts_base = np.array(
        [np.asarray(T, dtype=np.float64)[:3, :3] @ tcp + np.asarray(T, dtype=np.float64)[:3, 3] for T in touch_flange_poses_base],
        dtype=np.float64,
    )
    pts_board = np.asarray(board_points_m, dtype=np.float64).reshape(-1, 3)

    c_base = np.mean(pts_base, axis=0)
    c_board = np.mean(pts_board, axis=0)
    P = pts_board - c_board
    Q = pts_base - c_base
    H = P.T @ Q
    U, _, Vt = np.linalg.svd(H)
    S = np.eye(3, dtype=np.float64)
    if np.linalg.det(Vt.T @ U.T) < 0.0:
        S[2, 2] = -1.0
    R_base_board = Vt.T @ S @ U.T
    t_base_board = c_base - R_base_board @ c_board

    T_base_board = np.eye(4, dtype=np.float64)
    T_base_board[:3, :3] = R_base_board
    T_base_board[:3, 3] = t_base_board

    reconstructed = (R_base_board @ pts_board.T).T + t_base_board
    rms_m = float(np.sqrt(np.mean(np.sum((reconstructed - pts_base) ** 2, axis=1))))
    table_top_z_m = float(np.mean(pts_base[:, 2]) - float(board_thickness_m))
    return T_base_board, table_top_z_m, rms_m


def solve_eye_to_hand_from_board(
    T_base_board: np.ndarray,
    T_cam_board_samples: Sequence[np.ndarray],
) -> tuple[np.ndarray, float]:
    """Compute fixed top-camera pose T_{base <- cam_cv} from stationary board observations."""
    T_bb = np.asarray(T_base_board, dtype=np.float64)
    candidates = [T_bb @ np.linalg.inv(np.asarray(T_cb, dtype=np.float64)) for T_cb in T_cam_board_samples]
    return _average_transforms(candidates)


def solve_eye_in_hand(
    T_base_gripper_samples: Sequence[np.ndarray],
    T_cam_board_samples: Sequence[np.ndarray],
) -> tuple[np.ndarray, float]:
    """Solve T_{gripper <- cam_cv} using OpenCV Tsai hand-eye calibration and verify board consistency."""
    if len(T_base_gripper_samples) < 3 or len(T_base_gripper_samples) != len(T_cam_board_samples):
        raise ValueError("Eye-in-hand calibration requires >= 3 matching pose pairs")
    R_g2b = [np.asarray(T, dtype=np.float64)[:3, :3] for T in T_base_gripper_samples]
    t_g2b = [np.asarray(T, dtype=np.float64)[:3, 3].reshape(3, 1) for T in T_base_gripper_samples]
    R_t2c = [np.asarray(T, dtype=np.float64)[:3, :3] for T in T_cam_board_samples]
    t_t2c = [np.asarray(T, dtype=np.float64)[:3, 3].reshape(3, 1) for T in T_cam_board_samples]

    R_c2g, t_c2g = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, method=cv2.CALIB_HAND_EYE_TSAI
    )
    T_gripper_cam = np.eye(4, dtype=np.float64)
    T_gripper_cam[:3, :3] = R_c2g
    T_gripper_cam[:3, 3] = t_c2g.reshape(3)

    board_poses = [
        np.asarray(T_bg, dtype=np.float64) @ T_gripper_cam @ np.asarray(T_cb, dtype=np.float64)
        for T_bg, T_cb in zip(T_base_gripper_samples, T_cam_board_samples, strict=True)
    ]
    _, spread_mm = _average_transforms(board_poses)
    return T_gripper_cam, spread_mm


def solve_captures_document(
    document: dict[str, Any],
    *,
    urdf_path: str | Path = DEFAULT_URDF_PATH,
) -> dict[str, Any]:
    """Solve all available hand-eye, table_top_z, and TCP terms from a capture JSON document."""
    spec = CharucoBoardSpec(**document.get("board", {}))
    all_corners = board_corner_points(spec)
    chain = UrdfArmChain(urdf_path)
    out: dict[str, Any] = {"board": document.get("board", {})}

    def fk_for(entry: dict[str, Any], side: str, target_link: str) -> np.ndarray:
        if "matrix" in entry:
            return np.asarray(entry["matrix"], dtype=np.float64)
        j_names = arm_joint_names(side)
        j_rad = {n: math.radians(float(d)) for n, d in zip(j_names, entry["joints_deg"], strict=True)}
        return chain.forward_kinematics(target_link, j_rad)

    tcp_solved_m: dict[str, list[float]] = {}
    for side in ("left", "right"):
        link7 = f"Arm_{'L' if side == 'left' else 'R'}7_Link"
        pivot_entries = document.get(f"pivot_touches_{side}", [])
        if pivot_entries:
            poses = [fk_for(e, side, link7) for e in pivot_entries]
            tcp, pivot_world, rms_m = solve_tcp_offset_from_pivot_touches(poses)
            tcp_solved_m[side] = [float(v) for v in tcp.tolist()]
            out[f"tcp_{side}"] = {
                "flange_link": link7,
                "tcp_offset_flange_m": tcp_solved_m[side],
                "pivot_world_m": [float(v) for v in pivot_world.tolist()],
                "rms_residual_mm": rms_m * 1000.0,
            }

    T_base_board = None
    board_touches = document.get("board_touches")
    if board_touches:
        side = board_touches.get("side", "right")
        link7 = f"Arm_{'L' if side == 'left' else 'R'}7_Link"
        tcp_vec = board_touches.get("tcp_offset_flange_m") or tcp_solved_m.get(side)
        if tcp_vec is None:
            raise ValueError("board_touches requires tcp_offset_flange_m or pivot_touches")
        touch_poses = [fk_for(e, side, link7) for e in board_touches["touches"]]
        if "board_points_m" in board_touches:
            pts_board = board_touches["board_points_m"]
        else:
            pts_board = [all_corners[int(cid)].tolist() for cid in board_touches["corner_ids"]]
        T_base_board, table_z, rms_m = solve_table_board_pose_from_touches(
            touch_poses, pts_board, tcp_vec, board_thickness_m=spec.board_thickness_m
        )
        out["table"] = {
            "top_z_m": round(table_z, 6),
            "rms_fit_mm": rms_m * 1000.0,
            "T_base_board": T_base_board.tolist(),
        }

    if "cam_high_samples" in document:
        if T_base_board is None and "T_base_board" in document:
            T_base_board = np.asarray(document["T_base_board"], dtype=np.float64)
        if T_base_board is None:
            raise ValueError("cam_high_samples requires solved or provided T_base_board")
        T_cb_list = [np.asarray(s["T_cam_board"], dtype=np.float64) for s in document["cam_high_samples"]]
        T_base_cam, spread_mm = solve_eye_to_hand_from_board(T_base_board, T_cb_list)
        mj = mujoco_camera_pose(T_base_cam)
        out["cam_high"] = {
            "parent": "ZJ_Robot_link",
            "spread_mm": spread_mm,
            "T_base_cam_cv": T_base_cam.tolist(),
            "mujoco": mj,
        }

    for side, key in (("left", "cam_left_wrist_samples"), ("right", "cam_right_wrist_samples")):
        if key not in document:
            continue
        gripper_link = f"Arm_{'L' if side == 'left' else 'R'}_gripper_base"
        pairs = document[key]
        T_bg_list = [fk_for(p, side, gripper_link) for p in pairs]
        T_cb_list = [np.asarray(p["T_cam_board"], dtype=np.float64) for p in pairs]
        T_gc, spread_mm = solve_eye_in_hand(T_bg_list, T_cb_list)
        mj = mujoco_camera_pose(T_gc)
        out[f"cam_{side}_wrist"] = {
            "parent": gripper_link,
            "consistency_spread_mm": spread_mm,
            "T_gripper_cam_cv": T_gc.tolist(),
            "mujoco": mj,
        }

    return out


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("/app/configs/devices.yaml"))
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF_PATH)
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-length-m", type=float, default=0.030)
    parser.add_argument("--marker-length-m", type=float, default=0.022)
    parser.add_argument("--board-thickness-m", type=float, default=0.003)
    parser.add_argument("--generate-board-png", type=Path, default=None)
    parser.add_argument("--solve-captures", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_PATH)
    args = parser.parse_args(argv)

    spec = CharucoBoardSpec(
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length_m=args.square_length_m,
        marker_length_m=args.marker_length_m,
        board_thickness_m=args.board_thickness_m,
    )
    print(
        f"[calibrate_handeye] config={args.config} urdf={args.urdf} "
        f"board={spec.squares_x}x{spec.squares_y} ({spec.square_length_m*1000:.1f}mm sq) "
        f"output={args.output}"
    )

    if args.generate_board_png is not None:
        dest = generate_board_image(spec, args.generate_board_png)
        print(f"[calibrate_handeye] generated board PNG: {dest}")
        if args.solve_captures is None:
            return 0

    if args.solve_captures is None:
        raise SystemExit("--solve-captures or --generate-board-png is required")

    doc = json.loads(args.solve_captures.read_text(encoding="utf-8"))
    result = solve_captures_document(doc, urdf_path=args.urdf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(f"[calibrate_handeye] saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
