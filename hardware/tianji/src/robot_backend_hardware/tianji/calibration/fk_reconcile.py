"""Forward-kinematics reconciliation between Marvin URDF and APEX controller."""
from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from robot_backend_hardware.tianji.arms import _euler_xyz_to_quat
from robot_backend_hardware.tianji.config import load_config


DEFAULT_URDF_PATH = Path("/shared/robots/marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf")
DEFAULT_OUTPUT_PATH = Path("/data/_system/debug/tianji_calibration/fk_reconcile.json")
POS_PASS_THRESHOLD_MM = 0.5
ROT_PASS_THRESHOLD_DEG = 0.05


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert URDF origin rpy=(roll, pitch, yaw) to 3x3 rotation matrix Rz(yaw)@Ry(pitch)@Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _axis_angle_to_matrix(axis: Sequence[float], angle_rad: float) -> np.ndarray:
    """Rodrigues formula around a unit axis vector."""
    ax = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(ax))
    if norm <= 0.0 or angle_rad == 0.0:
        return np.eye(3, dtype=np.float64)
    kx, ky, kz = (ax / norm).tolist()
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    v = 1.0 - c
    return np.array(
        [
            [kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s],
            [kx * ky * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s],
            [kx * kz * v - ky * s, ky * kz * v + kx * s, kz * kz * v + c],
        ],
        dtype=np.float64,
    )


def _pose_to_matrix(x: float, y: float, z: float, rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """Convert APEX pose (m, extrinsic XYZ deg) into a 4x4 homogeneous matrix."""
    qx, qy, qz, qw = _euler_xyz_to_quat(
        math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
    )
    T = np.eye(4, dtype=np.float64)
    T[0, 0] = 1.0 - 2.0 * (qy * qy + qz * qz)
    T[0, 1] = 2.0 * (qx * qy - qz * qw)
    T[0, 2] = 2.0 * (qx * qz + qy * qw)
    T[1, 0] = 2.0 * (qx * qy + qz * qw)
    T[1, 1] = 1.0 - 2.0 * (qx * qx + qz * qz)
    T[1, 2] = 2.0 * (qy * qz - qx * qw)
    T[2, 0] = 2.0 * (qx * qz - qy * qw)
    T[2, 1] = 2.0 * (qy * qz + qx * qw)
    T[2, 2] = 1.0 - 2.0 * (qx * qx + qy * qy)
    T[:3, 3] = [float(x), float(y), float(z)]
    return T


def _geodesic_angle_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    trace = float(np.trace(R1.T @ R2))
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return math.degrees(math.acos(cos_theta))


@dataclass(frozen=True)
class _UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


class UrdfArmChain:
    """Pure-NumPy URDF kinematic chain evaluator."""

    def __init__(self, urdf_path: str | Path) -> None:
        self.urdf_path = Path(urdf_path)
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        self._joints_by_child: dict[str, _UrdfJoint] = {}
        links: set[str] = set()
        children: set[str] = set()
        for link_el in root.findall("link"):
            name = link_el.attrib.get("name", "")
            if name:
                links.add(name)
        for joint_el in root.findall("joint"):
            name = joint_el.attrib["name"]
            joint_type = joint_el.attrib.get("type", "fixed")
            parent = joint_el.find("parent").attrib["link"]
            child = joint_el.find("child").attrib["link"]
            origin_el = joint_el.find("origin")
            xyz_str = origin_el.attrib.get("xyz", "0 0 0") if origin_el is not None else "0 0 0"
            rpy_str = origin_el.attrib.get("rpy", "0 0 0") if origin_el is not None else "0 0 0"
            axis_el = joint_el.find("axis")
            axis_str = axis_el.attrib.get("xyz", "0 0 1") if axis_el is not None else "0 0 1"
            xyz = tuple(float(v) for v in xyz_str.split())
            rpy = tuple(float(v) for v in rpy_str.split())
            axis = tuple(float(v) for v in axis_str.split())
            self._joints_by_child[child] = _UrdfJoint(
                name=name,
                joint_type=joint_type,
                parent=parent,
                child=child,
                origin_xyz=xyz,  # type: ignore[arg-type]
                origin_rpy=rpy,  # type: ignore[arg-type]
                axis=axis,  # type: ignore[arg-type]
            )
            links.update((parent, child))
            children.add(child)
        roots = sorted(links - children)
        if not roots:
            raise ValueError(f"Could not determine root link in URDF: {self.urdf_path}")
        self.root_link = roots[0]

    def chain_to(self, target_link: str, base_link: str | None = None) -> list[_UrdfJoint]:
        """Return ordered joints from base_link (default URDF root) to target_link."""
        stop = base_link or self.root_link
        chain: list[_UrdfJoint] = []
        cursor = target_link
        while cursor != stop:
            joint = self._joints_by_child.get(cursor)
            if joint is None:
                raise KeyError(f"No URDF path from {stop!r} to {target_link!r} (stopped at {cursor!r})")
            chain.append(joint)
            cursor = joint.parent
        chain.reverse()
        return chain

    def forward_kinematics(
        self,
        target_link: str,
        joint_values_rad: Mapping[str, float],
        *,
        base_link: str | None = None,
    ) -> np.ndarray:
        """Compute 4x4 transform T_{base_link <- target_link} at given joint positions (rad/m)."""
        T = np.eye(4, dtype=np.float64)
        for joint in self.chain_to(target_link, base_link=base_link):
            T_origin = np.eye(4, dtype=np.float64)
            T_origin[:3, :3] = _rpy_to_matrix(*joint.origin_rpy)
            T_origin[:3, 3] = joint.origin_xyz
            T = T @ T_origin

            val = float(joint_values_rad.get(joint.name, 0.0))
            if joint.joint_type in ("revolute", "continuous"):
                T_motion = np.eye(4, dtype=np.float64)
                T_motion[:3, :3] = _axis_angle_to_matrix(joint.axis, val)
                T = T @ T_motion
            elif joint.joint_type == "prismatic":
                ax = np.asarray(joint.axis, dtype=np.float64)
                norm = float(np.linalg.norm(ax))
                if norm > 0.0:
                    T_motion = np.eye(4, dtype=np.float64)
                    T_motion[:3, 3] = (ax / norm) * val
                    T = T @ T_motion
        return T


def arm_joint_names(side: str) -> tuple[str, ...]:
    prefix = "Arm_L" if side == "left" else "Arm_R"
    return tuple(f"{prefix}{i}_Joint" for i in range(1, 8))


def reconcile_samples(
    chain: UrdfArmChain,
    side: str,
    samples: Sequence[Mapping[str, Any]],
    *,
    target_link: str | None = None,
    base_link: str | None = None,
) -> dict[str, Any]:
    """Compare URDF FK vs APEX FK poses across joint configurations."""
    if not samples:
        raise ValueError("FK reconciliation requires at least one sample")
    if side not in ("left", "right"):
        raise ValueError(f"Unsupported arm side {side!r}; expected 'left' or 'right'")

    resolved_target = target_link or ("Arm_L7_Link" if side == "left" else "Arm_R7_Link")
    j_names = arm_joint_names(side)

    pos_errors_mm: list[float] = []
    rot_errors_deg: list[float] = []
    T_urdf_list: list[np.ndarray] = []
    T_apex_list: list[np.ndarray] = []

    for sample in samples:
        joints_deg = [float(v) for v in sample["joints_deg"]]
        if len(joints_deg) != 7:
            raise ValueError(f"Expected 7 joint angles in degrees, got {len(joints_deg)}")
        joint_map = {name: math.radians(deg) for name, deg in zip(j_names, joints_deg, strict=True)}
        T_urdf = chain.forward_kinematics(resolved_target, joint_map, base_link=base_link)
        pose = sample["apex_pose"]
        T_apex = _pose_to_matrix(
            float(pose["x"]),
            float(pose["y"]),
            float(pose["z"]),
            float(pose["rx"]),
            float(pose["ry"]),
            float(pose["rz"]),
        )
        pos_err_mm = float(np.linalg.norm(T_urdf[:3, 3] - T_apex[:3, 3]) * 1000.0)
        rot_err_deg = float(_geodesic_angle_deg(T_urdf[:3, :3], T_apex[:3, :3]))
        pos_errors_mm.append(pos_err_mm)
        rot_errors_deg.append(rot_err_deg)
        T_urdf_list.append(T_urdf)
        T_apex_list.append(T_apex)

    # Check common base-frame transforms if direct frame match exceeds threshold
    candidate_bases = {
        "root_direct": np.eye(4, dtype=np.float64),
        "arm_base_1": np.linalg.inv(chain.forward_kinematics(f"{'Arm_L1' if side == 'left' else 'Arm_R1'}_Link", {})),
    }
    for yaw_deg in (90, -90, 180):
        T_z = np.eye(4, dtype=np.float64)
        T_z[:3, :3] = _rpy_to_matrix(0.0, 0.0, math.radians(yaw_deg))
        candidate_bases[f"yaw_{yaw_deg}deg"] = T_z

    candidate_summary: dict[str, dict[str, float]] = {}
    for label, T_conv in candidate_bases.items():
        c_pos = [float(np.linalg.norm((T_conv @ Tu)[:3, 3] - Ta[:3, 3]) * 1000.0) for Tu, Ta in zip(T_urdf_list, T_apex_list, strict=True)]
        c_rot = [float(_geodesic_angle_deg((T_conv @ Tu)[:3, :3], Ta[:3, :3])) for Tu, Ta in zip(T_urdf_list, T_apex_list, strict=True)]
        candidate_summary[label] = {
            "max_pos_err_mm": max(c_pos),
            "max_rot_err_deg": max(c_rot),
        }

    max_pos_mm = max(pos_errors_mm)
    max_rot_deg = max(rot_errors_deg)
    best_label = min(
        candidate_summary,
        key=lambda k: (candidate_summary[k]["max_pos_err_mm"], candidate_summary[k]["max_rot_err_deg"]),
    )
    return {
        "side": side,
        "target_link": resolved_target,
        "base_link": base_link or chain.root_link,
        "sample_count": len(samples),
        "max_pos_err_mm": max_pos_mm,
        "mean_pos_err_mm": float(np.mean(pos_errors_mm)),
        "max_rot_err_deg": max_rot_deg,
        "mean_rot_err_deg": float(np.mean(rot_errors_deg)),
        "passed": max_pos_mm < POS_PASS_THRESHOLD_MM and max_rot_deg < ROT_PASS_THRESHOLD_DEG,
        "best_candidate_frame": best_label,
        "candidate_frames": candidate_summary,
        "per_sample_pos_err_mm": pos_errors_mm,
        "per_sample_rot_err_deg": rot_errors_deg,
    }


def _collect_live_samples(host: str, left_arm: str, right_arm: str, *, num_samples: int = 1) -> dict[str, list[dict[str, Any]]]:
    """Read current joint states and APEX FK poses without enabling or moving arms."""
    from robot_backend_hardware.tianji.apex_sdk import ApexClient

    collected: dict[str, list[dict[str, Any]]] = {"left": [], "right": []}
    with ApexClient(host) as client:
        for side, arm_name in (("left", left_arm), ("right", right_arm)):
            for _ in range(num_samples):
                joints_deg = client.get_joint_positions(arm_name, timeout=2.0)
                fk_resp = client.forward_kinematics(arm_name, joints_deg)
                pose = fk_resp["pose"] if "pose" in fk_resp else fk_resp
                collected[side].append({"joints_deg": joints_deg, "apex_pose": pose})
    return collected


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("/app/configs/devices.yaml"))
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF_PATH)
    parser.add_argument("--target-link", type=str, default="", help="Optional link override (both sides if empty)")
    parser.add_argument("--offline-samples", type=Path, default=None, help="JSON with {'left': [...], 'right': [...]}")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_PATH)
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    print(f"[reconcile_fk] config={args.config.resolve()} urdf={args.urdf} output={args.output}")
    chain = UrdfArmChain(args.urdf)

    if args.offline_samples is not None:
        raw_samples = json.loads(args.offline_samples.read_text(encoding="utf-8"))
    else:
        raw_samples = _collect_live_samples(cfg.arms.host, cfg.arms.left_arm, cfg.arms.right_arm)

    results: dict[str, Any] = {"urdf": str(args.urdf), "arms": {}}
    all_passed = True
    for side in ("left", "right"):
        side_samples = raw_samples.get(side, [])
        if not side_samples:
            continue
        res = reconcile_samples(
            chain,
            side,
            side_samples,
            target_link=args.target_link or None,
        )
        results["arms"][side] = res
        all_passed = all_passed and res["passed"]
        print(
            f"[reconcile_fk] {side}: samples={res['sample_count']} "
            f"max_pos={res['max_pos_err_mm']:.3f}mm max_rot={res['max_rot_err_deg']:.4f}deg "
            f"passed={res['passed']} best_frame={res['best_candidate_frame']}"
        )

    results["all_passed"] = bool(results["arms"]) and all_passed
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2, sort_keys=True), encoding="utf-8")
    print(f"[reconcile_fk] saved {args.output}")
    return 0 if results["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
