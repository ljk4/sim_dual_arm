"""Offline Marvin handover scene check: compile, reset, step, measure, render.

This is a model/scene diagnostic, not a control backend and not a handover
attempt.  It reads the task environment from configuration, measures the
compiled model, verifies gripper coupling and renders three camera views.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import sys
import time
import uuid
from collections.abc import Mapping
from pathlib import Path

import mujoco
import numpy as np
from robot_contracts.profiles import (
    load_profile,
    semantic_hash,
)

from robot_backend_sim.common.config.camera_config import camera_declared_description
from robot_backend_sim.marvin_sim.handover.scene_config import load_marvin_scene_config
from robot_backend_sim.marvin_sim.assembly import SIDES, read_urdf_semantics
from robot_backend_sim.marvin_sim.assembly import gripper_opening
from robot_backend_sim.marvin_sim.handover.scene import build_marvin_scene
from robot_backend_sim.marvin_sim.robot_config import MARVIN_CONTROL_JOINT_NAMES
from robot_backend_sim.common.render.rendering import SceneRenderer
from robot_backend_sim.common.render.visibility import count_visible_pixels

__all__ = ["main", "parse_arguments", "run"]

_SHARED_ROOT = Path(os.environ.get("SHARED_ROOT", "/shared"))
_SCENARIO = _SHARED_ROOT / "scenarios/marvin_sim/handover"
DEFAULT_SCENE_CONFIG = _SCENARIO / "configs/simulation/scene/marvin_handover.yaml"
DEFAULT_PROFILE = _SCENARIO / "profiles/robot/profile.json"
DEFAULT_SCHEMAS = _SHARED_ROOT / "common/interfaces/schemas"
PUBLIC_JOINT_ORDER = MARVIN_CONTROL_JOINT_NAMES
READY_POSE = {
    "left": (-0.450, -1.239, -0.610, -0.842, -0.620, -0.370, -1.545),
    "right": (0.395, -1.983, 1.405, -0.339, 1.571, -0.442, -1.571),
}
# Verified reachable configurations used only to prove that the active
# collision geometry detects contact.  They are not task or grasp poses.
SAME_ARM_COLLISION_POSE = {
    "right": (0.74432, 0.33872, 0.80518, -2.47078, -1.67433, -0.17901, 1.49686),
}
CROSS_ARM_COLLISION_POSE = {
    "left": (1.54525, -1.69055, -0.10567, -1.90513, 1.01637, -0.50605, 0.98071),
    "right": (-0.76212, -1.99988, 1.04755, -2.23433, -0.36641, 0.23354, -0.98902),
}
TABLE_COLLISION_POSE = {
    "right": (2.39295, 1.59484, -2.25148, 0.13239, 1.23707, -0.07975, -0.32199),
}
BOX_COLLISION_POSE = {
    "right": (2.78133, 1.94594, -0.18191, -1.08069, 2.10245, -1.00087, 0.74346),
}
FINGER_PROBE_CLOSURE = 0.02
MEASUREMENT_TOLERANCE = 5e-4
TCP_MIDPOINT_TOLERANCE = 1e-4
JOINT_TEST_DELTA = 0.05
JOINT_TEST_STEPS = 300
GRIPPER_TEST_STEPS = 600
STATIC_STEPS = 1000


class CheckFailure(RuntimeError):
    def __init__(self, reason: str, message: str) -> None:
        super().__init__(message)
        self.reason = reason
        self.message = message


def _close(left: float, right: float, tolerance: float = 1e-6) -> bool:
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)


def _utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _write_json(path: Path, document: dict[str, object]) -> None:
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, default=DEFAULT_SCENE_CONFIG)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--schemas", type=Path, default=DEFAULT_SCHEMAS)
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument(
        "--run-id",
        default=None,
        help="fresh report run identity (default: marvin-scene-check-<UTC>-<pid>)",
    )
    return parser


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parsed = _parser().parse_args(argv)
    if not parsed.run_id:
        parsed.run_id = (
            f"marvin-scene-check-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{os.getpid()}"
        )
    return parsed


def _semantics_check(scene, semantics) -> dict[str, object]:
    """Compare imported link mass and principal inertia with the source URDF."""
    model = scene.model
    max_mass_error = 0.0
    max_inertia_error = 0.0
    total_mass = 0.0
    for name, mass in semantics.link_mass.items():
        try:
            body = int(model.body(name).id)
        except KeyError as error:
            raise CheckFailure("urdf_link_missing", f"compiled model lacks {name}") from error
        max_mass_error = max(max_mass_error, abs(float(model.body_mass[body]) - mass))
        total_mass += float(model.body_mass[body])
        source = np.linalg.eigvalsh(np.array(semantics.link_inertia[name], dtype=float))
        compiled = np.sort(np.array(model.body_inertia[body], dtype=float))
        max_inertia_error = max(
            max_inertia_error, float(np.max(np.abs(np.sort(source) - compiled)))
        )
    if max_mass_error > 1e-6:
        raise CheckFailure(
            "urdf_mass_mismatch", f"compiled link mass differs by {max_mass_error:.6g} kg"
        )
    if max_inertia_error > 1e-4:
        raise CheckFailure(
            "urdf_inertia_mismatch",
            f"compiled principal inertia differs by {max_inertia_error:.6g}",
        )
    return {
        "link_count": len(semantics.link_mass),
        "mesh_reference_count": len(semantics.mesh_files),
        "compiled_mesh_count": int(model.nmesh),
        "total_link_mass_kg": total_mass,
        "max_mass_error_kg": max_mass_error,
        "max_principal_inertia_error": max_inertia_error,
        "stand_mesh_split": "ZJ_Robot_link source STL exceeds the 200000-face decoder limit",
    }


def _profile_checks(scene, profile_path: Path, schemas_path: Path) -> dict[str, object]:
    profile = load_profile(profile_path, schemas_path)
    if tuple(profile.joint_names) != PUBLIC_JOINT_ORDER:
        raise CheckFailure(
            "profile_joint_order",
            f"profile joint order {profile.joint_names} differs from {PUBLIC_JOINT_ORDER}",
        )
    robot = scene.indices.robot
    model = scene.model
    joints: dict[str, object] = {}
    for index, (side, prefix) in enumerate(SIDES):
        for joint_index in range(1, 8):
            spec = profile.joints[index * 8 + joint_index - 1]
            joint = model.joint(f"Arm_{prefix}{joint_index}_Joint")
            lower, upper = (float(value) for value in model.jnt_range[joint.id])
            if not (_close(spec.minimum, lower) and _close(spec.maximum, upper)):
                raise CheckFailure(
                    "profile_joint_range",
                    f"{spec.name}: profile [{spec.minimum}, {spec.maximum}] "
                    f"differs from compiled [{lower}, {upper}]",
                )
            if not _close(spec.initial, scene.config.robot.home_joint_positions[joint_index - 1]):
                raise CheckFailure(
                    "profile_joint_initial", f"{spec.name}: initial does not match reset pose"
                )
            joints[spec.name] = {
                "min": spec.minimum,
                "max": spec.maximum,
                "initial": spec.initial,
                "max_velocity": spec.max_velocity,
            }
        gripper = profile.joints[index * 8 + 7]
        opening = gripper_opening(model, scene.data, robot, side)
        if not _close(gripper.initial, scene.config.robot.gripper_open_total, 5e-4):
            raise CheckFailure(
                "profile_gripper_initial",
                f"{gripper.name}: initial {gripper.initial} differs from measured open {opening}",
            )
        if not (
            _close(gripper.minimum, scene.config.robot.gripper_closed_total, 5e-4)
            and _close(gripper.maximum, scene.config.robot.gripper_open_total, 5e-4)
        ):
            raise CheckFailure(
                "profile_gripper_range",
                f"{gripper.name}: profile range does not match measured finger opening",
            )
        joints[gripper.name] = {
            "min": gripper.minimum,
            "max": gripper.maximum,
            "initial": gripper.initial,
            "max_velocity": gripper.max_velocity,
        }
    cameras: list[dict[str, object]] = []
    for scene_camera, profile_camera in zip(
        scene.config.cameras, profile.cameras, strict=True
    ):
        if (
            scene_camera.name != profile_camera.name
            or scene_camera.width != profile_camera.width
            or scene_camera.height != profile_camera.height
        ):
            raise CheckFailure("profile_camera", "profile camera declaration mismatch")
        declaration = camera_declared_description(scene_camera)
        digest = semantic_hash(declaration)
        if digest != profile_camera.calibration_digest:
            raise CheckFailure(
                "profile_calibration_digest",
                f"{scene_camera.name}: scene declaration digest differs from profile",
            )
        calibration_path = profile_path.parent / f"{scene_camera.name}_calibration.json"
        calibration = None
        if calibration_path.is_file():
            calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
            if calibration != declaration:
                raise CheckFailure(
                    "profile_calibration_file",
                    f"{calibration_path} differs from the scene camera declaration",
                )
        cameras.append(
            {
                "name": scene_camera.name,
                "parent": scene_camera.parent,
                "width": scene_camera.width,
                "height": scene_camera.height,
                "frame_id": profile_camera.frame_id,
                "calibration_digest": digest,
                "calibration_file": (
                    calibration_path.name if calibration is not None else None
                ),
            }
        )
    return {
        "robot_id": profile.robot_id,
        "profile_id": profile.profile_id,
        "digest": profile.digest,
        "joints": joints,
        "cameras": cameras,
    }


def _static_check(scene) -> dict[str, object]:
    scene.reset(0)
    home = scene.config.robot.home_joint_positions
    model = scene.model
    data = scene.data
    for _ in range(STATIC_STEPS):
        mujoco.mj_step(model, data)
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        raise CheckFailure("static_nonfinite", "resting step produced a non-finite state")
    drift: dict[str, float] = {}
    for side, _prefix in SIDES:
        for joint_index, _ in enumerate(home, start=1):
            joint = model.joint(f"Arm_{side[0].upper()}{joint_index}_Joint")
            drift[f"{side}_joint{joint_index}"] = abs(
                float(data.qpos[model.jnt_qposadr[joint.id]]) - home[joint_index - 1]
            )
    max_drift = max(drift.values())
    if max_drift > 0.02:
        raise CheckFailure(
            "static_joint_drift", f"resting joint drift {max_drift:.4f} rad exceeds 0.02"
        )
    block_address = scene.indices.environment.block_free_joint_qpos
    block = data.qpos[block_address : block_address + 7]
    expected_z = (
        scene.config.robot.base_position[2]
        + scene.config.task.table.top_z
        + scene.config.task.block.half_extents[2]
    )
    if abs(float(block[2]) - expected_z) > 0.005:
        raise CheckFailure(
            "static_block_z",
            f"block z {block[2]:.4f} left the table surface {expected_z:.4f}",
        )
    table_contacts = sum(
        1
        for index in range(data.ncon)
        if {
            int(data.contact[index].geom1),
            int(data.contact[index].geom2),
        }
        == {scene.indices.environment.block_geom_id, scene.indices.environment.table_geom_id}
    )
    if table_contacts == 0:
        raise CheckFailure("static_block_contact", "block has no contact with the table")
    return {
        "steps": STATIC_STEPS,
        "sim_time_s": float(data.time),
        "max_arm_drift_rad": max_drift,
        "joint_drift_rad": drift,
        "block_qpos": [float(value) for value in block],
        "block_table_contacts": table_contacts,
    }


def _set_pose(scene, pose: Mapping[str, tuple[float, ...]], gripper_closure: float) -> None:
    robot = scene.indices.robot
    for side, _ in SIDES:
        for actuator, value in zip(
            robot.arm_actuator_ids[side], pose[side], strict=True
        ):
            scene.data.ctrl[actuator] = value
        scene.data.ctrl[robot.gripper_actuator_ids[side]] = gripper_closure


def _joint_motion_checks(scene) -> list[dict[str, object]]:
    model = scene.model
    robot = scene.indices.robot
    home = scene.config.robot.home_joint_positions
    results: list[dict[str, object]] = []
    for moved_side, prefix in SIDES:
        other_side = "right" if moved_side == "left" else "left"
        for joint_index in range(1, 8):
            scene.reset(0)
            actuator = model.actuator(f"Arm_{prefix}{joint_index}_Actuator").id
            scene.data.ctrl[actuator] = home[joint_index - 1] + JOINT_TEST_DELTA
            for _ in range(JOINT_TEST_STEPS):
                mujoco.mj_step(model, scene.data)
            joint = model.joint(f"Arm_{prefix}{joint_index}_Joint")
            position = float(scene.data.qpos[model.jnt_qposadr[joint.id]])
            delta = position - home[joint_index - 1]
            if delta * JOINT_TEST_DELTA <= 0.0 or abs(delta) < 0.01:
                raise CheckFailure(
                    "joint_motion_direction",
                    f"{moved_side}_joint{joint_index} moved {delta:.4f} rad "
                    f"for command {JOINT_TEST_DELTA}",
                )
            other_joint = model.joint(f"Arm_{other_side[0].upper()}{joint_index}_Joint")
            other_delta = abs(
                float(scene.data.qpos[model.jnt_qposadr[other_joint.id]])
                - home[joint_index - 1]
            )
            if other_delta > 0.01:
                raise CheckFailure(
                    "joint_motion_isolation",
                    f"{other_side}_joint{joint_index} moved {other_delta:.4f} rad "
                    "while the other arm was commanded",
                )
            moved_tcp = scene.data.site_xpos[robot.tcp_site_ids[moved_side]].copy()
            other_tcp = scene.data.site_xpos[robot.tcp_site_ids[other_side]].copy()
            results.append(
                {
                    "side": moved_side,
                    "joint": joint_index,
                    "commanded_delta_rad": JOINT_TEST_DELTA,
                    "measured_delta_rad": delta,
                    "other_arm_delta_rad": other_delta,
                    "tcp_x": float(moved_tcp[0]),
                    "tcp_y": float(moved_tcp[1]),
                    "tcp_z": float(moved_tcp[2]),
                    "other_tcp": [float(value) for value in other_tcp],
                }
            )
    return results


def _gripper_checks(scene) -> list[dict[str, object]]:
    model = scene.model
    robot = scene.indices.robot
    home = scene.config.robot.home_joint_positions
    open_total = scene.config.robot.gripper_open_total
    closed_total = scene.config.robot.gripper_closed_total
    closures = (0.0, (open_total - closed_total) / 2.0, open_total - closed_total)
    results: list[dict[str, object]] = []
    for side, prefix in SIDES:
        for closure in closures:
            scene.reset(0)
            for actuator, value in zip(
                robot.arm_actuator_ids[side], home, strict=True
            ):
                scene.data.ctrl[actuator] = value
            scene.data.ctrl[robot.gripper_actuator_ids[side]] = closure
            for _ in range(GRIPPER_TEST_STEPS):
                mujoco.mj_step(model, scene.data)
            first = model.joint(f"Arm_{prefix}10_Joint")
            second = model.joint(f"Arm_{prefix}11_Joint")
            q10 = float(scene.data.qpos[model.jnt_qposadr[first.id]])
            q11 = float(scene.data.qpos[model.jnt_qposadr[second.id]])
            if abs(q10 - q11) > 0.002:
                raise CheckFailure(
                    "gripper_coupling",
                    f"{side}: finger positions {q10:.5f} and {q11:.5f} are not coupled",
                )
            opening = gripper_opening(model, scene.data, robot, side)
            results.append(
                {
                    "side": side,
                    "closure_command_m": closure,
                    "finger10_qpos_m": q10,
                    "finger11_qpos_m": q11,
                    "coupling_error_m": abs(q10 - q11),
                    "measured_opening_m": opening,
                }
            )
        open_measured = results[-3]["measured_opening_m"]
        closed_measured = results[-1]["measured_opening_m"]
        if not _close(open_measured, open_total, 5e-4):
            raise CheckFailure(
                "gripper_open_measurement",
                f"{side}: measured open {open_measured:.5f} differs from {open_total:.5f}",
            )
        if not _close(closed_measured, closed_total, 5e-4):
            raise CheckFailure(
                "gripper_closed_measurement",
                f"{side}: measured closed {closed_measured:.5f} differs from {closed_total:.5f}",
            )
    return results


def _equality_checks(scene) -> dict[str, object]:
    """Enumerate the compiled equalities and their source."""
    model = scene.model
    if model.neq != 2:
        raise CheckFailure(
            "equality_count", f"compiled model has {model.neq} equalities, expected 2"
        )
    if model.ntendon != 2:
        raise CheckFailure(
            "tendon_count", f"compiled model has {model.ntendon} tendons, expected 2"
        )
    couplings: list[dict[str, object]] = []
    for _, prefix in SIDES:
        name = f"Arm_{prefix}_finger_coupling"
        try:
            equality = model.equality(name)
        except KeyError as error:
            raise CheckFailure(
                "equality_missing", f"compiled model lacks {name!r}"
            ) from error
        if int(model.eq_type[equality.id]) != int(mujoco.mjtEq.mjEQ_JOINT):
            raise CheckFailure("equality_type", f"{name!r} is not a joint equality")
        couplings.append(
            {
                "name": name,
                "type": "joint",
                "joints": [
                    model.joint(int(model.eq_obj1id[equality.id])).name,
                    model.joint(int(model.eq_obj2id[equality.id])).name,
                ],
                "data": [float(value) for value in model.eq_data[equality.id]],
                "solref": [float(value) for value in model.eq_solref[equality.id]],
                "solimp": [float(value) for value in model.eq_solimp[equality.id]],
                "active": bool(model.eq_active0[equality.id]),
            }
        )
    return {
        "count": int(model.neq),
        "tendons": int(model.ntendon),
        "origin": "URDF mimic Arm_*11_Joint <- Arm_*10_Joint (multiplier 1, offset 0)",
        "assembly_adds_equalities": False,
        "couplings": couplings,
    }


def _measurement_checks(scene, gripper_results) -> dict[str, object]:
    """Check TCP midpoint, command range and measured opening tolerances."""
    model = scene.model
    robot = scene.indices.robot
    scene.reset(0)
    data = scene.data
    open_total = scene.config.robot.gripper_open_total
    closed_total = scene.config.robot.gripper_closed_total
    full_closure = open_total - closed_total
    tolerance = MEASUREMENT_TOLERANCE
    if not _close(scene.gripper_ctrl(open_total), 0.0, 1e-9):
        raise CheckFailure(
            "gripper_command_open",
            f"open_total {open_total} does not map to command 0",
        )
    if not _close(scene.gripper_ctrl(closed_total), full_closure, 1e-9):
        raise CheckFailure(
            "gripper_command_closed",
            f"closed_total {closed_total} does not map to command {full_closure}",
        )
    tcp: dict[str, object] = {}
    for side, _ in SIDES:
        hand = robot.hand_body_ids[side]
        axis = data.xmat[hand].reshape(3, 3)[:, 2]
        tcp_position = data.site_xpos[robot.tcp_site_ids[side]]
        pad_offsets = []
        for key in ("finger10", "finger11"):
            pad = data.site_xpos[robot.pad_site_ids[side][key]]
            pad_offsets.append(float(np.dot(pad - tcp_position, axis)))
        opening = abs(pad_offsets[0] - pad_offsets[1])
        midpoint_error = abs(pad_offsets[0] + pad_offsets[1])
        if midpoint_error > TCP_MIDPOINT_TOLERANCE:
            raise CheckFailure(
                "tcp_midpoint",
                f"{side}: TCP is {midpoint_error:.6g} m from the finger-face midpoint",
            )
        if not _close(opening, open_total, tolerance):
            raise CheckFailure(
                "tcp_opening",
                f"{side}: TCP-relative finger gap {opening:.6f} differs from {open_total:.6f}",
            )
        actuator = model.actuator(robot.gripper_actuator_ids[side])
        lower, upper = (float(value) for value in model.actuator_ctrlrange[actuator.id])
        if not (_close(lower, 0.0, 1e-9) and _close(upper, full_closure, 1e-9)):
            raise CheckFailure(
                "gripper_ctrlrange",
                f"{side}: actuator ctrlrange [{lower}, {upper}] is not [0, {full_closure}]",
            )
        tcp[side] = {
            "site_id": robot.tcp_site_ids[side],
            "parent_body": model.body(int(model.site_bodyid[robot.tcp_site_ids[side]])).name,
            "local_position_m": [
                float(value) for value in model.site_pos[robot.tcp_site_ids[side]]
            ],
            "pad_offsets_m": pad_offsets,
            "midpoint_error_m": midpoint_error,
            "actuator_ctrlrange_m": [lower, upper],
        }
    measured: dict[str, object] = {}
    for side, _ in SIDES:
        rows = [row for row in gripper_results if row["side"] == side]
        open_row = next(
            row for row in rows if _close(row["closure_command_m"], 0.0, 1e-12)
        )
        mid_row = next(
            row for row in rows if _close(row["closure_command_m"], full_closure / 2.0, 1e-12)
        )
        closed_row = next(
            row for row in rows if _close(row["closure_command_m"], full_closure, 1e-12)
        )
        expected_mid = (open_total + closed_total) / 2.0
        if not _close(mid_row["measured_opening_m"], expected_mid, tolerance):
            raise CheckFailure(
                "gripper_mid_measurement",
                f"{side}: measured midpoint {mid_row['measured_opening_m']:.6f} "
                f"differs from {expected_mid:.6f}",
            )
        measured[side] = {
            "open_m": open_row["measured_opening_m"],
            "mid_m": mid_row["measured_opening_m"],
            "closed_m": closed_row["measured_opening_m"],
            "coupling_error_m": max(row["coupling_error_m"] for row in rows),
        }
    return {
        "tolerance_m": tolerance,
        "tcp_midpoint_tolerance_m": TCP_MIDPOINT_TOLERANCE,
        "command_range_m": [0.0, full_closure],
        "declared_open_total_m": open_total,
        "declared_closed_total_m": closed_total,
        "command_at_declared_open_m": scene.gripper_ctrl(open_total),
        "command_at_declared_closed_m": scene.gripper_ctrl(closed_total),
        "tcp": tcp,
        "measured": measured,
    }


def _contact_pairs(model, data) -> list[dict[str, object]]:
    pairs: list[dict[str, object]] = []
    for index in range(data.ncon):
        contact = data.contact[index]
        first_geom = int(contact.geom1)
        second_geom = int(contact.geom2)
        pairs.append(
            {
                "bodies": sorted(
                    (
                        model.body(int(model.geom_bodyid[first_geom])).name,
                        model.body(int(model.geom_bodyid[second_geom])).name,
                    )
                ),
                "geoms": sorted((model.geom(first_geom).name, model.geom(second_geom).name)),
                "distance_m": float(contact.dist),
            }
        )
    return pairs


def _set_arm_pose(scene, pose: Mapping[str, tuple[float, ...]]) -> None:
    robot = scene.indices.robot
    for side, values in pose.items():
        for address, value in zip(
            robot.arm_joint_qpos[side], values, strict=True
        ):
            scene.data.qpos[address] = value


def _probe_pose(scene, pose: Mapping[str, tuple[float, ...]]) -> list[dict[str, object]]:
    scene.reset(0)
    _set_arm_pose(scene, pose)
    mujoco.mj_forward(scene.model, scene.data)
    return _contact_pairs(scene.model, scene.data)


def _probe_finger_contact(scene) -> list[dict[str, object]]:
    scene.reset(0)
    model = scene.model
    data = scene.data
    robot = scene.indices.robot
    pads = robot.pad_site_ids["left"]
    midpoint = 0.5 * (
        data.site_xpos[pads["finger10"]] + data.site_xpos[pads["finger11"]]
    )
    address = scene.indices.environment.block_free_joint_qpos
    data.qpos[address : address + 7] = (*midpoint, 1.0, 0.0, 0.0, 0.0)
    for finger_address in robot.finger_joint_qpos["left"]:
        data.qpos[finger_address] = FINGER_PROBE_CLOSURE
    mujoco.mj_forward(model, data)
    pairs = _contact_pairs(model, data)
    scene.reset(0)
    return pairs


def _effective_collision_checks(scene) -> dict[str, object]:
    """Prove that same-arm, cross-arm, finger and environment contacts fire."""
    same_arm = _probe_pose(scene, SAME_ARM_COLLISION_POSE)
    if not any(
        pair["bodies"] == ["Arm_R1_Link", "Arm_R9_Link"] for pair in same_arm
    ):
        raise CheckFailure(
            "same_arm_collision",
            f"verified same-arm pose produced {[pair['bodies'] for pair in same_arm]}",
        )
    cross_arm = _probe_pose(scene, CROSS_ARM_COLLISION_POSE)
    if not any(
        pair["bodies"] == ["Arm_L5_Link", "Arm_R5_Link"] for pair in cross_arm
    ):
        raise CheckFailure(
            "cross_arm_collision",
            f"verified cross-arm pose produced {[pair['bodies'] for pair in cross_arm]}",
        )
    table = _probe_pose(scene, TABLE_COLLISION_POSE)
    if not any(pair["bodies"] == ["Arm_R5_Link", "table"] for pair in table):
        raise CheckFailure(
            "arm_table_collision",
            f"verified table pose produced {[pair['bodies'] for pair in table]}",
        )
    box = _probe_pose(scene, BOX_COLLISION_POSE)
    if not any(pair["bodies"] == ["Arm_R5_Link", "box"] for pair in box):
        raise CheckFailure(
            "arm_box_collision",
            f"verified box pose produced {[pair['bodies'] for pair in box]}",
        )
    fingers = _probe_finger_contact(scene)
    for finger in ("Arm_L10_Link", "Arm_L11_Link"):
        if not any(pair["bodies"] == sorted((finger, "block")) for pair in fingers):
            raise CheckFailure(
                "finger_block_collision",
                f"{finger} did not contact the block in the finger probe",
            )
    scene.reset(0)
    mujoco.mj_forward(scene.model, scene.data)
    reset_pairs = _contact_pairs(scene.model, scene.data)
    if any(
        pair["bodies"] == ["Arm_L5_Link", "Arm_L7_Link"]
        or pair["bodies"] == ["Arm_R5_Link", "Arm_R7_Link"]
        for pair in reset_pairs
    ):
        raise CheckFailure(
            "self_exclusion_ineffective", "the excluded 5/7 pair still contacts at reset"
        )
    return {
        "method": "set verified reachable qpos, call mj_forward, inspect contacts",
        "same_arm": {
            "pose_rad": {side: list(values) for side, values in SAME_ARM_COLLISION_POSE.items()},
            "contacts": same_arm,
        },
        "cross_arm": {
            "pose_rad": {side: list(values) for side, values in CROSS_ARM_COLLISION_POSE.items()},
            "contacts": cross_arm,
        },
        "arm_table": {
            "pose_rad": {side: list(values) for side, values in TABLE_COLLISION_POSE.items()},
            "contacts": table,
        },
        "arm_box": {
            "pose_rad": {side: list(values) for side, values in BOX_COLLISION_POSE.items()},
            "contacts": box,
        },
        "finger_block": {
            "finger_qpos_m": FINGER_PROBE_CLOSURE,
            "contacts": fingers,
        },
        "reset_contacts": reset_pairs,
    }


def _collision_checks(scene) -> dict[str, object]:
    model = scene.model
    data = scene.data
    task = scene.config.task
    box = task.box
    box_origin = np.array(
        data.xpos[scene.indices.environment.box_body_id], dtype=float, copy=True
    )
    box_bottom = float(box_origin[2]) + box.bottom_thickness
    box_top = float(box_origin[2]) + box.height
    geomid = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(
        model,
        data,
        np.array([box_origin[0], box_origin[1], box_top + 0.10]),
        np.array([0.0, 0.0, -1.0]),
        None,
        True,
        -1,
        geomid,
    )
    bottom_id = scene.indices.environment.box_geom_ids["box_bottom"]
    if distance < 0.0 or int(geomid[0]) != bottom_id:
        raise CheckFailure(
            "box_opening_closed",
            f"downward ray through the box centre hit geom {int(geomid[0])}, "
            f"expected the bottom geom {bottom_id}",
        )
    expected_distance = box_top + 0.10 - box_bottom
    if abs(float(distance) - expected_distance) > 0.01:
        raise CheckFailure(
            "box_bottom_depth",
            f"box bottom distance {distance:.4f} differs from {expected_distance:.4f}",
        )

    # Drop the block into the open box and verify it is supported by the bottom.
    scene.reset(0)
    address = scene.indices.environment.block_free_joint_qpos
    block = task.block
    half = block.half_extents
    data.qpos[address : address + 7] = (
        box_origin[0],
        box_origin[1],
        box_bottom + half[2] + 0.01,
        1.0,
        0.0,
        0.0,
        0.0,
    )
    mujoco.mj_forward(model, data)
    for _ in range(STATIC_STEPS):
        mujoco.mj_step(model, data)
    placed = np.array(data.qpos[address : address + 7], copy=True)
    inner_x, inner_y = box.inner_xy
    if (
        abs(float(placed[0]) - box_origin[0]) > inner_x / 2.0
        or abs(float(placed[1]) - box_origin[1]) > inner_y / 2.0
        or float(placed[2]) < box_bottom + half[2] - 0.01
    ):
        raise CheckFailure(
            "box_block_support",
            f"block did not settle inside the box: {[float(value) for value in placed[:3]]}",
        )
    bottom_contacts = sum(
        1
        for index in range(data.ncon)
        if {
            int(data.contact[index].geom1),
            int(data.contact[index].geom2),
        }
        == {scene.indices.environment.block_geom_id, bottom_id}
    )
    if bottom_contacts == 0:
        raise CheckFailure("box_block_support", "block has no contact with the box bottom")
    finger_geoms: dict[str, dict[str, int]] = {}
    for prefix in ("L", "R"):
        for finger in (10, 11):
            name = f"Arm_{prefix}{finger}_Link"
            body = model.body(name)
            if int(model.body_geomnum[body.id]) != 1:
                raise CheckFailure(
                    "finger_contact_geom",
                    f"{name} has {int(model.body_geomnum[body.id])} geoms, expected one",
                )
            geom_id = int(model.body_geomadr[body.id])
            if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_MESH):
                raise CheckFailure(
                    "finger_contact_geom", f"{name} is not a mesh collision geom"
                )
            if int(model.geom_contype[geom_id]) == 0 or int(model.geom_conaffinity[geom_id]) == 0:
                raise CheckFailure(
                    "finger_contact_disabled", f"{name} excludes itself from contacts"
                )
            finger_geoms[name] = {
                "contype": int(model.geom_contype[geom_id]),
                "conaffinity": int(model.geom_conaffinity[geom_id]),
                "nmesh_vertices": int(model.mesh_vertnum[model.geom_dataid[geom_id]]),
            }
    cross_arm_exclusions = 0
    exclusion_pairs: list[list[str]] = []
    for index in range(model.nexclude):
        signature = int(model.exclude_signature[index])
        first = model.body(signature >> 16).name
        second = model.body(signature & 0xFFFF).name
        exclusion_pairs.append(sorted((first, second)))
        if first.startswith("Arm_L") != second.startswith("Arm_L"):
            cross_arm_exclusions += 1
    expected_exclusions = [
        ["Arm_L5_Link", "Arm_L7_Link"],
        ["Arm_R5_Link", "Arm_R7_Link"],
    ]
    if cross_arm_exclusions:
        raise CheckFailure(
            "cross_arm_exclusion",
            f"{cross_arm_exclusions} exclusions disable left/right arm collision",
        )
    if sorted(exclusion_pairs) != expected_exclusions:
        raise CheckFailure(
            "self_exclusion_set",
            f"self exclusions {sorted(exclusion_pairs)} are not the verified 5/7 pairs",
        )
    scene.reset(0)
    return {
        "box_opening_ray_geom": int(geomid[0]),
        "box_opening_ray_distance_m": float(distance),
        "box_bottom_geom": bottom_id,
        "box_block_rest_z_m": float(placed[2]),
        "box_bottom_contacts": bottom_contacts,
        "block_geom_contype": int(model.geom_contype[scene.indices.environment.block_geom_id]),
        "box_geom_count": len(scene.indices.environment.box_geom_ids),
        "finger_geoms": finger_geoms,
        "self_exclusions": int(model.nexclude),
        "self_exclusion_pairs": sorted(exclusion_pairs),
        "cross_arm_exclusions": cross_arm_exclusions,
        "base_collision": {
            "enabled": False,
            "visual_geom_count": 2,
            "reason": (
                "216k-face concave source hull is visual only; the task table starts "
                "at x=0.40 m and the verified reset, low-amplitude and collision "
                "probes do not enter the base volume"
            ),
            "proxy_trigger": (
                "a P02B path entering about 0.15 m of the base vertical axis"
            ),
        },
    }


def _render(scene, run_dir: Path) -> dict[str, object]:
    first = scene.config.cameras[0]
    renderer = SceneRenderer(
        scene.model,
        [camera.name for camera in scene.config.cameras],
        width=first.width,
        height=first.height,
    )
    gl = renderer.gl_info
    artifacts: dict[str, object] = {
        "views": {},
        "ready_pose_rad": {side: list(values) for side, values in READY_POSE.items()},
        "gl": {
            "vendor": gl.vendor,
            "renderer": gl.renderer,
            "version": gl.version,
            "nvidia": gl.nvidia,
        },
    }
    try:
        for label in ("reset", "ready"):
            if label == "reset":
                scene.reset(0)
            else:
                scene.reset(0)
                _set_pose(scene, READY_POSE, 0.0)
                for _ in range(500):
                    mujoco.mj_step(scene.model, scene.data)
                if not np.all(np.isfinite(scene.data.qpos)):
                    raise CheckFailure(
                        "ready_pose_nonfinite", "diagnostic ready pose diverged"
                    )
            snapshot = renderer.capture(scene.data)
            rgb = renderer.render_rgb(snapshot)
            segmentation = renderer.render_segmentation(snapshot)
            directory = run_dir / "artifacts" / "renders" / label
            directory.mkdir(parents=True, exist_ok=False)
            block_pixels: dict[str, int] = {}
            for camera, image in rgb.items():
                from PIL import Image

                path = directory / f"{camera}.png"
                Image.fromarray(image, mode="RGB").save(path, format="PNG")
                if image.shape != (first.height, first.width, 3):
                    raise CheckFailure(
                        "render_shape", f"{label}/{camera}: {image.shape} is not the declared size"
                    )
                if len(np.unique(image.reshape(-1, 3), axis=0)) < 256:
                    raise CheckFailure(
                        "render_content", f"{label}/{camera}: image has almost no content"
                    )
                block_pixels[camera] = count_visible_pixels(
                    segmentation[camera], scene.indices.environment.block_geom_id
                )
            artifacts["views"][label] = {
                "block_pixels": block_pixels,
                "images": {
                    camera: (directory / f"{camera}.png")
                    .relative_to(run_dir)
                    .as_posix()
                    for camera in rgb
                },
            }
        return artifacts
    finally:
        renderer.close()


def run(arguments) -> int:
    run_dir = arguments.report_root / arguments.run_id
    if os.path.lexists(run_dir):
        raise FileExistsError(f"report run directory already exists: {run_dir}")
    arguments.report_root.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(exist_ok=False)
    started = time.monotonic()
    report: dict[str, object] = {
        "kind": "marvin_scene",
        "run_id": arguments.run_id,
        "instance_id": str(uuid.uuid4()),
        "status": "failed",
        "reason": "not_started",
        "started_at_utc": _utc_now(),
        "finished_at_utc": None,
        "elapsed_s": None,
        "python": {
            "version": platform.python_version(),
            "executable": sys.executable,
        },
        "environment": {
            "MUJOCO_GL": os.environ.get("MUJOCO_GL"),
            "mujoco": mujoco.__version__,
        },
        "scene": None,
        "profile": None,
        "checks": None,
        "artifacts": None,
        "error": None,
    }
    try:
        config = load_marvin_scene_config(arguments.scene_config)
        scene = build_marvin_scene(config)
        semantics = read_urdf_semantics(config.urdf_path)
        model = scene.model
        robot = scene.indices.robot
        report["scene"] = {
            "config": str(config.source),
            "model_shape": {
                "nq": int(model.nq),
                "nv": int(model.nv),
                "nu": int(model.nu),
                "ncam": int(model.ncam),
                "nbody": int(model.nbody),
                "ngeom": int(model.ngeom),
                "neq": int(model.neq),
                "ntendon": int(model.ntendon),
            },
            "robot": {
                "asset_path": str(scene.config.urdf_path),
                "base_body": model.body(robot.base_body_id).name,
                "arm_joint_qpos": {
                    side: list(addresses)
                    for side, addresses in robot.arm_joint_qpos.items()
                },
                "finger_joint_qpos": {
                    side: list(addresses)
                    for side, addresses in robot.finger_joint_qpos.items()
                },
                "arm_actuator_ids": {
                    side: list(ids) for side, ids in robot.arm_actuator_ids.items()
                },
                "gripper_actuator_ids": dict(robot.gripper_actuator_ids),
                "tcp_site_ids": dict(robot.tcp_site_ids),
                "tcp_sites": {
                    side: {
                        "site_id": site_id,
                        "parent_body": model.body(
                            int(model.site_bodyid[site_id])
                        ).name,
                        "local_position_m": [
                            float(value) for value in model.site_pos[site_id]
                        ],
                    }
                    for side, site_id in robot.tcp_site_ids.items()
                },
                "hand_body_ids": dict(robot.hand_body_ids),
                "pad_site_ids": {
                    side: dict(pads) for side, pads in robot.pad_site_ids.items()
                },
                "finger_travel_m": robot.finger_travel,
            },
            "environment": {
                "block_free_joint_qpos": scene.indices.environment.block_free_joint_qpos,
                "block_geom_id": scene.indices.environment.block_geom_id,
                "table_geom_id": scene.indices.environment.table_geom_id,
                "box_geom_ids": dict(scene.indices.environment.box_geom_ids),
                "box_body_id": scene.indices.environment.box_body_id,
                "camera_mount_body_id": scene.indices.environment.camera_mount_body_id,
            },
            "urdf_links": len(semantics.link_mass),
            "urdf_meshes": len(semantics.mesh_files),
        }
        report["profile"] = _profile_checks(scene, arguments.profile, arguments.schemas)
        report["checks"] = {}
        report["checks"]["semantics"] = _semantics_check(scene, semantics)
        report["checks"]["static"] = _static_check(scene)
        report["checks"]["joint_motion"] = _joint_motion_checks(scene)
        gripper_results = _gripper_checks(scene)
        report["checks"]["gripper"] = gripper_results
        report["checks"]["measurement"] = _measurement_checks(scene, gripper_results)
        report["checks"]["collision"] = _collision_checks(scene)
        report["checks"]["effective_collision"] = _effective_collision_checks(scene)
        report["checks"]["equality"] = _equality_checks(scene)
        report["artifacts"] = _render(scene, run_dir)
        report["status"] = "succeeded"
        report["reason"] = None
    except CheckFailure as error:
        report["reason"] = error.reason
        report["error"] = {"type": type(error).__name__, "message": error.message[:2000]}
    except Exception as error:
        report["reason"] = "unexpected_error"
        report["error"] = {"type": type(error).__name__, "message": str(error)[:2000]}
    finally:
        report["elapsed_s"] = round(time.monotonic() - started, 3)
        report["finished_at_utc"] = _utc_now()
        _write_json(run_dir / "report.json", report)
        print(f"report: {run_dir / 'report.json'}")
    return 0 if report["status"] == "succeeded" else 1


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    try:
        return run(arguments)
    except FileExistsError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
