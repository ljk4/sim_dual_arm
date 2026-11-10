"""Thin orchestration for the S2 key-pose, IK and clearance diagnostics.

The module validates all configurations before native work, loads the accepted
scene once, then keeps every layout on private snapshots.  It writes bounded
JSON evidence and RGB/segmentation-derived visibility counts; it does not step
physics, execute a grasp, start ROS or create training data.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import re
import sys
import time
import uuid
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
from typing import TYPE_CHECKING

from .. import assets, kinematics, visibility
from ..diagnostics import (
    code_provenance,
    scene_config_document,
    sha256_file,
    utc_now,
    write_json_atomic,
)
from ..kinematics import (
    IKResult,
    KinematicsError,
    PathResult,
    build_collision_pairs,
    build_targets,
    check_initial_pose,
    check_path,
    extract_object_pose,
    solve_key_poses,
)
from ..kinematics_config import (
    KinematicsCheckConfig,
    KinematicsConfigError,
    TargetConfig,
    load_kinematics_config,
)
from ..scene_config import SceneConfig, SceneConfigError, load_scene_config
from ..visibility import (
    LayoutSpec,
    SceneCheckConfig,
    SceneCheckConfigError,
    load_check_config,
)

if TYPE_CHECKING:
    import numpy as np

    from ..rendering import SceneRenderer, Snapshot
    from ..scene import Scene

from . import probe_environment

COMPONENT = "kinematics_check"
KIND = "kinematics"
REPORT_SCHEMA_VERSION = 1
DEFAULT_SCENE_CONFIG = Path("configs/scene/panda_stack.yaml")
DEFAULT_CHECK_CONFIG = Path("configs/checks/scene_check.yaml")
DEFAULT_KINEMATICS_CONFIG = Path("configs/checks/kinematics_check.yaml")
_RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class KinematicsCheckUsageError(ValueError):
    """Invalid invocation before the run directory is created."""


class KinematicsCheckFailure(RuntimeError):
    """A stable check failure with a bounded reason and message."""

    def __init__(self, reason: str, message: str = "") -> None:
        super().__init__(message or reason)
        self.reason = reason
        self.message = message or reason


@dataclass(frozen=True, slots=True)
class KinematicsCheckArguments:
    scene_config: Path
    check_config: Path
    kinematics_config: Path
    report_root: Path
    run_id: str


@dataclass
class ArtifactBudget:
    limit_bytes: int
    total_bytes: int = 0
    count: int = 0


def _validate_run_id(value: str) -> str:
    if not isinstance(value, str) or not _RUN_ID_PATTERN.fullmatch(value):
        raise KinematicsCheckUsageError(
            "run id must contain 1..128 ASCII letters, digits, '.', '_' or '-' "
            "and must start with a letter or digit"
        )
    if value in {".", ".."} or ".." in value:
        raise KinematicsCheckUsageError("run id must not be '.' or '..' or contain '..'")
    return value


def _validate_report_root(path: Path) -> Path:
    resolved = path.resolve()
    if resolved == Path("/"):
        raise KinematicsCheckUsageError("report root must not be the filesystem root")
    home = Path.home().resolve()
    if resolved == home or resolved in home.parents:
        raise KinematicsCheckUsageError("report root must not be the home directory or a parent")
    return resolved


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, required=True, help="scene parameters")
    parser.add_argument("--check-config", type=Path, required=True, help="layout parameters")
    parser.add_argument(
        "--kinematics-config", type=Path, required=True, help="kinematics check parameters"
    )
    parser.add_argument("--report-root", type=Path, required=True, help="report parent directory")
    parser.add_argument("--run-id", required=True, help="fresh report run identity")
    return parser


def parse_arguments(argv: list[str] | None = None) -> KinematicsCheckArguments:
    parsed = _parser().parse_args(argv)
    return KinematicsCheckArguments(
        scene_config=parsed.scene_config.resolve(),
        check_config=parsed.check_config.resolve(),
        kinematics_config=parsed.kinematics_config.resolve(),
        report_root=_validate_report_root(parsed.report_root),
        run_id=_validate_run_id(parsed.run_id),
    )


def _emit(
    logger: object, event: str, message: str, fields: dict[str, object] | None = None
) -> None:
    try:
        logger.emit(event, message=message, fields=fields, critical=True)
    except Exception as error:
        raise KinematicsCheckFailure(
            "event_log_failed", f"{event}: {type(error).__name__}: {error}"
        ) from error


def _load_configs(
    arguments: KinematicsCheckArguments,
) -> tuple[SceneConfig, SceneCheckConfig, KinematicsCheckConfig]:
    try:
        scene_config = load_scene_config(arguments.scene_config)
    except SceneConfigError as error:
        raise KinematicsCheckFailure("scene_config_invalid", str(error)) from error
    try:
        check_config = load_check_config(arguments.check_config, scene_config)
    except SceneCheckConfigError as error:
        raise KinematicsCheckFailure("check_config_invalid", str(error)) from error
    try:
        kinematics_config = load_kinematics_config(arguments.kinematics_config, scene_config)
    except KinematicsConfigError as error:
        raise KinematicsCheckFailure("kinematics_config_invalid", str(error)) from error
    layouts = visibility.build_layouts(check_config, scene_config)
    if len(layouts) > kinematics_config.budgets.max_layouts:
        raise KinematicsCheckFailure(
            "layout_budget_exceeded",
            f"{len(layouts)} layouts exceed max_layouts={kinematics_config.budgets.max_layouts}",
        )
    if len(layouts) != check_config.layout_count:
        raise KinematicsCheckFailure(
            "layout_count_mismatch",
            f"built {len(layouts)} layouts but check config declares {check_config.layout_count}",
        )
    return scene_config, check_config, kinematics_config


def _kinematics_config_document(config: KinematicsCheckConfig) -> dict[str, object]:
    return {
        "source": config.source,
        "schema_version": config.schema_version,
        "ik": {
            "dt_s": config.ik.dt_s,
            "max_iterations": config.ik.max_iterations,
            "position_tolerance_m": config.ik.position_tolerance_m,
            "orientation_tolerance_rad": config.ik.orientation_tolerance_rad,
            "posture_cost": config.ik.posture_cost,
            "position_cost": config.ik.position_cost,
            "orientation_cost": config.ik.orientation_cost,
            "lm_damping": config.ik.lm_damping,
            "solver": config.ik.solver,
            "initial_right_joint_positions": list(config.ik.initial_right_joint_positions),
        },
        "clearance": {
            "minimum_clearance_m": config.clearance.minimum_clearance_m,
            "collision_detection_distance_m": config.clearance.collision_detection_distance_m,
            "sample_step_rad": config.clearance.sample_step_rad,
            "max_samples_per_segment": config.clearance.max_samples_per_segment,
            "max_total_samples": config.clearance.max_total_samples,
        },
        "targets": {
            "pregrasp_height_m": config.targets.pregrasp_height_m,
            "grasp_height_m": config.targets.grasp_height_m,
            "lift_height_m": config.targets.lift_height_m,
            "place_hover_height_m": config.targets.place_hover_height_m,
            "place_clearance_m": config.targets.place_clearance_m,
            "retreat_height_m": config.targets.retreat_height_m,
        },
        "visibility": {
            "rules": [
                {
                    "camera": rule.camera,
                    "object": rule.object,
                    "phases": list(rule.phases),
                    "mode": rule.mode,
                    "min_pixels": rule.min_pixels,
                }
                for rule in config.visibility_rules
            ]
        },
        "budgets": {
            "max_layouts": config.budgets.max_layouts,
            "max_png_bytes": config.budgets.max_png_bytes,
            "max_report_bytes": config.budgets.max_report_bytes,
        },
        "camera_names": list(config.camera_names),
        "object_names": list(config.object_names),
        "camera_resolutions": {
            camera: list(resolution) for camera, resolution in config.camera_resolutions.items()
        },
    }


def _build_scene(scene_config: SceneConfig) -> object:
    from ..scene import SceneBuildError, build_scene

    try:
        return build_scene(scene_config)
    except SceneBuildError as error:
        raise KinematicsCheckFailure("scene_build_failed", str(error)) from error


def _scene_info(scene: object, scene_config: SceneConfig) -> dict[str, object]:
    model = scene.model
    try:
        object_geom_ids = {
            obj.name: int(model.geom(f"{obj.name}_geom").id) for obj in scene_config.objects
        }
        table_geom_id = int(model.geom("table_top").id)
        tcp_site_id = int(model.site("right_tcp").id)
        camera_ids = {
            camera.name: int(model.camera(camera.name).id) for camera in scene_config.cameras
        }
    except KeyError as error:
        raise KinematicsCheckFailure(
            "scene_model_missing_name", f"compiled model is missing {error}"
        ) from error
    return {
        "model": {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "ncam": int(model.ncam),
            "ngeom": int(model.ngeom),
            "nkey": int(model.nkey),
        },
        "camera_ids": camera_ids,
        "object_geom_ids": object_geom_ids,
        "table_geom_id": table_geom_id,
        "right_tcp_site_id": tcp_site_id,
        "offscreen": {
            "width": int(model.vis.global_.offwidth),
            "height": int(model.vis.global_.offheight),
        },
    }


def _make_renderer(scene: object, scene_config: SceneConfig) -> object:
    from ..rendering import SceneRenderer

    first = scene_config.cameras[0]
    return SceneRenderer(
        scene.model,
        [camera.name for camera in scene_config.cameras],
        width=first.width,
        height=first.height,
    )


def _open_logger(run_dir: Path, run_id: str, instance_id: str) -> object:
    from robot_run_logging import EventLogger, LogSettings

    return EventLogger(
        run_dir / "logs" / COMPONENT / instance_id,
        run_id=run_id,
        component=COMPONENT,
        instance_id=instance_id,
        settings=LogSettings(
            queue_capacity=512,
            max_event_bytes=32768,
            max_file_bytes=8 * 1024 * 1024,
            max_total_bytes=32 * 1024 * 1024,
            close_timeout_s=5.0,
            flush_interval_s=0.2,
            console=False,
        ),
    )


def _boundary_updates(
    layout: LayoutSpec, scene: object, scene_config: SceneConfig
) -> dict[int, tuple[float, ...]]:
    by_name = {placement.name: placement for placement in layout.placements}
    updates: dict[int, tuple[float, ...]] = {}
    for obj in scene_config.objects:
        placement = by_name[obj.name]
        address = scene.indices.object_free_joint_qpos[obj.name]
        z = scene_config.table.top_z + obj.half_extents[2]
        updates[address] = (
            placement.x,
            placement.y,
            z,
            *visibility.yaw_quaternion(placement.yaw_deg),
        )
    return updates


def _apply_updates(qpos: object, updates: dict[int, tuple[float, ...]]) -> object:
    import numpy as np

    result = np.array(qpos, dtype=np.float64, copy=True)
    for address, values in updates.items():
        result[address : address + len(values)] = values
    return result


def _point_document(point: kinematics.ClearancePoint) -> dict[str, object]:
    return {
        "distance_m": point.distance_m,
        "geom1": point.geom1,
        "geom2": point.geom2,
        "geom1_name": point.geom1_name,
        "geom2_name": point.geom2_name,
        "body1": point.body1,
        "body2": point.body2,
        "sample_index": point.sample_index,
        "fraction": round(point.fraction, 6),
        "measured_pairs": point.measured_pairs,
        "broadphase_skipped": point.broadphase_skipped,
        "lower_bound": point.lower_bound,
    }


def _path_document(result: PathResult) -> dict[str, object]:
    return {
        "segment": result.segment,
        "sample_count": result.sample_count,
        "requested_max_step_rad": result.requested_max_step_rad,
        "actual_max_step_rad": result.actual_max_step_rad,
        "minimum_clearance": _point_document(result.minimum),
        "passed": result.passed,
        "elapsed_s": round(result.elapsed_s, 6),
        "failure_reason": result.failure_reason,
        "failure_message": result.failure_message,
    }


def _finite_or_none(value: float) -> float | None:
    return value if math.isfinite(value) else None


def _ik_document(result: IKResult) -> dict[str, object]:
    qpos_right = list(result.qpos_right)
    return {
        "phase": result.phase,
        "target_position": list(result.target.position),
        "target_rotation": [list(row) for row in result.target.rotation],
        "converged": result.converged,
        "iterations": result.iterations,
        "position_error_m": _finite_or_none(result.position_error_m),
        "orientation_error_rad": _finite_or_none(result.orientation_error_rad),
        "qpos_right": qpos_right if all(math.isfinite(value) for value in qpos_right) else None,
        "frozen_max_deviation": _finite_or_none(result.frozen_max_deviation),
        "elapsed_s": round(result.elapsed_s, 6),
        "reason": result.reason,
        "message": result.message,
    }


def _save_png(path: Path, image: object, budget: ArtifactBudget) -> None:
    """Encode one bounded image and reserve its bytes before touching disk."""
    buffer = BytesIO()
    try:
        image.save(buffer, format="PNG")
        payload = buffer.getvalue()
        if len(payload) > budget.limit_bytes - budget.total_bytes:
            raise KinematicsCheckFailure(
                "png_budget_exceeded",
                f"PNG evidence needs {len(payload)} more bytes; "
                f"{budget.total_bytes}/{budget.limit_bytes} already reserved",
            )
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(payload)
        budget.total_bytes += len(payload)
        budget.count += 1
    except KinematicsCheckFailure:
        raise
    except Exception as error:
        raise KinematicsCheckFailure(
            "artifact_write_failed",
            f"cannot write {path}: {type(error).__name__}: {error}",
        ) from error


def _save_rgb(
    run_dir: Path,
    budget: ArtifactBudget,
    layout_id: str,
    phase: str,
    camera: str,
    image: object,
) -> str:
    from PIL import Image

    path = run_dir / "artifacts" / "layouts" / layout_id / phase / f"{camera}.png"
    _save_png(path, Image.fromarray(image), budget)
    return path.relative_to(run_dir).as_posix()


def _camera_objects(config: KinematicsCheckConfig, camera: str) -> tuple[str, ...]:
    objects = list(config.object_names)
    if camera == "left_wrist":
        objects.append("table")
    return tuple(objects)


def _vision_document(
    layout_id: str,
    phase: str,
    segmentations: dict[str, object],
    geom_ids: dict[str, int],
    config: KinematicsCheckConfig,
    rgb_paths: dict[str, str],
) -> dict[str, object]:
    cameras: list[dict[str, object]] = []
    first_failure: tuple[str, str, int, int] | None = None
    for camera in config.camera_names:
        object_documents: list[dict[str, object]] = []
        camera_passed = True
        for obj in _camera_objects(config, camera):
            rule = config.thresholds(camera, obj, phase)
            geom_id = geom_ids[obj]
            pixels = visibility.count_visible_pixels(segmentations[camera], geom_id)
            passed = rule.record_only or (rule.min_pixels is not None and pixels >= rule.min_pixels)
            if not passed and first_failure is None:
                assert rule.min_pixels is not None
                first_failure = (camera, obj, pixels, rule.min_pixels)
            camera_passed = camera_passed and passed
            object_documents.append(
                {
                    "object": obj,
                    "geom_id": geom_id,
                    "pixels": pixels,
                    "mode": rule.mode,
                    "threshold": rule.min_pixels,
                    "passed": passed,
                    "fully_occluded": pixels == 0,
                }
            )
        cameras.append(
            {
                "camera": camera,
                "passed": camera_passed,
                "objects": object_documents,
            }
        )
    passed = all(camera["passed"] for camera in cameras)
    return {
        "layout_id": layout_id,
        "phase": phase,
        "cameras": cameras,
        "passed": passed,
        "first_failure": None if first_failure is None else list(first_failure),
        "rgb": rgb_paths,
    }


def _right_updates(
    qpos_right: tuple[float, ...], addresses: tuple[int, ...]
) -> dict[int, tuple[float, ...]]:
    return {address: (value,) for address, value in zip(addresses, qpos_right, strict=True)}


def _write_overview(
    run_dir: Path,
    results: list[dict[str, object]],
    config: KinematicsCheckConfig,
    budget: ArtifactBudget,
) -> str | None:
    from PIL import Image, ImageDraw

    selected = next((item for item in results if item.get("vision")), None)
    if selected is None:
        return None
    layout_id = str(selected["layout_id"])
    vision = selected["vision"]  # type: ignore[index]
    phases = [item["phase"] for item in vision]
    paths = {item["phase"]: item["rgb"] for item in vision}
    cameras = list(config.camera_names)
    try:
        opened: list[list[Image.Image]] = []
        for phase in phases:
            row: list[Image.Image] = []
            for camera in cameras:
                relative = paths[phase][camera]
                with Image.open(run_dir / relative) as source:
                    row.append(source.convert("RGB"))
            opened.append(row)
        tile_width = max(image.width for row in opened for image in row)
        tile_height = max(image.height for row in opened for image in row)
        margin = 8
        gap = 6
        header = 18
        label = 14
        columns = len(cameras)
        rows = len(opened)
        sheet_width = margin * 2 + columns * (tile_width + gap) - gap
        sheet_height = margin * 2 + rows * (header + tile_height + label + gap) - gap
        sheet = Image.new("RGB", (sheet_width, sheet_height), (245, 245, 245))
        draw = ImageDraw.Draw(sheet)
        for row_index, (phase, images) in enumerate(zip(phases, opened, strict=True)):
            top = margin + row_index * (header + tile_height + label + gap)
            draw.text(
                (margin, top),
                f"layout={layout_id} phase={phase}",
                fill=(0, 0, 0),
            )
            image_top = top + header
            for column, (camera, image) in enumerate(zip(cameras, images, strict=True)):
                left = margin + column * (tile_width + gap)
                sheet.paste(image, (left, image_top))
                draw.rectangle((left, image_top, left + 150, image_top + label), fill=(0, 0, 0))
                draw.text(
                    (left + 3, image_top + 1),
                    f"{camera}",
                    fill=(255, 255, 255),
                )
        path = run_dir / "artifacts" / "overview.png"
        _save_png(path, sheet, budget)
        return path.relative_to(run_dir).as_posix()
    except KinematicsCheckFailure:
        raise
    except Exception as error:
        raise KinematicsCheckFailure(
            "overview_write_failed", f"{type(error).__name__}: {error}"
        ) from error


@dataclass(frozen=True)
class PreparedLayout:
    snapshot: Snapshot
    qpos: np.ndarray
    objects: tuple[dict[str, object], ...]
    targets: tuple[kinematics.KeyPoseTarget, ...]


@dataclass(frozen=True)
class MotionChecks:
    initial: PathResult
    ik: tuple[IKResult, ...]
    paths: tuple[PathResult, ...]
    expected_target_count: int

    @property
    def all_targets_solved(self) -> bool:
        return len(self.ik) == self.expected_target_count and all(
            result.converged for result in self.ik
        )


def _prepare_layout(
    scene: Scene,
    renderer: SceneRenderer,
    scene_config: SceneConfig,
    layout: LayoutSpec,
    *,
    base_seed: int,
    target_config: TargetConfig,
) -> PreparedLayout:
    """Build the initial diagnostic snapshot; live objects are never moved."""
    scene.reset(layout.seed if layout.kind == "seed" else base_seed)
    snapshot = renderer.capture(scene.data)
    qpos = scene.data.qpos.copy()
    if layout.kind == "boundary":
        updates = _boundary_updates(layout, scene, scene_config)
        snapshot = renderer.derive_snapshot(snapshot, updates)
        qpos = _apply_updates(qpos, updates)
    poses = {
        obj.name: extract_object_pose(
            qpos, scene.indices.object_free_joint_qpos[obj.name], obj.name
        )
        for obj in scene_config.objects
    }
    half_z = {obj.name: obj.half_extents[2] for obj in scene_config.objects}
    targets = build_targets(
        poses["source"],
        poses["target"],
        table_top_z=scene_config.table.top_z,
        source_half_z=half_z["source"],
        target_half_z=half_z["target"],
        config=target_config,
    )
    objects = tuple(
        {"name": name, "center": list(pose.center), "yaw_deg": math.degrees(pose.yaw_rad)}
        for name, pose in poses.items()
    )
    qpos.flags.writeable = False
    return PreparedLayout(snapshot, qpos, objects, targets)


def _check_motion(
    scene: Scene,
    prepared: PreparedLayout,
    pairs: kinematics.CollisionPairSet,
    config: KinematicsCheckConfig,
    *,
    remaining_samples: int,
) -> MotionChecks:
    """Solve targets, then check paths starting at the actual initial home."""
    initial = check_initial_pose(scene.model, prepared.qpos, pairs, config.clearance)
    solved = solve_key_poses(
        scene.model, prepared.qpos, prepared.targets, config.ik, config.clearance, pairs
    )
    if len(solved) != len(prepared.targets) or not all(result.converged for result in solved):
        return MotionChecks(initial, solved, (), len(prepared.targets))
    addresses = scene.indices.arm_joint_qpos["right"]
    previous = tuple(float(prepared.qpos[address]) for address in addresses)
    previous_phase = "initial"
    paths = []
    for target in solved:
        sample_count = len(
            kinematics.sample_joint_path(
                previous,
                target.qpos_right,
                max_step_rad=config.clearance.sample_step_rad,
                max_samples=config.clearance.max_samples_per_segment,
            )
        )
        if sample_count > remaining_samples:
            raise KinematicsCheckFailure(
                "sample_budget_exceeded",
                f"segment {previous_phase}_to_{target.phase} needs {sample_count} samples; "
                f"{remaining_samples} remain",
            )
        path = check_path(
            scene.model,
            prepared.qpos,
            previous,
            target.qpos_right,
            pairs,
            config.clearance,
            segment=f"{previous_phase}_to_{target.phase}",
        )
        paths.append(path)
        remaining_samples -= path.sample_count
        previous, previous_phase = target.qpos_right, target.phase
    return MotionChecks(initial, solved, tuple(paths), len(prepared.targets))


def _render_key_pose(
    run_dir: Path,
    renderer: SceneRenderer,
    prepared: PreparedLayout,
    solved: IKResult,
    *,
    layout_id: str,
    right_addresses: tuple[int, ...],
    geom_ids: dict[str, int],
    config: KinematicsCheckConfig,
    budget: ArtifactBudget,
) -> dict[str, object]:
    snapshot = renderer.derive_snapshot(
        prepared.snapshot, _right_updates(solved.qpos_right, right_addresses)
    )
    rgb = renderer.render_rgb(snapshot)
    paths = {
        camera: _save_rgb(run_dir, budget, layout_id, solved.phase, camera, image)
        for camera, image in rgb.items()
    }
    segmentation = renderer.render_segmentation(snapshot)
    return _vision_document(layout_id, solved.phase, segmentation, geom_ids, config, paths)


def _motion_failures(
    motion: MotionChecks, vision: list[dict[str, object]]
) -> list[dict[str, object]]:
    failures = []
    for path in (motion.initial, *motion.paths):
        if not path.passed:
            failures.append(
                {
                    "reason": path.failure_reason or "clearance_below_minimum",
                    "message": path.failure_message or f"{path.segment} failed",
                    "phase": path.segment,
                }
            )
    for solved in motion.ik:
        if not solved.converged:
            failures.append(
                {
                    "reason": solved.reason or "ik_not_converged",
                    "message": solved.message or f"{solved.phase} did not converge",
                    "phase": solved.phase,
                }
            )
    if len(motion.ik) != motion.expected_target_count and all(item.converged for item in motion.ik):
        failures.append(
            {
                "reason": "ik_target_count_mismatch",
                "message": (
                    f"expected {motion.expected_target_count} IK results, got {len(motion.ik)}"
                ),
                "phase": None,
            }
        )
    for view in vision:
        if not view["passed"]:
            camera, obj, pixels, threshold = view["first_failure"]
            failures.append(
                {
                    "reason": "visibility_threshold_not_met",
                    "message": (
                        f"layout {view['layout_id']!r} phase {view['phase']!r} "
                        f"camera {camera!r} object {obj!r} has {pixels} pixels < {threshold}"
                    ),
                    "phase": view["phase"],
                    "camera": camera,
                    "object": obj,
                }
            )
    return failures


def _check_one_layout(
    run_dir: Path,
    scene: Scene,
    scene_config: SceneConfig,
    layout: LayoutSpec,
    *,
    base_seed: int,
    config: KinematicsCheckConfig,
    renderer: SceneRenderer,
    pairs: kinematics.CollisionPairSet,
    geom_ids: dict[str, int],
    budget: ArtifactBudget,
    remaining_samples: int,
) -> dict[str, object]:
    started = time.perf_counter()
    result = {
        "layout_id": layout.layout_id,
        "kind": layout.kind,
        "seed": layout.seed,
        "objects": [],
        "initial_clearance": None,
        "ik": [],
        "paths": [],
        "vision": [],
        "failures": [],
    }
    try:
        prepared = _prepare_layout(
            scene, renderer, scene_config, layout, base_seed=base_seed, target_config=config.targets
        )
        result["objects"] = list(prepared.objects)
        motion = _check_motion(scene, prepared, pairs, config, remaining_samples=remaining_samples)
        result["initial_clearance"] = _path_document(motion.initial)
        result["ik"] = [_ik_document(solved) for solved in motion.ik]
        result["paths"] = [_path_document(path) for path in motion.paths]
        if motion.all_targets_solved:
            result["vision"] = [
                _render_key_pose(
                    run_dir,
                    renderer,
                    prepared,
                    solved,
                    layout_id=layout.layout_id,
                    right_addresses=tuple(scene.indices.arm_joint_qpos["right"]),
                    geom_ids=geom_ids,
                    config=config,
                    budget=budget,
                )
                for solved in motion.ik
            ]
        result["failures"] = _motion_failures(motion, result["vision"])
    except KinematicsError as error:
        if error.reason == "mink_constraint_api_mismatch":
            raise KinematicsCheckFailure(error.reason, error.message) from error
        result["failures"].append({"reason": error.reason, "message": error.message, "phase": None})
    except KinematicsCheckFailure:
        raise
    except Exception as error:
        raise KinematicsCheckFailure(
            str(getattr(error, "reason", "kinematics_check_failed")),
            str(getattr(error, "message", f"{type(error).__name__}: {error}")),
        ) from error
    result["passed"] = not result["failures"]
    result["elapsed_s"] = round(time.perf_counter() - started, 6)
    return result


def _run_layouts(
    run_dir: Path,
    scene: Scene,
    scene_config: SceneConfig,
    check_config: SceneCheckConfig,
    kinematics_config: KinematicsCheckConfig,
    renderer: SceneRenderer,
    logger: object,
    report: dict[str, object],
    budget: ArtifactBudget,
) -> tuple[list[dict[str, object]], KinematicsCheckFailure | None]:
    pairs = build_collision_pairs(scene.model)
    report["collision_pairs"] = {
        "total_effective_pairs": len(pairs.pairs),
        "categories": pairs.categories,
        "exclusions": pairs.exclusions,
        "total_physical_pairs_before_robot_filter": pairs.total_physical_pairs,
        "explicit_pair_count": pairs.explicit_pair_count,
        "fixed_connection_note": (
            "world-welded and fixed connections do not count as free-motion clearance"
        ),
    }
    _emit(
        logger,
        "collision_pairs_ready",
        "effective robot collision pairs enumerated",
        {"pairs": len(pairs.pairs), **pairs.categories},
    )
    geom_ids = {**report["scene"]["object_geom_ids"], "table": report["scene"]["table_geom_id"]}
    results = []
    report["layouts"] = results  # Preserve completed layouts if a later resource fails.
    first_failure = None
    remaining = kinematics_config.clearance.max_total_samples
    for layout in visibility.build_layouts(check_config, scene_config):
        result = _check_one_layout(
            run_dir,
            scene,
            scene_config,
            layout,
            base_seed=check_config.seeds[0],
            config=kinematics_config,
            renderer=renderer,
            pairs=pairs,
            geom_ids=geom_ids,
            budget=budget,
            remaining_samples=remaining,
        )
        results.append(result)
        remaining -= sum(path["sample_count"] for path in result["paths"])
        if first_failure is None and result["failures"]:
            failure = result["failures"][0]
            first_failure = KinematicsCheckFailure(failure["reason"], failure["message"])
        _emit(
            logger,
            "layout_passed" if result["passed"] else "layout_failed",
            "kinematics layout diagnostic finished",
            {
                "layout": layout.layout_id,
                "kind": layout.kind,
                "passed": result["passed"],
                "ik_targets": len(result["ik"]),
                "paths": len(result["paths"]),
                "phases": len(result["vision"]),
                "elapsed_s": result["elapsed_s"],
            },
        )
    return results, first_failure


def _serialized_size(document: object) -> int:
    # Must match write_json_atomic's actual encoding byte for byte.
    return len((json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n").encode())


def _minimal_report(report: dict[str, object], reason: str, message: str) -> dict[str, object]:
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "kind": KIND,
        "component": COMPONENT,
        "run_id": report.get("run_id"),
        "instance_id": report.get("instance_id"),
        "status": "failed",
        "exit_code": 1,
        "reason": reason,
        "error": {"type": "KinematicsCheckFailure", "message": message[:2000]},
        "logs_closed": report.get("logs_closed", False),
        "report_budget_exceeded": True,
        "started_at_utc": report.get("started_at_utc"),
        "finished_at_utc": report.get("finished_at_utc"),
        "elapsed_s": report.get("elapsed_s"),
    }


def _new_report(arguments: KinematicsCheckArguments, instance_id: str) -> dict[str, object]:
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "kind": KIND,
        "component": COMPONENT,
        "run_id": arguments.run_id,
        "instance_id": instance_id,
        "status": "failed",
        "exit_code": 1,
        "reason": "not_started",
        "started_at_utc": utc_now(),
        "finished_at_utc": None,
        "elapsed_s": None,
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
            "executable": sys.executable,
        },
        "environment": {
            "MUJOCO_GL": os.environ.get("MUJOCO_GL"),
            "PYOPENGL_PLATFORM": os.environ.get("PYOPENGL_PLATFORM"),
        },
        "configs": None,
        "dependencies": None,
        "source": None,
        "scene": None,
        "collision_pairs": None,
        "layouts": [],
        "summary": None,
        "failures": [],
        "artifact": None,
        "artifacts": None,
        "log_directory": None,
        "logs_closed": False,
        "renderer_close_error": None,
        "logger_close_error": None,
        "error": None,
        "limitations": [
            "Linear joint-space sampling is discrete, not a continuous collision proof.",
            "The gripper stays open and both objects stay at their initial poses; "
            "lift/place are empty-hand key-pose previews, not object transport or stacking.",
            "IK dt is a solver step, not a controller velocity or execution period.",
        ],
    }


def _close_resources(
    logger: object | None,
    renderer: object | None,
    report: dict[str, object],
    failure: KinematicsCheckFailure | None,
) -> KinematicsCheckFailure | None:
    if logger is not None:
        try:
            _emit(
                logger,
                "finalizing",
                "kinematics check finalizing resources",
                {"failed": failure is not None},
            )
        except KinematicsCheckFailure as error:
            if failure is None:
                failure = error
    if renderer is not None:
        try:
            renderer.close()
        except Exception as error:
            report["renderer_close_error"] = (
                f"{getattr(error, 'reason', 'renderer_close_failed')}: {error}"
            )
            if failure is None:
                failure = KinematicsCheckFailure(
                    "renderer_close_failed", report["renderer_close_error"]
                )
    try:
        if logger is not None:
            if failure is None:
                _emit(
                    logger,
                    "checks_finished",
                    "kinematics diagnostics and artifact writes finished; closing resources",
                )
            else:
                _emit(
                    logger,
                    "kinematics_check_failed",
                    "kinematics key-pose check failed",
                    {"reason": failure.reason},
                )
            logger.check_health()
    except Exception as error:
        if failure is None:
            failure = KinematicsCheckFailure(
                "event_log_failed", f"logger finalization: {type(error).__name__}: {error}"
            )
    log_close_error: str | None = None
    if logger is not None:
        try:
            logger.close()
        except Exception as error:
            log_close_error = f"{type(error).__name__}: {error}"
            if failure is None:
                failure = KinematicsCheckFailure(
                    "logger_close_failed", f"event logger close failed: {log_close_error}"
                )
    report["logs_closed"] = logger is not None and log_close_error is None
    report["logger_close_error"] = log_close_error
    return failure


def _summarize_results(
    results: list[dict[str, object]], check_config: SceneCheckConfig
) -> dict[str, object]:
    all_paths = [
        path
        for result in results
        for path in result["paths"]  # type: ignore[index]
    ]
    clearance_entries = []
    for result in results:
        initial = result["initial_clearance"]
        if initial is not None:
            clearance_entries.append(
                {
                    "layout": result["layout_id"],
                    "segment": "initial",
                    "minimum_clearance": initial["minimum_clearance"],
                }
            )
        for path in result["paths"]:
            clearance_entries.append(
                {
                    "layout": result["layout_id"],
                    "segment": path["segment"],
                    "minimum_clearance": path["minimum_clearance"],
                }
            )
    minimum = min(
        clearance_entries,
        key=lambda entry: entry["minimum_clearance"]["distance_m"],
        default=None,
    )
    return {
        "layout_count": len(results),
        "layouts_passed": sum(1 for result in results if result["passed"]),
        "layouts_failed": sum(1 for result in results if not result["passed"]),
        "ik_targets": sum(len(result["ik"]) for result in results),
        "ik_targets_converged": sum(
            1 for result in results for target in result["ik"] if target["converged"]
        ),
        "segments": len(all_paths),
        "path_samples": sum(path["sample_count"] for path in all_paths),
        "minimum_clearance": minimum,
        "phases_with_vision": sum(len(result["vision"]) for result in results),
        "boundary_count": check_config.boundary_layout_count,
        "seed_count": len(check_config.seeds),
    }


def _publish_report(
    run_dir: Path,
    report: dict[str, object],
    failure: KinematicsCheckFailure | None,
    budget: ArtifactBudget | None,
    *,
    started: float,
    max_report_bytes: int,
) -> int:
    report["elapsed_s"] = round(time.monotonic() - started, 3)
    report["finished_at_utc"] = utc_now()
    if failure is None:
        report["status"] = "succeeded"
        report["exit_code"] = 0
        report["reason"] = None
    else:
        report["status"] = "failed"
        report["exit_code"] = 1
        report["reason"] = failure.reason
        report["error"] = {
            "type": type(failure).__name__,
            "message": failure.message[:2000],
        }
        recorded = report.get("failures")
        if not isinstance(recorded, list):
            recorded = []
        if not any(
            isinstance(item, dict)
            and item.get("reason") == failure.reason
            and item.get("message") == failure.message[:2000]
            for item in recorded
        ):
            recorded.append(
                {
                    "layout": None,
                    "phase": None,
                    "reason": failure.reason,
                    "message": failure.message[:2000],
                }
            )
        report["failures"] = recorded
    if budget is not None:
        report["artifacts"] = report.get("artifacts") or {
            "png_count": budget.count,
            "png_bytes": budget.total_bytes,
            "png_limit_bytes": budget.limit_bytes,
        }
    try:
        size = _serialized_size(report)
        if size > max_report_bytes:
            reason = failure.reason if failure is not None else "report_budget_exceeded"
            message = (
                failure.message
                if failure is not None
                else f"report needs {size} bytes, limit {max_report_bytes}"
            )
            report = _minimal_report(report, reason, message)
            if failure is None:
                failure = KinematicsCheckFailure("report_budget_exceeded", message)
            report["reason"] = failure.reason
            report["error"] = {
                "type": type(failure).__name__,
                "message": failure.message[:2000],
            }
    except (TypeError, ValueError) as error:
        report = _minimal_report(
            report,
            "report_serialization_failed",
            f"{type(error).__name__}: {error}",
        )
        failure = failure or KinematicsCheckFailure("report_serialization_failed", str(error))
    report_path = run_dir / "report.json"
    try:
        write_json_atomic(report_path, report)
    except Exception as error:
        print(f"error: report_write_failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(f"report: {report_path}")
    return int(report["exit_code"])


def run(arguments: KinematicsCheckArguments) -> int:
    run_dir = arguments.report_root / arguments.run_id
    if os.path.lexists(run_dir):
        raise KinematicsCheckUsageError(f"report run directory already exists: {run_dir}")
    arguments.report_root.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(exist_ok=False)
    started = time.monotonic()
    instance_id = str(uuid.uuid4())
    report = _new_report(arguments, instance_id)
    logger: object | None = None
    failure: KinematicsCheckFailure | None = None
    renderer: object | None = None
    budget: ArtifactBudget | None = None
    try:
        logger = _open_logger(run_dir, arguments.run_id, instance_id)
        report["log_directory"] = Path(logger.directory).relative_to(run_dir).as_posix()
        _emit(logger, "kinematics_check_start", "kinematics key-pose check started")
        scene_config, check_config, kinematics_config = _load_configs(arguments)
        report["configs"] = {
            "scene": {
                "path": str(arguments.scene_config),
                "sha256": sha256_file(arguments.scene_config),
                "resolved": scene_config_document(scene_config),
            },
            "check": {
                "path": str(arguments.check_config),
                "sha256": sha256_file(arguments.check_config),
                "resolved": visibility_document(check_config),
            },
            "kinematics": {
                "path": str(arguments.kinematics_config),
                "sha256": sha256_file(arguments.kinematics_config),
                "resolved": _kinematics_config_document(kinematics_config),
            },
        }
        _emit(
            logger,
            "configs_loaded",
            "scene, layout and kinematics configurations validated before GL",
            {
                "seeds": len(check_config.seeds),
                "boundary_layouts": check_config.boundary_layout_count,
                "layouts": check_config.layout_count,
                "max_iterations": kinematics_config.ik.max_iterations,
            },
        )
        try:
            dependencies = probe_environment.check_dependencies()
            probe_environment._verify_mounted_source(dependencies)
        except probe_environment.ProbeFailure as error:
            raise KinematicsCheckFailure(error.reason, error.message) from error
        report["dependencies"] = dependencies
        report["source"] = {"model_directory": str(assets.MODEL_DIR), "code": code_provenance()}
        scene = _build_scene(scene_config)
        report["scene"] = _scene_info(scene, scene_config)
        _emit(
            logger,
            "model_loaded",
            "local Panda model compiled into the scene",
            {"model_directory": str(assets.MODEL_DIR)},
        )
        renderer = _make_renderer(scene, scene_config)
        budget = ArtifactBudget(kinematics_config.budgets.max_png_bytes)
        results, layout_failure = _run_layouts(
            run_dir,
            scene,
            scene_config,
            check_config,
            kinematics_config,
            renderer,
            logger,
            report,
            budget,
        )
        report["layouts"] = results
        report["failures"] = [
            {"layout": result["layout_id"], **failure_document}
            for result in results
            for failure_document in result["failures"]  # type: ignore[index]
        ]
        report["summary"] = _summarize_results(results, check_config)
        overview = _write_overview(run_dir, results, kinematics_config, budget)
        report["artifact"] = overview
        report["artifacts"] = {
            "overview": overview,
            "layout_rgb_root": "artifacts/layouts",
            "png_count": budget.count,
            "png_bytes": budget.total_bytes,
            "png_limit_bytes": budget.limit_bytes,
        }
        if layout_failure is not None:
            failure = layout_failure
    except KinematicsCheckFailure as error:
        failure = error
    except KinematicsError as error:
        failure = KinematicsCheckFailure(error.reason, error.message)
    except SceneConfigError as error:
        failure = KinematicsCheckFailure("scene_config_invalid", str(error))
    except SceneCheckConfigError as error:
        failure = KinematicsCheckFailure("check_config_invalid", str(error))
    except KinematicsConfigError as error:
        failure = KinematicsCheckFailure("kinematics_config_invalid", str(error))
    except Exception as error:
        if logger is None:
            failure = KinematicsCheckFailure(
                "logger_init_failed", f"{type(error).__name__}: {error}"
            )
        else:
            failure = KinematicsCheckFailure("unexpected_error", f"{type(error).__name__}: {error}")
    finally:
        failure = _close_resources(logger, renderer, report, failure)
    max_report_bytes = (
        kinematics_config.budgets.max_report_bytes
        if "kinematics_config" in locals()
        else 2 * 1024 * 1024
    )
    return _publish_report(
        run_dir,
        report,
        failure,
        budget,
        started=started,
        max_report_bytes=max_report_bytes,
    )


def visibility_document(config: SceneCheckConfig) -> dict[str, object]:
    """Serialize the accepted layout check config without importing its private code."""
    return {
        "source": config.source,
        "schema_version": config.schema_version,
        "seeds": list(config.seeds),
        "boundary_xy_corners": [list(corner) for corner in config.boundary_xy_corners],
        "boundary_yaw_signs": list(config.boundary_yaw_signs),
        "min_visible_pixels": {
            camera: dict(objects) for camera, objects in config.min_visible_pixels.items()
        },
        "max_layouts": config.max_layouts,
        "throughput": {
            "warmup_groups": config.throughput.warmup_groups,
            "measure_groups": config.throughput.measure_groups,
        },
        "camera_names": list(config.camera_names),
        "object_names": list(config.object_names),
        "camera_resolutions": {
            camera: list(resolution) for camera, resolution in config.camera_resolutions.items()
        },
        "layout_count": config.layout_count,
        "boundary_layout_count": config.boundary_layout_count,
    }


def main(argv: list[str] | None = None) -> int:
    try:
        arguments = parse_arguments(argv)
    except KinematicsCheckUsageError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    try:
        return run(arguments)
    except KinematicsCheckUsageError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    except OSError as error:
        print(f"error: report_io_error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
