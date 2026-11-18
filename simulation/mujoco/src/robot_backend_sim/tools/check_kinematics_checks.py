"""Layout preparation, motion checks and bounded PNG evidence helpers."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from robot_backend_sim.panda_sim.kinematics_config import KinematicsCheckConfig, TargetConfig
from robot_backend_sim.panda_sim.stack.scene_config import SceneConfig
from robot_backend_sim.common.kinematics import common
from robot_backend_sim.panda_sim import kinematics
from robot_backend_sim.common.kinematics.common import extract_object_pose
from robot_backend_sim.panda_sim.kinematics import (
    IKResult,
    KinematicsError,
    PathResult,
    build_collision_pairs,
    build_targets,
    check_initial_pose,
    check_path,
    solve_key_poses,
)
from robot_backend_sim.common.render import visibility
from robot_backend_sim.panda_sim.stack import visibility as panda_visibility
from robot_backend_sim.panda_sim.stack.visibility import LayoutSpec, SceneCheckConfig
from robot_backend_sim.tools.check_kinematics_common import (
    ArtifactBudget,
    KinematicsCheckFailure,
    _emit,
)

if TYPE_CHECKING:
    from robot_backend_sim.panda_sim.stack.scene import Scene
    from robot_backend_sim.common.render.rendering import SceneRenderer, Snapshot


def _apply_updates(qpos: object, updates: dict[int, tuple[float, ...]]) -> object:
    import numpy as np

    result = np.array(qpos, dtype=np.float64, copy=True)
    for address, values in updates.items():
        result[address : address + len(values)] = values
    return result

def _boundary_updates(
    layout: LayoutSpec, scene: object, scene_config: SceneConfig
) -> dict[int, tuple[float, ...]]:
    by_name = {placement.name: placement for placement in layout.placements}
    updates: dict[int, tuple[float, ...]] = {}
    for obj in scene_config.objects:
        placement = by_name[obj.name]
        address = scene.indices.environment.object_free_joint_qpos[obj.name]
        z = scene_config.table.top_z + obj.half_extents[2]
        updates[address] = (
            placement.x,
            placement.y,
            z,
            *visibility.yaw_quaternion(placement.yaw_deg),
        )
    return updates

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
            qpos, scene.indices.environment.object_free_joint_qpos[obj.name], obj.name
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
    addresses = scene.indices.robot.arm_joint_qpos["right"]
    previous = tuple(float(prepared.qpos[address]) for address in addresses)
    previous_phase = "initial"
    paths = []
    for target in solved:
        sample_count = len(
            common.sample_joint_path(
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
                    right_addresses=tuple(scene.indices.robot.arm_joint_qpos["right"]),
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
    pairs = build_collision_pairs(
        scene.model,
        robot_prefixes=("left_", "right_"),
        robot_sides={"left": ("left_",), "right": ("right_",)},
        object_names=("source", "target"),
    )
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
    for layout in panda_visibility.build_layouts(check_config, scene_config):
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

