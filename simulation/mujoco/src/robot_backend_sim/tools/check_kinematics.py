"""Thin orchestration for the S2 key-pose, IK and clearance diagnostics.

The module validates all configurations before native work, loads the accepted
scene once, then keeps every layout on private snapshots.  It writes bounded
JSON evidence and RGB/segmentation-derived visibility counts; it does not step
physics, execute a grasp, start ROS or create training data.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from robot_backend_sim.common.physics import assets
from robot_backend_sim.common.runtime.diagnostics import code_provenance, sha256_file
from robot_backend_sim.marvin_sim.stack import visibility
from robot_backend_sim.marvin_sim.stack.diagnostics import scene_config_document
from robot_backend_sim.marvin_sim.stack.kinematics_config import (
    KinematicsCheckConfig,
    KinematicsConfigError,
    load_kinematics_config,
)
from robot_backend_sim.marvin_sim.stack.plan import KinematicsError
from robot_backend_sim.marvin_sim.stack.scene_config import (
    MarvinStackSceneConfig,
    StackSceneConfigError,
    load_marvin_stack_scene_config,
)
from robot_backend_sim.marvin_sim.stack.visibility import (
    SceneCheckConfig,
    SceneCheckConfigError,
    load_check_config,
)

if TYPE_CHECKING:

    pass

from robot_backend_sim.tools import probe_environment
from robot_backend_sim.tools.check_kinematics_checks import _run_layouts, _write_overview
from robot_backend_sim.tools.check_kinematics_checks import _save_png as _save_png
from robot_backend_sim.tools.check_kinematics_checks import _vision_document as _vision_document
from robot_backend_sim.tools.check_kinematics_common import (
    ArtifactBudget,
    KinematicsCheckFailure,
    KinematicsCheckUsageError,
    _emit,
)
from robot_backend_sim.tools.check_kinematics_report import (
    _close_resources,
    _new_report,
    _publish_report,
    _summarize_results,
)

COMPONENT = "kinematics_check"
KIND = "kinematics"
_SHARED_ROOT = Path(os.environ.get("SHARED_ROOT", "/shared"))
_SHARED_RECIPES = (
    _SHARED_ROOT / "scenarios/marvin_sim/stack/configs/simulation"
)
DEFAULT_SCENE_CONFIG = _SHARED_RECIPES / "scene/marvin_stack.yaml"
DEFAULT_CHECK_CONFIG = _SHARED_RECIPES / "checks/scene_check.yaml"
DEFAULT_KINEMATICS_CONFIG = _SHARED_RECIPES / "checks/kinematics_check.yaml"
_RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


@dataclass(frozen=True, slots=True)
class KinematicsCheckArguments:
    scene_config: Path
    check_config: Path
    kinematics_config: Path
    report_root: Path
    run_id: str


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
    parser.add_argument(
        "--run-id",
        default=None,
        help="fresh report run identity (default: kinematics-check-<UTC>-<pid>)",
    )
    return parser


def parse_arguments(argv: list[str] | None = None) -> KinematicsCheckArguments:
    parsed = _parser().parse_args(argv)
    run_id = (
        f"kinematics-check-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{os.getpid()}"
        if parsed.run_id is None
        else parsed.run_id
    )
    return KinematicsCheckArguments(
        scene_config=parsed.scene_config.resolve(),
        check_config=parsed.check_config.resolve(),
        kinematics_config=parsed.kinematics_config.resolve(),
        report_root=_validate_report_root(parsed.report_root),
        run_id=_validate_run_id(run_id),
    )


def _load_configs(
    arguments: KinematicsCheckArguments,
) -> tuple[MarvinStackSceneConfig, SceneCheckConfig, KinematicsCheckConfig]:
    try:
        scene_config = load_marvin_stack_scene_config(arguments.scene_config)
    except StackSceneConfigError as error:
        raise KinematicsCheckFailure("scene_config_invalid", str(error)) from error
    try:
        check_config = load_check_config(arguments.check_config, scene_config)
    except SceneCheckConfigError as error:
        raise KinematicsCheckFailure("check_config_invalid", str(error)) from error
    try:
        kinematics_config = load_kinematics_config(arguments.kinematics_config, scene_config)
    except KinematicsConfigError as error:
        raise KinematicsCheckFailure("kinematics_config_invalid", str(error)) from error
    return scene_config, check_config, kinematics_config


def _kinematics_config_document(config: KinematicsCheckConfig) -> dict[str, object]:
    return {
        "source": config.source,
        "ik": {
            "dt_s": config.ik.dt_s,
            "max_iterations": config.ik.max_iterations,
            "position_tolerance_m": config.ik.position_tolerance_m,
            "orientation_tolerance_rad": config.ik.orientation_tolerance_rad,
            "posture_cost": config.ik.posture_cost,
            "position_cost": config.ik.position_cost,
            "orientation_cost": config.ik.orientation_cost,
            "lm_damping": config.ik.lm_damping,
            "limit_margin_rad": config.ik.limit_margin_rad,
            "solver": config.ik.solver,
            "initial_right_joint_positions": list(config.ik.initial_right_joint_positions),
        },
        "clearance": {
            "minimum_clearance_m": config.clearance.minimum_clearance_m,
            "collision_detection_distance_m": config.clearance.collision_detection_distance_m,
            "sample_step_rad": config.clearance.sample_step_rad,
            "max_samples_per_segment": config.clearance.max_samples_per_segment,
            "max_total_samples": config.budgets.max_total_samples,
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
                    "phase": rule.phase,
                    "mode": rule.mode,
                    "min_pixels": rule.min_pixels,
                }
                for rule in config.visibility_rules
            ]
        },
        "budgets": {
            "max_png_bytes": config.budgets.max_png_total_bytes,
            "max_report_bytes": config.budgets.max_report_bytes,
        },
        "camera_names": list(config.camera_names),
        "object_names": list(config.object_names),
        "camera_resolutions": {
            camera: list(resolution) for camera, resolution in config.camera_resolutions.items()
        },
    }


def _build_scene(scene_config: MarvinStackSceneConfig) -> object:
    from robot_backend_sim.marvin_sim.stack.scene import (
        SceneBuildError,
        build_marvin_stack_scene,
    )

    try:
        return build_marvin_stack_scene(scene_config)
    except SceneBuildError as error:
        raise KinematicsCheckFailure("scene_build_failed", str(error)) from error


def _scene_info(scene: object, scene_config: MarvinStackSceneConfig) -> dict[str, object]:
    model = scene.model
    try:
        object_geom_ids = {
            obj.name: int(model.geom(f"{obj.name}_geom").id) for obj in scene_config.objects
        }
        table_geom_id = int(model.geom("table_top").id)
        tcp_site_id = int(model.site("Arm_R_tcp").id)
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


def _make_renderer(scene: object, scene_config: MarvinStackSceneConfig) -> object:
    from robot_backend_sim.common.render.rendering import SceneRenderer

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


def _load_check_configs(arguments, report: dict):
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
    return scene_config, kinematics_config


def _execute_checks(arguments, run_dir: Path, report: dict, logger):
    _emit(logger, "kinematics_check_start", "kinematics key-pose check started")
    scene_config, kinematics_config = _load_check_configs(arguments, report)
    _emit(
        logger,
        "configs_loaded",
        "scene, layout and kinematics configurations validated before GL",
        {
            "layout": visibility.NOMINAL_LAYOUT.layout_id,
            "max_iterations": kinematics_config.ik.max_iterations,
        },
    )
    try:
        dependencies = probe_environment.check_dependencies()
        probe_environment._verify_mounted_source(dependencies)
    except probe_environment.ProbeFailure as error:
        raise KinematicsCheckFailure(error.reason, error.message) from error
    report["dependencies"] = dependencies
    model_dir = str(assets.robot_model_path(scene_config.robot.assembly).parent)
    report["source"] = {"model_directory": model_dir, "code": code_provenance()}
    scene = _build_scene(scene_config)
    report["scene"] = _scene_info(scene, scene_config)
    _emit(
        logger,
        "model_loaded",
        "local robot model compiled into the scene",
        {"model_directory": model_dir},
    )
    renderer = _make_renderer(scene, scene_config)
    budget = ArtifactBudget(kinematics_config.budgets.max_png_total_bytes)
    results, layout_failure = _run_layouts(
        run_dir,
        scene,
        scene_config,
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
    report["summary"] = _summarize_results(results)
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
        raise layout_failure
    return renderer, budget, kinematics_config


def _classify_failure(logger, error: BaseException) -> KinematicsCheckFailure:
    if isinstance(error, KinematicsCheckFailure):
        return error
    if isinstance(error, KinematicsError):
        return KinematicsCheckFailure(error.reason, error.message)
    if isinstance(error, StackSceneConfigError):
        return KinematicsCheckFailure("scene_config_invalid", str(error))
    if isinstance(error, SceneCheckConfigError):
        return KinematicsCheckFailure("check_config_invalid", str(error))
    if isinstance(error, KinematicsConfigError):
        return KinematicsCheckFailure("kinematics_config_invalid", str(error))
    if logger is None:
        return KinematicsCheckFailure(
            "logger_init_failed", f"{type(error).__name__}: {error}"
        )
    return KinematicsCheckFailure("unexpected_error", f"{type(error).__name__}: {error}")


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
    kinematics_config = None
    try:
        logger = _open_logger(run_dir, arguments.run_id, instance_id)
        report["log_directory"] = Path(logger.directory).relative_to(run_dir).as_posix()
        renderer, budget, kinematics_config = _execute_checks(
            arguments, run_dir, report, logger
        )
    except BaseException as error:
        failure = _classify_failure(logger, error)
    finally:
        failure = _close_resources(logger, renderer, report, failure)
    max_report_bytes = (
        kinematics_config.budgets.max_report_bytes
        if kinematics_config is not None
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
        "min_visible_pixels": {
            camera: dict(objects) for camera, objects in config.min_visible_pixels.items()
        },
        "throughput": {
            "warmup_groups": config.throughput.warmup_groups,
            "measure_groups": config.throughput.measure_groups,
        },
        "camera_names": list(config.camera_names),
        "object_names": list(config.object_names),
        "camera_resolutions": {
            camera: list(resolution) for camera, resolution in config.camera_resolutions.items()
        },
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
