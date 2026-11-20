"""MuJoCo three-camera visibility check for the nominal Marvin stack layout.

The module loads the scene and check configurations, builds the scene once,
takes explicit full-physics snapshots, renders three named RGB/segmentation
views per snapshot and publishes bounded PNG evidence and one JSON report.  It
does not step physics, start ROS or move objects in a live scene.
"""

from __future__ import annotations

import argparse
import os
import platform
import re
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

from robot_backend_sim.common.physics import assets
from robot_backend_sim.common.runtime.diagnostics import code_provenance as _shared_code_provenance
from robot_backend_sim.common.runtime.diagnostics import (
    python_source_manifest as _python_source_manifest,  # noqa: F401
)
from robot_backend_sim.common.runtime.diagnostics import sha256_file as _sha256_file
from robot_backend_sim.common.runtime.diagnostics import utc_now as _utc_now
from robot_backend_sim.common.runtime.diagnostics import write_json_atomic as _write_json_atomic
from robot_backend_sim.marvin_sim.stack import visibility
from robot_backend_sim.marvin_sim.stack.diagnostics import (
    scene_config_document as _scene_config_document,
)
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
from robot_backend_sim.tools import probe_environment
from robot_backend_sim.tools.check_scene_common import SceneCheckFailure, SceneCheckUsageError
from robot_backend_sim.tools.check_scene_views import (
    _actual_object_poses,
    _first_visibility_failure,
    _layout_document,
    _measure_throughput,
    _save_rgb_images,
    _write_contact_sheet,
)

COMPONENT = "scene_check"
KIND = "scene"
_SHARED_ROOT = Path(os.environ.get("SHARED_ROOT", "/shared"))
_SHARED_RECIPES = (
    _SHARED_ROOT / "scenarios/marvin_sim/stack/configs/simulation"
)
DEFAULT_SCENE_CONFIG = _SHARED_RECIPES / "scene/marvin_stack.yaml"
DEFAULT_CHECK_CONFIG = _SHARED_RECIPES / "checks/scene_check.yaml"
_RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_LABEL_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")






@dataclass(frozen=True, slots=True)
class SceneCheckArguments:
    scene_config: Path
    check_config: Path
    report_root: Path
    run_id: str


def _validate_run_id(value: str) -> str:
    if not isinstance(value, str) or not _RUN_ID_PATTERN.fullmatch(value):
        raise SceneCheckUsageError(
            "run id must contain 1..128 ASCII letters, digits, '.', '_' or '-' "
            "and must start with a letter or digit"
        )
    if value in {".", ".."} or ".." in value:
        raise SceneCheckUsageError("run id must not be '.' or '..' or contain '..'")
    return value


def _validate_report_root(path: Path) -> Path:
    resolved = path.resolve()
    if resolved == Path("/"):
        raise SceneCheckUsageError("report root must not be the filesystem root")
    home = Path.home().resolve()
    if resolved == home or resolved in home.parents:
        raise SceneCheckUsageError("report root must not be the home directory or a parent")
    return resolved


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, required=True, help="scene parameters")
    parser.add_argument("--check-config", type=Path, required=True, help="check parameters")
    parser.add_argument("--report-root", type=Path, required=True, help="report parent directory")
    parser.add_argument(
        "--run-id",
        default=None,
        help="fresh report run identity (default: scene-check-<UTC>-<pid>)",
    )
    return parser


def parse_arguments(argv: list[str] | None = None) -> SceneCheckArguments:
    parsed = _parser().parse_args(argv)
    run_id = (
        f"scene-check-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{os.getpid()}"
        if parsed.run_id is None
        else parsed.run_id
    )
    return SceneCheckArguments(
        scene_config=parsed.scene_config.resolve(),
        check_config=parsed.check_config.resolve(),
        report_root=_validate_report_root(parsed.report_root),
        run_id=_validate_run_id(run_id),
    )


def _emit(
    logger: object, event: str, message: str, fields: dict[str, object] | None = None
) -> None:
    try:
        logger.emit(event, message=message, fields=fields, critical=True)
    except Exception as error:
        raise SceneCheckFailure(
            "event_log_failed", f"{event}: {type(error).__name__}: {error}"
        ) from error


def _check_config_document(config: SceneCheckConfig) -> dict[str, object]:
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


def _load_configs(
    arguments: SceneCheckArguments,
) -> tuple[MarvinStackSceneConfig, SceneCheckConfig]:
    try:
        scene_config = load_marvin_stack_scene_config(arguments.scene_config)
    except StackSceneConfigError as error:
        raise SceneCheckFailure("scene_config_invalid", str(error)) from error
    try:
        check_config = load_check_config(arguments.check_config, scene_config)
    except SceneCheckConfigError as error:
        raise SceneCheckFailure("check_config_invalid", str(error)) from error
    return scene_config, check_config


def _code_provenance() -> dict[str, object]:
    try:
        return _shared_code_provenance()
    except OSError as error:
        raise SceneCheckFailure("source_provenance_failed", str(error)) from error


def _build_scene(scene_config: MarvinStackSceneConfig) -> object:
    from robot_backend_sim.marvin_sim.stack.scene import (
        SceneBuildError,
        build_marvin_stack_scene,
    )

    try:
        return build_marvin_stack_scene(scene_config)
    except SceneBuildError as error:
        raise SceneCheckFailure("scene_build_failed", str(error)) from error


def _scene_info(scene: object, scene_config: MarvinStackSceneConfig) -> dict[str, object]:
    model = scene.model
    try:
        object_geom_ids = {
            obj.name: int(model.geom(f"{obj.name}_geom").id) for obj in scene_config.objects
        }
        camera_ids = {
            camera.name: int(model.camera(camera.name).id) for camera in scene_config.cameras
        }
    except KeyError as error:
        raise SceneCheckFailure(
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
        "offscreen": {
            "width": int(model.vis.global_.offwidth),
            "height": int(model.vis.global_.offheight),
        },
        "camera_resolution": {
            camera.name: [int(camera.width), int(camera.height)] for camera in scene_config.cameras
        },
    }


def _make_renderer(scene: object, scene_config: MarvinStackSceneConfig) -> object:
    from robot_backend_sim.common.render.rendering import SceneRenderer, SceneRenderError

    first = scene_config.cameras[0]
    try:
        return SceneRenderer(
            scene.model,
            [camera.name for camera in scene_config.cameras],
            width=first.width,
            height=first.height,
        )
    except SceneRenderError as error:
        raise SceneCheckFailure(error.reason, error.message) from error


def _gl_document(gl: object) -> dict[str, object]:
    return {
        "vendor": gl.vendor,
        "renderer": gl.renderer,
        "version": gl.version,
        "nvidia": gl.nvidia,
    }














def _layout_result(
    run_dir: Path,
    layout,
    scene: object,
    scene_config: MarvinStackSceneConfig,
    check_config: SceneCheckConfig,
    renderer: object,
    logger: object,
    object_geom_ids,
):
    try:
        scene.reset(layout.seed)
        snapshot = renderer.capture(scene.data)
        poses = _actual_object_poses(scene, scene_config)
        rgb = renderer.render_rgb(snapshot)
        rgb_paths = _save_rgb_images(run_dir, layout, rgb)
        segmentation = renderer.render_segmentation(snapshot)
        result = visibility.evaluate_layout_visibility(
            layout.layout_id,
            segmentation,
            object_geom_ids,
            check_config,
        )
    except Exception as error:
        if isinstance(error, SceneCheckFailure):
            raise
        reason = getattr(error, "reason", "render_failed")
        message = getattr(error, "message", f"{type(error).__name__}: {error}")
        try:
            logger.emit(
                "render_failed",
                message="scene layout rendering failed",
                reason=reason,
                fields={"layout": layout.layout_id},
                critical=True,
            )
        except Exception as log_error:
            raise SceneCheckFailure(
                "event_log_failed",
                f"render_failed: {type(log_error).__name__}: {log_error}",
            ) from log_error
        raise SceneCheckFailure(str(reason), str(message)) from error
    return result, poses, rgb_paths


def _run_layouts(
    run_dir: Path,
    scene: object,
    scene_config: MarvinStackSceneConfig,
    check_config: SceneCheckConfig,
    renderer: object,
    logger: object,
    report: dict[str, object],
) -> SceneCheckFailure | None:
    object_geom_ids = report["scene"]["object_geom_ids"]  # type: ignore[index]
    layout = visibility.NOMINAL_LAYOUT
    result, poses, rgb_paths = _layout_result(
        run_dir, layout, scene, scene_config, check_config, renderer, logger, object_geom_ids
    )
    report["layouts"] = [_layout_document(layout, result, poses, rgb_paths)]
    report["layout_summary"] = {
        "count": 1,
        "passed": int(result.passed),
        "failed": int(not result.passed),
    }
    _emit(
        logger,
        "layout_checked" if result.passed else "layout_failed",
        "scene layout visibility counted",
        {
            "layout": layout.layout_id,
            "passed": result.passed,
            "total_pixels": result.total_pixels,
            "minimum_margin": result.minimum_margin,
        },
    )
    failure = _first_visibility_failure(result)
    if failure is None:
        return None
    layout_id, camera, obj, pixels = failure
    return SceneCheckFailure(
        "visibility_threshold_not_met",
        f"layout {layout_id!r} camera {camera!r} object {obj!r} has {pixels} visible pixels",
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


def _report_template(arguments: SceneCheckArguments, instance_id: str) -> dict[str, object]:
    return {
        "kind": KIND,
        "component": COMPONENT,
        "run_id": arguments.run_id,
        "instance_id": instance_id,
        "status": "failed",
        "exit_code": 1,
        "reason": "not_started",
        "started_at_utc": _utc_now(),
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
        "gl": None,
        "layouts": [],
        "layout_summary": None,
        "throughput": None,
        "artifact": None,
        "artifacts": None,
        "log_directory": None,
        "logs_closed": False,
        "renderer_close_error": None,
        "logger_close_error": None,
        "error": None,
    }


def _load_and_report_configs(arguments, report):
    scene_config, check_config = _load_configs(arguments)
    report["configs"] = {
        "scene": {
            "path": str(arguments.scene_config),
            "sha256": _sha256_file(arguments.scene_config),
            "resolved": _scene_config_document(scene_config),
        },
        "check": {
            "path": str(arguments.check_config),
            "sha256": _sha256_file(arguments.check_config),
            "resolved": _check_config_document(check_config),
        },
    }
    return scene_config, check_config


def _execute_checks(arguments, run_dir: Path, report: dict, logger: object):
    _emit(logger, "scene_check_start", "scene visibility check started")
    scene_config, check_config = _load_and_report_configs(arguments, report)
    _emit(
        logger,
        "configs_loaded",
        "scene and check configurations validated and frozen before GL",
        {"layout": visibility.NOMINAL_LAYOUT.layout_id},
    )
    dependencies = probe_environment.check_dependencies()
    probe_environment._verify_mounted_source(dependencies)
    report["dependencies"] = dependencies
    model_dir = str(assets.robot_model_path(scene_config.robot.assembly).parent)
    report["source"] = {"model_directory": model_dir, "code": _code_provenance()}
    scene = _build_scene(scene_config)
    report["scene"] = _scene_info(scene, scene_config)
    _emit(
        logger,
        "model_loaded",
        "local robot model compiled into the scene",
        {"model_directory": model_dir},
    )
    renderer = _make_renderer(scene, scene_config)
    report["gl"] = _gl_document(renderer.gl_info)
    _emit(
        logger,
        "renderer_ready",
        "EGL renderer created with the local NVIDIA driver",
        {
            "vendor": str(report["gl"]["vendor"]),
            "renderer": str(report["gl"]["renderer"]),
            "width": int(renderer.width),
            "height": int(renderer.height),
        },
    )
    layout_failure = _run_layouts(
        run_dir, scene, scene_config, check_config, renderer, logger, report
    )
    report["throughput"] = _measure_throughput(
        renderer,
        renderer.capture(scene.data),
        check_config,
    )
    _emit(
        logger,
        "throughput_measured",
        "RGB-only three-camera render throughput measured",
        {
            "groups": int(report["throughput"]["measured_groups"]),
            "p50_s": float(report["throughput"]["p50_s"]),
            "p95_s": float(report["throughput"]["p95_s"]),
        },
    )
    contact = _write_contact_sheet(run_dir, report["layouts"][0], check_config)
    report["artifact"] = contact
    report["artifacts"] = {
        "contact_sheet": contact,
        "layout_rgb_root": "artifacts/layouts",
        "layout_png_count": sum(len(result["rgb"]) for result in report["layouts"]),
    }
    if layout_failure is not None:
        raise layout_failure
    return renderer


def _classify_failure(logger, error: BaseException) -> SceneCheckFailure:
    if isinstance(error, SceneCheckFailure):
        return error
    if isinstance(error, probe_environment.ProbeFailure):
        return SceneCheckFailure(error.reason, error.message)
    if isinstance(error, StackSceneConfigError):
        return SceneCheckFailure("scene_config_invalid", str(error))
    if isinstance(error, SceneCheckConfigError):
        return SceneCheckFailure("check_config_invalid", str(error))
    if logger is None:
        return SceneCheckFailure("logger_init_failed", f"{type(error).__name__}: {error}")
    return SceneCheckFailure("unexpected_error", f"{type(error).__name__}: {error}")


def _close_resources(logger, renderer, failure, report) -> SceneCheckFailure | None:
    if logger is not None:
        try:
            _emit(
                logger,
                "finalizing",
                "scene check finalizing resources",
                {"failed": failure is not None},
            )
        except SceneCheckFailure as error:
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
                failure = SceneCheckFailure(
                    "renderer_close_failed", report["renderer_close_error"]
                )
    return failure


def _close_logger(logger, failure):
    log_close_error: str | None = None
    try:
        if logger is not None:
            if failure is None:
                _emit(
                    logger,
                    "checks_finished",
                    "scene checks and artifact writes finished; closing resources",
                )
            else:
                _emit(
                    logger,
                    "scene_check_failed",
                    "scene visibility check failed",
                    {"reason": failure.reason},
                )
            logger.check_health()
    except Exception as error:
        if failure is None:
            failure = SceneCheckFailure(
                "event_log_failed", f"logger finalization: {type(error).__name__}: {error}"
            )
    if logger is not None:
        try:
            logger.close()
        except Exception as error:
            log_close_error = f"{type(error).__name__}: {error}"
            if failure is None:
                failure = SceneCheckFailure(
                    "logger_close_failed", f"event logger close failed: {log_close_error}"
                )
    return failure, log_close_error


def _finalize_report(
    report, failure, run_dir: Path, started: float, log_close_error, logger_present: bool
) -> int:
    report["logs_closed"] = logger_present and log_close_error is None
    report["logger_close_error"] = log_close_error
    report["elapsed_s"] = round(time.monotonic() - started, 3)
    report["finished_at_utc"] = _utc_now()
    if failure is None:
        report["status"] = "succeeded"
        report["exit_code"] = 0
        report["reason"] = None
    else:
        report["status"] = "failed"
        report["exit_code"] = 1
        report["reason"] = failure.reason
        report["error"] = {"type": type(failure).__name__, "message": failure.message[:2000]}
    report_path = run_dir / "report.json"
    try:
        _write_json_atomic(report_path, report)
    except Exception as error:
        print(f"error: report_write_failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(f"report: {report_path}")
    return int(report["exit_code"])


def run(arguments: SceneCheckArguments) -> int:
    run_dir = arguments.report_root / arguments.run_id
    if os.path.lexists(run_dir):
        raise SceneCheckUsageError(f"report run directory already exists: {run_dir}")
    arguments.report_root.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(exist_ok=False)
    started = time.monotonic()
    instance_id = str(uuid.uuid4())
    report = _report_template(arguments, instance_id)
    logger: object | None = None
    failure: SceneCheckFailure | None = None
    renderer: object | None = None
    try:
        logger = _open_logger(run_dir, arguments.run_id, instance_id)
        report["log_directory"] = Path(logger.directory).relative_to(run_dir).as_posix()
        renderer = _execute_checks(arguments, run_dir, report, logger)
    except BaseException as error:
        failure = _classify_failure(logger, error)
    finally:
        failure = _close_resources(logger, renderer, failure, report)
    failure, log_close_error = _close_logger(logger, failure)
    return _finalize_report(
        report, failure, run_dir, started, log_close_error, logger is not None
    )


def main(argv: list[str] | None = None) -> int:
    try:
        arguments = parse_arguments(argv)
    except SceneCheckUsageError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    try:
        return run(arguments)
    except SceneCheckUsageError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    except OSError as error:
        print(f"error: report_io_error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
