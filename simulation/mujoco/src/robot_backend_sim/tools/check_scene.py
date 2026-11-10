"""MuJoCo three-camera static-layout visibility check for the S2 scene.

The module loads the scene and check configurations, builds the scene once,
takes explicit full-physics snapshots, renders three named RGB/segmentation
views per snapshot and publishes bounded PNG evidence and one JSON report.  It
does not step physics, start ROS or move objects in a live scene.
"""

from __future__ import annotations

import argparse
import math
import os
import platform
import re
import statistics
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

from .. import assets, visibility
from ..diagnostics import (
    code_provenance as _shared_code_provenance,
)
from ..diagnostics import (
    python_source_manifest as _python_source_manifest,  # noqa: F401
)
from ..diagnostics import (
    scene_config_document as _scene_config_document,
)
from ..diagnostics import (
    sha256_file as _sha256_file,
)
from ..diagnostics import (
    utc_now as _utc_now,
)
from ..diagnostics import (
    write_json_atomic as _write_json_atomic,
)
from ..scene_config import SceneConfig, SceneConfigError, load_scene_config
from ..visibility import (
    LayoutSpec,
    LayoutVisibility,
    SceneCheckConfig,
    SceneCheckConfigError,
    load_check_config,
)
from . import probe_environment

COMPONENT = "scene_check"
KIND = "scene"
REPORT_SCHEMA_VERSION = 1
DEFAULT_SCENE_CONFIG = Path("configs/scene/panda_stack.yaml")
DEFAULT_CHECK_CONFIG = Path("configs/checks/scene_check.yaml")
_RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_LABEL_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class SceneCheckUsageError(ValueError):
    """Invalid invocation before the run directory is created."""


class SceneCheckFailure(RuntimeError):
    """A stable scene-check failure with a bounded message."""

    def __init__(self, reason: str, message: str = "") -> None:
        super().__init__(message or reason)
        self.reason = reason
        self.message = message or reason


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
    parser.add_argument("--run-id", required=True, help="fresh report run identity")
    return parser


def parse_arguments(argv: list[str] | None = None) -> SceneCheckArguments:
    parsed = _parser().parse_args(argv)
    return SceneCheckArguments(
        scene_config=parsed.scene_config.resolve(),
        check_config=parsed.check_config.resolve(),
        report_root=_validate_report_root(parsed.report_root),
        run_id=_validate_run_id(parsed.run_id),
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


def _load_configs(
    arguments: SceneCheckArguments,
) -> tuple[SceneConfig, SceneCheckConfig]:
    try:
        scene_config = load_scene_config(arguments.scene_config)
    except SceneConfigError as error:
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


def _build_scene(scene_config: SceneConfig) -> object:
    from ..scene import SceneBuildError, build_scene

    try:
        return build_scene(scene_config)
    except SceneBuildError as error:
        raise SceneCheckFailure("scene_build_failed", str(error)) from error


def _scene_info(scene: object, scene_config: SceneConfig) -> dict[str, object]:
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


def _make_renderer(scene: object, scene_config: SceneConfig) -> object:
    from ..rendering import SceneRenderer, SceneRenderError

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


def _actual_object_poses(scene: object, scene_config: SceneConfig) -> list[dict[str, object]]:
    poses: list[dict[str, object]] = []
    for obj in scene_config.objects:
        address = scene.indices.object_free_joint_qpos[obj.name]
        qpos = scene.data.qpos[address : address + 7]
        yaw = 2.0 * math.atan2(float(qpos[6]), float(qpos[3]))
        poses.append(
            {
                "name": obj.name,
                "x": float(qpos[0]),
                "y": float(qpos[1]),
                "z": float(qpos[2]),
                "yaw_deg": math.degrees(yaw),
            }
        )
    return poses


def _boundary_object_poses(
    layout: LayoutSpec, scene_config: SceneConfig
) -> list[dict[str, object]]:
    by_name = {placement.name: placement for placement in layout.placements}
    poses: list[dict[str, object]] = []
    for obj in scene_config.objects:
        placement = by_name[obj.name]
        poses.append(
            {
                "name": obj.name,
                "x": placement.x,
                "y": placement.y,
                "z": scene_config.table.top_z + obj.half_extents[2],
                "yaw_deg": placement.yaw_deg,
            }
        )
    return poses


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


def _save_rgb_images(
    run_dir: Path,
    layout: LayoutSpec,
    rgb: dict[str, object],
) -> dict[str, str]:
    from PIL import Image

    directory = run_dir / "artifacts" / "layouts" / layout.layout_id
    try:
        directory.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise SceneCheckFailure(
            "artifact_path_exists",
            f"layout artifact directory already exists: {directory}",
        ) from error
    except OSError as error:
        raise SceneCheckFailure(
            "artifact_write_failed",
            f"cannot create layout artifact directory {directory}: {error}",
        ) from error
    paths: dict[str, str] = {}
    for camera, image in rgb.items():
        path = directory / f"{camera}.png"
        try:
            Image.fromarray(image, mode="RGB").save(path, format="PNG")
        except Exception as error:
            raise SceneCheckFailure(
                "artifact_write_failed",
                f"cannot write {path}: {type(error).__name__}: {error}",
            ) from error
        paths[camera] = path.relative_to(run_dir).as_posix()
    return paths


def _layout_document(
    layout: LayoutSpec,
    result: LayoutVisibility,
    poses: list[dict[str, object]],
    rgb_paths: dict[str, str],
) -> dict[str, object]:
    return {
        "layout_id": layout.layout_id,
        "kind": layout.kind,
        "seed": layout.seed,
        "objects": poses,
        "cameras": [
            {
                "camera": camera.camera,
                "width": camera.width,
                "height": camera.height,
                "total_pixels": camera.total_pixels,
                "passed": camera.passed,
                "objects": [
                    {
                        "object": item.object,
                        "geom_id": item.geom_id,
                        "pixels": item.pixels,
                        "threshold": item.threshold,
                        "passed": item.passed,
                    }
                    for item in camera.objects
                ],
            }
            for camera in result.cameras
        ],
        "total_pixels": result.total_pixels,
        "minimum_margin": result.minimum_margin,
        "passed": result.passed,
        "rgb": rgb_paths,
    }


def _first_visibility_failure(result: LayoutVisibility) -> tuple[str, str, int, int] | None:
    for camera in result.cameras:
        for item in camera.objects:
            if not item.passed:
                return (result.layout_id, camera.camera, item.object, item.pixels)
    return None


def _run_layouts(
    run_dir: Path,
    scene: object,
    scene_config: SceneConfig,
    check_config: SceneCheckConfig,
    renderer: object,
    logger: object,
    report: dict[str, object],
) -> SceneCheckFailure | None:
    layouts = visibility.build_layouts(check_config, scene_config)
    object_geom_ids = report["scene"]["object_geom_ids"]  # type: ignore[index]
    boundary_base = None
    if any(layout.kind == "boundary" for layout in layouts):
        scene.reset(check_config.seeds[0])
        boundary_base = renderer.capture(scene.data)

    results: list[dict[str, object]] = []
    first_failure: SceneCheckFailure | None = None
    for layout in layouts:
        try:
            if layout.kind == "boundary":
                assert boundary_base is not None
                snapshot = renderer.derive_snapshot(
                    boundary_base, _boundary_updates(layout, scene, scene_config)
                )
                poses = _boundary_object_poses(layout, scene_config)
            else:
                scene.reset(layout.seed)
                snapshot = renderer.capture(scene.data)
                poses = _actual_object_poses(scene, scene_config)
            rgb = renderer.render_rgb(snapshot)
            rgb_paths = _save_rgb_images(run_dir, layout, rgb)
            segmentation = renderer.render_segmentation(snapshot)
            result = visibility.summarize_layout(
                layout,
                segmentation,
                object_geom_ids,
                check_config.min_visible_pixels,
                check_config.camera_resolutions,
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
        document = _layout_document(layout, result, poses, rgb_paths)
        results.append(document)
        fields = {
            "layout": layout.layout_id,
            "kind": layout.kind,
            "passed": result.passed,
            "total_pixels": result.total_pixels,
            "minimum_margin": result.minimum_margin,
        }
        _emit(
            logger,
            "layout_checked" if result.passed else "layout_failed",
            "scene layout visibility counted",
            fields,
        )
        failure = _first_visibility_failure(result)
        if failure is not None and first_failure is None:
            layout_id, camera, obj, pixels = failure
            first_failure = SceneCheckFailure(
                "visibility_threshold_not_met",
                f"layout {layout_id!r} camera {camera!r} object {obj!r} "
                f"has {pixels} visible pixels",
            )
    report["layouts"] = results
    report["layout_summary"] = {
        "count": len(results),
        "passed": sum(1 for result in results if bool(result["passed"])),
        "failed": sum(1 for result in results if not bool(result["passed"])),
        "boundary_count": check_config.boundary_layout_count,
        "seed_count": len(check_config.seeds),
    }
    return first_failure


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def _measure_throughput(
    renderer: object,
    snapshot: object,
    check_config: SceneCheckConfig,
) -> dict[str, object]:
    for _ in range(check_config.throughput.warmup_groups):
        views = renderer.render_rgb(snapshot)
        del views
    durations: list[float] = []
    started = time.perf_counter()
    for _ in range(check_config.throughput.measure_groups):
        group_started = time.perf_counter()
        views = renderer.render_rgb(snapshot)
        durations.append(time.perf_counter() - group_started)
        del views
    wall = time.perf_counter() - started
    gl = renderer.gl_info
    return {
        "group_definition": "one RGB render of all three named cameras from one snapshot",
        "warmup_groups": check_config.throughput.warmup_groups,
        "measure_groups": check_config.throughput.measure_groups,
        "measured_groups": len(durations),
        "width": renderer.width,
        "height": renderer.height,
        "cameras": list(renderer.camera_names),
        "wall_s": round(wall, 6),
        "mean_s": round(statistics.fmean(durations), 6),
        "p50_s": round(statistics.median(durations), 6),
        "p95_s": round(_percentile(durations, 0.95), 6),
        "rgb_only": True,
        "gpu": {
            "vendor": gl.vendor,
            "renderer": gl.renderer,
            "version": gl.version,
        },
        "note": (
            "Pure render throughput only; this is not evidence that 500 Hz physics "
            "plus 20 Hz cameras are concurrently achieved."
        ),
    }


def _select_contact_layouts(results: list[dict[str, object]]) -> list[dict[str, object]]:
    if not results:
        return []
    first = results[0]
    worst = min(
        results,
        key=lambda item: (
            int(item["minimum_margin"]),
            int(item["total_pixels"]),
            str(item["layout_id"]),
        ),
    )
    if worst["layout_id"] == first["layout_id"]:
        return [first, first]
    return [first, worst]


def _write_contact_sheet(
    run_dir: Path, results: list[dict[str, object]], check_config: SceneCheckConfig
) -> str:
    try:
        return _compose_contact_sheet(run_dir, results, check_config)
    except SceneCheckFailure:
        raise
    except Exception as error:
        raise SceneCheckFailure(
            "contact_sheet_write_failed",
            f"cannot write contact sheet: {type(error).__name__}: {error}",
        ) from error


def _compose_contact_sheet(
    run_dir: Path, results: list[dict[str, object]], check_config: SceneCheckConfig
) -> str:
    from PIL import Image, ImageDraw

    selected = _select_contact_layouts(results)
    cameras = list(check_config.camera_names)
    opened: list[list[Image.Image]] = []
    for layout in selected:
        row: list[Image.Image] = []
        for camera in cameras:
            with Image.open(run_dir / str(layout["rgb"][camera])) as source:
                row.append(source.convert("RGB"))
        opened.append(row)
    tile_width = max(image.width for row in opened for image in row)
    tile_height = max(image.height for row in opened for image in row)
    margin = 8
    gap = 6
    header = 16
    label = 14
    columns = len(cameras)
    rows = len(selected)
    sheet_width = margin * 2 + columns * (tile_width + gap) - gap
    sheet_height = margin * 2 + rows * (header + tile_height + label + gap) - gap
    sheet = Image.new("RGB", (sheet_width, sheet_height), (245, 245, 245))
    draw = ImageDraw.Draw(sheet)
    for row_index, (layout, images) in enumerate(zip(selected, opened, strict=True)):
        top = margin + row_index * (header + tile_height + label + gap)
        draw.text(
            (margin, top),
            f"{layout['layout_id']}  kind={layout['kind']}  "
            f"total_pixels={layout['total_pixels']}  passed={layout['passed']}",
            fill=(0, 0, 0),
        )
        image_top = top + header
        for column, (camera, image) in enumerate(zip(cameras, images, strict=True)):
            left = margin + column * (tile_width + gap)
            sheet.paste(image, (left, image_top))
            draw.rectangle(
                (left, image_top, left + 110, image_top + label),
                fill=(0, 0, 0),
            )
            draw.text((left + 3, image_top + 1), camera, fill=(255, 255, 255))
    path = run_dir / "artifacts" / "contact_sheet.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(path, format="PNG")
    return path.relative_to(run_dir).as_posix()


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


def run(arguments: SceneCheckArguments) -> int:
    run_dir = arguments.report_root / arguments.run_id
    if os.path.lexists(run_dir):
        raise SceneCheckUsageError(f"report run directory already exists: {run_dir}")
    arguments.report_root.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(exist_ok=False)
    started = time.monotonic()
    instance_id = str(uuid.uuid4())
    report: dict[str, object] = {
        "schema_version": REPORT_SCHEMA_VERSION,
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
    logger: object | None = None
    failure: SceneCheckFailure | None = None
    renderer: object | None = None
    try:
        logger = _open_logger(run_dir, arguments.run_id, instance_id)
        report["log_directory"] = Path(logger.directory).relative_to(run_dir).as_posix()
        _emit(logger, "scene_check_start", "scene visibility check started")
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
        _emit(
            logger,
            "configs_loaded",
            "scene and check configurations validated and frozen before GL",
            {
                "seeds": len(check_config.seeds),
                "boundary_layouts": check_config.boundary_layout_count,
                "layouts": check_config.layout_count,
            },
        )
        dependencies = probe_environment.check_dependencies()
        probe_environment._verify_mounted_source(dependencies)
        report["dependencies"] = dependencies
        report["source"] = {"model_directory": str(assets.MODEL_DIR), "code": _code_provenance()}
        scene = _build_scene(scene_config)
        report["scene"] = _scene_info(scene, scene_config)
        _emit(
            logger,
            "model_loaded",
            "local Panda model compiled into the scene",
            {"model_directory": str(assets.MODEL_DIR)},
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
        contact = _write_contact_sheet(run_dir, report["layouts"], check_config)
        report["artifact"] = contact
        report["artifacts"] = {
            "contact_sheet": contact,
            "layout_rgb_root": "artifacts/layouts",
            "layout_png_count": sum(len(result["rgb"]) for result in report["layouts"]),
        }
        if layout_failure is not None:
            raise layout_failure
    except SceneCheckFailure as error:
        failure = error
    except probe_environment.ProbeFailure as error:
        failure = SceneCheckFailure(error.reason, error.message)
    except SceneConfigError as error:
        failure = SceneCheckFailure("scene_config_invalid", str(error))
    except SceneCheckConfigError as error:
        failure = SceneCheckFailure("check_config_invalid", str(error))
    except Exception as error:
        if logger is None:
            failure = SceneCheckFailure("logger_init_failed", f"{type(error).__name__}: {error}")
        else:
            failure = SceneCheckFailure("unexpected_error", f"{type(error).__name__}: {error}")
    finally:
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
    log_close_error: str | None = None
    if logger is not None:
        try:
            logger.close()
        except Exception as error:
            log_close_error = f"{type(error).__name__}: {error}"
            if failure is None:
                failure = SceneCheckFailure(
                    "logger_close_failed", f"event logger close failed: {log_close_error}"
                )
    report["logs_closed"] = logger is not None and log_close_error is None
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
        report["error"] = {
            "type": type(failure).__name__,
            "message": failure.message[:2000],
        }
    report_path = run_dir / "report.json"
    try:
        _write_json_atomic(report_path, report)
    except Exception as error:
        print(f"error: report_write_failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(f"report: {report_path}")
    return int(report["exit_code"])


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
