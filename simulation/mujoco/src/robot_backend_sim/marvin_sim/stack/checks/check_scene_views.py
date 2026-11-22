"""Scene snapshot, RGB evidence and contact-sheet helpers."""

from __future__ import annotations

import math
import statistics
import time
from pathlib import Path

from robot_backend_sim.marvin_sim.stack.scene_config import MarvinStackSceneConfig
from robot_backend_sim.marvin_sim.stack.visibility import (
    LayoutSpec,
    LayoutVisibility,
    SceneCheckConfig,
)
from robot_backend_sim.marvin_sim.stack.checks.check_scene_common import SceneCheckFailure


def _actual_object_poses(
    scene: object, scene_config: MarvinStackSceneConfig
) -> list[dict[str, object]]:
    poses: list[dict[str, object]] = []
    for obj in scene_config.objects:
        address = scene.indices.environment.object_free_joint_qpos[obj.name]
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

def _first_visibility_failure(result: LayoutVisibility) -> tuple[str, str, str, int] | None:
    for camera in result.cameras:
        for item in camera.objects:
            if not item.passed:
                return (result.layout_id, camera.camera, item.object, item.pixels)
    return None

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

def _write_contact_sheet(
    run_dir: Path, layout: dict[str, object], check_config: SceneCheckConfig
) -> str:
    try:
        return _compose_contact_sheet(run_dir, layout, check_config)
    except SceneCheckFailure:
        raise
    except Exception as error:
        raise SceneCheckFailure(
            "contact_sheet_write_failed",
            f"cannot write contact sheet: {type(error).__name__}: {error}",
        ) from error

def _compose_contact_sheet(
    run_dir: Path, layout: dict[str, object], check_config: SceneCheckConfig
) -> str:
    from PIL import Image, ImageDraw

    cameras = check_config.camera_names
    images = []
    for camera in cameras:
        with Image.open(run_dir / str(layout["rgb"][camera])) as source:
            images.append(source.convert("RGB"))
    tile_width = max(image.width for image in images)
    tile_height = max(image.height for image in images)
    margin = 8
    gap = 6
    header = 16
    label = 14
    sheet_width = margin * 2 + len(cameras) * (tile_width + gap) - gap
    sheet_height = margin * 2 + header + tile_height + label
    sheet = Image.new("RGB", (sheet_width, sheet_height), (245, 245, 245))
    draw = ImageDraw.Draw(sheet)
    draw.text(
        (margin, margin),
        f"{layout['layout_id']}  total_pixels={layout['total_pixels']}  passed={layout['passed']}",
        fill=(0, 0, 0),
    )
    image_top = margin + header
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

