"""Preview media generated only from recorded RGB frames."""

from __future__ import annotations

import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw


def _json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _frame_rows(
    export: Path, limit: int = 120
) -> tuple[list[tuple[int, list[Path]]], tuple[str, ...]]:
    labels = _json(export / "labels.json")
    if labels is None:
        return [], ()
    rows = labels.get("labels")
    raw_order = labels.get("camera_order")
    if (
        not isinstance(rows, list)
        or not isinstance(raw_order, list)
        or not raw_order
        or not all(isinstance(name, str) and name for name in raw_order)
        or len(set(raw_order)) != len(raw_order)
    ):
        return [], ()
    camera_order = tuple(raw_order)
    result: list[tuple[int, list[Path]]] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        label_time = row.get("label_backend_time_ns")
        rgb = row.get("rgb")
        if not isinstance(label_time, int) or not isinstance(rgb, dict):
            continue
        paths: list[Path] = []
        for camera in camera_order:
            record = rgb.get(camera)
            if not isinstance(record, dict) or not isinstance(record.get("path"), str):
                paths = []
                break
            paths.append(export / record["path"])
        if paths and all(path.is_file() for path in paths):
            result.append((label_time, paths))
    if len(result) > limit:
        step = max(1, math.ceil(len(result) / limit))
        result = result[::step][:limit]
    return result, camera_order


def _montage(
    paths: list[Path], caption: str, cameras: tuple[str, ...]
) -> Image.Image | None:
    try:
        images = [Image.open(path).convert("RGB").resize((320, 240)) for path in paths]
    except OSError:
        return None
    width = 320 * len(images)
    canvas = Image.new("RGB", (width, 270), (0, 0, 0))
    for index, image in enumerate(images):
        canvas.paste(image, (index * 320, 30))
    draw = ImageDraw.Draw(canvas)
    draw.text((8, 8), caption, fill=(255, 255, 255))
    for index, camera in enumerate(cameras[: len(images)]):
        draw.text((index * 320 + 8, 250), camera, fill=(255, 255, 255))
    return canvas


def _triptych(
    paths: list[Path], out: Path, caption: str, cameras: tuple[str, ...]
) -> bool:
    canvas = _montage(paths, caption, cameras)
    if canvas is None:
        return False
    out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out)
    return True


def _ffmpeg_video(
    rows: list[tuple[int, list[Path]]], cameras: tuple[str, ...], out: Path
) -> str:
    temporary = Path(tempfile.mkdtemp(prefix="act-eval-"))
    try:
        for index, (_label_time, paths) in enumerate(rows):
            _triptych(paths, temporary / f"{index:06d}.png", f"frame {index}", cameras)
        command = [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            "8",
            "-i",
            str(temporary / "%06d.png"),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(out),
        ]
        completed = subprocess.run(command, capture_output=True, timeout=60)
        if completed.returncode == 0 and out.is_file() and out.stat().st_size > 0:
            return str(out)
    except (OSError, subprocess.SubprocessError):
        pass
    finally:
        shutil.rmtree(temporary, ignore_errors=True)
    return ""


def _gif_video(
    rows: list[tuple[int, list[Path]]], cameras: tuple[str, ...], out: Path
) -> str:
    images = []
    for label_time, paths in rows:
        canvas = _montage(paths, f"t={label_time} ns", cameras)
        if canvas is not None:
            images.append(canvas)
    if not images:
        return ""
    gif = out.with_suffix(".gif")
    images[0].save(gif, save_all=True, append_images=images[1:], duration=125, loop=0)
    return str(gif)


def write_video(export: Path, out: Path, max_frames: int = 80) -> str:
    rows, cameras = _frame_rows(export, limit=max_frames)
    if not rows:
        return ""
    out.parent.mkdir(parents=True, exist_ok=True)
    if shutil.which("ffmpeg"):
        video = _ffmpeg_video(rows, cameras, out)
        if video:
            return video
    return _gif_video(rows, cameras, out)


def keyframes(export: Path, out_dir: Path, prefix: str) -> list[str]:
    rows, cameras = _frame_rows(export, limit=1000)
    if not rows:
        return []
    positions = sorted({0, len(rows) // 2, len(rows) - 1})
    result = []
    for index in positions:
        label_time, paths = rows[index]
        out = out_dir / f"{prefix}-{index:04d}.png"
        if _triptych(paths, out, f"{prefix} t={label_time} ns", cameras):
            result.append(str(out))
    return result


def media_for(metric: dict[str, Any], output: Path) -> None:
    export = Path(metric["export"]) if metric["export"] else None
    if export is None or not export.is_dir():
        return
    preview = output / "preview"
    prefix = f"seed-{metric['seed']:05d}"
    metric["preview_video"] = write_video(export, preview / f"{prefix}-three-view.mp4")
    metric["preview_keyframes"] = keyframes(export, preview, prefix)


__all__ = ["keyframes", "media_for", "write_video"]
