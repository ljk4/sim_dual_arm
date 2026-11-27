"""Decode recorded rosbag2/MCAP camera & task topics into composited RGB24 video streams."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from PIL import Image, ImageDraw, ImageFont

_DEFAULT_FPS = 20.0
_HUD_HEIGHT = 46
_LABEL_HEIGHT = 22
_MAX_PENDING_FRAMES = 8
_FONT_PATH = Path("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf")
_FONT_BOLD_PATH = Path("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf")


@dataclass(frozen=True)
class EpisodePaths:
    episode_dir: Path
    bag_dir: Path
    manifest_path: Path | None
    task_result_path: Path | None


@dataclass(frozen=True)
class VideoLayout:
    cameras: tuple[str, ...]
    camera_sizes: dict[str, tuple[int, int]]
    tile_width: int
    tile_height: int
    cols: int
    rows: int
    hud_height: int
    label_height: int
    width: int
    height: int


def resolve_episode_paths(input_dir: Path) -> EpisodePaths:
    """Resolve episode root, rosbag2 directory, and optional sidecar JSON paths."""
    root = Path(input_dir).resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"录制目录不存在: {root}")
    if (root / "recording" / "metadata.yaml").is_file():
        episode_dir = root
        bag_dir = root / "recording"
    elif (root / "raw" / "recording" / "metadata.yaml").is_file():
        episode_dir = root / "raw"
        bag_dir = episode_dir / "recording"
    elif (root / "metadata.yaml").is_file():
        episode_dir = root.parent if root.name == "recording" else root
        bag_dir = root
    else:
        raise FileNotFoundError(
            f"未找到录制元数据 metadata.yaml: {root}（请传入单局录制目录或其 recording/ 子目录）"
        )
    if not list(bag_dir.glob("*.mcap")):
        raise FileNotFoundError(f"录制目录中缺少 .mcap 文件: {bag_dir}")
    manifest_path = episode_dir / "manifest.json"
    task_result_path = episode_dir / "task_result.json"
    return EpisodePaths(
        episode_dir=episode_dir,
        bag_dir=bag_dir,
        manifest_path=manifest_path if manifest_path.is_file() else None,
        task_result_path=task_result_path if task_result_path.is_file() else None,
    )


def _read_json_dict(path: Path | None) -> dict[str, Any]:
    if path is None or not path.is_file():
        return {}
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return loaded if isinstance(loaded, dict) else {}


def _metadata_topics(bag_dir: Path) -> tuple[dict[str, str], dict[str, int]]:
    doc = yaml.safe_load((bag_dir / "metadata.yaml").read_text(encoding="utf-8")) or {}
    info = doc.get("rosbag2_bagfile_information") or {}
    topics: dict[str, str] = {}
    counts: dict[str, int] = {}
    for entry in info.get("topics_with_message_count") or ():
        meta = entry.get("topic_metadata") or {}
        name = str(meta.get("name") or "")
        msg_type = str(meta.get("type") or "")
        if name:
            topics[name] = msg_type
            counts[name] = int(entry.get("message_count") or 0)
    return topics, counts


def _camera_name_from_topic(topic: str) -> str | None:
    parts = topic.split("/")
    if len(parts) >= 4 and parts[-1] == "frame" and parts[-3] == "cameras":
        return parts[-2]
    return None


def _select_cameras(
    available: dict[str, str],
    manifest: dict[str, Any],
    requested: list[str] | None,
) -> tuple[str, ...]:
    if not available:
        raise ValueError("录制包中未包含任何相机帧话题 (*/cameras/*/frame)")
    if requested:
        missing = [name for name in requested if name not in available]
        if missing:
            raise ValueError(
                f"请求的相机在录制中不存在: {', '.join(missing)}（可用: {', '.join(sorted(available))}）"
            )
        return tuple(requested)
    ordered: list[str] = []
    obs_cams = (
        (manifest.get("configuration") or {}).get("observation") or {}
    ).get("camera_names") or ()
    for name in obs_cams:
        name_str = str(name)
        if name_str in available and name_str not in ordered:
            ordered.append(name_str)
    remaining = sorted(name for name in available if name not in ordered)
    ordered.extend(remaining)
    return tuple(ordered)


def compute_layout(
    cameras: tuple[str, ...],
    camera_sizes: dict[str, tuple[int, int]],
    *,
    overlay: bool = True,
) -> VideoLayout:
    """Compute grid dimensions guaranteed to have even width and height for H.264."""
    if not cameras:
        raise ValueError("至少需要一路相机用于视频合成")
    tile_w = max(camera_sizes[c][0] for c in cameras)
    tile_h = max(camera_sizes[c][1] for c in cameras)
    count = len(cameras)
    if count <= 3:
        cols, rows = count, 1
    else:
        cols = math.ceil(math.sqrt(count))
        rows = math.ceil(count / cols)
    hud_h = _HUD_HEIGHT if overlay else 0
    label_h = _LABEL_HEIGHT if overlay else 0
    raw_w = cols * tile_w
    raw_h = hud_h + rows * (label_h + tile_h)
    width = raw_w + (raw_w % 2)
    height = raw_h + (raw_h % 2)
    return VideoLayout(
        cameras=cameras,
        camera_sizes=dict(camera_sizes),
        tile_width=tile_w,
        tile_height=tile_h,
        cols=cols,
        rows=rows,
        hud_height=hud_h,
        label_height=label_h,
        width=width,
        height=height,
    )


def _probe_bag_streams(
    bag_dir: Path,
    camera_topics: dict[str, str],
    cameras: tuple[str, ...],
    topic_types: dict[str, str],
) -> tuple[dict[str, tuple[int, int]], float]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    wanted_topics = {camera_topics[cam]: cam for cam in cameras}
    msg_types = {topic: get_message(topic_types[topic]) for topic in wanted_topics}
    sizes: dict[str, tuple[int, int]] = {}
    primary_cam = cameras[0]
    primary_stamps: list[int] = []

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    try:
        while reader.has_next():
            topic, payload, ts = reader.read_next()
            cam = wanted_topics.get(topic)
            if cam is None:
                continue
            need_size = cam not in sizes
            need_stamp = cam == primary_cam and len(primary_stamps) < 25
            if not need_size and not need_stamp:
                continue
            msg = deserialize_message(payload, msg_types[topic])
            if need_size:
                sizes[cam] = (int(msg.image.width), int(msg.image.height))
            if need_stamp:
                stamp = getattr(msg.image.header, "stamp", None)
                stamp_ns = (
                    int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                    if stamp is not None
                    else int(ts)
                )
                primary_stamps.append(stamp_ns if stamp_ns > 0 else int(ts))
            if len(sizes) == len(cameras) and len(primary_stamps) >= 25:
                break
    finally:
        reader.close()

    missing_sizes = [cam for cam in cameras if cam not in sizes]
    if missing_sizes:
        raise ValueError(f"未能读取相机分辨率: {', '.join(missing_sizes)}")

    deltas = [
        b - a
        for a, b in zip(primary_stamps, primary_stamps[1:], strict=False)
        if b > a
    ]
    if deltas:
        median_dt_ns = statistics.median(deltas)
        fps = round(1_000_000_000.0 / median_dt_ns, 3)
        fps = min(120.0, max(1.0, fps))
    else:
        fps = _DEFAULT_FPS
    return sizes, fps


def _task_outcome_summary(
    manifest: dict[str, Any], task_result: dict[str, Any]
) -> tuple[str, str]:
    task = task_result.get("task") if isinstance(task_result.get("task"), dict) else {}
    state = str(
        task.get("state")
        or (task_result.get("result") or {}).get("outcome")
        or (manifest.get("task") or {}).get("outcome")
        or ""
    )
    reason = str(
        task.get("failure_reason")
        or task.get("reason")
        or (task_result.get("result") or {}).get("reason")
        or (manifest.get("task") or {}).get("reason")
        or ""
    )
    return state or "UNKNOWN", reason


def inspect_episode(
    input_dir: Path,
    *,
    requested_cameras: list[str] | None = None,
    overlay: bool = True,
) -> dict[str, Any]:
    """Inspect a recorded episode directory and return layout & stream metadata."""
    paths = resolve_episode_paths(input_dir)
    topic_types, topic_counts = _metadata_topics(paths.bag_dir)
    camera_topics: dict[str, str] = {}
    for topic in sorted(topic_types):
        cam = _camera_name_from_topic(topic)
        if cam is not None:
            camera_topics[cam] = topic

    manifest = _read_json_dict(paths.manifest_path)
    task_result = _read_json_dict(paths.task_result_path)
    cameras = _select_cameras(camera_topics, manifest, requested_cameras)
    camera_sizes, fps = _probe_bag_streams(paths.bag_dir, camera_topics, cameras, topic_types)
    layout = compute_layout(cameras, camera_sizes, overlay=overlay)

    config = manifest.get("configuration") or {}
    outcome, reason = _task_outcome_summary(manifest, task_result)
    total_frames = max(topic_counts.get(camera_topics[cam], 0) for cam in cameras)
    return {
        "episode_dir": str(paths.episode_dir),
        "bag_dir": str(paths.bag_dir),
        "episode_id": str(manifest.get("episode_id") or paths.episode_dir.name),
        "cameras": list(cameras),
        "camera_topics": {cam: camera_topics[cam] for cam in cameras},
        "camera_sizes": {cam: list(camera_sizes[cam]) for cam in cameras},
        "tile_width": layout.tile_width,
        "tile_height": layout.tile_height,
        "cols": layout.cols,
        "rows": layout.rows,
        "hud_height": layout.hud_height,
        "label_height": layout.label_height,
        "width": layout.width,
        "height": layout.height,
        "fps": fps,
        "total_frames": total_frames,
        "action_source": str(config.get("action_source") or "-"),
        "seed": config.get("reset_seed"),
        "task_outcome": outcome,
        "task_reason": reason,
    }


def _load_fonts() -> tuple[ImageFont.ImageFont, ImageFont.ImageFont]:
    try:
        regular = ImageFont.truetype(str(_FONT_PATH), 13)
        bold = ImageFont.truetype(str(_FONT_BOLD_PATH), 13)
        return regular, bold
    except OSError:
        default = ImageFont.load_default()
        return default, default


def _parse_task_state(msg: object) -> dict[str, Any]:
    phase = str(getattr(msg, "phase", "") or "")
    succeeded = bool(getattr(msg, "succeeded", False))
    terminal = bool(getattr(msg, "terminal", False))
    failure_reason = str(getattr(msg, "failure_reason", "") or "")
    passed = 0
    total = 0
    raw_metrics = getattr(msg, "metrics_json", "") or ""
    if raw_metrics:
        try:
            metrics = json.loads(raw_metrics)
            conds = metrics.get("conditions") if isinstance(metrics, dict) else None
            if isinstance(conds, list):
                total = len(conds)
                passed = sum(1 for item in conds if isinstance(item, dict) and item.get("passed"))
        except (json.JSONDecodeError, TypeError):
            pass
    return {
        "phase": phase,
        "succeeded": succeeded,
        "terminal": terminal,
        "failure_reason": failure_reason,
        "cond_passed": passed,
        "cond_total": total,
    }


def compose_frame(
    layout: VideoLayout,
    frames: dict[str, tuple[int, int, bytes]],
    *,
    frame_index: int,
    total_frames: int,
    rel_time_s: float,
    fps: float,
    metadata: dict[str, Any],
    task_state: dict[str, Any] | None,
    fonts: tuple[ImageFont.ImageFont, ImageFont.ImageFont],
) -> bytes:
    """Compose one multi-camera RGB24 canvas frame."""
    canvas = Image.new("RGB", (layout.width, layout.height), (18, 20, 26))
    draw = ImageDraw.Draw(canvas)
    regular_font, bold_font = fonts

    if layout.hud_height > 0:
        draw.rectangle((0, 0, layout.width, layout.hud_height - 1), fill=(26, 30, 38))
        draw.line((0, layout.hud_height - 1, layout.width, layout.hud_height - 1), fill=(58, 66, 82))
        ep_id = metadata.get("episode_id", "-")
        line1 = (
            f"{ep_id}  |  t={rel_time_s:05.2f}s  |  "
            f"frame {frame_index}/{max(frame_index, total_frames)}  |  {fps:.1f} fps"
        )
        draw.text((10, 5), line1, fill=(235, 240, 248), font=bold_font)

        src = metadata.get("action_source", "-")
        seed = metadata.get("seed")
        seed_text = "-" if seed is None else str(seed)
        if task_state:
            phase = task_state.get("phase") or "-"
            cond_text = (
                f"{task_state['cond_passed']}/{task_state['cond_total']}"
                if task_state.get("cond_total")
                else "-"
            )
            if task_state.get("succeeded"):
                status_text = "SUCCEEDED"
                status_color = (96, 224, 136)
            elif task_state.get("terminal"):
                reason = task_state.get("failure_reason") or "FAILED"
                status_text = reason[:38]
                status_color = (245, 110, 110)
            else:
                status_text = phase
                status_color = (120, 200, 255)
        else:
            cond_text = "-"
            outcome = str(metadata.get("task_outcome") or "-")
            status_text = outcome
            status_color = (
                (96, 224, 136)
                if outcome in {"SUCCEEDED", "succeeded"}
                else (245, 110, 110)
                if outcome in {"FAILED", "failed"}
                else (200, 206, 218)
            )
        line2_prefix = f"src={src}  seed={seed_text}  |  cond={cond_text}  |  "
        draw.text((10, 25), line2_prefix, fill=(182, 192, 208), font=regular_font)
        prefix_w = int(draw.textlength(line2_prefix, font=regular_font))
        draw.text((10 + prefix_w, 25), status_text, fill=status_color, font=bold_font)

    for idx, cam in enumerate(layout.cameras):
        col = idx % layout.cols
        row = idx // layout.cols
        x0 = col * layout.tile_width
        y0 = layout.hud_height + row * (layout.label_height + layout.tile_height)
        if layout.label_height > 0:
            draw.rectangle(
                (x0, y0, x0 + layout.tile_width - 1, y0 + layout.label_height - 1),
                fill=(34, 39, 50),
            )
            draw.text((x0 + 8, y0 + 4), cam, fill=(210, 220, 235), font=regular_font)
        img_y = y0 + layout.label_height
        sample = frames.get(cam)
        if sample is not None:
            w, h, raw = sample
            tile_img = Image.frombytes("RGB", (w, h), raw)
            if (w, h) != (layout.tile_width, layout.tile_height):
                tile_img = tile_img.resize(
                    (layout.tile_width, layout.tile_height), Image.Resampling.BILINEAR
                )
            canvas.paste(tile_img, (x0, img_y))
        else:
            draw.rectangle(
                (x0, img_y, x0 + layout.tile_width - 1, img_y + layout.tile_height - 1),
                fill=(24, 24, 28),
            )
            draw.text(
                (x0 + 14, img_y + layout.tile_height // 2 - 8),
                f"{cam}: NO FRAME",
                fill=(180, 110, 110),
                font=regular_font,
            )
        if col > 0:
            draw.line(
                (x0, y0, x0, y0 + layout.label_height + layout.tile_height),
                fill=(48, 54, 68),
            )
    return canvas.tobytes()


def stream_frames(
    input_dir: Path,
    *,
    requested_cameras: list[str] | None = None,
    overlay: bool = True,
) -> int:
    """Stream composited RGB24 frames to stdout and progress markers to stderr."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    info = inspect_episode(input_dir, requested_cameras=requested_cameras, overlay=overlay)
    cameras = tuple(info["cameras"])
    camera_sizes = {cam: (int(v[0]), int(v[1])) for cam, v in info["camera_sizes"].items()}
    layout = compute_layout(cameras, camera_sizes, overlay=overlay)
    bag_dir = Path(info["bag_dir"])
    topic_types, _ = _metadata_topics(bag_dir)
    camera_topics = info["camera_topics"]
    topic_to_cam = {topic: cam for cam, topic in camera_topics.items()}
    task_topics = {
        topic for topic in topic_types if topic.endswith("/task_state")
    }
    msg_types = {
        topic: get_message(topic_types[topic])
        for topic in (*topic_to_cam, *task_topics)
    }

    fonts = _load_fonts()
    pending: dict[int, dict[str, tuple[int, int, int, bytes]]] = {}
    first_stamp_ns: int | None = None
    latest_task_state: dict[str, Any] | None = None
    written = 0
    total_frames = int(info["total_frames"] or 0)
    fps = float(info["fps"] or _DEFAULT_FPS)
    out_stream = sys.stdout.buffer

    def flush_seq(seq: int) -> None:
        nonlocal first_stamp_ns, written
        bucket = pending.pop(seq)
        stamps = [item[0] for item in bucket.values() if item[0] > 0]
        frame_stamp_ns = min(stamps) if stamps else 0
        if first_stamp_ns is None and frame_stamp_ns > 0:
            first_stamp_ns = frame_stamp_ns
        rel_time_s = (
            max(0.0, (frame_stamp_ns - first_stamp_ns) / 1_000_000_000.0)
            if first_stamp_ns is not None and frame_stamp_ns > 0
            else written / fps
        )
        rgb_map = {cam: (item[1], item[2], item[3]) for cam, item in bucket.items()}
        written += 1
        raw_canvas = compose_frame(
            layout,
            rgb_map,
            frame_index=written,
            total_frames=total_frames,
            rel_time_s=rel_time_s,
            fps=fps,
            metadata=info,
            task_state=latest_task_state,
            fonts=fonts,
        )
        out_stream.write(raw_canvas)
        if written == 1 or written % 5 == 0:
            sys.stderr.write(f"PROGRESS {written} {max(written, total_frames)}\n")
            sys.stderr.flush()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    try:
        while reader.has_next():
            topic, payload, ts = reader.read_next()
            if topic in task_topics:
                msg = deserialize_message(payload, msg_types[topic])
                latest_task_state = _parse_task_state(msg)
                continue
            cam = topic_to_cam.get(topic)
            if cam is None:
                continue
            msg = deserialize_message(payload, msg_types[topic])
            image = msg.image
            w, h = int(image.width), int(image.height)
            if image.encoding != "rgb8" or int(image.step) != w * 3:
                raise ValueError(f"相机 {cam} 帧编码不是 rgb8 ({image.encoding})")
            stamp = getattr(image.header, "stamp", None)
            stamp_ns = (
                int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                if stamp is not None
                else int(ts)
            )
            seq = int(getattr(msg, "frame_seq", written + 1))
            pending.setdefault(seq, {})[cam] = (
                stamp_ns if stamp_ns > 0 else int(ts),
                w,
                h,
                bytes(image.data),
            )
            while pending:
                oldest = min(pending)
                if len(pending[oldest]) == len(cameras) or len(pending) > _MAX_PENDING_FRAMES:
                    flush_seq(oldest)
                else:
                    break
        while pending:
            flush_seq(min(pending))
        out_stream.flush()
    except BrokenPipeError:
        return 1
    finally:
        reader.close()

    sys.stderr.write(f"PROGRESS {written} {written}\n")
    sys.stderr.flush()
    return 0 if written > 0 else 1


def parser() -> argparse.ArgumentParser:
    cli = argparse.ArgumentParser(description=__doc__)
    sub = cli.add_subparsers(dest="command", required=True)
    for name in ("inspect", "stream"):
        item = sub.add_parser(name)
        item.add_argument("input_dir", type=Path)
        item.add_argument("--cameras", default="", help="comma-separated camera names")
        item.add_argument("--no-overlay", action="store_true")
    return cli


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    requested = [c.strip() for c in args.cameras.split(",") if c.strip()] or None
    overlay = not args.no_overlay
    try:
        if args.command == "inspect":
            doc = inspect_episode(
                args.input_dir, requested_cameras=requested, overlay=overlay
            )
            print(json.dumps(doc, ensure_ascii=False))
            return 0
        return stream_frames(
            args.input_dir, requested_cameras=requested, overlay=overlay
        )
    except (FileNotFoundError, ValueError, OSError) as error:
        print(f"错误: {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
