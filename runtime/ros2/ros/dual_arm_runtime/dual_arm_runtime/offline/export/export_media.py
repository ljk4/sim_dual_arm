"""Recorded RGB extraction, PNG hashing and preview GIFs for export_episode."""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any


def _frame_label_map(
    rows: list[dict[str, Any]],
    frame_map: dict[int, tuple[int, ...]],
    camera_names: tuple[str, ...],
    phase_observations: dict[str, int],
) -> tuple[dict[tuple[str, int], list[tuple[int, int, int]]], dict[tuple[str, int], str]]:
    frame_to_labels: dict[tuple[str, int], list[tuple[int, int, int]]] = defaultdict(list)
    for index, row in enumerate(rows):
        observation_id = int(row["observation_id"])
        observation_time = int(row["observation_backend_time_ns"])
        sequences = frame_map.get(observation_id)
        if sequences is None or len(sequences) != len(camera_names):
            from robot_contracts.models import ContractError

            raise ContractError("LABEL_EXPORT_RGB", f"label {index} has no complete capture")
        for camera, sequence in zip(camera_names, sequences, strict=True):
            frame_to_labels[(camera, int(sequence))].append(
                (index, observation_id, observation_time)
            )
    phase_frames: dict[tuple[str, int], str] = {}
    for phase, observation_id in phase_observations.items():
        sequences = frame_map.get(observation_id)
        if sequences is None:
            continue
        for camera, sequence in zip(camera_names, sequences, strict=True):
            phase_frames[(camera, int(sequence))] = phase
    return frame_to_labels, phase_frames


def _check_camera_sizes(
    camera_names: tuple[str, ...], camera_sizes: dict[str, tuple[int, int]]
) -> None:
    for camera in camera_names:
        width, height = camera_sizes.get(camera, (0, 0))
        if width <= 0 or height <= 0:
            from robot_contracts.models import ContractError

            raise ContractError("LABEL_EXPORT_RGB", f"no expected size for {camera}")


def _save_recorded_frame(
    message: object,
    key: tuple[str, int],
    camera_sizes: dict[str, tuple[int, int]],
    frame_to_labels: dict[tuple[str, int], list[tuple[int, int, int]]],
    out: Path,
    mapping: dict[int, dict[str, dict[str, Any]]],
    preview_indices: set[int],
    gif_frames: dict[str, list[Any]],
) -> None:
    import hashlib as hash_module

    from PIL import Image
    from robot_contracts.models import ContractError

    camera, sequence = key
    image = message.image
    width, height = camera_sizes[camera]
    if (int(image.width), int(image.height)) != (width, height):
        raise ContractError(
            "LABEL_EXPORT_RGB",
            f"{camera}/{sequence} is {image.width}x{image.height}, expected {width}x{height}",
        )
    if image.encoding != "rgb8" or int(image.step) != width * 3:
        raise ContractError("LABEL_EXPORT_RGB", f"{camera}/{sequence} is not packed rgb8")
    raw = bytes(image.data)
    if len(raw) != width * height * 3:
        raise ContractError("LABEL_EXPORT_RGB", f"{camera}/{sequence} payload is short")
    frame = Image.frombytes("RGB", (width, height), raw)
    for index, observation_id, observation_time in frame_to_labels.get(key, ()):
        path = out / "rgb" / f"{index:04d}" / f"{camera}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        frame.save(path)
        png_bytes = path.read_bytes()
        mapping[index][camera] = {
            "path": path.relative_to(out).as_posix(),
            "frame_seq": sequence,
            "observation_id": observation_id,
            "observation_backend_time_ns": observation_time,
            "png_sha256": hash_module.sha256(png_bytes).hexdigest(),
            "rgb_sha256": hash_module.sha256(raw).hexdigest(),
        }
        if index in preview_indices:
            gif_frames[camera].append(frame.copy())


def _preview_indices(rows: list[dict[str, Any]]) -> set[int]:
    preview_count = min(60, len(rows))
    return {
        index * (len(rows) - 1) // max(1, preview_count - 1) for index in range(preview_count)
    }


def _write_videos(gif_frames: dict[str, list[Any]], out: Path, rows: list[dict[str, Any]]) -> None:
    video_dir = out / "video"
    video_dir.mkdir(parents=True, exist_ok=True)
    for camera, frames in gif_frames.items():
        if frames:
            frames[0].save(
                video_dir / f"{camera}.gif",
                save_all=True,
                append_images=frames[1:],
                duration=max(50, round(len(rows) * 50 / len(frames))),
                loop=0,
            )


def write_images(
    recording: Path,
    out: Path,
    camera_names: tuple[str, ...],
    rows: list[dict[str, Any]],
    frame_map: dict[int, tuple[int, ...]],
    phase_observations: dict[str, int],
    camera_sizes: dict[str, tuple[int, int]],
    *,
    previews: bool = True,
) -> dict[int, dict[str, dict[str, Any]]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from robot_contracts.models import ContractError
    from rosidl_runtime_py.utilities import get_message

    frame_to_labels, phase_frames = _frame_label_map(
        rows, frame_map, camera_names, phase_observations if previews else {}
    )
    selected = set(frame_to_labels) | set(phase_frames)
    _check_camera_sizes(camera_names, camera_sizes)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(recording), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {
        item.name: get_message(item.type)
        for item in reader.get_all_topics_and_types()
        if "/cameras/" in item.name and item.name.endswith("/frame")
    }
    mapping: dict[int, dict[str, dict[str, Any]]] = defaultdict(dict)
    preview_dir = out / "preview"
    if previews:
        preview_dir.mkdir(parents=True, exist_ok=True)
    gif_frames: dict[str, list[Any]] = defaultdict(list)
    preview_indices = _preview_indices(rows) if previews else set()
    seen: set[tuple[str, int]] = set()
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic not in types:
            continue
        parts = topic.split("/")
        if len(parts) < 3 or parts[-1] != "frame" or parts[-3] != "cameras":
            continue
        camera = parts[-2]
        if camera not in camera_names:
            continue
        message = deserialize_message(payload, types[topic])
        sequence = int(message.frame_seq)
        key = (camera, sequence)
        if key not in selected:
            continue
        if key in seen:
            raise ContractError("LABEL_EXPORT_RGB", f"duplicate frame in bag: {camera}/{sequence}")
        seen.add(key)
        _save_recorded_frame(
            message,
            key,
            camera_sizes,
            frame_to_labels,
            out,
            mapping,
            preview_indices,
            gif_frames,
        )
        phase = phase_frames.get(key)
        if phase is not None:
            from PIL import Image

            image = message.image
            width, height = camera_sizes[camera]
            Image.frombytes("RGB", (width, height), bytes(image.data)).save(
                preview_dir / f"{phase}_{camera}.png"
            )
    missing = {key for key in selected if key not in seen}
    if missing:
        first = sorted(missing)[0]
        raise ContractError(
            "LABEL_EXPORT_RGB", f"missing required camera frame: {first[0]}/{first[1]}"
        )
    if previews:
        _write_videos(gif_frames, out, rows)
    return mapping


__all__ = ["write_images"]
