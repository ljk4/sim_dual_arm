"""Recorded RGB extraction and PNG hashing for export_episode."""

from __future__ import annotations

import hashlib
from collections import defaultdict
from collections.abc import Mapping
from pathlib import Path
from typing import Any


def _frame_label_map(
    rows: list[dict[str, Any]],
    frame_map: dict[int, tuple[int, ...]],
    camera_names: tuple[str, ...],
) -> dict[tuple[str, int], list[tuple[int, int, int]]]:
    from robot_contracts.models import ContractError

    frame_to_labels: dict[tuple[str, int], list[tuple[int, int, int]]] = defaultdict(list)
    for index, row in enumerate(rows):
        observation_id = int(row["observation_id"])
        observation_time = int(row["observation_backend_time_ns"])
        sequences = frame_map.get(observation_id)
        if sequences is None or len(sequences) != len(camera_names):
            raise ContractError("LABEL_EXPORT_RGB", f"label {index} has no complete capture")
        for camera, sequence in zip(camera_names, sequences, strict=True):
            frame_to_labels[(camera, int(sequence))].append(
                (index, observation_id, observation_time)
            )
    return frame_to_labels


def _check_camera_sizes(
    camera_names: tuple[str, ...], camera_sizes: dict[str, tuple[int, int]]
) -> None:
    from robot_contracts.models import ContractError

    for camera in camera_names:
        width, height = camera_sizes.get(camera, (0, 0))
        if width <= 0 or height <= 0:
            raise ContractError("LABEL_EXPORT_RGB", f"no expected size for {camera}")


def _save_recorded_frame(
    message: object,
    key: tuple[str, int],
    camera_sizes: dict[str, tuple[int, int]],
    frame_to_labels: dict[tuple[str, int], list[tuple[int, int, int]]],
    out: Path,
    mapping: dict[int, dict[str, dict[str, Any]]],
) -> None:
    from PIL import Image
    from robot_contracts.models import ContractError

    camera, sequence = key
    image = message.image  # type: ignore[attr-defined]
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
    rgb_sha = hashlib.sha256(raw).hexdigest()
    for index, observation_id, observation_time in frame_to_labels.get(key, ()):
        path = out / "rgb" / f"{index:04d}" / f"{camera}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        frame.save(path)
        png_sha = hashlib.sha256(path.read_bytes()).hexdigest()
        mapping[index][camera] = {
            "path": path.relative_to(out).as_posix(),
            "frame_seq": sequence,
            "observation_id": observation_id,
            "observation_backend_time_ns": observation_time,
            "png_sha256": png_sha,
            "rgb_sha256": rgb_sha,
        }


def write_images(
    source: Path | Mapping[tuple[str, int], object],
    out: Path,
    camera_names: tuple[str, ...],
    rows: list[dict[str, Any]],
    frame_map: dict[int, tuple[int, ...]],
    camera_sizes: dict[str, tuple[int, int]],
) -> dict[int, dict[str, dict[str, Any]]]:
    """Extract required label PNGs from either already-decoded frames or an MCAP bag."""
    from robot_contracts.models import ContractError

    frame_to_labels = _frame_label_map(rows, frame_map, camera_names)
    selected = set(frame_to_labels)
    _check_camera_sizes(camera_names, camera_sizes)
    mapping: dict[int, dict[str, dict[str, Any]]] = defaultdict(dict)
    seen: set[tuple[str, int]] = set()

    if isinstance(source, Mapping):
        for key in sorted(selected):
            message = source.get(key)
            if message is None:
                continue
            seen.add(key)
            _save_recorded_frame(message, key, camera_sizes, frame_to_labels, out, mapping)
    else:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(source), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        try:
            types = {
                item.name: get_message(item.type)
                for item in reader.get_all_topics_and_types()
                if "/cameras/" in item.name and item.name.endswith("/frame")
            }
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
                key = (camera, int(message.frame_seq))
                if key not in selected:
                    continue
                if key in seen:
                    raise ContractError("LABEL_EXPORT_RGB", f"duplicate frame in bag: {camera}/{key[1]}")
                seen.add(key)
                _save_recorded_frame(message, key, camera_sizes, frame_to_labels, out, mapping)
        finally:
            reader.close()

    missing = selected - seen
    if missing:
        first = sorted(missing)[0]
        raise ContractError(
            "LABEL_EXPORT_RGB", f"missing required camera frame: {first[0]}/{first[1]}"
        )
    return mapping
