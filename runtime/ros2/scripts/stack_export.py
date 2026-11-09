#!/usr/bin/env python3
"""Auditable, bounded export of one stack episode.

Outputs only JSON, PNG and GIF.  No pickle, framework database or ROS import is
needed by later learning conversion.  Labels are the actually applied follower
reference at observation + 50 ms; raw 60-point predictions are stored apart.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

import yaml

_PROJECT = Path(__file__).resolve().parents[3]
for _path in (
    _PROJECT / "contracts/python/src",
    _PROJECT / "libraries/ros_support/src",
    _PROJECT / "runtime/ros2/ros/dual_arm_runtime",
):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from dual_arm_runtime.label_export import (  # noqa: E402
    AppliedBlock,
    BrakeInterval,
    ObservationFact,
    RawPrediction,
    build_labels,
)
from robot_contracts.models import RobotProfile  # noqa: E402

PHASES = ("pregrasp", "grasp", "close", "lift", "transport", "lower", "open", "retreat", "stable")


def _json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _expand_logs(paths: list[Path]) -> list[Path]:
    expanded: list[Path] = []
    for path in paths:
        if path.is_dir():
            expanded.extend(sorted(set(path.glob("*.log*")) | set(path.rglob("events.*.jsonl"))))
        else:
            expanded.append(path)
    return expanded


def _parse_log_events(paths: list[Path]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for path in _expand_logs(paths):
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        for line in lines:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(event, dict):
                events.append(event)
    return events


def _phase_observations(
    runtime_events: list[dict[str, Any]], observation_times: dict[int, int]
) -> dict[str, int]:
    result: dict[str, int] = {}
    by_time = {value: key for key, value in observation_times.items()}
    for event in runtime_events:
        if event.get("event") != "follower.chunk_accepted":
            continue
        fields = event.get("fields")
        if not isinstance(fields, dict):
            continue
        phase = str(fields.get("phase") or "")
        sim_time = event.get("sim_time_ns")
        observation_id = by_time.get(sim_time)
        if phase in PHASES and observation_id is not None and phase not in result:
            result[phase] = observation_id
    return result


def quality_summary(
    quality_events: list[dict[str, Any]],
    phase_observations: dict[str, int],
    observation_times: dict[int, int],
    rules: dict[str, Any],
    *,
    action_source: str = "",
) -> dict[str, Any]:
    """Evaluate required/record_only rules over full phase intervals."""
    max_blackout_ns = round(float(rules.get("max_blackout_s", 0.35)) * 1e9)
    min_ratio = float(rules.get("min_qualified_ratio", 0.85))
    raw_rules = rules.get("rules")
    if not isinstance(raw_rules, list) or not raw_rules:
        return {
            "schema_version": 1,
            "qualified": False,
            "phase_based": bool(phase_observations),
            "failure_reasons": ["no quality rules configured"],
            "rules": [],
            "phases": {},
            "max_blackout_s": max_blackout_ns / 1e9,
            "min_qualified_ratio": min_ratio,
        }
    timelines: dict[tuple[str, str], list[tuple[int, int]]] = defaultdict(list)
    for event in quality_events:
        if event.get("event") != "backend.render.quality":
            continue
        fields = event.get("fields")
        if not isinstance(fields, dict):
            continue
        sim_time = event.get("sim_time_ns")
        if not isinstance(sim_time, int) or sim_time < 0:
            continue
        for name, value in fields.items():
            if not isinstance(name, str) or not name.endswith("_pixels"):
                continue
            camera, _, obj = name[: -len("_pixels")].rpartition("_")
            if camera and obj and isinstance(value, int):
                timelines[(camera, obj)].append((sim_time, value))
    for values in timelines.values():
        values.sort()
    ordered_times = sorted(observation_times.values())
    last_time = ordered_times[-1] if ordered_times else 0
    phase_starts = {
        phase: observation_times[observation_id]
        for phase, observation_id in phase_observations.items()
        if observation_id in observation_times
    }
    ordered_phases = sorted(phase_starts.items(), key=lambda item: item[1])
    phase_intervals: dict[str, tuple[int, int]] = {}
    for index, (phase, start) in enumerate(ordered_phases):
        end = (
            ordered_phases[index + 1][1]
            if index + 1 < len(ordered_phases)
            else last_time + 250_000_000
        )
        phase_intervals[phase] = (start, end)
    phase_based = bool(phase_intervals)
    if not phase_based:
        phase_intervals = {"global": (ordered_times[0] if ordered_times else 0, last_time + 1)}
    failures: list[str] = []
    phase_report: dict[str, Any] = {}
    rule_report: list[dict[str, Any]] = []
    for rule in raw_rules:
        if not isinstance(rule, dict):
            raise ValueError("quality rule must be an object")
        camera = str(rule["camera"])
        obj = str(rule["object"])
        mode = str(rule["mode"])
        if mode not in {"required", "record_only"}:
            raise ValueError(f"unsupported quality mode {mode!r}")
        minimum = int(rule.get("min_pixels", 0))
        phases = [str(value) for value in rule.get("phases", [])]
        if not phase_based:
            phases = ["global"]
        for phase in phases:
            interval = phase_intervals.get(phase)
            if interval is None:
                if mode == "required":
                    failures.append(f"{phase}: no teacher phase interval for {camera}.{obj}")
                continue
            start, end = interval
            samples = [
                (time, pixels)
                for time, pixels in timelines.get((camera, obj), [])
                if start <= time <= end
            ]
            entry = _rule_metrics(samples, minimum, max_blackout_ns)
            entry.update({"camera": camera, "object": obj, "mode": mode, "phase": phase})
            rule_report.append(entry)
            phase_entry = phase_report.setdefault(
                phase,
                {
                    "present": phase in phase_observations if phase_based else True,
                    "start_sim_time_ns": start,
                    "end_sim_time_ns": end,
                    "rules": [],
                },
            )
            phase_entry["rules"].append(entry)
            if mode == "required":
                if not samples:
                    failures.append(f"{phase}: no {camera}.{obj} quality samples")
                elif entry["qualified_ratio"] < min_ratio:
                    failures.append(
                        f"{phase}: {camera}.{obj} ratio {entry['qualified_ratio']:.3f} "
                        f"< {min_ratio:.3f}"
                    )
                elif entry["max_blackout_s"] > max_blackout_ns / 1e9:
                    failures.append(
                        f"{phase}: {camera}.{obj} blackout {entry['max_blackout_s']:.3f}s "
                        f"> {max_blackout_ns / 1e9:.3f}s"
                    )
    if phase_based:
        for phase in PHASES:
            if phase not in phase_report:
                phase_report[phase] = {
                    "present": phase in phase_observations,
                    "rules": [],
                }
    qualified = bool(timelines) and not failures and bool(rule_report)
    return {
        "schema_version": 1,
        "qualified": qualified,
        "phase_based": phase_based,
        "action_source": action_source,
        "failure_reasons": failures,
        "max_blackout_s": max_blackout_ns / 1e9,
        "min_qualified_ratio": min_ratio,
        "phases": phase_report,
        "rules": rule_report,
        "samples": {f"{camera}.{obj}": len(values) for (camera, obj), values in timelines.items()},
    }


def _rule_metrics(
    samples: list[tuple[int, int]], minimum: int, max_blackout_ns: int
) -> dict[str, Any]:
    if not samples:
        return {
            "samples": 0,
            "qualified_ratio": 0.0,
            "max_blackout_s": 0.0,
            "min_pixels": 0,
            "required_pixels": minimum,
            "ok": False,
        }
    passed = sum(1 for _time, pixels in samples if pixels >= minimum)
    longest = 0
    run_start: int | None = None
    for time, pixels in samples:
        if pixels < minimum:
            if run_start is None:
                run_start = time
        elif run_start is not None:
            longest = max(longest, time - run_start)
            run_start = None
    if run_start is not None:
        longest = max(longest, samples[-1][0] - run_start)
    ratio = passed / len(samples)
    return {
        "samples": len(samples),
        "qualified_ratio": round(ratio, 6),
        "max_blackout_s": round(longest / 1e9, 6),
        "min_pixels": min(pixels for _time, pixels in samples),
        "required_pixels": minimum,
    }


def _read_metadata(recording: Path, root: str) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from robot_contracts.stream_request import StreamRequest
    from robot_ros_support.follower_conversions import follower_reference_from_message
    from robot_ros_support.stream_conversions import (
        control_event_from_message,
        execution_feedback_from_message,
        stream_command_from_message,
    )
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(recording), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
    states: dict[int, Any] = {}
    frames: dict[tuple[str, int], Any] = {}
    indices: dict[int, Any] = {}
    requests: dict[int, tuple[str, int, tuple[int, ...], tuple[tuple[float, ...], ...]]] = {}
    references: dict[tuple[str, int], Any] = {}
    chunk_events: dict[tuple[str, int], dict[str, int]] = {}
    brakes: list[BrakeInterval] = []
    camera_names: tuple[str, ...] = ()
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic not in types:
            continue
        message = deserialize_message(payload, types[topic])
        if topic == "/runtime/observation_index":
            indices[int(message.state_seq)] = message
            if not camera_names:
                camera_names = tuple(message.camera_names)
        elif topic == f"{root}/state":
            states[int(message.state_seq)] = message
        elif topic.startswith(f"{root}/cameras/") and topic.endswith("/frame"):
            camera = topic.split("/")[-2]
            image = message.image
            if image.encoding != "rgb8" or image.step != int(image.width) * 3:
                from robot_contracts.models import ContractError

                raise ContractError(
                    "LABEL_EXPORT_RGB",
                    f"{camera} frame {message.frame_seq} is not packed rgb8",
                )
            key = (camera, int(message.frame_seq))
            if key in frames:
                previous = frames[key]
                if _stamp_ns(previous.image.header.stamp) != _stamp_ns(image.header.stamp) or bytes(
                    previous.image.data
                ) != bytes(image.data):
                    from robot_contracts.models import ContractError

                    raise ContractError(
                        "LABEL_EXPORT_CAPTURE",
                        f"duplicate capture has different pixels: {camera}/{key[1]}",
                    )
            else:
                frames[key] = message
        elif topic == "/runtime/commands":
            request = stream_command_from_message(message)
            if isinstance(request, StreamRequest) and request.anchor is not None:
                key = int(request.trajectory.observation_id)
                targets = request.trajectory.targets
                requests.setdefault(
                    key,
                    (
                        request.trajectory.command_id,
                        request.anchor.predecessor_seq + 1,
                        tuple(int(target.time_ns) for target in targets),
                        tuple(
                            tuple(float(value) for value in target.positions) for target in targets
                        ),
                    ),
                )
        elif topic == f"{root}/follower_references":
            record = follower_reference_from_message(message)
            references[(record.command_id, record.chunk_seq)] = record
        elif topic == f"{root}/control_events":
            event = control_event_from_message(message)
            if not event.is_chunk and event.phase == "STOPPING":
                # This is the controller's switch from the follower to BRAKE.
                # Overall started_time is the stream start, and canceled_time
                # is when braking finishes; neither is the switch instant.
                brakes.append(BrakeInterval(int(event.sim_time_ns), (1 << 63) - 1))
            if event.is_chunk and event.chunk_seq is not None:
                key = (event.command_id, int(event.chunk_seq))
                phases = chunk_events.setdefault(key, {})
                if event.phase in {"STARTED", "RETIRED", "CANCELED", "REJECTED"}:
                    phases.setdefault(event.phase, int(event.sim_time_ns))
        elif topic == f"{root}/execution_feedback":
            execution = execution_feedback_from_message(message)
            if execution.reference_kind == "BRAKE":
                start = (
                    int(execution.reference_application_sim_time_ns)
                    if execution.reference_application_sim_time_ns > 0
                    else int(execution.measurement_sim_time_ns)
                )
                end = (
                    int(execution.finished_sim_time_ns)
                    if execution.has_finished
                    else int(execution.measurement_sim_time_ns)
                )
                if end > start:
                    brakes.append(BrakeInterval(start, end))
    if not camera_names:
        from robot_contracts.models import ContractError

        raise ContractError("LABEL_EXPORT_CAPTURE", "no camera names in observation index")
    return {
        "states": states,
        "frames": frames,
        "indices": indices,
        "requests": requests,
        "references": references,
        "chunk_events": chunk_events,
        "brakes": brakes,
        "camera_names": camera_names,
    }


def _observations(
    metadata: dict[str, Any], camera_names: tuple[str, ...]
) -> tuple[list[ObservationFact], dict[int, int], dict[int, tuple[int, ...]]]:
    observations: list[ObservationFact] = []
    times: dict[int, int] = {}
    frame_map: dict[int, tuple[int, ...]] = {}
    for state_seq in sorted(metadata["states"]):
        state = metadata["states"][state_seq]
        stamp_ns = _stamp_ns(state.stamp)
        if not camera_names:
            continue
        sequences = []
        complete = True
        for camera in camera_names:
            frame = metadata["frames"].get((camera, state_seq))
            if frame is None or _stamp_ns(frame.image.header.stamp) != stamp_ns:
                complete = False
                break
            sequences.append(state_seq)
        if not complete:
            continue
        index = metadata["indices"].get(state_seq)
        if (
            index is not None
            and _stamp_ns(index.reference_time) == stamp_ns
            and tuple(int(value) for value in index.frame_seqs) == tuple(sequences)
        ):
            observation_id = state_seq
            source = "observation_index"
        else:
            observation_id = state_seq
            source = "state_rebuild" if index is None else "state_rebuild_index_mismatch"
        observations.append(
            ObservationFact(
                observation_id=observation_id,
                sim_time_ns=stamp_ns,
                state_seq=state_seq,
                measured_positions=tuple(float(value) for value in state.joints.position),
                measured_velocities=tuple(float(value) for value in state.joints.velocity),
                frame_seqs=tuple(sequences),
                source=source,
            )
        )
        times[observation_id] = stamp_ns
        frame_map[observation_id] = tuple(sequences)
    if len({item.observation_id for item in observations}) != len(observations):
        from robot_contracts.models import ContractError

        raise ContractError("LABEL_EXPORT_CAPTURE", "rebuilt observation IDs collide")
    return observations, times, frame_map


def _build(
    metadata: dict[str, Any],
    camera_names: tuple[str, ...],
    runtime_events: list[dict[str, Any]],
    max_labels: int,
    profile: RobotProfile,
) -> tuple[
    list[dict[str, Any]],
    list[RawPrediction],
    dict[str, Any],
    dict[int, int],
    dict[int, tuple[int, ...]],
]:
    observations, times, frame_map = _observations(metadata, camera_names)
    index_time = {
        int(message.observation_id): _stamp_ns(message.reference_time)
        for message in metadata["indices"].values()
    }
    raw_by_chunk: dict[int, RawPrediction] = {}
    for source_observation_id, (
        command_id,
        chunk_seq,
        times_from_observation_ns,
        positions,
    ) in sorted(metadata["requests"].items()):
        reference = metadata["references"].get((command_id, chunk_seq))
        source_time = (
            reference.request.observation_time_ns
            if reference is not None
            else index_time.get(source_observation_id, 0)
        )
        raw_by_chunk.setdefault(
            chunk_seq,
            RawPrediction(
                chunk_seq=chunk_seq,
                command_id=command_id,
                source_observation_id=source_observation_id,
                source_observation_time_ns=source_time,
                times_from_observation_ns=times_from_observation_ns,
                positions=positions,
            ),
        )
    blocks: list[AppliedBlock] = []
    for (command_id, chunk_seq), reference in sorted(
        metadata["references"].items(), key=lambda item: item[1].start_sim_time_ns
    ):
        events = metadata["chunk_events"].get((command_id, chunk_seq), {})
        canceled = None
        if "CANCELED" in events or "REJECTED" in events:
            canceled = min(
                value for key, value in events.items() if key in {"CANCELED", "REJECTED"}
            )
        blocks.append(
            AppliedBlock(
                chunk_seq=chunk_seq,
                command_id=command_id,
                start_sim_time_ns=reference.start_sim_time_ns,
                end_sim_time_ns=reference.end_sim_time_ns,
                started_sim_time_ns=events.get("STARTED"),
                retired_sim_time_ns=events.get("RETIRED"),
                canceled_sim_time_ns=canceled,
                reference=reference,
            )
        )
    rows, summary = build_labels(
        observations,
        blocks,
        metadata["brakes"],
        raw_by_chunk,
        max_labels=max_labels,
        profile=profile,
    )
    summary["phase_observations"] = _phase_observations(
        runtime_events,
        {item.observation_id: item.sim_time_ns for item in observations},
    )
    return rows, list(raw_by_chunk.values()), summary, times, frame_map


def _stamp_ns(value: object) -> int:
    return int(getattr(value, "sec", 0)) * 1_000_000_000 + int(getattr(value, "nanosec", 0))


def _write_images(
    recording: Path,
    out: Path,
    camera_names: tuple[str, ...],
    rows: list[dict[str, Any]],
    frame_map: dict[int, tuple[int, ...]],
    phase_observations: dict[str, int],
    camera_sizes: dict[str, tuple[int, int]],
) -> dict[int, dict[str, dict[str, Any]]]:
    import hashlib

    import rosbag2_py
    from PIL import Image
    from rclpy.serialization import deserialize_message
    from robot_contracts.models import ContractError
    from rosidl_runtime_py.utilities import get_message

    frame_to_labels: dict[tuple[str, int], list[tuple[int, int, int]]] = defaultdict(list)
    for index, row in enumerate(rows):
        observation_id = int(row["observation_id"])
        observation_time = int(row["observation_sim_time_ns"])
        sequences = frame_map.get(observation_id)
        if sequences is None or len(sequences) != len(camera_names):
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
    selected = set(frame_to_labels) | set(phase_frames)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(recording), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
    mapping: dict[int, dict[str, dict[str, Any]]] = defaultdict(dict)
    preview_dir = out / "preview"
    preview_dir.mkdir(parents=True, exist_ok=True)
    gif_frames: dict[str, list[Any]] = defaultdict(list)
    preview_count = min(60, len(rows))
    preview_indices = {
        index * (len(rows) - 1) // max(1, preview_count - 1) for index in range(preview_count)
    }
    seen: set[tuple[str, int]] = set()
    for camera in camera_names:
        width, height = camera_sizes.get(camera, (0, 0))
        if width <= 0 or height <= 0:
            raise ContractError("LABEL_EXPORT_RGB", f"no expected size for {camera}")
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
                "observation_sim_time_ns": observation_time,
                "png_sha256": hashlib.sha256(png_bytes).hexdigest(),
                "rgb_sha256": hashlib.sha256(raw).hexdigest(),
            }
            if index in preview_indices:
                gif_frames[camera].append(frame.copy())
        phase = phase_frames.get(key)
        if phase is not None:
            frame.save(preview_dir / f"{phase}_{camera}.png")
    missing = {
        (camera, sequence) for (camera, sequence) in selected if (camera, sequence) not in seen
    }
    if missing:
        first = sorted(missing)[0]
        raise ContractError(
            "LABEL_EXPORT_RGB", f"missing required camera frame: {first[0]}/{first[1]}"
        )
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
    return mapping


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--campaign-config",
        type=Path,
        default=_PROJECT / "runtime/ros2/configs/experiments/stack_campaign.yaml",
    )
    parser.add_argument("--runtime-log", type=Path, action="append", default=[])
    parser.add_argument("--backend-log", type=Path, action="append", default=[])
    parser.add_argument("--max-labels", type=int, default=700)
    args = parser.parse_args(argv)
    episode = args.episode_dir.resolve()
    out = args.out.resolve()
    manifest = json.loads((episode / "manifest.json").read_text(encoding="utf-8"))
    from robot_contracts.episodes import manifest_schema_path
    from robot_contracts.profiles import profile_from_document, validate_document

    validate_document(manifest, manifest_schema_path(_PROJECT / "contracts/schemas", 1))
    if not manifest["recording"]["complete"]:
        raise ValueError(manifest["recording"]["failure_reason"])
    configuration = manifest.get("configuration", {})
    profile = profile_from_document(configuration["profile"], _PROJECT / "contracts/schemas")
    if profile.digest != manifest["robot_profile_hash"]:
        raise ValueError("recorded profile snapshot does not match the manifest")
    recipe = configuration.get("recipe", {})
    root = str(recipe.get("robot_namespace", "/robot")).rstrip("/")
    recording = episode / "recording"
    metadata = _read_metadata(recording, root)
    context = configuration["run_context"]
    runtime_events = [
        event
        for event in _parse_log_events(list(args.runtime_log) or [episode / "log"])
        if event.get("episode_id") == context["episode_id"]
        and event.get("clock_epoch") == context["clock_epoch"]
    ]
    camera_names = metadata["camera_names"]
    rows, raw, summary, times, frame_map = _build(
        metadata, camera_names, runtime_events, args.max_labels, profile
    )
    camera_sizes: dict[str, tuple[int, int]] = {}
    for camera in configuration.get("profile", {}).get("cameras", []):
        if isinstance(camera, dict) and camera.get("name"):
            camera_sizes[str(camera["name"])] = (int(camera["width"]), int(camera["height"]))
    rgb_index = _write_images(
        recording,
        out,
        camera_names,
        rows,
        frame_map,
        summary["phase_observations"],
        camera_sizes,
    )
    for index, row in enumerate(rows):
        row["rgb"] = rgb_index.get(index, {})
    campaign = yaml.safe_load(args.campaign_config.read_text(encoding="utf-8"))
    backend_events = _parse_log_events(
        list(args.backend_log) or sorted(episode.rglob("backend*.log*"))
    )
    backend_events = [
        event for event in backend_events if event.get("clock_epoch") == context["clock_epoch"]
    ]
    quality = quality_summary(
        backend_events,
        summary["phase_observations"],
        times,
        dict(campaign.get("quality") or {}),
        action_source=str(recipe.get("action_source", "")),
    )
    continuous = bool(rows) and all(
        right["observation_sim_time_ns"] - left["observation_sim_time_ns"] == 50_000_000
        for left, right in zip(rows, rows[1:], strict=False)
    )
    complete_rgb = bool(rows) and all(set(row["rgb"]) == set(camera_names) for row in rows)
    quality["visibility_qualified"] = quality["qualified"]
    quality["trajectory_qualified"] = continuous and complete_rgb
    quality["qualified"] = quality["qualified"] and quality["trajectory_qualified"]
    if not quality["trajectory_qualified"]:
        quality["failure_reasons"].append(
            "training labels or RGB do not form a continuous 20 Hz segment"
        )
    _json(
        out / "labels.json",
        {
            "schema_version": 1,
            "episode_id": manifest["episode_id"],
            "robot_profile_hash": profile.digest,
            "seed": configuration["reset_seed"],
            "summary": summary,
            "labels": rows,
        },
    )
    _json(
        out / "raw_predictions.json",
        {
            "schema_version": 1,
            "predictions": [
                {
                    "chunk_seq": item.chunk_seq,
                    "command_id": item.command_id,
                    "source_observation_id": item.source_observation_id,
                    "source_observation_time_ns": item.source_observation_time_ns,
                    "times_from_observation_ns": list(item.times_from_observation_ns),
                    "positions": [list(point) for point in item.positions],
                }
                for item in raw
            ],
        },
    )
    _json(
        out / "frame_index.json",
        {
            "schema_version": 1,
            "camera_order": list(camera_names),
            "labels": {str(index): value for index, value in sorted(rgb_index.items())},
        },
    )
    _json(out / "quality.json", quality)
    _json(
        out / "summary.json",
        {
            "schema_version": 1,
            "episode_id": manifest.get("episode_id"),
            "action_source": recipe.get("action_source"),
            "task": manifest.get("task"),
            "runtime": manifest.get("runtime"),
            "recording": manifest.get("recording"),
            "labels": summary,
            "quality_qualified": quality["qualified"],
            "raw_predictions": len(raw),
        },
    )
    print(json.dumps({"labels": summary["labels"], "quality_qualified": quality["qualified"]}))
    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
