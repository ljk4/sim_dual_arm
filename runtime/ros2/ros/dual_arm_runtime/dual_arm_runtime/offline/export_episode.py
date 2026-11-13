#!/usr/bin/env python3
"""Auditable, bounded export of one recorded episode.

Outputs only JSON, PNG and GIF.  No pickle, framework database or ROS import is
needed by later learning conversion.  Labels are the actually applied follower
reference at observation + 50 ms; raw 60-point predictions are stored apart.

Log parsing and quality rules live in :mod:`export_quality`; RGB extraction and
preview GIFs live in :mod:`export_media`.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any

import yaml

_PROJECT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[6])

from robot_contracts.models import RobotProfile  # noqa: E402
from robot_contracts.overrides import OverrideError, apply_overrides  # noqa: E402

from dual_arm_runtime.offline.audit.label_export import (  # noqa: E402
    AppliedBlock,
    BrakeInterval,
    ObservationFact,
    RawPrediction,
    build_labels,
)
from dual_arm_runtime.offline.campaign_ledger import CampaignError  # noqa: E402
from dual_arm_runtime.offline.export_media import write_images  # noqa: E402
from dual_arm_runtime.offline.export_quality import (  # noqa: E402
    parse_log_events,
    phase_observations,
    quality_summary,
)


def _json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _stamp_ns(value: object) -> int:
    return int(getattr(value, "sec", 0)) * 1_000_000_000 + int(getattr(value, "nanosec", 0))


def _consume_index(message: object, metadata: dict[str, Any]) -> None:
    metadata["indices"][int(message.state_seq)] = message
    if not metadata["camera_names"]:
        metadata["camera_names"] = tuple(message.camera_names)


def _consume_state(message: object, metadata: dict[str, Any]) -> None:
    metadata["states"][int(message.state_seq)] = message


def _consume_frame(topic: str, message: object, metadata: dict[str, Any]) -> None:
    from robot_contracts.models import ContractError

    camera = topic.split("/")[-2]
    image = message.image
    if image.encoding != "rgb8" or image.step != int(image.width) * 3:
        raise ContractError(
            "LABEL_EXPORT_RGB",
            f"{camera} frame {message.frame_seq} is not packed rgb8",
        )
    key = (camera, int(message.frame_seq))
    frames = metadata["frames"]
    if key not in frames:
        frames[key] = message
        return
    previous = frames[key]
    if _stamp_ns(previous.image.header.stamp) != _stamp_ns(image.header.stamp) or bytes(
        previous.image.data
    ) != bytes(image.data):
        raise ContractError(
            "LABEL_EXPORT_CAPTURE",
            f"duplicate capture has different pixels: {camera}/{key[1]}",
        )


def _consume_command(message: object, metadata: dict[str, Any]) -> None:
    from robot_contracts.stream_request import StreamRequest
    from robot_ros_support.stream_conversions import stream_command_from_message

    request = stream_command_from_message(message)
    if not isinstance(request, StreamRequest) or request.anchor is None:
        return
    key = int(request.trajectory.observation_id)
    targets = request.trajectory.targets
    metadata["requests"].setdefault(
        key,
        (
            request.trajectory.command_id,
            request.anchor.predecessor_seq + 1,
            tuple(int(target.time_ns) for target in targets),
            tuple(tuple(float(value) for value in target.positions) for target in targets),
        ),
    )


def _consume_reference(message: object, metadata: dict[str, Any]) -> None:
    from robot_ros_support.follower_conversions import follower_reference_from_message

    record = follower_reference_from_message(message)
    metadata["references"][(record.command_id, record.chunk_seq)] = record


def _consume_event(message: object, metadata: dict[str, Any]) -> None:
    from robot_ros_support.stream_conversions import control_event_from_message

    event = control_event_from_message(message)
    if not event.is_chunk and event.phase == "STOPPING":
        # This is the controller's switch from the follower to BRAKE.
        # Overall started_time is the stream start, and canceled_time
        # is when braking finishes; neither is the switch instant.
        metadata["brakes"].append(BrakeInterval(int(event.sim_time_ns), (1 << 63) - 1))
    if event.is_chunk and event.chunk_seq is not None:
        key = (event.command_id, int(event.chunk_seq))
        phases = metadata["chunk_events"].setdefault(key, {})
        if event.phase in {"STARTED", "RETIRED", "CANCELED", "REJECTED"}:
            phases.setdefault(event.phase, int(event.sim_time_ns))


def _consume_feedback(message: object, metadata: dict[str, Any]) -> None:
    from robot_ros_support.stream_conversions import execution_feedback_from_message

    execution = execution_feedback_from_message(message)
    if execution.reference_kind != "BRAKE":
        return
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
        metadata["brakes"].append(BrakeInterval(start, end))


def _consume_message(
    topic: str,
    message: object,
    root: str,
    metadata: dict[str, Any],
) -> None:
    if topic == "/runtime/observation_index":
        _consume_index(message, metadata)
    elif topic == f"{root}/state":
        _consume_state(message, metadata)
    elif topic.startswith(f"{root}/cameras/") and topic.endswith("/frame"):
        _consume_frame(topic, message, metadata)
    elif topic == "/runtime/commands":
        _consume_command(message, metadata)
    elif topic == f"{root}/follower_references":
        _consume_reference(message, metadata)
    elif topic == f"{root}/control_events":
        _consume_event(message, metadata)
    elif topic == f"{root}/execution_feedback":
        _consume_feedback(message, metadata)


def _read_metadata(recording: Path, root: str) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    from dual_arm_runtime.offline.follower_definition import (
        require_current_follower_reference_definition,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(recording), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    try:
        required = {f"{root}/state", f"{root}/follower_references", f"{root}/control_events",
                    f"{root}/execution_feedback", "/runtime/commands",
                    "/runtime/observation_index"}
        topics = reader.get_all_topics_and_types()
        topic_types = {item.name: item.type for item in topics}
        require_current_follower_reference_definition(reader, topic_types, rosbag2_py)
        types = {item.name: get_message(item.type) for item in topics
                 if item.name in required or (item.name.startswith(root + "/cameras/")
                                              and item.name.endswith("/frame"))}
        metadata: dict[str, Any] = {
            "states": {},
            "frames": {},
            "indices": {},
            "requests": {},
            "references": {},
            "chunk_events": {},
            "brakes": [],
            "camera_names": (),
        }
        while reader.has_next():
            topic, payload, _ = reader.read_next()
            if topic not in types:
                continue
            message = deserialize_message(payload, types[topic])
            _consume_message(topic, message, root, metadata)
        if not metadata["camera_names"]:
            from robot_contracts.models import ContractError

            raise ContractError("LABEL_EXPORT_CAPTURE", "no camera names in observation index")
        return metadata
    finally:
        reader.close()


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


def _raw_predictions(
    metadata: dict[str, Any], index_time: dict[int, int]
) -> dict[int, RawPrediction]:
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
    return raw_by_chunk


def _applied_blocks(metadata: dict[str, Any]) -> list[AppliedBlock]:
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
    return blocks


def _build(
    metadata: dict[str, Any],
    camera_names: tuple[str, ...],
    runtime_events: list[dict[str, Any]],
    max_labels: int,
    profile: RobotProfile,
    phases: list[str] | None = None,
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
    raw_by_chunk = _raw_predictions(metadata, index_time)
    blocks = _applied_blocks(metadata)
    rows, summary = build_labels(
        observations,
        blocks,
        metadata["brakes"],
        raw_by_chunk,
        max_labels=max_labels,
        profile=profile,
    )
    summary["phase_observations"] = phase_observations(
        runtime_events,
        {item.observation_id: item.sim_time_ns for item in observations},
        phases,
    )
    return rows, list(raw_by_chunk.values()), summary, times, frame_map


def _load_episode(
    episode: Path,
) -> tuple[dict[str, Any], dict[str, Any], RobotProfile, dict, str, dict]:
    from robot_contracts.episodes import manifest_schema_path
    from robot_contracts.profiles import profile_from_document, validate_document

    manifest = json.loads((episode / "manifest.json").read_text(encoding="utf-8"))
    validate_document(
        manifest, manifest_schema_path(_PROJECT / "shared/common/interfaces/schemas", 1)
    )
    if not manifest["recording"]["complete"]:
        raise ValueError(manifest["recording"]["failure_reason"])
    configuration = manifest.get("configuration", {})
    profile = profile_from_document(
        configuration["profile"], _PROJECT / "shared/common/interfaces/schemas"
    )
    if profile.digest != manifest["robot_profile_hash"]:
        raise ValueError("recorded profile snapshot does not match the manifest")
    recipe = configuration.get("recipe", {})
    root = str(recipe.get("robot_namespace", "/robot")).rstrip("/")
    return manifest, configuration, profile, recipe, root, configuration["run_context"]


def _runtime_events(
    args: argparse.Namespace, episode: Path, context: dict[str, Any]
) -> list[dict[str, Any]]:
    return [
        event
        for event in parse_log_events(list(args.runtime_log) or [episode / "log"])
        if event.get("episode_id") == context["episode_id"]
        and event.get("clock_epoch") == context["clock_epoch"]
    ]


def _camera_sizes(configuration: dict[str, Any]) -> dict[str, tuple[int, int]]:
    result: dict[str, tuple[int, int]] = {}
    for camera in configuration.get("profile", {}).get("cameras", []):
        if isinstance(camera, dict) and camera.get("name"):
            result[str(camera["name"])] = (int(camera["width"]), int(camera["height"]))
    return result


def _quality(
    args: argparse.Namespace,
    episode: Path,
    context: dict[str, Any],
    summary: dict[str, Any],
    times: dict[int, int],
    recipe: dict[str, Any],
    campaign: dict[str, Any],
) -> dict[str, Any]:
    backend_events = parse_log_events(
        list(args.backend_log) or sorted(episode.rglob("backend*.log*"))
    )
    backend_events = [
        event for event in backend_events if event.get("clock_epoch") == context["clock_epoch"]
    ]
    return quality_summary(
        backend_events,
        summary["phase_observations"],
        times,
        dict(campaign.get("quality") or {}),
        action_source=str(recipe.get("action_source", "")),
    )


def _trajectory_flags(
    rows: list[dict[str, Any]],
    camera_names: tuple[str, ...],
    quality: dict[str, Any],
) -> None:
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


def _write_documents(
    out: Path,
    manifest: dict[str, Any],
    profile: RobotProfile,
    configuration: dict[str, Any],
    recipe: dict[str, Any],
    rows: list[dict[str, Any]],
    raw: list[RawPrediction],
    camera_names: tuple[str, ...],
    summary: dict[str, Any],
    quality: dict[str, Any],
) -> None:
    _json(
        out / "labels.json",
        {
            "schema_version": 1,
            "episode_id": manifest["episode_id"],
            "robot_profile_hash": profile.digest,
            "seed": configuration["reset_seed"],
            "camera_order": list(camera_names),
            "action_source": recipe.get("action_source"),
            "task": manifest.get("task"),
            "runtime": manifest.get("runtime"),
            "recording": manifest.get("recording"),
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
    _json(out / "quality.json", quality)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--campaign-config",
        type=Path,
        default=None,
        help="campaign recipe used for quality rules (conflicts with --scenario)",
    )
    parser.add_argument("--scenario")
    parser.add_argument("--shared-root", type=Path)
    parser.add_argument("--runtime-log", type=Path, action="append", default=[])
    parser.add_argument("--backend-log", type=Path, action="append", default=[])
    parser.add_argument(
        "--max-labels",
        type=int,
        default=None,
        help="override the campaign export.max_labels bound",
    )
    parser.add_argument("--no-previews", action="store_true", help="skip preview PNGs and GIFs")
    parser.add_argument(
        "--set", dest="overrides", action="append", default=[], metavar="PATH=VALUE"
    )
    args = parser.parse_args(argv)
    from dual_arm_runtime.offline.scenario_selection import campaign_config_path

    if args.scenario is not None and args.campaign_config is not None:
        print(
            json.dumps({"error": "--scenario cannot be combined with --campaign-config"}),
            file=sys.stderr,
        )
        return 2
    if args.scenario is None and args.campaign_config is None:
        print(
            json.dumps({"error": "one of --scenario or --campaign-config is required"}),
            file=sys.stderr,
        )
        return 2
    try:
        args.campaign_config = campaign_config_path(
            campaign_config=args.campaign_config,
            scenario=args.scenario,
            shared_root=args.shared_root,
            project_root=_PROJECT,
        )
    except CampaignError as error:
        print(json.dumps({"error": str(error)}), file=sys.stderr)
        return 2
    campaign = yaml.safe_load(args.campaign_config.read_text(encoding="utf-8"))
    try:
        campaign = apply_overrides(campaign, list(args.overrides))
    except OverrideError as error:
        print(json.dumps({"error": str(error)}), file=sys.stderr)
        return 2
    export = campaign.get("export") if isinstance(campaign.get("export"), dict) else {}
    max_labels = args.max_labels
    if max_labels is None:
        max_labels = int(export.get("max_labels", 700))
    quality_rules = campaign.get("quality") if isinstance(campaign.get("quality"), dict) else {}
    phases = quality_rules.get("phases")
    phases = [str(value) for value in phases] if isinstance(phases, list) else None
    episode = args.episode_dir.resolve()
    out = args.out.resolve()
    manifest, configuration, profile, recipe, root, context = _load_episode(episode)
    metadata = _read_metadata(episode / "recording", root)
    camera_names = metadata["camera_names"]
    rows, raw, summary, times, frame_map = _build(
        metadata,
        camera_names,
        _runtime_events(args, episode, context),
        max_labels,
        profile,
        phases,
    )
    rgb_index = write_images(
        episode / "recording",
        out,
        camera_names,
        rows,
        frame_map,
        summary["phase_observations"],
        _camera_sizes(configuration),
        previews=not args.no_previews,
    )
    for index, row in enumerate(rows):
        row["rgb"] = rgb_index.get(index, {})
    quality = _quality(args, episode, context, summary, times, recipe, campaign)
    _trajectory_flags(rows, camera_names, quality)
    _write_documents(
        out,
        manifest,
        profile,
        configuration,
        recipe,
        rows,
        raw,
        camera_names,
        summary,
        quality,
    )
    print(
        json.dumps(
            {
                "labels": summary["labels"],
                "quality_qualified": quality["qualified"],
                "max_labels": max_labels,
            }
        )
    )
    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
