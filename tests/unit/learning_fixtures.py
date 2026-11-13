"""Small 034-format export fixtures for DSH-035 component tests.

The generated files are explicitly synthetic.  They reproduce the accepted
flat export shape (labels with camera order and source facts, raw chunks and
quality) without reading or changing the live campaign.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from dual_arm_learning.config import DatasetConfig
from PIL import Image

LEAD_NS = 50_000_000


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def profile_hash(config: DatasetConfig) -> str:
    document = json.loads(config.profile_path.read_text(encoding="utf-8"))
    return hashlib.sha256(
        json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def make_export(
    *,
    attempt_dir: Path,
    config: DatasetConfig,
    episode_id: str,
    seed: int,
    frames: int,
    value: float,
    start_ns: int,
) -> dict[str, Any]:
    export = attempt_dir / "export"
    export.mkdir(parents=True)
    rows: list[dict[str, Any]] = []
    for index in range(frames):
        observation_ns = start_ns + index * LEAD_NS
        rgb: dict[str, dict[str, Any]] = {}
        for camera_index, camera in enumerate(config.camera_names):
            array = np.zeros((config.camera_height, config.camera_width, 3), dtype=np.uint8)
            array[:, :, 0] = 10 + camera_index * 40 + index
            array[:, :, 1] = 20 + camera_index * 10
            array[:, :, 2] = 30 + index * 2
            png = export / "rgb" / f"{index:04d}" / f"{camera}.png"
            png.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(array, "RGB").save(png)
            entry = {
                "path": png.relative_to(export).as_posix(),
                "frame_seq": index + 1,
                "observation_id": index + 1,
                "observation_sim_time_ns": observation_ns,
                "png_sha256": sha256_bytes(png.read_bytes()),
                "rgb_sha256": sha256_bytes(array.tobytes()),
            }
            rgb[camera] = entry
        rows.append(
            {
                "observation_id": index + 1,
                "state_seq": index + 1,
                "capture_source": "observation_index",
                "observation_sim_time_ns": observation_ns,
                "label_sim_time_ns": observation_ns + LEAD_NS,
                "observation_period_ns": LEAD_NS,
                "chunk_seq": 1,
                "command_id": "command",
                "started_sim_time_ns": start_ns,
                "block_end_sim_time_ns": start_ns + (frames + 5) * LEAD_NS,
                "applied_positions": [value] * config.state_dim,
                "sampled_positions": [value] * config.state_dim,
                "applied_velocities": [0.0] * config.state_dim,
                "measured_positions": [value] * config.state_dim,
                "measured_velocities": [0.0] * config.state_dim,
                "frame_seqs": [index + 1] * len(config.camera_names),
                "raw_chunk_seq": 1,
                "raw_source_observation_id": 1,
                "raw_source_observation_time_ns": start_ns,
                "rgb": rgb,
            }
        )
    write_json(
        export / "labels.json",
        {
            "schema_version": 1,
            "episode_id": episode_id,
            "robot_profile_hash": profile_hash(config),
            "seed": seed,
            "camera_order": list(config.camera_names),
            "action_source": "teacher",
            "task": {"outcome": "succeeded", "reason": "TASK_SUCCEEDED"},
            "runtime": {"outcome": "stopped", "reason": "TASK_SUCCEEDED"},
            "recording": {"complete": True, "failure_reason": ""},
            "summary": {
                "labels": frames,
                "observations": frames,
                "rejected": {},
                "lead_ns": LEAD_NS,
                "expected_period_ns": LEAD_NS,
                "max_labels": 700,
            },
            "labels": rows,
        },
    )
    write_json(
        export / "raw_predictions.json",
        {
            "schema_version": 1,
            "predictions": [
                {
                    "chunk_seq": 1,
                    "command_id": "command",
                    "source_observation_id": 1,
                    "source_observation_time_ns": start_ns,
                    "times_from_observation_ns": [(offset + 1) * LEAD_NS for offset in range(60)],
                    "positions": [[value] * config.state_dim for _ in range(60)],
                }
            ],
        },
    )
    write_json(
        export / "quality.json",
        {
            "schema_version": 1,
            "qualified": True,
            "visibility_qualified": True,
            "trajectory_qualified": True,
            "failure_reasons": [],
        },
    )
    return {"attempt_id": episode_id, "seed": seed, "attempt_dir": str(attempt_dir)}


def make_select(root: Path, config: DatasetConfig, *, first_frames: int = 61) -> Path:
    training: list[dict[str, Any]] = []
    validation: list[dict[str, Any]] = []
    for index in range(config.expected_formal_episodes):
        episode_id = f"synthetic-{index:03d}"
        frames = first_frames if index == 0 else 2
        value = 0.01 if index not in config.validation_positions else 1.01
        entry = make_export(
            attempt_dir=root / "attempts" / episode_id,
            config=config,
            episode_id=episode_id,
            seed=10_000 + index,
            frames=frames,
            value=value,
            start_ns=1_000_000_000 + index * 10_000_000_000,
        )
        (validation if index in config.validation_positions else training).append(entry)
    select = root / "select.json"
    write_json(
        select,
        {
            "status": "SELECTED",
            "qualified": config.expected_formal_episodes,
            "training": training,
            "validation": validation,
            "rule": "first 50 qualified formal attempts; 10th/20th/30th/40th/50th validate",
        },
    )
    return select
