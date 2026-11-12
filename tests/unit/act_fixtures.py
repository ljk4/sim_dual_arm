"""Small synthetic ACT bundle used only by component tests.

The fixture builds a real 035-shaped safetensors bundle with a tiny ACT policy.
It is explicitly synthetic and can never be used by the formal service config.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import make_default_pre_post_processors
from safetensors.torch import save_file

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "shared/scenarios/panda_stack/profiles/robots/panda_stack_learning_v1/profile.json"
OBSERVATION = ROOT / "shared/scenarios/panda_stack/profiles/observations/panda_rgb_joint_v1.json"
ACTION = ROOT / "shared/scenarios/panda_stack/profiles/actions/panda_joint_position_v1.json"
CAMERAS = ("global", "left_wrist", "right_wrist")
LEARNING_REVISION = "7e241bd630a3719a56157a497ce5d08f244784f1"
ARCHIVE_SHA = "710397ad753090743c1aadf04b2bd40584fbca8448b1409131138172913c8a0c"


def _canonical(value: object) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _tree(root: Path) -> dict[str, dict[str, Any]]:
    return {
        path.relative_to(root).as_posix(): {
            "sha256": _sha(path),
            "bytes": path.stat().st_size,
        }
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _stats() -> dict[str, Any]:
    stats: dict[str, Any] = {}
    for key, dim in (("observation.state", 16), ("action", 16)):
        stats[key] = {
            "mean": torch.zeros(dim),
            "std": torch.ones(dim),
            "min": torch.full((dim,), -1.0),
            "max": torch.full((dim,), 1.0),
        }
    for camera in CAMERAS:
        stats[f"observation.images.{camera}"] = {
            "mean": torch.zeros(3, 1, 1),
            "std": torch.ones(3, 1, 1),
            "min": torch.zeros(3, 1, 1),
            "max": torch.ones(3, 1, 1),
        }
    return stats


def make_act_bundle(root: Path, *, synthetic: bool = True) -> Path:
    """Create a real, verifiable bundle; callers must set require_formal=False."""
    root.mkdir(parents=True, exist_ok=False)
    profile = json.loads(PROFILE.read_text())
    observation_schema = json.loads(OBSERVATION.read_text())
    action_schema = json.loads(ACTION.read_text())
    bundle_profile = dict(profile)
    _write(root / "metadata/profile.json", bundle_profile)
    profile_semantic = _canonical(bundle_profile)
    profile_sha = _sha(root / "metadata/profile.json")

    inputs = {
        "observation.state": PolicyFeature(FeatureType.STATE, (16,)),
        **{
            f"observation.images.{camera}": PolicyFeature(FeatureType.VISUAL, (3, 240, 320))
            for camera in CAMERAS
        },
    }
    config = ACTConfig(
        device="cpu",
        input_features=inputs,
        output_features={"action": PolicyFeature(FeatureType.ACTION, (16,))},
        chunk_size=60,
        n_action_steps=60,
        vision_backbone="resnet18",
        pretrained_backbone_weights=None,
        dim_model=32,
        n_heads=2,
        dim_feedforward=64,
        n_encoder_layers=1,
        n_decoder_layers=1,
        use_vae=True,
        latent_dim=2,
        n_vae_encoder_layers=1,
        dropout=0.0,
        kl_weight=10.0,
    )
    policy = ACTPolicy(config)
    save_file(policy.state_dict(), root / "model.safetensors")
    config._save_pretrained(root)
    config_sha = _sha(root / "config.json")

    full_stats = _stats()
    pre, post = make_default_pre_post_processors(config, full_stats, normalizer_device="cpu")
    processors = root / "processors"
    processors.mkdir()
    pre.save_pretrained(processors, config_filename="policy_preprocessor.json")
    post.save_pretrained(processors, config_filename="policy_postprocessor.json")
    summary = {
        "source": "training_split_only",
        "features": {
            "observation.state": {
                "mean": [0.0] * 16,
                "std": [1.0] * 16,
                "min": [-1.0] * 16,
                "max": [1.0] * 16,
            },
            "action": {
                "mean": [0.0] * 16,
                "std": [1.0] * 16,
                "min": [-1.0] * 16,
                "max": [1.0] * 16,
            },
        },
    }
    _write(processors / "train_stats_summary.json", summary)
    _write(root / "metadata/train_stats_summary.json", summary)
    (root / "metadata/training_config.yaml").write_text("schema_version: 1\n", encoding="utf-8")
    training_config_sha = _sha(root / "metadata/training_config.yaml")
    (root / "metadata/checkpoint.json").write_text(
        json.dumps({"mode": "full", "step": 10, "config_sha256": training_config_sha}) + "\n",
        encoding="utf-8",
    )

    observation = {
        "state_dim": 16,
        "state_names": [joint["name"] for joint in profile["joints"]],
        "state_units": [joint["unit"] for joint in profile["joints"]],
        "cameras": list(CAMERAS),
        "image_height": 240,
        "image_width": 320,
        "channels": 3,
    }
    action = {
        "dim": 16,
        "names": [joint["name"] for joint in profile["joints"]],
        "units": [joint["unit"] for joint in profile["joints"]],
        "chunk_size": 60,
        "action_lead_ns": 50_000_000,
        "fps": 20,
    }
    _write(root / "metadata/schema.json", {"observation": observation, "action": action})

    validation = [9, 19, 29, 39, 49]
    training = [index for index in range(50) if index not in validation]

    def facts(episode_id: str) -> dict[str, Any]:
        payload = {
            "episode_id": episode_id,
            "task": {"outcome": "succeeded", "reason": "TASK_SUCCEEDED"},
            "runtime": {"outcome": "stopped", "reason": "TASK_SUCCEEDED"},
            "recording": {"complete": True, "failure_reason": ""},
            "quality_qualified": True,
        }
        return {**payload, "facts_sha256": _canonical(payload)}

    def split(indices: list[int]) -> dict[str, Any]:
        root_name = "train" if indices is training else "validation"
        return {
            "root": root_name,
            "episode_count": len(indices),
            "stats_source": f"{root_name}/meta/stats.json",
            "episodes": [
                {
                    "global_success_index": index,
                    "source_facts": facts(f"episode-{index:03d}"),
                }
                for index in indices
            ],
            "frame_count": len(indices),
            "files": {},
        }

    dataset = {
        "schema_version": 1,
        "status": "complete",
        "synthetic": bool(synthetic),
        "profile": {
            "semantic_sha256": profile_semantic,
            "joint_names": observation["state_names"],
            "camera_names": list(CAMERAS),
        },
        "lerobot": {
            "source_revision": LEARNING_REVISION,
            "source_archive_sha256": ARCHIVE_SHA,
            "chunk_size": 60,
            "action_lead_ns": 50_000_000,
            "fps": 20,
        },
        "splits": {"training": split(training), "validation": split(validation)},
        "processors": {"training_stats_only": True},
    }
    _write(root / "metadata/dataset_manifest.json", dataset)

    files = _tree(root)
    processor_digest = hashlib.sha256()
    for name, record in sorted(
        (name.removeprefix("processors/"), record)
        for name, record in files.items()
        if name.startswith("processors/")
    ):
        processor_digest.update(name.encode())
        processor_digest.update(record["sha256"].encode())
    manifest: dict[str, Any] = {
        "format": "dual-arm-act-bundle-v1",
        "created_wall_time_ns": 1,
        "policy": "act",
        "synthetic": bool(synthetic),
        "chunk_size": 60,
        "state_dim": 16,
        "observation": observation,
        "action": action,
        "checkpoint": {
            "format": "dual-arm-act-checkpoint-v1",
            "step": 10,
            "best_val_l1": 0.1,
            "reason": "completed",
            "budget_run_id": "test",
            "budget_segment_id": "test",
            "budget_instance_id": "test",
            "config_sha256": training_config_sha,
            "mode": "full",
            "seed": 20261102,
        },
        "config": {"path": "config.json", "sha256": config_sha},
        "dataset": {
            "manifest": "metadata/dataset_manifest.json",
            "manifest_sha256": files["metadata/dataset_manifest.json"]["sha256"],
            "train_episodes": 45,
            "validation_episodes": 5,
            "training_global_indices": training,
            "validation_global_indices": validation,
            "synthetic": bool(synthetic),
            "seed": 20261102,
            "split": "45train/5validation",
            "stats_source": "processors/train_stats_summary.json",
        },
        "profile": {
            "path": "metadata/profile.json",
            "sha256": profile_sha,
            "semantic_sha256": profile_semantic,
        },
        "processors": {
            "root": "processors",
            "digest": processor_digest.hexdigest(),
            "files": {
                name: record for name, record in files.items() if name.startswith("processors/")
            },
        },
        "source": {
            "lerobot_revision": LEARNING_REVISION,
            "lerobot_archive_sha256": ARCHIVE_SHA,
            "training_config_sha256": training_config_sha,
        },
        "model": {"path": "model.safetensors", "sha256": files["model.safetensors"]["sha256"]},
        "state_dict_sha256": files["model.safetensors"]["sha256"],
        "files": {name: record for name, record in files.items() if name != "manifest.json"},
        "deployment": {
            "format": "safetensors_only",
            "api": "predict_action_chunk",
            "fresh_chunk_each_call": True,
            "select_action_queue": False,
            "crop": None,
            "privileged_inputs": [],
        },
    }
    manifest["bundle_digest"] = _canonical(manifest)
    _write(root / "manifest.json", manifest)
    _ = observation_schema, action_schema
    return root


def observation_payloads() -> dict[str, np.ndarray]:
    images = {}
    for index, camera in enumerate(CAMERAS):
        array = np.zeros((240, 320, 3), dtype=np.uint8)
        array[:, :, 0] = 10 + index * 40
        array[:, :, 1] = 20 + index * 10
        array[:, :, 2] = 30
        images[camera] = array
    return images


def reseal_bundle(root: Path, mutate=None) -> dict[str, Any]:
    """Recompute all manifest hashes after a test mutation."""
    if mutate is not None:
        mutate(root)
    manifest = json.loads((root / "manifest.json").read_text())
    files = _tree(root)
    manifest["files"] = {name: record for name, record in files.items() if name != "manifest.json"}
    digest = hashlib.sha256()
    for name, record in sorted(
        (name.removeprefix("processors/"), record)
        for name, record in files.items()
        if name.startswith("processors/")
    ):
        digest.update(name.encode())
        digest.update(record["sha256"].encode())
    manifest["processors"]["digest"] = digest.hexdigest()
    manifest["processors"]["files"] = {
        name: record for name, record in files.items() if name.startswith("processors/")
    }
    manifest["dataset"]["manifest_sha256"] = files["metadata/dataset_manifest.json"]["sha256"]
    manifest["profile"]["sha256"] = files["metadata/profile.json"]["sha256"]
    manifest["config"]["sha256"] = files["config.json"]["sha256"]
    manifest["checkpoint"]["config_sha256"] = files["metadata/training_config.yaml"]["sha256"]
    manifest["model"]["sha256"] = files["model.safetensors"]["sha256"]
    manifest["state_dict_sha256"] = files["model.safetensors"]["sha256"]
    manifest["source"]["training_config_sha256"] = files[
        "metadata/training_config.yaml"
    ]["sha256"]
    manifest.pop("bundle_digest", None)
    manifest["bundle_digest"] = _canonical(manifest)
    _write(root / "manifest.json", manifest)
    return manifest
