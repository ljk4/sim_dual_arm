"""Small synthetic ACT model package used only by component tests.

The fixture builds a real ``model.safetensors`` with a tiny ACT policy, the
normalizer/unnormalizer tensors and the JSON metadata needed for inference.  It
is explicitly synthetic and can never be used by the formal service config.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import torch
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import make_default_pre_post_processors
from safetensors.torch import load_file, save_file
from scenario_fixtures import MARVIN_SHARED_PROFILES

ROOT = Path(__file__).resolve().parents[3]
PROFILE = (
    ROOT / "shared/scenarios/marvin/mujoco/stack/profiles/robot/profile.json"
)
OBSERVATION = MARVIN_SHARED_PROFILES / "observations/marvin_rgb_joint.json"
ACTION = MARVIN_SHARED_PROFILES / "actions/marvin_joint_position.json"
CAMERAS = ("cam_high", "cam_left_wrist", "cam_right_wrist")


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


def _act_config_values() -> dict[str, Any]:
    return {
        "chunk_size": 60,
        "n_action_steps": 60,
        "vision_backbone": "resnet18",
        "dim_model": 32,
        "n_heads": 2,
        "dim_feedforward": 64,
        "n_encoder_layers": 1,
        "n_decoder_layers": 1,
        "use_vae": True,
        "latent_dim": 2,
        "n_vae_encoder_layers": 1,
        "dropout": 0.0,
        "kl_weight": 10.0,
    }


def make_act_model(root: Path, *, synthetic: bool = True) -> Path:
    """Create a real, loadable model package."""
    root.mkdir(parents=True, exist_ok=False)
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
        pretrained_backbone_weights=None,
        **_act_config_values(),
    )
    policy = ACTPolicy(config)

    full_stats = _stats()
    pre, post = make_default_pre_post_processors(config, full_stats, normalizer_device="cpu")
    processors = root / ".processors"
    processors.mkdir()
    pre.save_pretrained(processors, config_filename="policy_preprocessor.json")
    post.save_pretrained(processors, config_filename="policy_postprocessor.json")
    pre_file = next(
        processors.glob("policy_preprocessor_step_*_normalizer_processor.safetensors")
    )
    post_file = next(
        processors.glob("policy_postprocessor_step_*_unnormalizer_processor.safetensors")
    )
    pre_tensors = load_file(pre_file)
    post_tensors = load_file(post_file)
    tensors = {
        **policy.state_dict(),
        **{f"_preprocessor.{name}": value for name, value in pre_tensors.items()},
        **{f"_postprocessor.{name}": value for name, value in post_tensors.items()},
    }
    observation = {
        "state_dim": 16,
        "state_names": [],
        "state_units": [],
        "cameras": list(CAMERAS),
        "image_height": 240,
        "image_width": 320,
        "channels": 3,
    }
    action = {
        "dim": 16,
        "names": [],
        "units": [],
        "chunk_size": 60,
        "action_lead_ns": 50_000_000,
        "fps": 20,
    }
    metadata = {
        "policy": "act",
        "step": "10",
        "act_config_json": json.dumps(_act_config_values(), sort_keys=True),
        "preprocessor_config_json": (processors / "policy_preprocessor.json").read_text(),
        "postprocessor_config_json": (processors / "policy_postprocessor.json").read_text(),
        "observation_schema_json": json.dumps(observation, sort_keys=True),
        "action_schema_json": json.dumps(action, sort_keys=True),
        "synthetic": str(bool(synthetic)).lower(),
        "config_sha256": "0" * 64,
    }
    save_file(tensors, root / "model.safetensors", metadata=metadata)
    shutil.rmtree(processors)
    (root / "training_args.yaml").write_text(
        "model_name: act\nmode: full\nsteps_completed: 10\n", encoding="utf-8"
    )
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
