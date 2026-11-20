"""Self-contained ACT model package (one ``model.safetensors`` + one memo).

A trained model lives in ``models/act_<timestamp>/`` and contains exactly
``model.safetensors`` and ``training_args.yaml``.  The safetensors file carries
the policy state dict, the dataset normalizer/unnormalizer tensors and the JSON
metadata needed to rebuild the policy and processors without any bundle
directory.  ``training_args.yaml`` is a human memo and is never read here.
"""

from __future__ import annotations

import hashlib
import json
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

MODEL_FILENAME = "model.safetensors"
TRAINING_ARGS_FILENAME = "training_args.yaml"
PREPROCESSOR_FILE = "policy_preprocessor_step_3_normalizer_processor.safetensors"
POSTPROCESSOR_FILE = "policy_postprocessor_step_0_unnormalizer_processor.safetensors"
_PREPROCESSOR_PREFIX = "_preprocessor."
_POSTPROCESSOR_PREFIX = "_postprocessor."


class ModelError(RuntimeError):
    """An ACT model package failed an integrity, shape or contract check."""


@dataclass(frozen=True)
class ModelHandle:
    root: Path
    model_file: Path
    policy: Any
    preprocessor: Any
    postprocessor: Any
    act_config: Any
    metadata: dict[str, Any]
    observation_schema: dict[str, Any]
    action_schema: dict[str, Any]
    device: str

    def prediction_metadata(self) -> dict[str, Any]:
        return {
            "model": str(self.root),
            "model_sha256": _file_sha256(self.model_file),
            "step": self.metadata.get("step"),
            "config_sha256": self.metadata.get("config_sha256"),
            "synthetic": self.metadata.get("synthetic"),
        }


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve_model_path(path: str | Path) -> Path:
    """Accept either a model directory or its ``model.safetensors`` file."""
    candidate = Path(path)
    if candidate.is_dir():
        verify_model_dir(candidate)
        candidate = candidate / MODEL_FILENAME
    if not candidate.is_file():
        raise ModelError(f"{candidate}: model.safetensors is missing")
    return candidate


def verify_model_dir(root: str | Path) -> Path:
    """Check that a model directory contains only the two contract files."""
    directory = Path(root)
    if not directory.is_dir():
        raise ModelError(f"{directory}: model directory is missing")
    names = sorted(path.name for path in directory.iterdir())
    expected = sorted((MODEL_FILENAME, TRAINING_ARGS_FILENAME))
    if names != expected:
        raise ModelError(
            f"{directory}: expected exactly {expected}, found {names}"
        )
    return directory


def read_model_metadata(path: str | Path) -> tuple[Path, dict[str, Any]]:
    """Read safetensors metadata without loading any tensor."""
    from safetensors import safe_open

    model_file = resolve_model_path(path)
    try:
        with safe_open(model_file, framework="pt") as handle:
            metadata = handle.metadata() or {}
    except Exception as error:  # noqa: BLE001 - surface one model error type
        raise ModelError(f"{model_file}: cannot read model metadata: {error}") from error
    if metadata.get("policy") != "act":
        raise ModelError(f"{model_file}: model metadata must declare policy 'act'")
    return model_file, {str(key): str(value) for key, value in metadata.items()}


def _json_metadata(metadata: dict[str, Any], key: str) -> dict[str, Any]:
    value = metadata.get(key)
    if value is None:
        raise ModelError(f"model metadata has no {key}")
    try:
        document = json.loads(value)
    except json.JSONDecodeError as error:
        raise ModelError(f"model metadata {key} is not JSON: {error}") from error
    if not isinstance(document, dict):
        raise ModelError(f"model metadata {key} must be an object")
    return document


def _load_processors(root: Path) -> tuple[Any, Any]:
    from lerobot.processor import (
        PolicyProcessorPipeline,
        policy_action_to_transition,
        transition_to_policy_action,
    )

    pre = PolicyProcessorPipeline.from_pretrained(
        root,
        config_filename="policy_preprocessor.json",
        local_files_only=True,
    )
    post = PolicyProcessorPipeline.from_pretrained(
        root,
        config_filename="policy_postprocessor.json",
        local_files_only=True,
        to_transition=policy_action_to_transition,
        to_output=transition_to_policy_action,
    )
    return pre, post


def _processors_from_metadata(
    metadata: dict[str, Any], tensors: dict[str, Any]
) -> tuple[Any, Any]:
    from safetensors.torch import save_file

    pre_tensors = {
        name.removeprefix(_PREPROCESSOR_PREFIX): tensor
        for name, tensor in tensors.items()
        if name.startswith(_PREPROCESSOR_PREFIX)
    }
    post_tensors = {
        name.removeprefix(_POSTPROCESSOR_PREFIX): tensor
        for name, tensor in tensors.items()
        if name.startswith(_POSTPROCESSOR_PREFIX)
    }
    if not pre_tensors or not post_tensors:
        raise ModelError("model package has no normalizer/unnormalizer tensors")
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        (root / "policy_preprocessor.json").write_text(
            metadata["preprocessor_config_json"], encoding="utf-8"
        )
        (root / "policy_postprocessor.json").write_text(
            metadata["postprocessor_config_json"], encoding="utf-8"
        )
        save_file(pre_tensors, root / PREPROCESSOR_FILE)
        save_file(post_tensors, root / POSTPROCESSOR_FILE)
        return _load_processors(root)


def _build_act_config(metadata: dict[str, Any]) -> Any:
    from lerobot.configs import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig

    values = _json_metadata(metadata, "act_config_json")
    observation = _json_metadata(metadata, "observation_schema_json")
    action = _json_metadata(metadata, "action_schema_json")
    inputs = {
        "observation.state": PolicyFeature(
            FeatureType.STATE, (int(observation["state_dim"]),)
        )
    }
    for camera in observation["cameras"]:
        inputs[f"observation.images.{camera}"] = PolicyFeature(
            FeatureType.VISUAL,
            (3, int(observation["image_height"]), int(observation["image_width"])),
        )
    outputs = {
        "action": PolicyFeature(FeatureType.ACTION, (int(action["dim"]),))
    }
    values.pop("pretrained_backbone_weights", None)
    return ACTConfig(
        input_features=inputs,
        output_features=outputs,
        pretrained_backbone_weights=None,
        device="cpu",
        **values,
    )


def load_model(path: str | Path, *, device: str = "cpu") -> ModelHandle:
    """Rebuild the ACT policy and processors from one safetensors file."""
    from lerobot.policies.act.modeling_act import ACTPolicy
    from safetensors.torch import load_file

    model_file, metadata = read_model_metadata(path)
    act_config = _build_act_config(metadata)
    policy = ACTPolicy(act_config)
    tensors = load_file(model_file)
    state = {
        name: tensor
        for name, tensor in tensors.items()
        if not name.startswith((_PREPROCESSOR_PREFIX, _POSTPROCESSOR_PREFIX))
    }
    missing, unexpected = policy.load_state_dict(state, strict=False)
    if missing or unexpected:
        raise ModelError(
            f"model state dict does not match ACT: missing={list(missing)[:4]} "
            f"unexpected={list(unexpected)[:4]}"
        )
    policy.to(device)
    policy.eval()
    pre, post = _processors_from_metadata(metadata, tensors)
    return ModelHandle(
        root=model_file.parent,
        model_file=model_file,
        policy=policy,
        preprocessor=pre,
        postprocessor=post,
        act_config=act_config,
        metadata=metadata,
        observation_schema=_json_metadata(metadata, "observation_schema_json"),
        action_schema=_json_metadata(metadata, "action_schema_json"),
        device=device,
    )


def _observation_tensors(
    observation: dict[str, Any], handle: ModelHandle
) -> dict[str, Any]:
    import torch

    schema = handle.observation_schema
    cameras = list(schema.get("cameras", []))
    state = np.asarray(observation.get("observation.state"), dtype=np.float32)
    if state.shape != (int(schema.get("state_dim", 0)),):
        raise ModelError(f"observation.state shape {state.shape} does not match the model")
    if not np.isfinite(state).all():
        raise ModelError("observation.state contains non-finite values")
    images = observation.get("images")
    if not isinstance(images, dict) or set(images) != set(cameras):
        raise ModelError("observation.images must contain exactly the model cameras")
    sample: dict[str, Any] = {"observation.state": torch.from_numpy(state)}
    for camera in cameras:
        image = np.asarray(images[camera])
        expected = (
            int(schema.get("image_height", 0)),
            int(schema.get("image_width", 0)),
            3,
        )
        if image.shape != expected:
            raise ModelError(f"observation.images.{camera} shape {image.shape} != {expected}")
        if image.dtype == np.uint8:
            array = image.astype(np.float32) / 255.0
        elif np.issubdtype(image.dtype, np.floating):
            array = image.astype(np.float32)
            if not np.isfinite(array).all() or array.min() < 0.0 or array.max() > 1.0:
                raise ModelError(
                    f"observation.images.{camera} float pixels must be finite in [0,1]"
                )
        else:
            raise ModelError(f"observation.images.{camera} dtype must be uint8 or float")
        sample[f"observation.images.{camera}"] = (
            torch.from_numpy(array).permute(2, 0, 1).contiguous()
        )
    return sample


def _to_device(value: Any, device: str) -> Any:
    import torch

    if isinstance(value, torch.Tensor):
        return value.to(device)
    if isinstance(value, dict):
        return {key: _to_device(item, device) for key, item in value.items()}
    if isinstance(value, list):
        return [_to_device(item, device) for item in value]
    return value


def predict_action_chunk(handle: ModelHandle, observation: dict[str, Any]) -> np.ndarray:
    """Return one fresh complete (60,16) physical action chunk.

    The output is never cropped, padded by the caller, or altered per arm.
    """
    import torch

    if not isinstance(handle, ModelHandle):
        raise ModelError("handle must be a loaded ModelHandle")
    sample = _observation_tensors(observation, handle)
    with torch.no_grad():
        batch = handle.preprocessor(sample)
        batch = _to_device(batch, handle.device)
        if "action_is_pad" in batch:
            batch["action_is_pad"] = batch["action_is_pad"].bool()
        normalized = handle.policy.predict_action_chunk(batch)
        if handle.device != "cpu":
            normalized = normalized.to("cpu")
        actions = handle.postprocessor(normalized)
    if isinstance(actions, dict):
        if "action" not in actions:
            raise ModelError("postprocessor did not return an action tensor")
        actions = actions["action"]
    tensor = actions.detach().to("cpu", dtype=torch.float32)
    if tensor.dim() == 3:
        if tensor.shape[0] != 1:
            raise ModelError(f"prediction batch dimension is {tensor.shape[0]}, expected 1")
        tensor = tensor[0]
    expected = (
        int(handle.action_schema.get("chunk_size", 60)),
        int(handle.action_schema.get("dim", 16)),
    )
    if tuple(tensor.shape) != expected:
        raise ModelError(f"policy returned {tuple(tensor.shape)}, expected {expected}")
    result = tensor.numpy()
    if not np.isfinite(result).all():
        raise ModelError("policy returned non-finite actions")
    return result


def build_schema(config: Any, profile: dict[str, Any]) -> tuple[dict, dict]:
    """Build the observation/action schema stored in the model package."""
    joints = list(profile.get("joints", []))
    observation = {
        "state_dim": config.dataset.state_dim,
        "state_names": [item.get("name") for item in joints],
        "state_units": [item.get("unit") for item in joints],
        "cameras": list(config.dataset.camera_names),
        "image_height": config.dataset.camera_height,
        "image_width": config.dataset.camera_width,
        "channels": 3,
    }
    action = {
        "dim": config.dataset.state_dim,
        "names": [item.get("name") for item in joints],
        "units": [item.get("unit") for item in joints],
        "chunk_size": config.dataset.chunk_size,
        "action_lead_ns": config.dataset.action_lead_ns,
        "fps": config.dataset.fps,
    }
    return observation, action


def _processor_tensors(dataset_root: Path) -> tuple[dict[str, Any], dict[str, str]]:
    from safetensors.torch import load_file

    processors = dataset_root / "processors"
    pre_file = processors / PREPROCESSOR_FILE
    post_file = processors / POSTPROCESSOR_FILE
    if not pre_file.is_file() or not post_file.is_file():
        raise ModelError(f"{processors}: normalizer/unnormalizer tensors are missing")
    pre = load_file(pre_file)
    post = load_file(post_file)
    tensors = {
        f"{_PREPROCESSOR_PREFIX}{name}": tensor for name, tensor in pre.items()
    }
    tensors.update(
        {f"{_POSTPROCESSOR_PREFIX}{name}": tensor for name, tensor in post.items()}
    )
    configs = {
        "preprocessor_config_json": (processors / "policy_preprocessor.json").read_text(
            encoding="utf-8"
        ),
        "postprocessor_config_json": (processors / "policy_postprocessor.json").read_text(
            encoding="utf-8"
        ),
    }
    return tensors, configs


def save_model_package(
    model_dir: Path,
    *,
    policy: Any,
    act_config: Any,
    dataset_root: Path,
    step: int,
    synthetic: bool,
    config_sha256: str,
    observation_schema: dict[str, Any],
    action_schema: dict[str, Any],
    training_args: dict[str, Any],
) -> Path:
    """Write ``model.safetensors`` and the human-readable ``training_args.yaml``."""
    import torch
    from safetensors.torch import save_file

    model_dir.mkdir(parents=True, exist_ok=True)
    processor_tensors, processor_configs = _processor_tensors(dataset_root)
    policy_tensors = {
        name: tensor.detach().to("cpu").contiguous()
        for name, tensor in policy.state_dict().items()
    }
    for name, tensor in policy_tensors.items():
        if not torch.isfinite(tensor).all():
            raise ModelError(f"trained tensor {name} contains non-finite values")
    tensors = {**policy_tensors, **processor_tensors}
    metadata = {
        "policy": "act",
        "step": str(step),
        "act_config_json": json.dumps(_act_config_document(act_config), sort_keys=True),
        "preprocessor_config_json": processor_configs["preprocessor_config_json"],
        "postprocessor_config_json": processor_configs["postprocessor_config_json"],
        "observation_schema_json": json.dumps(observation_schema, sort_keys=True),
        "action_schema_json": json.dumps(action_schema, sort_keys=True),
        "synthetic": str(bool(synthetic)).lower(),
        "config_sha256": config_sha256,
    }
    temporary = model_dir / (MODEL_FILENAME + ".tmp")
    save_file(tensors, temporary, metadata=metadata)
    temporary.replace(model_dir / MODEL_FILENAME)
    _write_training_args(model_dir / TRAINING_ARGS_FILENAME, training_args)
    return model_dir


def _act_config_document(act_config: Any) -> dict[str, Any]:
    return {
        "chunk_size": act_config.chunk_size,
        "n_action_steps": act_config.n_action_steps,
        "vision_backbone": act_config.vision_backbone,
        "dim_model": act_config.dim_model,
        "n_heads": act_config.n_heads,
        "dim_feedforward": act_config.dim_feedforward,
        "n_encoder_layers": act_config.n_encoder_layers,
        "n_decoder_layers": act_config.n_decoder_layers,
        "use_vae": act_config.use_vae,
        "latent_dim": act_config.latent_dim,
        "n_vae_encoder_layers": act_config.n_vae_encoder_layers,
        "dropout": act_config.dropout,
        "kl_weight": act_config.kl_weight,
    }


def _write_training_args(path: Path, document: dict[str, Any]) -> None:
    import yaml

    path.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )


def deployment_summary(config: Any) -> dict[str, Any]:
    """Return JSON-safe deployment provenance from one loaded policy config."""
    summary: dict[str, Any] = {
        "policy": "act",
        "chunk_size": config.chunk_size,
        "state_dim": len(config.joint_names),
        "action_period_ns": config.action_period_ns,
        "action_lead_ns": 50_000_000,
        "fps": 20,
        "cameras": list(config.camera_names),
        "camera_width": config.camera_width,
        "camera_height": config.camera_height,
    }
    if config.model_path is None:
        return summary
    model_file, metadata = read_model_metadata(config.model_path)
    summary.update(
        model_sha256=_file_sha256(model_file),
        model_bundle_digest=_file_sha256(model_file),
        config_sha256=metadata.get("config_sha256", ""),
        checkpoint_step=int(metadata.get("step", 0)),
        synthetic=metadata.get("synthetic") == "true",
    )
    return summary


__all__ = [
    "MODEL_FILENAME",
    "ModelError",
    "ModelHandle",
    "TRAINING_ARGS_FILENAME",
    "build_schema",
    "deployment_summary",
    "load_model",
    "predict_action_chunk",
    "read_model_metadata",
    "resolve_model_path",
    "save_model_package",
    "verify_model_dir",
]
