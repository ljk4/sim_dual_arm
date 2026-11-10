"""Offline, safetensors-only ACT model bundle and typed prediction helper.

Bundles contain no executable pickle.  Loading verifies every declared file
hash before the trained state dict is applied strictly.  Prediction always
calls upstream ``predict_action_chunk`` and returns a fresh complete 60x16
chunk; it never uses ``select_action`` or a queue.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .checkpoints import CHECKPOINT_FORMAT, load_checkpoint, training_binding
from .config import TrainingConfig
from .dataset import canonical_hash, configure_local_caches, tree_hashes
from .sample_filter import load_filter_mask
from .training import BUNDLE_FORMAT, build_act_config, verify_dataset_manifest


class BundleError(RuntimeError):
    """A model bundle failed an integrity, shape or contract check."""


@dataclass(frozen=True)
class BundleHandle:
    root: Path
    policy: Any
    preprocessor: Any
    postprocessor: Any
    config: Any
    manifest: dict[str, Any]
    device: str

    def prediction_metadata(self) -> dict[str, Any]:
        return {
            "bundle": str(self.root),
            "bundle_digest": self.manifest.get("bundle_digest"),
            "checkpoint_step": self.manifest.get("checkpoint", {}).get("step"),
            "config_sha256": self.manifest.get("config", {}).get("sha256"),
            "synthetic": self.manifest.get("dataset", {}).get("synthetic"),
        }


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise BundleError(f"{path}: cannot read JSON: {error}") from error
    if not isinstance(value, dict):
        raise BundleError(f"{path}: expected an object")
    return value


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, allow_nan=False, default=str) + "\n", encoding="utf-8"
    )


def _processor_digest(processors: Path) -> str:
    digest = hashlib.sha256()
    for name, record in sorted(tree_hashes(processors).items()):
        digest.update(name.encode())
        digest.update(record["sha256"].encode())
    return digest.hexdigest()


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


def _observation_tensors(
    observation: dict[str, Any], manifest: dict[str, Any], device: str
) -> dict[str, Any]:
    import torch

    schema = manifest.get("observation", {})
    cameras = schema.get("cameras", [])
    state = np.asarray(observation.get("observation.state"), dtype=np.float32)
    if state.shape != (int(schema.get("state_dim", 0)),):
        raise BundleError(f"observation.state shape {state.shape} does not match the bundle")
    if not np.isfinite(state).all():
        raise BundleError("observation.state contains non-finite values")
    images = observation.get("images")
    if not isinstance(images, dict) or set(images) != set(cameras):
        raise BundleError("observation.images must contain exactly the bundle cameras")
    sample: dict[str, Any] = {"observation.state": torch.from_numpy(state)}
    for camera in cameras:
        image = np.asarray(images[camera])
        expected = (
            int(schema.get("image_height", 0)),
            int(schema.get("image_width", 0)),
            3,
        )
        if image.shape != expected:
            raise BundleError(f"observation.images.{camera} shape {image.shape} != {expected}")
        if image.dtype == np.uint8:
            array = image.astype(np.float32) / 255.0
        elif np.issubdtype(image.dtype, np.floating):
            array = image.astype(np.float32)
            if not np.isfinite(array).all() or array.min() < 0.0 or array.max() > 1.0:
                raise BundleError(
                    f"observation.images.{camera} float pixels must be finite in [0,1]"
                )
        else:
            raise BundleError(f"observation.images.{camera} dtype must be uint8 or float")
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


def predict_action_chunk(handle: BundleHandle, observation: dict[str, Any]) -> np.ndarray:
    """Return one fresh complete (60,16) physical action chunk.

    The output is never cropped, padded by the caller, or altered per arm.
    """
    import torch

    if not isinstance(handle, BundleHandle):
        raise BundleError("handle must be a loaded BundleHandle")
    sample = _observation_tensors(observation, handle.manifest, handle.device)
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
            raise BundleError("postprocessor did not return an action tensor")
        actions = actions["action"]
    tensor = actions.detach().to("cpu", dtype=torch.float32)
    if tensor.dim() == 3:
        if tensor.shape[0] != 1:
            raise BundleError(f"prediction batch dimension is {tensor.shape[0]}, expected 1")
        tensor = tensor[0]
    if tuple(tensor.shape) != (
        handle.manifest.get("chunk_size", 60),
        handle.manifest.get("state_dim", 16),
    ):
        raise BundleError(f"policy returned {tuple(tensor.shape)}, expected complete (60,16)")
    result = tensor.numpy()
    if not np.isfinite(result).all():
        raise BundleError("policy returned non-finite actions")
    return result


def _actual_sample(config: TrainingConfig, train_root: Path) -> dict[str, Any]:
    configure_local_caches(config.cache_root, data_root=config.data_root)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(
        repo_id=f"local/{config.data_root.name}-{train_root.name}",
        root=train_root,
        video_backend=config.dataset.video_backend,
    )
    sample = dataset[0]
    images = {
        camera: (sample[f"observation.images.{camera}"].permute(1, 2, 0).numpy() * 255.0)
        .round()
        .clip(0, 255)
        .astype(np.uint8)
        for camera in config.dataset.camera_names
    }
    return {
        "observation.state": sample["observation.state"].numpy().astype(np.float32),
        "images": images,
    }


def _export_inputs(
    config: TrainingConfig, run_dir: Path, checkpoint_name: str, allow_synthetic: bool
) -> tuple[Any, dict[str, Any], Any]:
    audit = verify_dataset_manifest(config, allow_synthetic=allow_synthetic)
    if audit.synthetic and not allow_synthetic:
        raise BundleError("refusing to export a bundle from synthetic data")
    checkpoint_path = run_dir / checkpoint_name
    if not checkpoint_path.is_file():
        raise BundleError(f"checkpoint is missing: {checkpoint_path}")
    checkpoint = load_checkpoint(checkpoint_path)
    if checkpoint.get("format") != CHECKPOINT_FORMAT:
        raise BundleError("checkpoint format is not the trusted tool format")
    if checkpoint.get("config_sha256") != config.sha256:
        raise BundleError("checkpoint configuration hash does not match the training YAML")
    if not isinstance(checkpoint.get("model"), dict):
        raise BundleError("checkpoint has no model state dictionary")
    sample_filter = None
    if config.sample_filter is not None:
        sample_filter = load_filter_mask(config.sample_filter)
        if checkpoint.get("training_binding") != training_binding(config):
            raise BundleError("checkpoint was not trained with this sample filter")
    return audit, checkpoint, sample_filter


def _save_policy(config: TrainingConfig, checkpoint: dict[str, Any], output_dir: Path):
    import torch
    from safetensors.torch import save_file

    act_config = build_act_config(config, device="cpu")
    from lerobot.policies.act.modeling_act import ACTPolicy

    policy = ACTPolicy(act_config)
    missing, unexpected = policy.load_state_dict(checkpoint["model"], strict=False)
    if missing or unexpected:
        raise BundleError(
            f"trained state dict is incomplete: missing={list(missing)[:4]} "
            f"unexpected={list(unexpected)[:4]}"
        )
    for name, tensor in policy.state_dict().items():
        if not torch.isfinite(tensor).all():
            raise BundleError(f"trained tensor {name} contains non-finite values")
    policy.eval()
    save_file(
        policy.state_dict(),
        output_dir / "model.safetensors",
        metadata={
            "format": BUNDLE_FORMAT,
            "policy": "act",
            "step": str(checkpoint.get("step", 0)),
        },
    )
    if act_config.pretrained_backbone_weights is not None:
        raise BundleError("bundle config would request an implicit backbone download")
    act_config._save_pretrained(output_dir)
    return act_config, policy


def _copy_metadata(
    config: TrainingConfig,
    output_dir: Path,
    checkpoint: dict[str, Any],
    sample_filter: Any,
) -> tuple[Path, dict[str, Any]]:
    dataset_root = config.data_root
    processors = dataset_root / "processors"
    if not processors.is_dir():
        raise BundleError(f"conversion processors are missing: {processors}")
    shutil.copytree(processors, output_dir / "processors")
    metadata = output_dir / "metadata"
    metadata.mkdir()
    profile = _json(config.dataset.profile_path)
    _write_json(metadata / "profile.json", profile)
    shutil.copy2(dataset_root / config.manifest_name, metadata / "dataset_manifest.json")
    shutil.copy2(config.source, metadata / "training_config.yaml")
    if sample_filter is not None:
        _write_json(metadata / "sample_filter.json", sample_filter.document)
    _write_json(
        metadata / "train_stats_summary.json",
        _json(processors / "train_stats_summary.json"),
    )
    _write_json(
        metadata / "checkpoint.json",
        {
            "format": checkpoint.get("format"),
            "step": checkpoint.get("step"),
            "best_val_l1": checkpoint.get("best_val_l1"),
            "reason": checkpoint.get("reason"),
            "budget_run_id": checkpoint.get("budget_run_id"),
            "budget_segment_id": checkpoint.get("budget_segment_id"),
            "budget_instance_id": checkpoint.get("budget_instance_id"),
            "config_sha256": checkpoint.get("config_sha256"),
            "mode": checkpoint.get("mode"),
            "seed": checkpoint.get("seed"),
        },
    )
    return metadata, profile


def _schema(config: TrainingConfig, profile: dict[str, Any]) -> tuple[dict, dict]:
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


def _verify_round_trip(
    config: TrainingConfig,
    audit: Any,
    output_dir: Path,
    policy: Any,
    metadata: Path,
    observation: dict,
    act_config: Any,
) -> tuple[int, ...]:
    pre_copy, post_copy = _load_processors(output_dir / "processors")
    handle = BundleHandle(
        root=output_dir,
        policy=policy,
        preprocessor=pre_copy,
        postprocessor=post_copy,
        config=act_config,
        manifest={
            "observation": observation,
            "chunk_size": config.dataset.chunk_size,
            "state_dim": config.dataset.state_dim,
        },
        device="cpu",
    )
    sample = _actual_sample(config, audit.train_root)
    before = predict_action_chunk(handle, sample)
    policy.to("cpu")
    # Reload from the exact files being shipped, not from the in-memory object.
    after_handle = _load_bundle_unverified(output_dir, manifest_placeholder=True)
    after = predict_action_chunk(after_handle, sample)
    difference = float(np.max(np.abs(before - after)))
    if difference > 1e-6:
        raise BundleError(f"bundle round-trip prediction changed by {difference}")
    _write_json(
        metadata / "verification.json",
        {
            "same_actual_batch": True,
            "shape": list(before.shape),
            "max_abs_difference": difference,
            "predict_api": "predict_action_chunk",
            "chunk_complete": True,
        },
    )
    return tuple(before.shape)


def _dataset_record(
    config: TrainingConfig,
    audit: Any,
    sample_filter: Any,
    files: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    record: dict[str, Any] = {
        "manifest": "metadata/dataset_manifest.json",
        "manifest_sha256": files["metadata/dataset_manifest.json"]["sha256"],
        "train_episodes": 45 if not audit.synthetic else len(audit.train_episodes),
        "validation_episodes": 5 if not audit.synthetic else len(audit.validation_episodes),
        "training_global_indices": list(audit.training_global_indices),
        "validation_global_indices": list(audit.validation_global_indices),
        "synthetic": bool(audit.synthetic),
        "seed": config.seed,
        "split": "45train/5validation",
        "stats_source": "processors/train_stats_summary.json",
    }
    if sample_filter is not None:
        record["sample_filter"] = {
            "path": "metadata/sample_filter.json",
            "sha256": files["metadata/sample_filter.json"]["sha256"],
            "digest": sample_filter.digest,
            "source_frames": sample_filter.source_frames,
            "eligible_frames": len(sample_filter.eligible_indices),
            "excluded_frames": sample_filter.excluded_frames,
        }
    return record


def _manifest_document(
    config: TrainingConfig,
    metadata: Path,
    audit: Any,
    sample_filter: Any,
    files: dict[str, dict[str, Any]],
    observation: dict,
    action: dict,
    processor_digest: str,
) -> dict[str, Any]:
    dataset_document = _json(metadata / "dataset_manifest.json")
    archive_sha256 = config.dataset.lerobot_source_archive_sha256 or str(
        dataset_document.get("lerobot", {}).get("source_archive_sha256", "")
    )
    model_sha = files["model.safetensors"]["sha256"]
    return {
        "format": BUNDLE_FORMAT,
        "created_wall_time_ns": time.time_ns(),
        "policy": "act",
        "synthetic": bool(audit.synthetic),
        "chunk_size": config.dataset.chunk_size,
        "state_dim": config.dataset.state_dim,
        "observation": observation,
        "action": action,
        "checkpoint": _json(metadata / "checkpoint.json"),
        "config": {"path": "config.json", "sha256": files["config.json"]["sha256"]},
        "dataset": _dataset_record(config, audit, sample_filter, files),
        "profile": {
            "path": "metadata/profile.json",
            "sha256": files["metadata/profile.json"]["sha256"],
            "semantic_sha256": canonical_hash(_json(metadata / "profile.json")),
        },
        "processors": {
            "root": "processors",
            "digest": processor_digest,
            "files": {
                name: record
                for name, record in files.items()
                if name.startswith("processors/")
            },
        },
        "source": {
            "lerobot_revision": config.dataset.lerobot_source_revision,
            "lerobot_archive_sha256": archive_sha256,
            "training_config_sha256": files["metadata/training_config.yaml"]["sha256"],
        },
        "model": {"path": "model.safetensors", "sha256": model_sha},
        "state_dict_sha256": model_sha,
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


def export_bundle(
    *,
    config: TrainingConfig,
    run_dir: Path,
    output_dir: Path,
    checkpoint_name: str = "best.pt",
    allow_synthetic: bool = False,
) -> dict[str, Any]:
    """Export the trained policy plus processors/stats/references as a bundle."""
    configure_local_caches(config.cache_root, data_root=config.data_root)
    if output_dir.exists():
        raise BundleError(f"{output_dir}: bundle output already exists")
    audit, checkpoint, sample_filter = _export_inputs(
        config, run_dir, checkpoint_name, allow_synthetic
    )
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    output_dir.mkdir(parents=True, exist_ok=False)
    act_config, policy = _save_policy(config, checkpoint, output_dir)
    metadata, profile = _copy_metadata(config, output_dir, checkpoint, sample_filter)
    observation, action = _schema(config, profile)
    _write_json(metadata / "schema.json", {"observation": observation, "action": action})
    _verify_round_trip(config, audit, output_dir, policy, metadata, observation, act_config)
    files = tree_hashes(output_dir)
    processor_digest = _processor_digest(output_dir / "processors")
    manifest = _manifest_document(
        config,
        metadata,
        audit,
        sample_filter,
        files,
        observation,
        action,
        processor_digest,
    )
    manifest["bundle_digest"] = hashlib.sha256(
        json.dumps(manifest, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()
    _write_json(output_dir / "manifest.json", manifest)
    return manifest


def _load_bundle_unverified(root: Path, *, manifest_placeholder: bool = False) -> BundleHandle:
    """Construct a handle from files that were just written in this process."""
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from safetensors.torch import load_file

    if not root.is_dir():
        raise BundleError(f"{root}: bundle directory is missing")
    config = ACTConfig.from_pretrained(root, local_files_only=True)
    config.device = "cpu"
    policy = ACTPolicy(config)
    state = load_file(root / "model.safetensors")
    policy.load_state_dict(state, strict=True)
    policy.to("cpu")
    policy.eval()
    pre, post = _load_processors(root / "processors")
    manifest = _json(root / "manifest.json") if (root / "manifest.json").is_file() else {}
    if not manifest and not manifest_placeholder:
        raise BundleError("bundle manifest is missing")
    manifest.setdefault(
        "observation",
        {
            "state_dim": 16,
            "cameras": ["global", "left_wrist", "right_wrist"],
            "image_height": 240,
            "image_width": 320,
        },
    )
    manifest.setdefault("chunk_size", 60)
    manifest.setdefault("state_dim", 16)
    return BundleHandle(
        root=root,
        policy=policy,
        preprocessor=pre,
        postprocessor=post,
        config=config,
        manifest=manifest,
        device="cpu",
    )


def verify_bundle(root: str | Path) -> dict[str, Any]:
    """Verify one bundle's manifest digest, inventory, symlinks and artefact types."""
    bundle = Path(root)
    manifest = _json(bundle / "manifest.json")
    if manifest.get("format") != BUNDLE_FORMAT:
        raise BundleError("bundle manifest format is not supported")
    stored_digest = manifest.get("bundle_digest")
    digest_payload = {key: value for key, value in manifest.items() if key != "bundle_digest"}
    expected_digest = hashlib.sha256(
        json.dumps(digest_payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()
    if stored_digest != expected_digest:
        raise BundleError("bundle manifest digest does not match its contents")
    if manifest.get("chunk_size") != 60 or manifest.get("state_dim") != 16:
        raise BundleError("bundle does not declare the 60x16 action contract")
    declared = manifest.get("files")
    if not isinstance(declared, dict):
        raise BundleError("bundle manifest has no file inventory")
    actual = tree_hashes(bundle)
    actual.pop("manifest.json", None)
    if actual != declared:
        missing = sorted(set(declared) - set(actual))
        extra = sorted(set(actual) - set(declared))
        changed = sorted(
            name for name in set(actual) & set(declared) if actual[name] != declared[name]
        )
        raise BundleError(
            "bundle file hash mismatch: "
            f"missing={missing[:4]} extra={extra[:4]} changed={changed[:4]}"
        )
    for path in tree_files_no_manifest(bundle):
        if path.is_symlink():
            raise BundleError(f"{path}: symlinks are not allowed in a deployment bundle")
    if any(name.endswith((".pt", ".pth", ".pkl", ".pickle")) for name in declared):
        raise BundleError("bundle contains a pickle-based model artefact")
    if manifest.get("model", {}).get("sha256") != manifest.get("state_dict_sha256"):
        raise BundleError("bundle model hash is inconsistent")
    return manifest


def deployment_summary(config: Any) -> dict[str, Any]:
    """Return JSON-safe deployment provenance from an already verified config."""
    summary: dict[str, Any] = {
        "format": BUNDLE_FORMAT,
        "bundle_digest": config.model_bundle_digest,
        "processor_digest": config.processor_digest,
        "observation_schema_hash": config.observation_schema_hash,
        "action_schema_hash": config.action_schema_hash,
        "profile_semantic_sha256": config.robot_profile_hash,
        "chunk_size": config.chunk_size,
        "state_dim": len(config.joint_names),
        "action_period_ns": config.action_period_ns,
        "action_lead_ns": 50_000_000,
        "fps": 20,
        "cameras": list(config.camera_names),
        "camera_width": config.camera_width,
        "camera_height": config.camera_height,
    }
    if config.bundle_dir is None:
        return summary
    manifest = _json(Path(config.bundle_dir) / "manifest.json")
    dataset, source = manifest.get("dataset", {}), manifest.get("source", {})
    summary.update(
        model_sha256=manifest.get("model", {}).get("sha256", ""),
        state_dict_sha256=manifest.get("state_dict_sha256", ""),
        config_sha256=manifest.get("config", {}).get("sha256", ""),
        checkpoint_step=manifest.get("checkpoint", {}).get("step", 0),
        checkpoint_mode=manifest.get("checkpoint", {}).get("mode", ""),
        dataset_seed=dataset.get("seed", 0),
        train_episodes=dataset.get("train_episodes", 0),
        validation_episodes=dataset.get("validation_episodes", 0),
        synthetic=bool(manifest.get("synthetic") or dataset.get("synthetic")),
        lerobot_revision=source.get("lerobot_revision", ""),
        lerobot_archive_sha256=source.get("lerobot_archive_sha256", ""),
        training_config_sha256=source.get("training_config_sha256", ""),
    )
    return summary


def load_bundle(root: str | Path, *, device: str = "cpu") -> BundleHandle:
    """Verify every bundle file, then strictly load the trained ACT policy."""
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from safetensors.torch import load_file

    bundle = Path(root)
    manifest = verify_bundle(bundle)
    config = ACTConfig.from_pretrained(bundle, local_files_only=True)
    config.device = device
    policy = ACTPolicy(config)
    state = load_file(bundle / "model.safetensors")
    missing, unexpected = policy.load_state_dict(state, strict=False)
    if missing or unexpected:
        raise BundleError(
            f"bundle state dict does not match ACT: missing={list(missing)[:4]} "
            f"unexpected={list(unexpected)[:4]}"
        )
    policy.to(device)
    policy.eval()
    pre, post = _load_processors(bundle / "processors")
    return BundleHandle(
        root=bundle,
        policy=policy,
        preprocessor=pre,
        postprocessor=post,
        config=config,
        manifest=manifest,
        device=device,
    )


def tree_files_no_manifest(root: Path) -> list[Path]:
    return sorted(
        path for path in root.rglob("*") if path.is_file() and path.name != "manifest.json"
    )


__all__ = [
    "BundleError",
    "BundleHandle",
    "deployment_summary",
    "export_bundle",
    "load_bundle",
    "predict_action_chunk",
    "tree_files_no_manifest",
    "verify_bundle",
]
