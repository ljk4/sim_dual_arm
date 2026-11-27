"""Unit tests for spatial-preserving training visual augmentation and wiring."""

from __future__ import annotations

import argparse
from pathlib import Path
from types import SimpleNamespace

import pytest
from dual_arm_learning import cli
from dual_arm_learning.config import VisualAugmentationSpec, load_training_config
from dual_arm_learning.scenario import select_scenario
from dual_arm_learning.training import engine
from dual_arm_learning.training.support import augment_visual_batch

torch = pytest.importorskip("torch")

ROOT = Path(__file__).resolve().parents[3]


def _synthetic_batch(batch_size: int = 4, height: int = 32, width: int = 32) -> dict[str, torch.Tensor]:
    y0, x0 = 11, 21
    batch: dict[str, torch.Tensor] = {
        "observation.state": torch.linspace(-1.0, 1.0, batch_size * 16).reshape(batch_size, 16),
        "action": torch.zeros((batch_size, 60, 16), dtype=torch.float32),
    }
    for idx, cam in enumerate(("cam_high", "cam_left_wrist", "cam_right_wrist")):
        img = torch.full((batch_size, 3, height, width), 0.25 + 0.05 * idx, dtype=torch.float32)
        img[:, 0, y0 - 1 : y0 + 2, x0 - 1 : x0 + 2] = 0.85
        img[:, 1, y0 - 1 : y0 + 2, x0 - 1 : x0 + 2] = 0.20
        img[:, 2, y0 - 1 : y0 + 2, x0 - 1 : x0 + 2] = 0.15
        img[:, :, y0, x0] = torch.tensor([0.95, 0.90, 0.85]).view(1, 3)
        batch[f"observation.images.{cam}"] = img
    return batch


def test_visual_augmentation_disabled_is_noop():
    batch = _synthetic_batch(batch_size=2)
    spec = VisualAugmentationSpec(enabled=False)
    out = augment_visual_batch(batch, spec)
    assert out is batch


def test_photometric_augmentation_preserves_spatial_geometry_and_bounds():
    torch.manual_seed(1234)
    batch = _synthetic_batch(batch_size=4)
    orig_global = batch["observation.images.cam_high"].clone()
    orig_state = batch["observation.state"].clone()
    spec = VisualAugmentationSpec(
        enabled=True,
        brightness=0.15,
        contrast=0.15,
        saturation=0.15,
        hue=0.03,
        noise_std=0.0,
        blur_prob=1.0,
        camera_dropout_prob=0.0,
    )

    out = augment_visual_batch(batch, spec)

    assert torch.equal(batch["observation.images.cam_high"], orig_global)
    assert torch.equal(out["observation.state"], orig_state)
    assert torch.equal(out["action"], batch["action"])

    for cam in ("cam_high", "cam_left_wrist", "cam_right_wrist"):
        img = out[f"observation.images.{cam}"]
        assert img.shape == (4, 3, 32, 32)
        assert float(img.min()) >= 0.0
        assert float(img.max()) <= 1.0
        luma = 0.2989 * img[:, 0] + 0.5870 * img[:, 1] + 0.1140 * img[:, 2]
        for b in range(4):
            flat_idx = int(torch.argmax(luma[b]).item())
            assert divmod(flat_idx, 32) == (11, 21)


def test_camera_dropout_zeros_at_most_one_camera_per_sample():
    torch.manual_seed(42)
    batch = _synthetic_batch(batch_size=16)
    spec = VisualAugmentationSpec(
        enabled=True,
        brightness=0.0,
        contrast=0.0,
        saturation=0.0,
        hue=0.0,
        noise_std=0.0,
        blur_prob=0.0,
        camera_dropout_prob=1.0,
    )

    out = augment_visual_batch(batch, spec)
    cams = ("observation.images.cam_high", "observation.images.cam_left_wrist", "observation.images.cam_right_wrist")
    for b in range(16):
        zero_count = sum(bool((out[k][b] == 0.0).all().item()) for k in cams)
        assert zero_count == 1


class _ToyLossPolicy(torch.nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.w = torch.nn.Parameter(torch.tensor([1.0]))

    def forward(self, _batch):
        return (self.w ** 2).sum(), {"l1_loss": 0.1, "kld_loss": 0.0}


class _NoopScaler:
    def is_enabled(self) -> bool:
        return False

    def scale(self, loss):
        return loss

    def unscale_(self, _opt) -> None:
        pass

    def step(self, opt) -> None:
        opt.step()

    def update(self) -> None:
        pass


@pytest.mark.parametrize(
    ("mode", "enabled", "expected_calls"),
    [
        ("full", True, 1),
        ("full", False, 0),
        ("smoke", True, 0),
        ("overfit", True, 0),
    ],
)
def test_train_step_and_cli_visual_augmentation_wiring(monkeypatch, mode, enabled, expected_calls):
    calls: list[tuple[bool, str | None]] = []
    monkeypatch.setattr(engine, "_sample_keys", lambda _cfg: ("observation.state",))
    monkeypatch.setattr(engine, "_process_batch", lambda _pre, item, _keys: item)
    monkeypatch.setattr(
        engine,
        "_augment_visual_batch",
        lambda batch, spec, device=None: calls.append((spec.enabled, device)) or batch,
    )

    policy = _ToyLossPolicy()
    batches = [{"observation.state": torch.zeros((1, 16))}]
    context = SimpleNamespace(
        config=SimpleNamespace(
            mode=mode,
            device="cpu",
            grad_clip_norm=10.0,
            visual_augmentation=VisualAugmentationSpec(enabled=enabled),
        ),
        stop_signal=None,
        policy=policy,
        optimizer=torch.optim.SGD(policy.parameters(), lr=0.1),
        scheduler=SimpleNamespace(step=lambda: None),
        pre=None,
        policy_loader=batches,
        iterator=iter(batches),
        step=0,
    )
    engine._train_step(context, (False, torch.float32, _NoopScaler()))
    assert len(calls) == expected_calls

    selection = select_scenario(
        robot="marvin",
        backend="mujoco",
        task="handover",
        recipe_field="training",
        stage="learning",
        project_root=ROOT,
    )
    cfg_default = load_training_config(selection.recipe_path, ROOT)
    assert cfg_default.visual_augmentation.enabled is True
    cfg_off = cli._training_config(
        argparse.Namespace(project_root=ROOT, mode="full", data_root=None, visual_augment=False),
        selection.recipe_path,
    )
    assert cfg_off.visual_augmentation.enabled is False
    cfg_on = cli._training_config(
        argparse.Namespace(project_root=ROOT, mode="full", data_root=None, visual_augment=True),
        selection.recipe_path,
    )
    assert cfg_on.visual_augmentation.enabled is True
