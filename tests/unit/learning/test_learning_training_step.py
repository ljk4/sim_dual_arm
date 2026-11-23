"""Unit tests for FP16 GradScaler overflow backoff inside ``_train_step``."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
from dual_arm_learning.training import engine
from dual_arm_learning.training.support import TrainingError

torch = pytest.importorskip("torch")


class _InfGrad(torch.autograd.Function):
    @staticmethod
    def forward(ctx, x):
        return x.sum()

    @staticmethod
    def backward(ctx, grad_output):
        return torch.full_like(grad_output, float("inf")).expand(1)


class _ToyPolicy(torch.nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.weight = torch.nn.Parameter(torch.tensor([1.0], dtype=torch.float32))

    def forward(self, batch):
        if batch.get("inf_grad"):
            loss = _InfGrad.apply(self.weight)
        else:
            loss = (self.weight ** 2).sum()
        return loss, {"l1_loss": 0.5, "kld_loss": 0.1}


class _CountingScheduler:
    def __init__(self) -> None:
        self.steps = 0

    def step(self) -> None:
        self.steps += 1


class _FakeGradScaler:
    def __init__(self, *, enabled: bool, overflow_attempts: int = 0, initial_scale: float = 65536.0) -> None:
        self._enabled = enabled
        self._overflow_attempts = overflow_attempts
        self._scale = initial_scale
        self._attempt = 0
        self._found_inf = False
        self.optimizer_steps = 0

    def is_enabled(self) -> bool:
        return self._enabled

    def get_scale(self) -> float:
        return self._scale

    def scale(self, loss):
        return loss

    def unscale_(self, optimizer) -> None:
        if self._enabled and self._attempt < self._overflow_attempts:
            for group in optimizer.param_groups:
                for param in group["params"]:
                    if param.grad is not None:
                        param.grad.fill_(float("inf"))

    def step(self, optimizer) -> None:
        if not self._enabled:
            optimizer.step()
            self.optimizer_steps += 1
            return
        found_inf = any(
            param.grad is not None and not torch.isfinite(param.grad).all()
            for group in optimizer.param_groups
            for param in group["params"]
        )
        self._found_inf = found_inf
        if not found_inf:
            optimizer.step()
            self.optimizer_steps += 1

    def update(self) -> None:
        if self._enabled and self._found_inf:
            self._scale *= 0.5
        self._found_inf = False
        self._attempt += 1


def _make_context(monkeypatch, tmp_path, *, batches):
    monkeypatch.setattr(engine, "_sample_keys", lambda _cfg: ("observation.state",))
    monkeypatch.setattr(engine, "_process_batch", lambda _pre, item, _keys: item)
    policy = _ToyPolicy()
    optimizer = torch.optim.SGD(policy.parameters(), lr=0.1)
    scheduler = _CountingScheduler()
    context = SimpleNamespace(
        config=SimpleNamespace(grad_clip_norm=10.0),
        stop_signal=None,
        policy=policy,
        optimizer=optimizer,
        scheduler=scheduler,
        pre=None,
        policy_loader=batches,
        iterator=iter(batches),
        step=0,
    )
    return context, scheduler


def test_grad_scaler_overflow_skips_and_retries_without_advancing_step(monkeypatch, tmp_path):
    batches = [{"observation.state": torch.zeros((1, 16))}]
    context, scheduler = _make_context(monkeypatch, tmp_path, batches=batches)
    scaler = _FakeGradScaler(enabled=True, overflow_attempts=2, initial_scale=65536.0)

    metrics = engine._train_step(context, (False, torch.float16, scaler))

    assert context.step == 1
    assert scheduler.steps == 1
    assert scaler.optimizer_steps == 1
    assert scaler.get_scale() == 16384.0
    assert math.isfinite(metrics["grad_norm"]) and metrics["grad_norm"] > 0.0
    assert math.isfinite(metrics["loss"])


def test_grad_scaler_repeated_overflow_raises_training_error(monkeypatch, tmp_path):
    batches = [{"observation.state": torch.zeros((1, 16))}]
    context, scheduler = _make_context(monkeypatch, tmp_path, batches=batches)
    scaler = _FakeGradScaler(
        enabled=True, overflow_attempts=engine._MAX_AMP_OVERFLOW_SKIPS + 5, initial_scale=65536.0
    )

    with pytest.raises(TrainingError, match="consecutive GradScaler backoffs"):
        engine._train_step(context, (False, torch.float16, scaler))

    assert context.step == 0
    assert scheduler.steps == 0
    assert scaler.optimizer_steps == 0


def test_non_finite_gradient_without_scaler_raises_immediately(monkeypatch, tmp_path):
    batches = [{"observation.state": torch.zeros((1, 16)), "inf_grad": True}]
    context, scheduler = _make_context(monkeypatch, tmp_path, batches=batches)
    scaler = _FakeGradScaler(enabled=False)

    with pytest.raises(TrainingError, match="non-finite gradient norm at step 0"):
        engine._train_step(context, (False, torch.float32, scaler))

    assert context.step == 0
    assert scheduler.steps == 0
    assert scaler.optimizer_steps == 0
