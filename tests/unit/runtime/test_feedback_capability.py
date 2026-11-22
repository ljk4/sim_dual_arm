"""The runtime consumer must reject a backend without the new feedback capability."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from dual_arm_runtime.episode_runner import EpisodeRunner
from dual_arm_runtime.runtime_config import (
    REQUIRED_CAPABILITIES,
    load_runtime_configuration,
    load_runtime_configuration_document,
)
from robot_contracts.models import ContractError

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "tests/fixtures/simulation/policy_unit.yaml"
CAPABILITY = "execution_feedback.gripper_set"


def test_runtime_configuration_requires_the_named_gripper_set_capability():
    configuration = load_runtime_configuration(CONFIG, ROOT)
    assert CAPABILITY in REQUIRED_CAPABILITIES
    assert CAPABILITY in configuration.required_capabilities
    assert CAPABILITY in configuration.snapshot["recipe"]["required_capabilities"]


def test_runner_rejects_a_backend_without_the_capability(tmp_path):
    configuration = load_runtime_configuration(CONFIG, ROOT)

    class IO:
        def notify(self) -> None:
            pass

        def wait_backend(self, episode_id, timeout_s, stop):
            return None, SimpleNamespace(
                test_only=False,
                backend_kind="mujoco",
                capabilities=tuple(
                    capability
                    for capability in REQUIRED_CAPABILITIES
                    if capability != CAPABILITY
                ),
            )

    class Control:
        def wait_ready(self) -> None:
            raise AssertionError("capability rejection must happen before ready")

    runner = EpisodeRunner(
        configuration,
        tmp_path,
        IO(),
        lambda stop_requested, report: Control(),
    )
    with pytest.raises(ContractError, match=CAPABILITY):
        runner._prepare("episode-1")  # noqa: SLF001 - narrow lifecycle check


@pytest.mark.parametrize("reset_enabled", [False, True])
def test_hardware_policy_readiness_requires_only_the_requested_capabilities(
    tmp_path, reset_enabled,
):
    document = yaml.safe_load(CONFIG.read_text())
    document.pop("reset")
    if reset_enabled:
        document["reset"] = {"enabled": True, "seed": 0}
    configuration = load_runtime_configuration_document(
        document,
        source=str(CONFIG),
        base_dir=CONFIG.parent,
        project_root=ROOT,
    )
    assert ("simulation_reset" in configuration.required_capabilities) is reset_enabled
    info = SimpleNamespace(
        test_only=False,
        backend_kind="hardware",
        capabilities=configuration.required_capabilities,
    )
    io = SimpleNamespace(notify=lambda: None, wait_backend=lambda *_: (None, info))
    wait_ready = []
    control = SimpleNamespace(wait_ready=lambda: wait_ready.append(True))
    runner = EpisodeRunner(configuration, tmp_path, io, lambda *_: control)
    if reset_enabled:
        with pytest.raises(ContractError, match="simulation features require"):
            runner._prepare("episode-hardware")
        assert not wait_ready
    else:
        assert runner._prepare("episode-hardware") is info
        assert wait_ready == [True]
