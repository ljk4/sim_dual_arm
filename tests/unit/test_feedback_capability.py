"""The runtime consumer must reject a backend without the new feedback capability."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
from dual_arm_runtime.episode_runner import EpisodeRunner
from dual_arm_runtime.runtime_config import (
    REQUIRED_CAPABILITIES,
    load_runtime_configuration,
)
from robot_contracts.models import ContractError

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "tests/fixtures/simulation/policy_unit.yaml"
CAPABILITY = "execution_feedback.gripper_set.v1"


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
