from pathlib import Path

import pytest
from robot_contracts.models import RunContext
from robot_contracts.profiles import load_profile

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def profile():
    return load_profile(
        ROOT / "shared/scenarios/panda_stack/profiles/robots/panda_stack_v1/profile.json",
        ROOT / "shared/common/interfaces/schemas/v1",
    )


@pytest.fixture
def controlled(profile):
    """A stable context for observation tests that do not drive a backend engine."""
    context = RunContext(
        profile.robot_id,
        profile.digest,
        "backend-test",
        "mujoco-sim",
        0,
        episode_id="episode-test",
        control_epoch=1,
    )
    return None, context, None
