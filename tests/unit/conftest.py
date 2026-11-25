from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
for source in (
    ROOT / "shared/common/python/contracts/src",
    ROOT / "shared/common/python/trajectory/src",
    ROOT / "shared/common/python/run_logging/src",
    ROOT / "shared/common/python/ros_support/src",
    ROOT / "shared/common/tools",
    ROOT / "runtime/ros2/ros/dual_arm_runtime",
    ROOT / "learning/lerobot/src",
    ROOT / "simulation/mujoco/src",
    ROOT / "tests/fakes/policy",
    ROOT / "tests/unit/fixtures",
    ROOT / "tests/unit/shared",
    ROOT / "tests/unit/runtime",
):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_contracts.models import RunContext  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402


@pytest.fixture
def profile():
    return load_profile(
        ROOT / "shared/scenarios/marvin/mujoco/stack/profiles/robot/profile.json",
        ROOT / "shared/common/interfaces/schemas",
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
