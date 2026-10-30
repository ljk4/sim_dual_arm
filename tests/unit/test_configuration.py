from copy import deepcopy
from pathlib import Path

import pytest
from dual_arm_runtime.configuration import load_configuration
from robot_contracts.models import ContractError
from robot_contracts.profiles import load_document, semantic_hash, validate_bindings

ROOT = Path(__file__).resolve().parents[2]


def test_recipe_and_stable_hash():
    configuration = load_configuration(ROOT, ROOT / "configs/runtime/mock_collect.yaml")
    document = configuration.snapshot["profile"]
    assert semantic_hash(document) == semantic_hash(dict(reversed(list(document.items()))))
    assert len(configuration.profile.joints) == 6


@pytest.mark.parametrize("change", ["order", "camera", "missing", "unknown"])
def test_invalid_bindings_fail_early(profile, change):
    observation = load_document(ROOT / "profiles/observations/mock_rgb_joint_v1.json")
    action = load_document(ROOT / "profiles/actions/mock_joint_position_v1.json")
    observation = deepcopy(observation)
    if change == "order":
        observation["joint_names"].reverse()
    elif change == "camera":
        observation["camera_names"] = ["absent"]
    elif change == "missing":
        del observation["max_age_ns"]
    else:
        observation["unknown"] = True
    with pytest.raises(ContractError, match="SCHEMA_MISMATCH"):
        validate_bindings(profile, observation, action, ROOT / "contracts/schemas/v1")
