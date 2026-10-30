"""Validated runtime recipes, distinct from robot semantics and deployment paths."""

from dataclasses import dataclass
from pathlib import Path

from robot_contracts.models import ContractError, RobotProfile
from robot_contracts.profiles import (
    load_document,
    load_profile,
    semantic_hash,
    validate_bindings,
    validate_document,
)


@dataclass(frozen=True)
class Configuration:
    recipe: dict
    profile: RobotProfile
    observation: dict
    action: dict
    snapshot: dict
    schema_root: Path


def load_configuration(root: Path, path: Path) -> Configuration:
    schemas = root / "contracts/schemas/v1"
    recipe = load_document(path)
    validate_document(recipe, schemas / "runtime_config.schema.json")
    profile_doc = load_document(root / recipe["robot_profile"])
    profile = load_profile(root / recipe["robot_profile"], schemas)
    observation = load_document(root / recipe["observation_schema"])
    action = load_document(root / recipe["action_schema"])
    validate_bindings(profile, observation, action, schemas)
    if recipe["renew_interval_ns"] * 2 >= recipe["lease_duration_ns"]:
        raise ContractError("SCHEMA_MISMATCH", "Renew interval needs lease headroom")
    for target in recipe["script_targets"]:
        if len(target["positions"]) != len(profile.joints):
            raise ContractError("SCHEMA_MISMATCH", "Script target dimension")
    snapshot = {
        "profile": profile_doc,
        "profile_hash": profile.digest,
        "observation": observation,
        "observation_hash": semantic_hash(observation),
        "action": action,
        "action_hash": semantic_hash(action),
        "recipe": recipe,
    }
    return Configuration(recipe, profile, observation, action, snapshot, schemas)
