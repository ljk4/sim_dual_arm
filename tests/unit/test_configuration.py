import json
import math
from copy import deepcopy
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest
import yaml
from dual_arm_runtime.configuration import (
    ActionSettings,
    ObservationSettings,
    RuntimeRecipe,
    ScriptTarget,
    load_configuration,
)
from robot_contracts.models import ContractError
from robot_contracts.profiles import load_document, semantic_hash, validate_bindings

ROOT = Path(__file__).resolve().parents[2]
BASE_CONFIG = ROOT / "configs/runtime/mock_collect.yaml"
SCHEMAS = ROOT / "contracts/schemas/v1"


def write_config(tmp_path, mutate, name="runtime.yaml"):
    document = yaml.safe_load(BASE_CONFIG.read_text())
    mutate(document)
    path = tmp_path / name
    path.write_text(yaml.safe_dump(document))
    return path


def assert_invalid(path, *fragments):
    with pytest.raises(ContractError) as caught:
        load_configuration(ROOT, path)
    error = caught.value
    assert error.reason == "SCHEMA_MISMATCH"
    message = str(error)
    for fragment in fragments:
        assert fragment in message
    return message


def test_valid_configuration_has_typed_fields_and_snapshot():
    configuration = load_configuration(ROOT, BASE_CONFIG)

    assert isinstance(configuration.recipe, RuntimeRecipe)
    assert isinstance(configuration.observation, ObservationSettings)
    assert isinstance(configuration.action, ActionSettings)
    assert isinstance(configuration.recipe.script_targets, tuple)
    assert isinstance(configuration.recipe.script_targets[0], ScriptTarget)
    assert configuration.recipe.lease_duration_ns == 2_000_000_000
    assert configuration.recipe.renew_interval_ns == 500_000_000
    assert configuration.observation.max_skew_ns == 100_000_000
    assert configuration.observation.max_age_ns == 500_000_000
    assert configuration.action.command_space == "joint_position"
    assert len(configuration.profile.joints) == 6

    snapshot = configuration.snapshot
    assert set(snapshot) == {
        "profile",
        "profile_hash",
        "observation",
        "observation_hash",
        "action",
        "action_hash",
        "recipe",
    }
    assert snapshot["recipe"] == load_document(BASE_CONFIG)
    assert snapshot["profile_hash"] == semantic_hash(snapshot["profile"])
    assert snapshot["observation_hash"] == semantic_hash(snapshot["observation"])
    assert snapshot["action_hash"] == semantic_hash(snapshot["action"])
    json.dumps(snapshot, allow_nan=False)


def test_snapshot_is_detached_and_configuration_fields_are_frozen():
    configuration = load_configuration(ROOT, BASE_CONFIG)
    first = configuration.snapshot

    first["recipe"]["lease_duration_ns"] = 1
    first["recipe"]["script_targets"][0]["positions"][0] = 9.0
    first["observation"]["max_age_ns"] = 1
    first["profile"]["joints"][0]["unit"] = "m"
    first["action"]["joint_names"].append("extra")

    second = configuration.snapshot
    assert second["recipe"]["lease_duration_ns"] == 2_000_000_000
    assert second["recipe"]["script_targets"][0]["positions"][0] == 0.2
    assert second["observation"]["max_age_ns"] == 500_000_000
    assert second["profile"]["joints"][0]["unit"] == "rad"
    assert "extra" not in second["action"]["joint_names"]
    assert configuration.recipe.lease_duration_ns == 2_000_000_000
    assert configuration.observation.max_age_ns == 500_000_000
    assert configuration.profile.joints[0].unit == "rad"
    assert configuration.recipe.script_targets[0].positions[0] == 0.2

    with pytest.raises(FrozenInstanceError):
        configuration.recipe.lease_duration_ns = 1_000_000_000


@pytest.mark.parametrize(
    ("lease", "renew"),
    [(100_000_000, 49_999_999), (10_000_000_000, 4_999_999_999)],
)
def test_supported_lease_range_is_accepted(tmp_path, lease, renew):
    def mutate(document):
        document["lease_duration_ns"] = lease
        document["renew_interval_ns"] = renew

    configuration = load_configuration(ROOT, write_config(tmp_path, mutate))
    assert configuration.recipe.lease_duration_ns == lease
    assert configuration.recipe.renew_interval_ns == renew


@pytest.mark.parametrize(
    ("lease", "renew"),
    [
        (30_000_000, 10_000_000),
        (99_999_999, 40_000_000),
        (10_000_000_001, 1_000_000_000),
    ],
)
def test_lease_outside_backend_range_is_rejected(tmp_path, lease, renew):
    def mutate(document):
        document["lease_duration_ns"] = lease
        document["renew_interval_ns"] = renew

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "lease_duration_ns", str(lease))


def test_renew_must_be_less_than_half_the_lease(tmp_path):
    def mutate(document):
        document["renew_interval_ns"] = 1_000_000_000

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "renew_interval_ns")


def test_renew_just_below_half_the_lease_is_accepted(tmp_path):
    def mutate(document):
        document["renew_interval_ns"] = 999_999_999

    configuration = load_configuration(ROOT, write_config(tmp_path, mutate))
    assert configuration.recipe.renew_interval_ns == 999_999_999


def test_renew_must_be_positive(tmp_path):
    def mutate(document):
        document["renew_interval_ns"] = 0

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "renew_interval_ns")


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("startup_timeout_s", math.nan),
        ("call_timeout_s", math.inf),
        ("command_timeout_s", -math.inf),
    ],
)
def test_non_finite_timeouts_are_rejected(tmp_path, field, value):
    def mutate(document):
        document[field] = value

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), field, "finite")


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_non_finite_script_positions_are_rejected(tmp_path, value):
    def mutate(document):
        document["script_targets"][0]["positions"][0] = value

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "script_targets[0].positions[0]", "finite")


def test_bool_script_position_is_rejected(tmp_path):
    def mutate(document):
        document["script_targets"][0]["positions"][0] = True

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "script_targets", "positions", "number")


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("lease_duration_ns", True),
        ("renew_interval_ns", False),
        ("startup_timeout_s", True),
        ("command_acceptance_ns", True),
    ],
)
def test_bool_is_not_accepted_as_a_number(tmp_path, field, value):
    def mutate(document):
        document[field] = value

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), field)


@pytest.mark.parametrize(
    "field",
    ["lease_duration_ns", "renew_interval_ns", "command_acceptance_ns"],
)
def test_nanosecond_fields_reject_integral_floats(tmp_path, field):
    def mutate(document):
        document[field] = float(document[field])

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), field, "integer")


def test_script_duration_rejects_integral_float(tmp_path):
    def mutate(document):
        document["script_targets"][0]["duration_ns"] = 800_000_000.0

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "script_targets[0].duration_ns", "integer")


def test_script_target_dimension_is_rejected(tmp_path):
    def mutate(document):
        document["script_targets"][0]["positions"].pop()

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "script_targets[0].positions", "expected 6")


def test_script_target_joint_limit_is_rejected(tmp_path):
    def mutate(document):
        document["script_targets"][0]["positions"][0] = 1.5

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "script_targets[0].positions[0]", "left_j1", "outside")


def test_script_target_velocity_from_initial_is_rejected(tmp_path):
    def mutate(document):
        document["script_targets"][0]["positions"][0] = 0.9

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "script_targets[0].positions[0]", "left_j1", "max_velocity")


def test_script_target_velocity_uses_the_previous_target(tmp_path):
    def mutate(document):
        document["script_targets"][0]["positions"] = [0.2, -0.2, 0.15, -0.15, 0.02, 0.02]
        document["script_targets"][1]["positions"] = [-0.9, 0.2, -0.15, 0.15, 0.06, 0.06]

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "script_targets[1].positions[0]", "left_j1", "max_velocity")


def test_command_timeout_must_exceed_the_longest_segment(tmp_path):
    def mutate(document):
        document["command_timeout_s"] = 0.8

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "command_timeout_s", "longest script segment")


def test_non_finite_external_observation_number_is_rejected(tmp_path):
    observation = load_document(ROOT / "profiles/observations/mock_rgb_joint_v1.json")
    observation["max_age_ns"] = math.nan
    observation_path = tmp_path / "observation.yaml"
    observation_path.write_text(yaml.safe_dump(observation))

    def mutate(document):
        document["observation_schema"] = str(observation_path)

    path = write_config(tmp_path, mutate, name="external_observation.yaml")
    assert_invalid(path, str(observation_path), "max_age_ns", "finite")


def test_non_finite_external_action_number_is_rejected(tmp_path):
    action = load_document(ROOT / "profiles/actions/mock_joint_position_v1.json")
    action["schema_version"] = math.inf
    action_path = tmp_path / "action.yaml"
    action_path.write_text(yaml.safe_dump(action))

    def mutate(document):
        document["action_schema"] = str(action_path)

    path = write_config(tmp_path, mutate, name="external_action.yaml")
    assert_invalid(path, str(action_path), "schema_version", "finite")


def test_external_observation_rejects_float_nanoseconds(tmp_path):
    observation = load_document(ROOT / "profiles/observations/mock_rgb_joint_v1.json")
    observation["max_age_ns"] = float(observation["max_age_ns"])
    observation_path = tmp_path / "observation.yaml"
    observation_path.write_text(yaml.safe_dump(observation))

    def mutate(document):
        document["observation_schema"] = str(observation_path)

    path = write_config(tmp_path, mutate, name="external_observation.yaml")
    assert_invalid(path, str(observation_path), "max_age_ns", "integer")


def test_binding_error_reports_the_external_source_path(tmp_path):
    observation = load_document(ROOT / "profiles/observations/mock_rgb_joint_v1.json")
    observation["camera_names"] = ["absent"]
    observation_path = tmp_path / "observation.yaml"
    observation_path.write_text(yaml.safe_dump(observation))

    def mutate(document):
        document["observation_schema"] = str(observation_path)

    path = write_config(tmp_path, mutate, name="binding.yaml")
    assert_invalid(path, str(observation_path), "camera_names")


def test_missing_configuration_file_is_a_schema_error(tmp_path):
    path = tmp_path / "absent.yaml"
    assert_invalid(path, str(path))


def test_invalid_yaml_is_a_schema_error(tmp_path):
    path = tmp_path / "broken.yaml"
    path.write_text("schema_version: [1\n")
    assert_invalid(path, str(path))


def test_missing_referenced_profile_reports_its_path(tmp_path):
    missing = tmp_path / "absent-profile.json"

    def mutate(document):
        document["robot_profile"] = str(missing)

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(missing))


def test_unknown_recipe_field_reports_path_and_field(tmp_path):
    def mutate(document):
        document["extra_parameter"] = 1

    path = write_config(tmp_path, mutate)
    assert_invalid(path, str(path), "extra_parameter")


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
        validate_bindings(profile, observation, action, SCHEMAS)
