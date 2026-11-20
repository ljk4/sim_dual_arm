from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
from pydantic import ValidationError

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "shared/scenarios/marvin_sim/stack/profiles"
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.config.stream_config import (  # noqa: E402
    STREAM_CONFIG_FIELDS,
    StreamConfig,
    StreamConfigError,
    load_stream_config,
    parse_stream_config,
)

VALID = {
    "initial_lead_s": 0.25,
    "min_lead_s": 0.05,
    "max_observation_age_s": 0.5,
    "stream_wall_timeout_s": 240.0,
    "progress_wall_timeout_s": 3.0,
    "tracking_timeout_s": 2.0,
    "contact_unload_timeout_s": 2.0,
}


def expect_error(document, field: str):
    with pytest.raises(StreamConfigError) as caught:
        parse_stream_config(document, "unit.yaml")
    assert caught.value.field == field
    assert "unit.yaml" in str(caught.value)
    return caught.value


def test_candidate_config_converts_to_exact_bounded_integer_nanoseconds():
    config = parse_stream_config(dict(VALID), "candidate.yaml")
    assert config.initial_lead_ns == 250_000_000
    assert config.min_lead_ns == 50_000_000
    assert config.max_observation_age_ns == 500_000_000
    assert config.stream_wall_timeout_ns == 240_000_000_000
    assert config.progress_wall_timeout_ns == 3_000_000_000
    assert config.tracking_timeout_ns == 2_000_000_000
    assert config.initial_lead_ns >= config.min_lead_ns
    assert config.min_lead_ns >= 2_000_000


def test_snapshot_is_frozen_fresh_and_complete():
    config = parse_stream_config(dict(VALID))
    document = config.to_document()
    assert set(document) == set(STREAM_CONFIG_FIELDS) | {"contact_unload_timeout_s"}
    assert document == VALID
    document["initial_lead_s"] = 999.0
    assert config.to_document()["initial_lead_s"] == 0.25
    with pytest.raises(ValidationError):
        config.initial_lead_s = 1.0  # type: ignore[misc]
    with pytest.raises(StreamConfigError):
        config.nanoseconds("unknown")


def test_candidate_file_loads_and_round_trips():
    config = load_stream_config(PROFILES / "control/marvin_stream_v1.yaml")
    assert config.to_document() == {
        name: value for name, value in VALID.items() if name != "contact_unload_timeout_s"
    }
    assert parse_stream_config(config.to_document()) == config


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("initial_lead_s", True),
        ("min_lead_s", "0.05"),
        ("max_observation_age_s", None),
        ("stream_wall_timeout_s", float("nan")),
        ("progress_wall_timeout_s", float("inf")),
        ("tracking_timeout_s", 0.0),
        ("tracking_timeout_s", -2.0),
    ],
)
def test_rejects_bool_non_number_non_finite_zero_and_negative(field, value):
    document = dict(VALID)
    document[field] = value
    expect_error(document, field)


def test_rejects_unknown_missing_and_non_mapping():
    unknown = dict(VALID)
    unknown["extra_s"] = 1.0
    expect_error(unknown, "extra_s")
    for field in STREAM_CONFIG_FIELDS:
        missing = dict(VALID)
        del missing[field]
        expect_error(missing, field)
    with pytest.raises(StreamConfigError) as caught:
        parse_stream_config([], "unit.yaml")
    assert caught.value.field == "<root>"


def test_rejects_overflowing_nanoseconds_at_the_field_path():
    document = dict(VALID)
    document["stream_wall_timeout_s"] = 10**20
    expect_error(document, "stream_wall_timeout_s")


def test_rejects_initial_lead_below_min_lead_and_accepts_equality():
    document = dict(VALID)
    document["initial_lead_s"] = 0.049
    error = expect_error(document, "initial_lead_s")
    assert "min_lead_s" in error.message
    document["initial_lead_s"] = 0.05
    assert parse_stream_config(document).initial_lead_ns == 50_000_000


def test_duplicate_yaml_key_is_rejected_with_source(tmp_path):
    path = tmp_path / "duplicate.yaml"
    path.write_text("initial_lead_s: 0.25\ninitial_lead_s: 0.3\n")
    with pytest.raises(StreamConfigError) as caught:
        load_stream_config(path)
    assert caught.value.field == "initial_lead_s"
    assert str(path) in str(caught.value)


def test_direct_constructor_runs_the_same_cross_field_validation():
    with pytest.raises(StreamConfigError):
        StreamConfig(0.01, 0.05, 0.5, 120.0, 3.0, 2.0)
    assert StreamConfig(0.25, 0.25, 0.5, 120.0, 3.0, 2.0).initial_lead_ns == 250_000_000


def test_rounding_uses_nearest_nanosecond_not_truncation():
    config = parse_stream_config(
        {
            "initial_lead_s": 0.250_000_000_6,
            "min_lead_s": 0.05,
            "max_observation_age_s": 0.5,
            "stream_wall_timeout_s": 240.0,
            "progress_wall_timeout_s": 3.0,
            "tracking_timeout_s": 2.0,
        }
    )
    assert config.initial_lead_ns == 250_000_001
    assert math.isfinite(config.initial_lead_s)
