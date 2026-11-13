"""``--set dot.path=value`` overrides stay small and strict."""

from __future__ import annotations

import pytest
from robot_contracts.overrides import OverrideError, apply_overrides, parse_override


def test_dotted_paths_are_applied_in_order_with_yaml_values():
    document = {
        "observation": {"max_age_ns": 500_000_000, "reliable": False},
        "reset": {"seed": 1},
    }
    result = apply_overrides(
        document,
        [
            "observation.max_age_ns=200000000",
            "observation.reliable=true",
            "reset.seed=42",
        ],
    )
    assert result["observation"] == {"max_age_ns": 200_000_000, "reliable": True}
    assert result["reset"]["seed"] == 42
    # The caller's document is never mutated.
    assert document["observation"]["max_age_ns"] == 500_000_000


def test_yaml_scalars_and_collections_are_parsed():
    document = {"run": {"values": [1], "seed": 0}}
    result = apply_overrides(
        document, ["run.values=[1, 2, 3]", "run.seed=null"]
    )
    assert result["run"]["values"] == [1, 2, 3]
    assert result["run"]["seed"] is None


@pytest.mark.parametrize(
    "spec, message",
    [
        ("observation", "dot.path=value"),
        ("=1", "empty or unsafe"),
        ("observation..max_age_ns=1", "empty or unsafe"),
        ("observation.max_age_ns..x=1", "empty or unsafe"),
        ("observation.missing=1", "does not exist"),
        ("observation.max_age_ns.deeper=1", "does not exist"),
    ],
)
def test_invalid_or_out_of_bounds_overrides_are_rejected(spec, message):
    with pytest.raises(OverrideError, match=message):
        apply_overrides({"observation": {"max_age_ns": 1}}, [spec])


def test_parse_override_rejects_traversal():
    with pytest.raises(OverrideError, match="empty or unsafe"):
        parse_override("observation.../etc=1")
