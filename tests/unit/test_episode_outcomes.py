"""Pure-Python checks for episode result vocabulary and schema selection."""

from pathlib import Path

import pytest
from robot_contracts.episodes import (
    RecordingResult,
    RuntimeOutcome,
    RuntimeResult,
    TaskOutcome,
    TaskResult,
    manifest_schema_path,
)
from robot_contracts.models import ContractError
from robot_contracts.profiles import validate_document

ROOT = Path(__file__).resolve().parents[2]


def test_manifest_schema_selection_is_an_explicit_version_mapping(tmp_path):
    base = tmp_path / "schemas"
    assert manifest_schema_path(base, 1) == base / "v1/episode_manifest.schema.json"
    assert manifest_schema_path(base / "v1", 1) == base / "v1/episode_manifest.schema.json"
    for version in (0, 2, 3, 4, True, "1"):
        with pytest.raises(ContractError, match="UNSUPPORTED_SCHEMA_VERSION"):
            manifest_schema_path(base, version)


def test_runtime_result_rejects_ambiguous_completed_and_missing_reason():
    assert RuntimeResult.completed() == RuntimeResult(RuntimeOutcome.COMPLETED, "")
    with pytest.raises(ValueError, match="completed runtime"):
        RuntimeResult(RuntimeOutcome.COMPLETED, "failure text")
    with pytest.raises(ValueError, match="must carry a reason"):
        RuntimeResult(RuntimeOutcome.STOPPED)
    assert TaskResult().outcome is TaskOutcome.NOT_EVALUATED
    assert RecordingResult(True, "").complete is True
    with pytest.raises(ValueError, match="complete recording"):
        RecordingResult(True, "corrupt")
    with pytest.raises(ValueError, match="incomplete recording"):
        RecordingResult(False, "")


def test_v1_schema_separates_dimensions_and_rejects_legacy_alias():
    schema = ROOT / "shared/common/interfaces/schemas/v1/episode_manifest.schema.json"
    manifest = {
        "schema_version": 1,
        "episode_id": "episode-test",
        "run_id": "run-test",
        "source": "mock",
        "test_only": True,
        "robot_profile_hash": "profile",
        "observation_schema_hash": "observation",
        "action_schema_hash": "action",
        "started_at_ns": 1,
        "ended_at_ns": 2,
        "message_counts": {},
        "files": {"recording/recording_0.mcap": "0" * 64},
        "recording": {"complete": True, "failure_reason": ""},
        "runtime": {"outcome": "stopped", "reason": "operator stopped"},
        "task": {"outcome": "not_evaluated", "reason": ""},
        "configuration": {},
    }
    validate_document(manifest, schema)

    manifest["runtime"] = {"outcome": "completed", "reason": "stale"}
    with pytest.raises(ContractError, match="reason"):
        validate_document(manifest, schema)

    manifest["runtime"] = {"outcome": "completed", "reason": ""}
    manifest["complete"] = True
    with pytest.raises(ContractError, match="complete"):
        validate_document(manifest, schema)
