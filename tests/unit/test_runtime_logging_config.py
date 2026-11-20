"""Pure-Python checks for runtime logging configuration, identity and metadata."""

from __future__ import annotations

import json
from dataclasses import FrozenInstanceError, asdict
from pathlib import Path

import pytest
import yaml
from dual_arm_runtime import run_logging
from dual_arm_runtime.runtime_config import load_runtime_configuration
from robot_contracts.models import ContractError
from robot_run_logging import LogFailure, LogSettings

ROOT = Path(__file__).resolve().parents[2]
BASE_CONFIG = ROOT / "shared/scenarios/marvin_sim/stack/configs/runtime/teacher_collect.yaml"
DEFAULTS = asdict(LogSettings())


def test_missing_health_check_is_rejected_instead_of_silently_disabled():
    with pytest.raises(LogFailure, match="check_health"):
        run_logging.check_health(object())


def write_config(tmp_path: Path, mutate) -> Path:
    document = yaml.safe_load(BASE_CONFIG.read_text(encoding="utf-8"))
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_config",
    ):
        if field in document:
            document[field] = str((BASE_CONFIG.parent / document[field]).resolve())
    mutate(document)
    path = tmp_path / "runtime.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def load_with(tmp_path: Path, mutate):
    return load_runtime_configuration(write_config(tmp_path, mutate), ROOT)


def test_old_config_without_logging_gets_effective_defaults_and_snapshot(tmp_path):
    configuration = load_with(tmp_path, lambda document: document.pop("logging"))

    assert asdict(configuration.recipe.logging) == DEFAULTS
    assert configuration.snapshot["recipe"]["logging"] == DEFAULTS
    # Configuration logging is deployment metadata; it must not perturb the
    # profile/observation/action semantic identity.
    configured = load_runtime_configuration(BASE_CONFIG, ROOT)
    for key in ("profile_hash", "observation_hash", "action_hash"):
        assert configuration.snapshot[key] == configured.snapshot[key]


def test_partial_logging_override_merges_with_defaults(tmp_path):
    def mutate(document):
        document["logging"] = {
            "queue_capacity": 8,
            "console": True,
        }

    configuration = load_with(tmp_path, mutate)

    assert configuration.recipe.logging.queue_capacity == 8
    assert configuration.recipe.logging.console is True
    assert configuration.recipe.logging.max_file_bytes == DEFAULTS["max_file_bytes"]
    assert configuration.snapshot["recipe"]["logging"] == {
        **DEFAULTS,
        "queue_capacity": 8,
        "console": True,
    }


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("queue_capacity", 0),
        ("queue_capacity", True),
        ("max_event_bytes", 1.5),
        ("max_total_bytes", -1),
        ("close_timeout_s", 0),
        ("flush_interval_s", float("nan")),
        ("console", 1),
    ],
)
def test_invalid_logging_value_reports_config_path(tmp_path, field, value):
    def mutate(document):
        document.setdefault("logging", {})[field] = value

    path = write_config(tmp_path, mutate)
    with pytest.raises(ContractError) as caught:
        load_runtime_configuration(path, ROOT)
    message = str(caught.value)
    assert caught.value.reason == "SCHEMA_MISMATCH"
    assert str(path) in message
    assert "logging" in message


@pytest.mark.parametrize(
    "logging_document",
    [
        {"max_event_bytes": 2, "max_file_bytes": 1},
        {"max_file_bytes": 2, "max_total_bytes": 1},
        {"unknown_setting": 1},
    ],
)
def test_invalid_logging_capacity_relations_report_config_path(tmp_path, logging_document):
    def mutate(document):
        document["logging"] = logging_document

    path = write_config(tmp_path, mutate)
    with pytest.raises(ContractError) as caught:
        load_runtime_configuration(path, ROOT)
    message = str(caught.value)
    assert str(path) in message
    assert "logging" in message


def test_log_settings_are_frozen_on_the_recipe(tmp_path):
    configuration = load_with(tmp_path, lambda document: None)
    with pytest.raises(FrozenInstanceError):
        configuration.recipe.logging.queue_capacity = 1


@pytest.mark.parametrize(
    "value",
    ["", ".", "..", "../outside", "a/b", "/absolute", "has space", "x" * 129],
)
def test_run_id_rejects_unsafe_directory_components(value):
    with pytest.raises(ValueError):
        run_logging.validate_run_id(value)


def test_run_id_accepts_generated_uuid_and_safe_names():
    generated = run_logging.new_run_id()
    assert run_logging.validate_run_id(generated) == generated
    assert run_logging.validate_run_id("sim-check_01.2") == "sim-check_01.2"


def test_initialize_run_writes_metadata_and_exclusive_directory(tmp_path, monkeypatch):
    monkeypatch.setenv("DSH008_SECRET_TOKEN", "secret-value")
    configuration = load_runtime_configuration(BASE_CONFIG, ROOT)
    session = run_logging.initialize_run(
        configuration,
        run_id="run-metadata",
        runs_output=tmp_path / "runs",
        output=tmp_path / "raw",
    )
    try:
        assert session.run_directory == (tmp_path / "runs/run-metadata").resolve()
        assert session.log_directory == session.run_directory / "logs/runtime" / session.instance_id
        assert session.log_directory.is_dir()

        resolved = json.loads((session.run_directory / "resolved/runtime.json").read_text())
        assert resolved["run_id"] == "run-metadata"
        assert resolved["instance_id"] == session.instance_id
        assert resolved["recipe"]["logging"] == asdict(configuration.recipe.logging)
        assert resolved["profile_hash"] == configuration.snapshot["profile_hash"]

        versions = json.loads((session.run_directory / "versions.json").read_text())
        assert set(versions) == {
            "run_id",
            "instance_id",
            "python",
            "dependencies",
            "sources",
            "configuration_sha256",
            "recipe_sha256",
            "logging",
            "image",
            "git",
        }
        assert versions["python"]["version"]
        assert set(versions["dependencies"]) == {"PyYAML", "jsonschema"}
        assert set(versions["sources"]) == {
            "dual_arm_runtime",
            "robot_contracts",
            "robot_run_logging",
            "robot_ros_support",
        }
        assert versions["image"]["digest"] is None
        assert versions["git"] == {"revision": None, "dirty": None}
        assert len(versions["configuration_sha256"]) == 64
        serialized = json.dumps(versions)
        assert "secret-value" not in serialized
    finally:
        session.logger.close()

    with pytest.raises(FileExistsError):
        run_logging.initialize_run(
            configuration,
            run_id="run-metadata",
            runs_output=tmp_path / "runs",
            output=tmp_path / "raw",
        )


def test_initialize_run_does_not_leave_completion_evidence_on_metadata_failure(
    tmp_path, monkeypatch
):
    configuration = load_runtime_configuration(BASE_CONFIG, ROOT)

    def fail_write(path, document):
        raise OSError("simulated metadata failure")

    monkeypatch.setattr(run_logging, "_write_json_atomic", fail_write)
    with pytest.raises(OSError, match="simulated metadata failure"):
        run_logging.initialize_run(
            configuration,
            run_id="run-broken-init",
            runs_output=tmp_path / "runs",
            output=tmp_path / "raw",
        )

    run_directory = tmp_path / "runs/run-broken-init"
    assert run_directory.is_dir()
    assert not (run_directory / "logs").exists()
    assert not list(run_directory.glob("**/summary.json"))
    assert not (run_directory / "status.json").exists()


def test_existing_directory_file_and_symlinks_are_never_reused(tmp_path):
    configuration = load_runtime_configuration(BASE_CONFIG, ROOT)
    runs = tmp_path / "runs"
    runs.mkdir()
    (runs / "existing-dir").mkdir()
    (runs / "existing-file").write_text("keep")
    dangling = runs / "dangling-link"
    dangling.symlink_to(tmp_path / "missing-target", target_is_directory=True)
    target = tmp_path / "external-target"
    target.mkdir()
    valid_link = runs / "valid-link"
    valid_link.symlink_to(target, target_is_directory=True)

    for run_id in ("existing-dir", "existing-file", "dangling-link", "valid-link"):
        with pytest.raises(FileExistsError):
            run_logging.initialize_run(
                configuration,
                run_id=run_id,
                runs_output=runs,
                output=tmp_path / "raw",
            )

    assert not (tmp_path / "missing-target").exists()
    assert list(target.iterdir()) == []
