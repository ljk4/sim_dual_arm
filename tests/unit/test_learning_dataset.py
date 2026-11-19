import json
from pathlib import Path

import pytest
from dual_arm_learning.config import load_dataset_config
from dual_arm_learning.dataset import ExportError, _validate_export, configure_local_caches
from learning_fixtures import make_export, profile_hash
from model_helpers import replace

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/panda_sim/stack"


@pytest.fixture
def config(tmp_path):
    value = load_dataset_config(SCENARIO / "configs/learning/act_dataset.yaml", ROOT)
    return replace(value, output_root=tmp_path / "output", cache_root=tmp_path / "cache")


def test_validate_export_accepts_034_shape_and_checks_png_pixels(config, tmp_path):
    make_export(
        attempt_dir=tmp_path / "attempt",
        config=config,
        episode_id="synthetic-ok",
        seed=1,
        frames=2,
        value=0.01,
        start_ns=1_000_000_000,
    )
    export = tmp_path / "attempt" / "export"
    rows, raw, quality = _validate_export(export, config, profile_hash(config))
    assert len(rows) == 2
    assert len(raw) == 1
    assert quality["qualified"] is True
    document = json.loads((export / "labels.json").read_text())
    assert document["action_source"] == "teacher"
    assert rows[0]["label_sim_time_ns"] - rows[0]["observation_sim_time_ns"] == 50_000_000


def test_wrong_label_clock_png_hash_and_raw_times_are_rejected(config, tmp_path):
    make_export(
        attempt_dir=tmp_path / "attempt",
        config=config,
        episode_id="synthetic-bad",
        seed=2,
        frames=2,
        value=0.02,
        start_ns=2_000_000_000,
    )
    export = tmp_path / "attempt" / "export"

    labels = json.loads((export / "labels.json").read_text())
    labels["labels"][1]["label_sim_time_ns"] += 1
    (export / "labels.json").write_text(json.dumps(labels))
    with pytest.raises(ExportError, match="50 ms"):
        _validate_export(export, config, profile_hash(config))

    labels["labels"][1]["label_sim_time_ns"] -= 1
    (export / "labels.json").write_text(json.dumps(labels))
    png = export / labels["labels"][1]["rgb"]["global"]["path"]
    png.write_bytes(png.read_bytes() + b"x")
    with pytest.raises(ExportError, match="PNG SHA"):
        _validate_export(export, config, profile_hash(config))

    # Rebuild a clean fixture and corrupt the raw future clock instead.
    import shutil

    shutil.rmtree(tmp_path / "attempt")
    make_export(
        attempt_dir=tmp_path / "attempt",
        config=config,
        episode_id="synthetic-bad",
        seed=2,
        frames=2,
        value=0.02,
        start_ns=2_000_000_000,
    )
    raw = json.loads((export / "raw_predictions.json").read_text())
    raw["predictions"][0]["times_from_observation_ns"][0] = 0
    (export / "raw_predictions.json").write_text(json.dumps(raw))
    with pytest.raises(ExportError, match="1..60"):
        _validate_export(export, config, profile_hash(config))


def test_quality_true_but_failed_task_or_runtime_is_rejected(config, tmp_path):
    make_export(
        attempt_dir=tmp_path / "attempt",
        config=config,
        episode_id="synthetic-failed",
        seed=3,
        frames=2,
        value=0.03,
        start_ns=3_000_000_000,
    )
    export = tmp_path / "attempt" / "export"
    labels = json.loads((export / "labels.json").read_text())
    labels["task"] = {"outcome": "failed", "reason": "TASK_FAILED"}
    (export / "labels.json").write_text(json.dumps(labels))
    with pytest.raises(ExportError, match="task outcome"):
        _validate_export(export, config, profile_hash(config))

    labels["task"] = {"outcome": "succeeded", "reason": "TASK_SUCCEEDED"}
    labels["runtime"] = {"outcome": "failed", "reason": "RUNTIME_FAILED"}
    (export / "labels.json").write_text(json.dumps(labels))
    with pytest.raises(ExportError, match="runtime outcome"):
        _validate_export(export, config, profile_hash(config))

    labels["runtime"] = {"outcome": "stopped", "reason": "TASK_SUCCEEDED"}
    labels["recording"] = {"complete": False, "failure_reason": "missing frame"}
    (export / "labels.json").write_text(json.dumps(labels))
    with pytest.raises(ExportError, match="recording is not complete"):
        _validate_export(export, config, profile_hash(config))


def test_cache_must_be_writable_and_outside_readonly_data(config, tmp_path, monkeypatch):
    monkeypatch.delenv("LEARNING_CACHE_ROOT", raising=False)
    monkeypatch.delenv("HF_HOME", raising=False)
    data = tmp_path / "readonly-data"
    data.mkdir()
    data.chmod(0o555)
    try:
        with pytest.raises(ExportError, match="inside data root"):
            configure_local_caches(data / "cache", data_root=data)
        cache = configure_local_caches(tmp_path / "rw-cache", data_root=data)
        assert cache == (tmp_path / "rw-cache").resolve()
        assert cache.is_dir()
        assert not (data / "cache").exists()
    finally:
        data.chmod(0o755)


def test_cache_environment_override_is_also_checked_against_data(tmp_path, monkeypatch):
    data = tmp_path / "accepted"
    data.mkdir()
    monkeypatch.setenv("LEARNING_CACHE_ROOT", str(data / "wrong-cache"))
    with pytest.raises(ExportError, match="inside data root"):
        configure_local_caches(tmp_path / "otherwise-valid", data_root=data)


def test_convert_fails_fast_when_output_root_already_exists(config, tmp_path, monkeypatch):
    import dual_arm_learning.dataset as dataset_mod

    config.output_root.mkdir(parents=True)

    def _unexpected_resolve(*_args, **_kwargs):
        raise AssertionError("_resolve_episodes should not run before output_root validation")

    monkeypatch.setattr(dataset_mod, "_resolve_episodes", _unexpected_resolve)
    with pytest.raises(ExportError, match="output root already exists"):
        dataset_mod.convert(select_path=tmp_path / "missing-select.json", config=config)
