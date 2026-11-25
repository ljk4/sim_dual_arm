"""Parallel inference helpers: stage classification and worker preparation."""

from __future__ import annotations

import json
import os
from pathlib import Path

import yaml
from dual_arm_runtime.offline.evaluation.inference_worker import (
    _classification,
    _stage_state,
    load_classification,
)
from dual_arm_runtime.offline.evaluation.inference_workers import prepare_workers

_STACK_EVALUATION = (
    Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[3])
    / "shared/scenarios/marvin_sim/stack/configs/runtime/evaluation.yaml"
)
CONFIG = load_classification(_STACK_EVALUATION)


def _task(reason: str, conditions: dict[str, bool], state: str = "FAILED") -> dict:
    return {
        "state": state,
        "reason": reason,
        "failure_reason": reason,
        "metrics": {
            "conditions": [
                {"name": name, "passed": passed} for name, passed in conditions.items()
            ]
        },
    }


def test_success_marks_every_milestone():
    stage, _reason, reached = _stage_state(_task("", {}, state="SUCCEEDED"), CONFIG)
    assert stage == ""
    assert reached == {"grasp": True, "place": True, "release": True, "stable": True}


def test_timeout_after_all_conditions_is_stable_wait():
    task = _task(
        "CONTROL_FAILED: TASK_TIMEOUT",
        {"source_not_on_table": True, "source_on_target_height": True,
         "right_gripper_opening": True},
    )
    stage, reason, reached = _stage_state(task, CONFIG)
    assert stage == "稳定确认"
    assert reason == "CONTROL_FAILED: TASK_TIMEOUT"
    assert all(reached.values())


def test_dropped_block_is_transport_failure():
    stage, _reason, _reached = _stage_state(
        _task("DROPPED: a block was dropped or left the table", {}), CONFIG
    )
    assert stage == "搬运/放置（掉落）"


def test_failed_grasp_is_first_stage():
    stage, _reason, reached = _stage_state(
        _task("CONTROL_FAILED: TASK_TIMEOUT", {"source_not_on_table": False}), CONFIG
    )
    assert stage == "抓取/抬起"
    assert reached["grasp"] is False


def test_failed_release_after_place_is_release_stage():
    stage, _reason, reached = _stage_state(
        _task(
            "CONTROL_FAILED: TASK_TIMEOUT",
            {"source_not_on_table": True, "source_on_target_height": True,
             "right_tcp_clearance": False},
        ),
        CONFIG,
    )
    assert stage == "释放/撤离"
    assert reached["place"] is True


def test_episode_classification_reads_recorded_artifacts(tmp_path):
    recordings = tmp_path / "recordings"
    episode = recordings / "run-1"
    episode.mkdir(parents=True)
    (episode / "task_result.json").write_text(
        json.dumps({"task": _task("", {}, state="SUCCEEDED")})
    )
    (episode / "manifest.json").write_text(
        json.dumps({"recording": {"complete": True}, "runtime": {"outcome": "stopped"}})
    )
    result = _classification(recordings, "run-1", 0, CONFIG)
    assert result["succeeded"] is True
    assert result["failure_stage"] == ""
    assert result["record_directory"] == str(episode)


def test_missing_artifacts_report_execution_failure(tmp_path):
    recordings = tmp_path / "recordings"
    recordings.mkdir()
    result = _classification(recordings, "missing-run", 1, CONFIG)
    assert result["succeeded"] is False
    assert result["failure_stage"] == "执行异常"


def _write(path: Path, document: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def test_prepare_workers_uses_recipe_speed_when_unspecified(tmp_path):
    project = tmp_path / "project"
    (project / "var").mkdir(parents=True)
    runtime = _write(
        project / "runtime.yaml",
        {"action_source": "policy", "robot_namespace": "/robot"},
    )
    backend = _write(
        project / "backend.yaml",
        {
            "robot": {"namespace": "/robot", "task_mode": "inference"},
            "run": {"realtime_speed": 0.5},
        },
    )
    directory = project / "var" / "robot" / "task" / "pool"
    manifest = prepare_workers(project, directory, 1, 120, runtime, backend, None)
    worker = json.loads(manifest.read_text())["workers"][0]
    assert worker["overrides"]["realtime_speed"] == 0.5
    assert worker["backend_config"] == str(backend.resolve())
    assert worker["backend_ready"].endswith("backend-runs/backend/ready.json")
    assert not (directory / worker["id"] / "backend.yaml").exists()


def test_prepare_workers_isolates_namespaces_and_domains(tmp_path):
    project = tmp_path / "project"
    (project / "var").mkdir(parents=True)
    runtime = _write(
        project / "runtime.yaml",
        {
            "action_source": "policy",
            "robot_namespace": "/robot",
        },
    )
    backend = _write(
        project / "backend.yaml",
        {
            "robot": {"namespace": "/robot", "task_mode": "inference"},
            "run": {"realtime_speed": 1.0},
        },
    )
    directory = project / "var" / "marvin_sim" / "stack" / "pool"
    manifest = prepare_workers(
        project, directory, 2, 120, runtime, backend, 0.5
    )
    document = json.loads(manifest.read_text())
    workers = document["workers"]
    assert len(workers) == 2
    assert workers[0]["namespace"] != workers[1]["namespace"]
    assert [worker["domain_id"] for worker in workers] == [120, 121]
    for worker in workers:
        runtime_config = yaml.safe_load(Path(worker["runtime_config"]).read_text())
        assert runtime_config["robot_namespace"] == worker["namespace"]
        assert worker["backend_config"] == str(backend.resolve())
        assert worker["backend_ready"].endswith("backend-runs/backend/ready.json")
        assert worker["overrides"]["realtime_speed"] == 0.5
        assert not (directory / worker["id"] / "backend.yaml").exists()


def test_inference_pool_tty_progress_stays_on_single_line(tmp_path, monkeypatch, capsys):
    from dual_arm_runtime.offline import progress as offline_progress
    from dual_arm_runtime.offline.evaluation.inference_worker import (
        GENERIC_CLASSIFICATION,
        InferencePool,
    )

    monkeypatch.setenv("RECORD_PROGRESS_TTY", "1")
    monkeypatch.setenv("COLUMNS", "80")
    monkeypatch.setattr(
        offline_progress.os,
        "get_terminal_size",
        lambda *_a: (_ for _ in ()).throw(OSError()),
    )
    args = type(
        "Args",
        (),
        {
            "recordings": tmp_path / "recordings",
            "runs_root": tmp_path / "runs",
            "console_root": tmp_path / "console",
            "count": 10,
            "seed_start": 100,
            "batch_id": "batch-1",
            "policy_endpoint": None,
            "max_consecutive_failures": 3,
        },
    )()
    pool = InferencePool(args, [{"id": "worker-01", "domain_id": 100}], GENERIC_CLASSIFICATION)
    pool.active = 2
    pool._render_progress()
    pool.episodes.append({"succeeded": False, "seed": 100})
    pool.worker_failures["worker-01"] = 1
    pool._render_progress()
    pool.episodes.append({"succeeded": True, "seed": 101})
    pool.worker_failures["worker-01"] = 0
    pool._render_progress()

    out = capsys.readouterr().out
    assert "\n" not in out
    assert "█" not in out and "░" not in out
    frames = [seg for seg in out.split("\r\x1b[2K") if seg]
    assert len(frames) == 3
    for frame in frames:
        assert offline_progress.display_width(frame) < 80

    monkeypatch.setenv("COLUMNS", "50")
    pool._render_progress()
    narrow = capsys.readouterr().out.split("\r\x1b[2K")[-1]
    assert offline_progress.display_width(narrow) < 50
