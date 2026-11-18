"""Parallel inference helpers: stage classification and worker preparation."""

from __future__ import annotations

import json
import os
from pathlib import Path

import yaml

from dual_arm_runtime.offline.inference_worker import (
    _classification,
    _stage_state,
    load_classification,
)
from dual_arm_runtime.offline.inference_workers import prepare_workers

_STACK_EVALUATION = (
    Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[2])
    / "shared/scenarios/panda_sim/stack/configs/runtime/evaluation.yaml"
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
    assert stage == "抓取/交接"
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
        {"schema_version": 1, "action_source": "policy", "robot_namespace": "/robot"},
    )
    backend = _write(
        project / "backend.yaml",
        {
            "robot": {"namespace": "/robot", "task_mode": "policy"},
            "run": {"realtime_speed": 0.5},
        },
    )
    directory = project / "var" / "robot" / "task" / "pool"
    manifest = prepare_workers(project, directory, 1, 120, runtime, backend, None)
    worker = json.loads(manifest.read_text())["workers"][0]
    prepared = yaml.safe_load((directory / worker["id"] / "backend.yaml").read_text())
    assert prepared["run"]["realtime_speed"] == 0.5


def test_prepare_workers_isolates_namespaces_and_domains(tmp_path):
    project = tmp_path / "project"
    (project / "var").mkdir(parents=True)
    runtime = _write(
        project / "runtime.yaml",
        {
            "schema_version": 1,
            "action_source": "policy",
            "robot_namespace": "/robot",
        },
    )
    backend = _write(
        project / "backend.yaml",
        {
            "robot": {"namespace": "/robot", "task_mode": "policy"},
            "run": {"realtime_speed": 1.0},
        },
    )
    directory = project / "var" / "panda_sim" / "stack" / "pool"
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
        backend_config = yaml.safe_load(
            (directory / worker["id"] / "backend.yaml").read_text()
        )
        assert runtime_config["robot_namespace"] == worker["namespace"]
        assert backend_config["robot"]["namespace"] == worker["namespace"]
        assert backend_config["robot"]["visualization_enabled"] is False
        assert backend_config["run"]["realtime_speed"] == 0.5
