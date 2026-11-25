"""Worker preparation consumes one explicit binding with per-worker isolation."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
import yaml
from collection_fixtures import BACKEND, ROBOT, SCENARIO_ID, TASK, build_collection_tree, sha
from dual_arm_runtime.offline.collection import record, record_pool
from dual_arm_runtime.offline.collection.campaign_ledger import CampaignError
from dual_arm_runtime.offline.collection.record_workers import prepare_workers
from dual_arm_runtime.offline.collection.scenario_selection import select_collection


def _selection(tree):
    return select_collection(
        robot=ROBOT,
        backend=BACKEND,
        task=TASK,
        shared_root=tree.shared,
        project_root=tree.repo,
        require_parallel=True,
    )


def test_workers_receive_binding_recipes_independent_identity_and_overrides(tmp_path):
    tree = build_collection_tree(tmp_path)
    selection = _selection(tree)
    manifest = prepare_workers(
        tree.repo,
        tree.repo / "output/pool",
        2,
        170,
        tree.runtime_recipe,
        tree.backend_recipe,
        1.5,
        selection,
    )
    document = json.loads(manifest.read_text())
    assert document["selection"] == selection
    workers = document["workers"]
    assert len({worker["domain_id"] for worker in workers}) == 2
    assert len({worker["namespace"] for worker in workers}) == 2
    assert len({worker["run_id_prefix"] for worker in workers}) == 2
    assert len({worker["output_root"] for worker in workers}) == 2
    for worker in workers:
        assert worker["binding"] == {
            "path": selection["binding"]["path"],
            "sha256": selection["binding"]["sha256"],
            "robot": ROBOT,
            "backend": BACKEND,
            "task": TASK,
            "scenario_id": SCENARIO_ID,
        }
        assert worker["recipes"]["runtime_teacher"]["sha256"] == sha(tree.runtime_recipe)
        assert worker["recipes"]["simulation_teacher"]["sha256"] == sha(tree.backend_recipe)
        assert worker["teacher_api"] == "marvin_teacher_api_msgs.teacher_chunk"
        assert worker["overrides"] == {
            "domain_id": worker["domain_id"],
            "namespace": worker["namespace"],
            "realtime_speed": 1.5,
            "visualization_enabled": False,
            "render_quality_enabled": True,
            "task_mode": "teacher",
            "action_source": "teacher",
        }
        runtime = yaml.safe_load((manifest.parent / worker["id"] / "runtime.yaml").read_text())
        assert runtime["robot_namespace"] == worker["namespace"]
        assert (
            worker["backend_config"]
            == "/shared/scenarios/synthetic_sim/mujoco/stack/configs/simulation_teacher.yaml"
        )
        assert worker["backend_ready"].endswith("backend-runs/backend/ready.json")
        assert worker["overrides"]["realtime_speed"] == 1.5
        assert not (tree.repo / "output/pool" / worker["id"] / "backend.yaml").exists()
        assert runtime["robot_profile"] == str(
            (tree.scenario.root / "profiles/robot.json").resolve()
        )


def test_direct_worker_manifest_keeps_explicit_recipe_identity(tmp_path):
    tree = build_collection_tree(tmp_path)
    manifest = prepare_workers(
        tree.repo,
        tree.repo / "output/pool-direct",
        1,
        180,
        tree.runtime_recipe,
        tree.backend_recipe,
        1.0,
        None,
    )
    document = json.loads(manifest.read_text())
    assert document["selection"]["mode"] == "direct"
    assert document["selection"]["binding"] is None
    worker = document["workers"][0]
    assert worker["binding"] is None
    assert worker["run_id_prefix"] == "pool-direct-worker-01"
    assert worker["recipes"]["runtime_teacher"]["path"] == str(tree.runtime_recipe.resolve())


def test_worker_plan_rejects_mixed_binding_identity(tmp_path):
    tree = build_collection_tree(tmp_path)
    manifest = prepare_workers(
        tree.repo,
        tree.repo / "output/pool",
        2,
        170,
        tree.runtime_recipe,
        tree.backend_recipe,
        1.0,
        _selection(tree),
    )
    document = json.loads(manifest.read_text())
    document["workers"][1]["binding"]["sha256"] = "0" * 64
    manifest.write_text(json.dumps(document))
    with pytest.raises(CampaignError, match="share the selected binding identity"):
        record_pool.load_worker_plan(manifest)


def test_worker_plan_rejects_tampered_recipe_identity(tmp_path):
    tree = build_collection_tree(tmp_path)
    manifest = prepare_workers(
        tree.repo,
        tree.repo / "output/pool",
        2,
        170,
        tree.runtime_recipe,
        tree.backend_recipe,
        1.0,
        _selection(tree),
    )
    document = json.loads(manifest.read_text())
    document["workers"][1]["recipes"]["runtime_teacher"]["sha256"] = "0" * 64
    manifest.write_text(json.dumps(document))
    with pytest.raises(CampaignError, match="selected runtime_teacher recipe"):
        record_pool.load_worker_plan(manifest)


def test_worker_plan_rejects_tampered_teacher_capability(tmp_path):
    tree = build_collection_tree(tmp_path)
    manifest = prepare_workers(
        tree.repo,
        tree.repo / "output/pool",
        2,
        170,
        tree.runtime_recipe,
        tree.backend_recipe,
        1.0,
        _selection(tree),
    )
    document = json.loads(manifest.read_text())
    document["workers"][1]["teacher_api"] = "other_api_msgs.teacher_chunk"
    manifest.write_text(json.dumps(document))
    with pytest.raises(CampaignError, match="share the selected Teacher capability"):
        record_pool.load_worker_plan(manifest)


def test_worker_plan_rejects_duplicate_run_prefix_or_output_root(tmp_path):
    tree = build_collection_tree(tmp_path)
    manifest = prepare_workers(
        tree.repo,
        tree.repo / "output/pool",
        2,
        170,
        tree.runtime_recipe,
        tree.backend_recipe,
        1.0,
        _selection(tree),
    )
    document = json.loads(manifest.read_text())
    document["workers"][1]["run_id_prefix"] = document["workers"][0]["run_id_prefix"]
    manifest.write_text(json.dumps(document))
    with pytest.raises(CampaignError, match="distinct run_id_prefix"):
        record_pool.load_worker_plan(manifest)


def test_record_worker_mode_passes_scenario_selection_to_pool(tmp_path, monkeypatch):
    tree = build_collection_tree(tmp_path)
    manifest = prepare_workers(
        tree.repo,
        tree.repo / "output/pool",
        1,
        170,
        tree.runtime_recipe,
        tree.backend_recipe,
        1.0,
        _selection(tree),
    )
    captured: dict = {}
    monkeypatch.setattr(
        record_pool,
        "record_parallel",
        lambda args, config: captured.update(args=vars(args), config=config) or 0,
    )
    code = record.main(
        [
            "--project-root",
            str(tree.repo),
            "--robot",
            ROBOT,
            "--backend",
            BACKEND,
            "--task",
            TASK,
            "--shared-root",
            str(tree.shared),
            "--worker-manifest",
            str(manifest),
        ]
    )
    assert code == 0
    assert captured["args"]["config"] == str(tree.campaign_recipe.resolve())
    assert captured["args"]["runtime_config"] == str(tree.runtime_recipe.resolve())
    assert captured["args"]["selection"]["binding"]["sha256"] == sha(tree.scenario.binding_path)
    assert captured["config"]["paths"]["ledger"] == (
        "output/synthetic_sim/mujoco/stack/logs/collection/attempts.jsonl"
    )


def test_parallel_pool_passes_worker_identity_and_isolation(tmp_path, monkeypatch):
    tree = build_collection_tree(tmp_path)
    manifest = prepare_workers(
        tree.repo,
        tree.repo / "output/pool",
        2,
        170,
        tree.runtime_recipe,
        tree.backend_recipe,
        1.0,
        _selection(tree),
    )
    document = json.loads(manifest.read_text())
    for worker in document["workers"]:
        log = tree.repo / "output/pool" / worker["id"] / "backend-runs/backend/log"
        log.mkdir(parents=True)
    monkeypatch.setattr(record_pool, "check_ros_sdk", lambda *args, **kwargs: 0)
    script = tmp_path / "fake_parallel.py"
    script.write_text(
        """import json
import os
from pathlib import Path
directory = Path(os.environ["CAMPAIGN_ATTEMPT_DIR"])
metadata = {
    "worker": os.environ["CAMPAIGN_WORKER_ID"],
    "domain_id": os.environ["ROS_DOMAIN_ID"],
    "scenario_id": os.environ["CAMPAIGN_SCENARIO_ID"],
    "binding": os.environ["CAMPAIGN_BINDING_SHA256"],
    "run_id_prefix": os.environ["CAMPAIGN_WORKER_RUN_ID_PREFIX"],
    "output_root": os.environ["CAMPAIGN_WORKER_OUTPUT_ROOT"],
    "runtime_config": os.environ["CAMPAIGN_RUNTIME_CONFIG"],
}
(directory / "worker.json").write_text(json.dumps(metadata))
raw = directory / "raw"
raw.mkdir()
export = directory / "export"
export.mkdir()
(raw / "manifest.json").write_text(json.dumps({
    "recording": {"complete": True},
    "runtime": {"outcome": "stopped", "reason": "TEST"},
}))
(raw / "task_result.json").write_text(
    json.dumps({"result": {"outcome": "succeeded"}})
)
(export / "quality.json").write_text(json.dumps({"qualified": True}))
""",
        encoding="utf-8",
    )
    collector_arguments = []

    def collector_command(args):
        collector_arguments.append(args)
        return [sys.executable, str(script)]

    monkeypatch.setattr(record, "_collector_command", collector_command)
    code = record.main(
        [
            "--project-root",
            str(tree.repo),
            "--robot",
            ROBOT,
            "--backend",
            BACKEND,
            "--task",
            TASK,
            "--shared-root",
            str(tree.shared),
            "--worker-manifest",
            str(manifest),
            "--count",
            "2",
        ]
    )
    assert code == 0
    assert len(collector_arguments) == 2
    assert {args.runtime_config for args in collector_arguments} == {
        Path(worker["runtime_config"]) for worker in document["workers"]
    }
    assert {args.robot_namespace for args in collector_arguments} == {
        worker["namespace"] for worker in document["workers"]
    }
    records = [
        json.loads(line)
        for line in (tree.repo / "output/synthetic_sim/mujoco/stack/logs/collection/attempts.jsonl")
        .read_text()
        .splitlines()
    ]
    reserved = [item for item in records if item["event"] == "RESERVED"]
    assert len(reserved) == 2
    facts = [
        json.loads((Path(item["attempt_dir"]) / "worker.json").read_text()) for item in reserved
    ]
    assert len({item["worker"] for item in facts}) == 2
    assert {item["domain_id"] for item in facts} == {"170", "171"}
    assert {item["scenario_id"] for item in facts} == {SCENARIO_ID}
    assert {item["binding"] for item in facts} == {sha(tree.scenario.binding_path)}
    assert len({item["run_id_prefix"] for item in facts}) == 2
    assert len({item["output_root"] for item in facts}) == 2
    assert len({item["runtime_config"] for item in facts}) == 2
    summary = json.loads((manifest.parent / "summary.json").read_text())
    assert summary["qualified"] == summary["attempted"] == 2
    assert summary["selection"]["binding"]["sha256"] == sha(tree.scenario.binding_path)
