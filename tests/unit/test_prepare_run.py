from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "runtime/ros2/scripts"))
import prepare_run  # noqa: E402


def _write(path: Path, value: object) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")
    return path


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fixture(tmp_path: Path, *, run_id: str = "run-42", endpoint: str = "learning:50051"):
    project = tmp_path / "project"
    files = {
        "profile.json": {"schema_version": 2},
        "observation.json": {"joint_names": []},
        "action.json": {"joint_names": []},
        "control.yaml": {"schema_version": 1},
        "follower.yaml": {"schema_version": 1},
    }
    paths = {}
    for name, content in files.items():
        path = project / name
        path.parent.mkdir(parents=True, exist_ok=True)
        if name.endswith(".json"):
            _write(path, content)
        else:
            path.write_text(yaml.safe_dump(content))
        paths[name] = path
    template = project / "runtime.yaml"
    template.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "robot_profile": "profile.json",
                "observation_schema": "observation.json",
                "action_schema": "action.json",
                "control_constraints": "control.yaml",
                "follower_constraints": "follower.yaml",
                "robot_namespace": "/robot",
                "action_source": "policy",
                "reset": {"enabled": True, "seed": 0},
                "policy": {
                    "endpoint": "act-policy:50051",
                    "model_bundle_digest": "FROM_VERIFIED_ACT_BUNDLE",
                    "processor_digest": "FROM_VERIFIED_ACT_BUNDLE",
                    "allow_test_policy": True,
                    "expected_test_only": True,
                    "connect_timeout_s": 1.0,
                },
            },
            sort_keys=False,
        )
    )
    binding = {
        "schema_version": 1,
        "kind": "policy-binding",
        "run_id": run_id,
        "endpoint": endpoint,
        "model": {
            "bundle_digest": "a" * 64,
            "processor_digest": "d" * 64,
        },
        "profile": {"path": str(paths["profile.json"]), "sha256": _sha(paths["profile.json"])},
        "observation_schema": {
            "path": str(paths["observation.json"]),
            "sha256": _sha(paths["observation.json"]),
        },
        "action_schema": {
            "path": str(paths["action.json"]),
            "sha256": _sha(paths["action.json"]),
        },
        "service": {
            "port": 50051,
            "task_id": "panda-stack",
            "task_text": "stack",
            "predict_deadline_s": 3.0,
            "open_deadline_s": 5.0,
            "close_deadline_s": 5.0,
            "max_message_bytes": 16777216,
        },
    }
    binding_path = _write(project / "policy_binding.json", binding)
    return project, template, binding_path, binding


def test_prepare_run_sets_known_policy_fields_and_refuses_overwrite(tmp_path):
    project, template, binding_path, _ = fixture(tmp_path)
    output = tmp_path / "out"
    manifest = prepare_run.prepare_run(
        template=template,
        binding=binding_path,
        seed=42,
        run_id="run-42",
        output=output,
        project_root=project,
    )
    runtime = yaml.safe_load((output / "runtime.yaml").read_text())
    assert runtime["policy"]["endpoint"] == "learning:50051"
    assert runtime["policy"]["model_bundle_digest"] == "a" * 64
    assert runtime["policy"]["processor_digest"] == "d" * 64
    assert runtime["policy"]["allow_test_policy"] is False
    assert runtime["policy"]["expected_test_only"] is False
    assert runtime["policy"]["task_id"] == "panda-stack"
    assert runtime["reset"] == {"enabled": True, "seed": 42}
    assert runtime["robot_profile"].startswith("/workspace/")
    assert "FROM_VERIFIED" not in (output / "runtime.yaml").read_text()
    assert manifest["kind"] == "runtime_prepare"
    assert manifest["policy"]["run_id"] == "run-42"
    assert manifest["outputs"]["runtime_sha256"] == _sha(output / "runtime.yaml")
    with pytest.raises(prepare_run.PrepareError, match="refusing to overwrite"):
        prepare_run.prepare_run(
            template=template,
            binding=binding_path,
            seed=43,
            run_id="run-42",
            output=output,
            project_root=project,
        )


def test_prepare_run_accepts_configurable_endpoint_matching_port(tmp_path):
    project, template, binding_path, _ = fixture(
        tmp_path, endpoint="policy-host.example:50051"
    )
    output = tmp_path / "custom"
    prepare_run.prepare_run(
        template=template,
        binding=binding_path,
        seed=7,
        run_id="run-42",
        output=output,
        project_root=project,
    )
    runtime = yaml.safe_load((output / "runtime.yaml").read_text())
    assert runtime["policy"]["endpoint"] == "policy-host.example:50051"


def test_prepare_run_rejects_run_id_mismatch_stale_digest_bad_port_and_placeholder(tmp_path):
    project, template, binding_path, binding = fixture(tmp_path)
    with pytest.raises(prepare_run.PrepareError, match="differs from runtime run_id"):
        prepare_run.prepare_run(
            template=template,
            binding=binding_path,
            seed=1,
            run_id="other-run",
            output=tmp_path / "out-run-id",
            project_root=project,
        )

    binding["profile"]["sha256"] = "0" * 64
    _write(binding_path, binding)
    with pytest.raises(prepare_run.PrepareError, match="file hash differs"):
        prepare_run.prepare_run(
            template=template,
            binding=binding_path,
            seed=1,
            run_id="run-42",
            output=tmp_path / "out-a",
            project_root=project,
        )

    _, _, binding_path, binding = fixture(tmp_path / "second", endpoint="learning:50052")
    with pytest.raises(prepare_run.PrepareError, match="must match endpoint port"):
        prepare_run.prepare_run(
            template=template,
            binding=binding_path,
            seed=1,
            run_id="run-42",
            output=tmp_path / "out-b",
            project_root=project,
        )

    _, template2, binding_path2, _ = fixture(tmp_path / "third")
    document = yaml.safe_load(template2.read_text())
    document["policy"]["unexpected"] = "FROM_VERIFIED_ACT_BUNDLE"
    template2.write_text(yaml.safe_dump(document))
    with pytest.raises(prepare_run.PrepareError, match="unresolved"):
        prepare_run.prepare_run(
            template=template2,
            binding=binding_path2,
            seed=1,
            run_id="run-42",
            output=tmp_path / "out-c",
            project_root=tmp_path / "third/project",
        )
