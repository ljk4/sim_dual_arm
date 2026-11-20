import json
from pathlib import Path

import pytest
import yaml
from dual_arm_runtime.offline.campaign_ledger import CampaignError  # noqa: E402
from dual_arm_runtime.offline.record_workers import prepare_workers  # noqa: E402

PROFILE_REL = "shared/scenarios/marvin_sim/stack/profiles/profile.json"


@pytest.fixture
def templates(tmp_path):
    runtime = tmp_path / "runtime.yaml"
    runtime.write_text(yaml.safe_dump({"action_source": "teacher",
                                      "teacher_api": "marvin_stack_api_msgs.teacher_chunk",
                                      "robot_namespace": "/robot",
                                      "robot_profile": PROFILE_REL}))
    backend = tmp_path / "backend.yaml"
    backend.write_text(yaml.safe_dump({
        "robot": {"namespace": "/robot", "task_mode": "teacher",
                  "profile": PROFILE_REL, "scene": "simulation/mujoco/scene.yaml",
                  "render_quality": {"enabled": True}},
        "run": {"realtime_speed": 1.0},
    }))
    return tmp_path, runtime, backend


def test_prepared_workers_isolate_domains_configs_and_disable_visualization(templates):
    root, runtime, backend = templates
    manifest = prepare_workers(root, root / "var/pool", 2, 70, runtime, backend, 1.0)
    workers = json.loads(manifest.read_text())["workers"]
    assert {item["domain_id"] for item in workers} == {70, 71}
    assert len({item["namespace"] for item in workers}) == 2
    assert len({item["backend_log"] for item in workers}) == 2
    for item in workers:
        runtime_config = yaml.safe_load(Path(item["runtime_config"]).read_text())
        backend_config = yaml.safe_load(
            (Path(item["runtime_config"]).parent / "backend.yaml").read_text()
        )
        assert runtime_config["robot_namespace"] == backend_config["robot"]["namespace"]
        assert backend_config["robot"]["visualization_enabled"] is False
        assert backend_config["robot"]["render_quality"]["enabled"] is True
        assert backend_config["robot"]["profile"] == f"/{PROFILE_REL}"
        assert backend_config["robot"]["scene"] == "/app/scene.yaml"
        assert runtime_config["robot_profile"] == str(root / PROFILE_REL)


@pytest.mark.parametrize("workers,domain,speed", [(0, 70, 1.0), (2, 232, 1.0),
                                                 (2, 70, float("nan")), (2, 70, 0.1)])
def test_invalid_worker_options_do_not_create_output(templates, workers, domain, speed):
    root, runtime, backend = templates
    with pytest.raises(CampaignError):
        prepare_workers(root, root / "var/pool", workers, domain, runtime, backend, speed)
    assert not (root / "var/pool").exists()


def test_preparation_refuses_to_overwrite_existing_pool(templates):
    root, runtime, backend = templates
    prepare_workers(root, root / "var/pool", 2, 70, runtime, backend, 1.0)
    with pytest.raises(FileExistsError):
        prepare_workers(root, root / "var/pool", 2, 70, runtime, backend, 1.0)
