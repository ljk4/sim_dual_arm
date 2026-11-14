"""Host resource selection and argument errors must not start Docker work."""

import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from dual_arm_runtime.offline import record_parallel


@pytest.mark.parametrize("cpus,memory_gib,gpu_free,expected", [
    (16, 12, 5600, 4), (4, 12, 5600, 1), (16, 12, 2560, 2), (16, 2.5, 5600, 1),
])
def test_auto_workers_respects_cpu_memory_and_gpu_headroom(
    monkeypatch, cpus, memory_gib, gpu_free, expected
):
    monkeypatch.setattr(record_parallel.os, "sched_getaffinity", lambda _: set(range(cpus)))
    monkeypatch.setattr(Path, "read_text", lambda self: f"MemAvailable: {int(memory_gib*2**20)} kB")
    monkeypatch.setattr(
        record_parallel.subprocess, "run",
        lambda *args, **kwargs: SimpleNamespace(returncode=0, stdout=f"{gpu_free}\n"),
    )
    assert record_parallel.automatic_workers() == expected


@pytest.mark.parametrize("arguments", [
    ["--count", "0"], ["--workers", "0"], ["--workers", "17"], ["--workers", "invalid"],
    ["--workers", "2", "--count", "2", "--domain-base", "232"],
    ["--realtime-speed", "nan"], ["--max-consecutive-failures", "0"],
    ["--target-total", "0"], ["--count", "1", "--target-total", "50"],
])
def test_invalid_options_fail_before_starting_containers(monkeypatch, arguments):
    prepare = Mock()
    monkeypatch.setattr(record_parallel.DockerWorkers, "prepare", prepare)
    with pytest.raises(SystemExit) as error:
        record_parallel.main(arguments)
    assert error.value.code == 2
    prepare.assert_not_called()


def test_cleanup_stops_own_container_even_if_log_capture_fails(tmp_path, monkeypatch):
    docker = record_parallel.DockerWorkers(SimpleNamespace(), tmp_path)
    docker.created = ["this-batch-worker"]
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if command[:2] == ["docker", "logs"]:
            raise OSError("disk unavailable")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(record_parallel.subprocess, "run", run)
    with pytest.raises(OSError, match="disk unavailable"):
        docker.cleanup()
    assert ["docker", "stop", "--timeout", "20", "this-batch-worker"] in calls


@pytest.mark.parametrize("current,remaining", [(0, 50), (17, 33), (50, 0)])
def test_target_total_counts_existing_formal_data(tmp_path, monkeypatch, current, remaining):
    args = SimpleNamespace(
        target_total=50,
        purpose="formal",
        config="campaign.yaml",
        count=None,
    )
    docker = record_parallel.DockerWorkers(args, tmp_path)
    docker.runtime = "runtime-id"
    status = {"formal_target_qualified": 50, "purposes": {"formal": {"qualified": current}}}
    run = Mock(return_value=SimpleNamespace(stdout=json.dumps(status)))
    monkeypatch.setattr(record_parallel.subprocess, "run", run)

    assert docker.resolve_target() is (remaining == 0)
    assert args.count == remaining
    assert "status" in run.call_args.args[0]


def test_target_total_cannot_exceed_formal_campaign_target(tmp_path, monkeypatch):
    args = SimpleNamespace(
        target_total=51,
        purpose="formal",
        config="campaign.yaml",
        count=None,
    )
    docker = record_parallel.DockerWorkers(args, tmp_path)
    docker.runtime = "runtime-id"
    status = {"formal_target_qualified": 50, "purposes": {"formal": {"qualified": 0}}}
    monkeypatch.setattr(record_parallel.subprocess, "run",
                        Mock(return_value=SimpleNamespace(stdout=json.dumps(status))))
    with pytest.raises(RuntimeError, match="exceeds"):
        docker.resolve_target()


def test_already_reached_target_does_not_start_simulation_workers(tmp_path, monkeypatch):
    plan = {"selection": {"mode": "direct"}, "campaign_status": {}}
    docker = SimpleNamespace(
        preflight=Mock(return_value=plan),
        resolve_target=Mock(return_value=True),
        prepare=Mock(),
        cleanup=Mock(),
    )
    monkeypatch.setattr(record_parallel, "DockerWorkers", lambda *args: docker)
    assert record_parallel.main(
        [
            "--project-root", str(tmp_path),
            "--config", "campaign.yaml",
            "--runtime-config", "runtime.yaml",
            "--backend-template", "backend.yaml",
            "--target-total", "50",
            "--workers", "4",
        ]
    ) == 0
    docker.preflight.assert_called_once()
    docker.prepare.assert_not_called()
    docker.cleanup.assert_called_once()


def test_host_container_path_mapping_is_explicit(tmp_path):
    root = tmp_path
    assert record_parallel.host_path_to_container("shared/x", root) == "/shared/x"
    assert record_parallel.host_path_to_container(root / "var/y", root) == "/data/y"
    assert record_parallel.host_path_to_container(root / "runtime/ros2/z", root) == "/app/z"
    assert record_parallel.container_path_to_host("/data/y", root) == (root / "var/y").resolve()
    assert record_parallel.container_path_to_host("/shared/z", root) == (
        root / "shared/z"
    ).resolve()
    assert record_parallel.container_path_to_host("/app/w", root) == (
        root / "runtime/ros2/w"
    ).resolve()
    assert record_parallel.shared_root_to_container(None, root) == "/shared"
    with pytest.raises(ValueError, match="outside the mounted project root"):
        record_parallel.host_path_to_container("/etc/passwd", root)
    with pytest.raises(ValueError, match="not mounted"):
        record_parallel.container_path_to_host("/opt/ros", root)
    with pytest.raises(ValueError, match="outside the mounted project root"):
        record_parallel.shared_root_to_container("/shared", root)


def test_scenario_conflicts_with_explicit_recipes_before_docker(tmp_path, monkeypatch):
    preflight = Mock()
    prepare = Mock()
    monkeypatch.setattr(record_parallel.DockerWorkers, "preflight", preflight)
    monkeypatch.setattr(record_parallel.DockerWorkers, "prepare", prepare)
    with pytest.raises(SystemExit) as error:
        record_parallel.main(
            [
                "--project-root", str(tmp_path),
                "--scenario", "synthetic_stack",
                "--config", "campaign.yaml",
            ]
        )
    assert error.value.code == 2
    preflight.assert_not_called()
    prepare.assert_not_called()
    with pytest.raises(SystemExit) as error:
        record_parallel.main(
            ["--project-root", str(tmp_path), "--shared-root", str(tmp_path / "shared")]
        )
    assert error.value.code == 2


class _FakeDocker:
    def __init__(self, args, directory, events, failure=None):
        self.args, self.directory = args, directory
        self.events, self.failure = events, failure

    def preflight(self):
        self.events.append("preflight")
        if self.failure:
            raise RuntimeError(self.failure)
        return {"selection": {"mode": "direct"}, "campaign_status": {}}

    def prepare(self, plan):
        self.events.append("prepare")
        return [{"id": "worker-01"}]

    def start_backend(self, worker):
        self.events.append("backend")

    def wait_ready(self, workers):
        self.events.append("ready")

    def run(self, plan):
        self.events.append("run")
        return 0

    def cleanup(self):
        self.events.append("cleanup")


def test_preflight_failure_starts_no_worker_or_pool_directory(tmp_path, monkeypatch):
    events = []
    monkeypatch.setattr(
        record_parallel,
        "DockerWorkers",
        lambda args, directory: _FakeDocker(
            args, directory, events, failure="preflight failed: bad binding"
        ),
    )
    assert record_parallel.main(
        ["--project-root", str(tmp_path), "--scenario", "synthetic_stack"]
    ) == 2
    assert events == ["preflight", "cleanup"]
    assert not (tmp_path / "var/synthetic_stack/logs/record-pools").exists()


def test_preflight_runs_before_worker_preparation_and_backends(tmp_path, monkeypatch):
    events = []
    monkeypatch.setattr(
        record_parallel,
        "DockerWorkers",
        lambda args, directory: _FakeDocker(args, directory, events),
    )
    assert record_parallel.main(
        ["--project-root", str(tmp_path), "--scenario", "synthetic_stack", "--workers", "1"]
    ) == 0
    assert events[:2] == ["preflight", "prepare"]
    assert events.index("backend") > events.index("prepare")


def test_worker_preparation_receives_binding_identity(tmp_path, monkeypatch):
    directory = tmp_path / "var/synthetic_stack/logs/record-pools/pool-test"
    directory.mkdir(parents=True)
    (directory / "workers.json").write_text('{"workers": []}\n')
    args = SimpleNamespace(
        project_root=tmp_path,
        scenario="synthetic_stack",
        shared_root=tmp_path / "shared",
        config=None,
        runtime_config=None,
        backend_template=None,
        workers=2,
        domain_base=170,
        realtime_speed=1.0,
        overrides=[],
    )
    docker = record_parallel.DockerWorkers(args, directory)
    docker.runtime = "runtime-id"
    calls = []
    monkeypatch.setattr(
        record_parallel.subprocess,
        "run",
        lambda command, **kwargs: calls.append(command)
        or SimpleNamespace(returncode=0, stdout="", stderr=""),
    )
    plan = {
        "selection": {
            "mode": "scenario",
            "scenario_id": "synthetic_stack",
            "binding": {"sha256": "a" * 64, "path": "/shared/scenarios/x/binding.yaml"},
        }
    }
    docker.prepare(plan)
    command = calls[-1]
    assert "--scenario" in command and "synthetic_stack" in command
    assert "--shared-root" in command and "/shared" in command
    assert "--binding-sha256" in command and "a" * 64 in command
    assert "--runtime-config" not in command
    assert "--backend-template" not in command


def test_preflight_command_converts_host_paths(tmp_path):
    args = SimpleNamespace(
        project_root=tmp_path,
        scenario=None,
        shared_root=None,
        config="shared/scenarios/x/campaign.yaml",
        runtime_config="shared/scenarios/x/runtime.yaml",
        backend_template="shared/scenarios/x/backend.yaml",
        purpose="formal",
        target_total=None,
        overrides=[],
    )
    command = record_parallel.DockerWorkers(args, tmp_path / "pool").preflight_command()
    assert "/shared/scenarios/x/campaign.yaml" in command
    assert "/shared/scenarios/x/runtime.yaml" in command
    assert "/shared/scenarios/x/backend.yaml" in command
    assert "--parallel" in command

    args.scenario = "synthetic_stack"
    args.shared_root = tmp_path / "shared"
    command = record_parallel.DockerWorkers(args, tmp_path / "pool").preflight_command()
    assert "--scenario" in command and "synthetic_stack" in command
    assert "/shared" in command
    assert "--config" not in command
