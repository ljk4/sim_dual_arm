"""Host resource selection and argument errors must not start Docker work."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from host import docker_pool, record_parallel


@pytest.mark.parametrize("cpus,memory_gib,gpu_free,expected", [
    (16, 12, 5600, 4), (4, 12, 5600, 1), (16, 12, 2560, 2), (16, 2.5, 5600, 1),
])
def test_auto_workers_respects_cpu_memory_and_gpu_headroom(
    monkeypatch, cpus, memory_gib, gpu_free, expected
):
    monkeypatch.setattr(docker_pool.os, "sched_getaffinity", lambda _: set(range(cpus)))
    monkeypatch.setattr(Path, "read_text", lambda self: f"MemAvailable: {int(memory_gib*2**20)} kB")
    monkeypatch.setattr(
        docker_pool.subprocess, "run",
        lambda *args, **kwargs: SimpleNamespace(returncode=0, stdout=f"{gpu_free}\n"),
    )
    assert record_parallel.automatic_workers() == expected


@pytest.mark.parametrize("arguments", [
    ["--count", "0"], ["--workers", "0"], ["--workers", "17"], ["--workers", "invalid"],
    ["--workers", "2", "--count", "2", "--domain-base", "232"],
    ["--realtime-speed", "nan"], ["--max-consecutive-failures", "0"],
    ["--target-total", "0"], ["--count", "1", "--target-total", "50"],
    ["--robot", "test_sim", "--task", "stack", "--set", "reset.enabled=false"],
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


@pytest.mark.parametrize("plan,wait_seconds", [(None, 35.0), ({"stop_grace_s": 45.0}, 55.0)])
def test_cleanup_honors_controller_stop_grace_before_stopping_backend(
    tmp_path, monkeypatch, plan, wait_seconds
):
    docker = record_parallel.DockerWorkers(SimpleNamespace(), tmp_path)
    docker.runtime = "runtime-id"
    docker.created = ["this-batch-worker"]
    docker.plan = plan
    docker.controller_pid.write_text("123")
    docker.controller = Mock()
    docker.controller.poll.return_value = None
    events = []
    docker.controller.wait.side_effect = lambda **kwargs: events.append("controller exited")

    def run(command, **kwargs):
        events.append(command[1])
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(record_parallel.subprocess, "run", run)
    docker.cleanup()

    docker.controller.wait.assert_called_once_with(timeout=wait_seconds)
    assert events == ["exec", "controller exited", "logs", "stop"]
    docker.controller.terminate.assert_not_called()


@pytest.mark.parametrize("current,target,remaining", [(0, 50, 50), (17, 50, 33), (50, 50, 0), (50, 60, 10)])
def test_target_total_counts_existing_formal_data(tmp_path, current, target, remaining):
    args = SimpleNamespace(
        target_total=target,
        purpose="formal",
        config="campaign.yaml",
        count=None,
    )
    docker = record_parallel.DockerWorkers(args, tmp_path)
    status = {"purposes": {"formal": {"qualified": current}}}

    assert docker.resolve_target({"campaign_status": status}) is (remaining == 0)
    assert args.count == remaining


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
                "--robot", "synthetic_sim",
                "--task", "stack",
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
        [
            "--project-root", str(tmp_path),
            "--robot", "synthetic_sim", "--task", "stack",
        ]
    ) == 2
    assert events == ["preflight", "cleanup"]
    assert not (tmp_path / "var/synthetic_sim/stack/logs/collection/record-pools").exists()


def test_preflight_runs_before_worker_preparation_and_backends(tmp_path, monkeypatch):
    events = []
    monkeypatch.setattr(
        record_parallel,
        "DockerWorkers",
        lambda args, directory: _FakeDocker(args, directory, events),
    )
    assert record_parallel.main(
        [
            "--project-root", str(tmp_path),
            "--robot", "synthetic_sim", "--task", "stack", "--workers", "1",
        ]
    ) == 0
    assert events[:2] == ["preflight", "prepare"]
    assert events.index("backend") > events.index("prepare")


def test_worker_preparation_receives_binding_identity(tmp_path, monkeypatch):
    directory = tmp_path / "var/synthetic_sim/stack/logs/collection/record-pools/pool-test"
    directory.mkdir(parents=True)
    (directory / "workers.json").write_text('{"workers": []}\n')
    args = SimpleNamespace(
        project_root=tmp_path,
        robot="synthetic_sim",
        task="stack",
        shared_root=tmp_path / "shared",
        config=None,
        runtime_config=None,
        backend_template=None,
        workers=2,
        domain_base=170,
        realtime_speed=1.0,
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
            "robot": "synthetic_sim",
            "task": "stack",
            "scenario_id": "synthetic_sim/stack",
            "binding": {"sha256": "a" * 64, "path": "/shared/scenarios/x/binding.yaml"},
        }
    }
    docker.prepare(plan)
    command = calls[-1]
    assert "--robot" in command and "synthetic_sim" in command
    assert "--task" in command and "stack" in command
    assert "--shared-root" in command and "/shared" in command
    assert "--binding-sha256" in command and "a" * 64 in command
    assert "--runtime-config" not in command
    assert "--backend-template" not in command


def test_preflight_command_converts_host_paths(tmp_path):
    args = SimpleNamespace(
        project_root=tmp_path,
        robot=None,
        task=None,
        shared_root=None,
        config="shared/scenarios/x/campaign.yaml",
        runtime_config="shared/scenarios/x/runtime.yaml",
        backend_template="shared/scenarios/x/backend.yaml",
        purpose="formal",
        target_total=None,
    )
    command = record_parallel.DockerWorkers(args, tmp_path / "pool").preflight_command()
    assert "/shared/scenarios/x/campaign.yaml" in command
    assert "/shared/scenarios/x/runtime.yaml" in command
    assert "/shared/scenarios/x/backend.yaml" in command
    assert "--parallel" in command

    args.robot = "synthetic_sim"
    args.task = "stack"
    args.shared_root = tmp_path / "shared"
    command = record_parallel.DockerWorkers(args, tmp_path / "pool").preflight_command()
    assert "--robot" in command and "synthetic_sim" in command
    assert "--task" in command and "stack" in command
    assert "/shared" in command
    assert "--config" not in command


def test_pool_and_inference_progress_include_eta_and_timestamp(tmp_path, capsys):
    from dual_arm_runtime.offline.collection import record_pool

    args = SimpleNamespace(
        project_root=tmp_path,
        count=10,
        max_consecutive_failures=3,
    )
    pool = record_pool.RecordingPool(
        args,
        {"paths": {"attempts_root": "var/x/y/datasets/raw"}, "launch": {}},
        None,  # type: ignore[arg-type]
        [{"id": "worker-01"}],
    )
    pool.started = record_pool.time.monotonic() - 12.0
    pool._render_progress(force=True)
    out = capsys.readouterr().out
    assert "用时 00:00:12 | 剩余 --:--:--" in out

    pool.qualified = 4
    pool._render_progress(force=True)
    out = capsys.readouterr().out
    assert "用时 00:00:12 | 剩余 00:00:18" in out

    pool.qualified = 10
    pool._render_progress(force=True)
    out = capsys.readouterr().out
    assert "用时 00:00:12 | 剩余 00:00:00" in out


def test_record_pool_tty_progress_is_single_line_and_fits_terminal_width(
    tmp_path, monkeypatch, capsys
):
    from dual_arm_runtime.offline import progress as offline_progress
    from dual_arm_runtime.offline.collection import record_pool

    monkeypatch.setenv("RECORD_PROGRESS_TTY", "1")
    monkeypatch.setenv("COLUMNS", "80")
    monkeypatch.setattr(offline_progress.os, "get_terminal_size", lambda *_a: (_ for _ in ()).throw(OSError()))
    args = SimpleNamespace(
        project_root=tmp_path,
        count=10,
        max_consecutive_failures=3,
    )
    pool = record_pool.RecordingPool(
        args,
        {"paths": {"attempts_root": "var/x/y/datasets/raw"}, "launch": {}},
        None,  # type: ignore[arg-type]
        [{"id": "worker-01"}],
    )
    for qualified in (0, 3, 10):
        pool.qualified = qualified
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


def test_runtime_command_forwards_tty_and_columns(tmp_path, monkeypatch):
    docker = record_parallel.DockerWorkers(SimpleNamespace(project_root=tmp_path), tmp_path)
    docker.runtime = "runtime-cid"
    monkeypatch.setattr(docker_pool.sys.stdout, "isatty", lambda: True)
    monkeypatch.setattr(docker_pool, "host_terminal_columns", lambda: 92)
    cmd = docker.runtime_command(["echo", "ok"])
    assert "RECORD_PROGRESS_TTY=1" in cmd
    assert "COLUMNS=92" in cmd


def test_inference_parallel_defaults_to_random_seed_start(tmp_path, monkeypatch):
    from host import inference_parallel

    parsed = inference_parallel.parser().parse_args(
        ["--robot", "marvin_sim", "--task", "stack"]
    )
    assert parsed.seed_start is None

    seen_seed_starts: list[int] = []
    monkeypatch.setattr(inference_parallel.secrets, "randbelow", lambda upper: 987654)

    class _FakeInferenceDocker:
        def __init__(self, args, directory, batch_id):
            seen_seed_starts.append(args.seed_start)

        def preflight(self):
            return {"selection": {"mode": "scenario", "robot": "marvin_sim", "task": "stack"}}

        def prepare(self, plan):
            return [{"id": "worker-01"}]

        def wait_ready(self, workers):
            pass

        def start_backend(self, worker):
            pass

        def run(self):
            return 0

        def cleanup(self):
            pass

    monkeypatch.setattr(inference_parallel, "DockerWorkers", _FakeInferenceDocker)
    assert (
        inference_parallel.main(
            [
                "--project-root",
                str(tmp_path),
                "--robot",
                "marvin_sim",
                "--task",
                "stack",
                "--workers",
                "1",
            ]
        )
        == 0
    )
    assert seen_seed_starts == [987654]

    with pytest.raises(SystemExit) as error:
        inference_parallel.main(
            [
                "--project-root",
                str(tmp_path),
                "--robot",
                "marvin_sim",
                "--task",
                "stack",
                "--seed-start",
                "-1",
            ]
        )
    assert error.value.code == 2
