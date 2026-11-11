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
    docker = SimpleNamespace(resolve_target=Mock(return_value=True), prepare=Mock(), cleanup=Mock())
    monkeypatch.setattr(record_parallel, "DockerWorkers", lambda *args: docker)
    assert record_parallel.main(["--project-root", str(tmp_path), "--target-total", "50",
                                 "--workers", "4"]) == 0
    docker.prepare.assert_not_called()
    docker.cleanup.assert_called_once()
