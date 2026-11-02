from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOLS_RUN = ROOT / "tools/run"
if str(TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(TOOLS_RUN))
_SPEC = importlib.util.spec_from_file_location("dsh012_mock", TOOLS_RUN / "mock.py")
assert _SPEC is not None and _SPEC.loader is not None
mock = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = mock
_SPEC.loader.exec_module(mock)

RUN_ID = "manual-001"
PROJECT = "test-project"
IMAGE = "sim-dual-arm/dev-ros:test"


def _completed(returncode: int = 0, stdout: str = "", stderr: str = ""):
    return SimpleNamespace(returncode=returncode, stdout=stdout, stderr=stderr)


def _container(
    tmp_path: Path,
    *,
    identifier: str,
    service: str,
    run_id: str = RUN_ID,
    project: str = PROJECT,
    image: str = IMAGE,
    state: str = "running",
    exit_code: int = 0,
    raw_source: Path | None = None,
    command: list[str] | None = None,
) -> dict:
    raw_source = raw_source or tmp_path / "raw"
    if command is None:
        if service == "runtime":
            command = [
                "python",
                "-m",
                "dual_arm_runtime.node",
                "--project-root",
                "/workspace",
                "--output",
                f"/data/raw/{run_id}",
                "--run-id",
                run_id,
                "--runs-output",
                "/data/runs",
            ]
        else:
            command = ["python", "-m", "mock_robot_backend.node"]
    mounts = [
        {"Destination": "/workspace", "Source": str(ROOT)},
        {"Destination": "/opt/ros-dev", "Source": str(tmp_path / "build")},
    ]
    if service == "runtime":
        mounts.extend(
            [
                {"Destination": "/data/raw", "Source": str(raw_source)},
                {"Destination": "/data/runs", "Source": str(tmp_path / "runs")},
            ]
        )
    return {
        "Id": identifier,
        "Name": f"/{project}-{service}-1",
        "Config": {
            "Image": image,
            "Cmd": command,
            "Labels": {
                "com.docker.compose.project": project,
                "com.docker.compose.service": service,
                "com.docker.compose.project.working_dir": str(ROOT),
            },
        },
        "State": {"Status": state, "ExitCode": exit_code},
        "Mounts": mounts,
    }


def _status_client(phase: str = "IDLE", episode_id: str = "") -> dict:
    return {
        "operation": "status",
        "run_id": RUN_ID,
        "success": True,
        "reason": "OK",
        "status": {
            "phase": phase,
            "episode_id": episode_id,
            "reason": "",
            "record_directory": "",
        },
        "services": {"start": True, "end": True},
        "healthy": phase != "FAULT",
        "audit_verified": False,
    }


class _FakeDocker:
    def __init__(
        self,
        *,
        project_results: list[list[str]] | None = None,
        documents: list[dict] | None = None,
        inspect_queue: list[list[dict]] | None = None,
        up_result=None,
        down_result=None,
        client_results: list[dict] | None = None,
        remove_result=None,
    ) -> None:
        self.project_results = list(project_results or [])
        self.documents = list(documents or [])
        self.inspect_queue = list(inspect_queue or [])
        self.up_result = up_result or _completed()
        self.down_result = down_result or _completed()
        self.client_results = list(client_results or [])
        self.remove_result = remove_result or _completed()
        self.calls: list[tuple[list[str], dict | None, float]] = []

    def _project(self, command):
        if self.project_results:
            return self.project_results.pop(0)
        return []

    def _inspect(self, command):
        ids = list(command[2:])
        documents = self.inspect_queue.pop(0) if self.inspect_queue else self.documents
        return [document for document in documents if not ids or document["Id"] in ids]

    def __call__(self, command, *, env, check, capture_output, text, timeout):
        self.calls.append((list(command), env, timeout))
        assert check is False
        assert capture_output is True
        assert text is True
        assert timeout > 0
        if command[1] == "ps":
            identifiers = self._project(command)
            return _completed(stdout="\n".join(identifiers))
        if command[1] == "inspect":
            documents = self._inspect(command)
            return _completed(stdout=json.dumps(documents))
        if command[1] == "rm":
            return self.remove_result
        if command[1] == "compose":
            if "up" in command:
                return self.up_result
            if "down" in command:
                return self.down_result
            if "exec" in command:
                assert "dual_arm_runtime.cli" in command
                result = self.client_results.pop(0)
                return _completed(
                    returncode=0 if result["success"] else 1,
                    stdout=json.dumps(result),
                )
        raise AssertionError(f"unexpected command: {command}")


def _argv(tmp_path: Path, operation: str, *extra: str, run_id: str = RUN_ID) -> list[str]:
    return [
        "--run-id",
        run_id,
        "--project",
        PROJECT,
        "--image",
        IMAGE,
        "--build-root",
        str(tmp_path / "build"),
        "--raw-root",
        str(tmp_path / "raw"),
        "--runs-root",
        str(tmp_path / "runs"),
        "--timeout",
        "5",
        operation,
        *extra,
    ]


def _json_output(capsys) -> dict:
    lines = [line for line in capsys.readouterr().out.splitlines() if line.strip()]
    assert len(lines) == 1, lines
    return json.loads(lines[0])


def test_timeout_rejects_bool_nan_infinity_zero_and_negative():
    for value in (True, False, 0, -1, float("nan"), float("inf"), "nan"):
        with pytest.raises(ValueError):
            mock.positive_timeout(value)
    assert mock.positive_timeout("1.5") == 1.5


def test_up_prepares_compose_waits_for_initial_idle_and_creates_only_parents(tmp_path, capsys):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    fake = _FakeDocker(
        project_results=[[], [runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
        client_results=[_status_client()],
    )

    code = mock.main(_argv(tmp_path, "up"), runner=fake)

    assert code == 0
    result = _json_output(capsys)
    assert result["success"] is True
    assert result["reason"] == "STACK_READY"
    assert result["status"]["phase"] == "IDLE"
    up_command, environment, _ = next(
        call for call in fake.calls if call[0][1] == "compose" and "up" in call[0]
    )
    assert up_command[-2:] == ["up", "-d"]
    assert environment["DEV_RUN_ID"] == RUN_ID
    exec_command = next(
        call[0] for call in fake.calls if call[0][1] == "compose" and "exec" in call[0]
    )
    assert exec_command[-1] == "status"
    assert 4.0 < float(exec_command[exec_command.index("--timeout") + 1]) <= 4.5
    assert (tmp_path / "build").is_dir()
    assert (tmp_path / "raw").is_dir()
    assert (tmp_path / "runs").is_dir()
    assert not (tmp_path / "raw" / RUN_ID).exists()
    assert not (tmp_path / "runs" / RUN_ID).exists()


def test_up_refuses_an_existing_compose_project_before_any_side_effect(tmp_path, capsys):
    fake = _FakeDocker(project_results=[["existing"]])

    code = mock.main(_argv(tmp_path, "up"), runner=fake)

    assert code == 1
    result = _json_output(capsys)
    assert result["reason"] == "PROJECT_EXISTS"
    assert [call[0][1] for call in fake.calls] == ["ps"]
    assert not (tmp_path / "build").exists()


def test_status_does_not_run_the_fresh_start_check_for_an_existing_run(tmp_path, capsys):
    (tmp_path / "raw" / RUN_ID).mkdir(parents=True)
    (tmp_path / "runs" / RUN_ID).mkdir(parents=True)
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
        client_results=[_status_client()],
    )

    code = mock.main(_argv(tmp_path, "status"), runner=fake)

    assert code == 0
    assert _json_output(capsys)["success"] is True


@pytest.mark.parametrize(
    ("change", "reason"),
    [
        ({"image": "old-image:stage1"}, "IMAGE_MISMATCH"),
        ({"project": "other-project"}, "PROJECT_MISMATCH"),
        ({"raw_source": Path("/somewhere/else")}, "MOUNT_MISMATCH"),
        (
            {"command": ["python", "-m", "dual_arm_runtime.node", "--run-id", "other"]},
            "RUN_MISMATCH",
        ),
    ],
)
def test_status_rejects_project_run_image_and_mount_mismatches(tmp_path, capsys, change, reason):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime", **change)
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
    )

    code = mock.main(_argv(tmp_path, "status"), runner=fake)

    assert code == 1
    assert _json_output(capsys)["reason"] == reason
    assert all(call[0][1] != "compose" for call in fake.calls)


def test_status_refuses_a_cached_status_when_runtime_has_exited(tmp_path, capsys):
    runtime = _container(
        tmp_path,
        identifier="a" * 64,
        service="runtime",
        state="exited",
        exit_code=17,
    )
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
    )

    code = mock.main(_argv(tmp_path, "status"), runner=fake)

    assert code == 1
    result = _json_output(capsys)
    assert result["reason"] == "RUNTIME_NOT_RUNNING"
    assert result["containers"]["runtime"]["exit_code"] == 17
    assert result["healthy"] is False
    assert all(call[0][1] != "compose" for call in fake.calls)


def test_status_rechecks_liveness_after_a_successful_client_response(tmp_path, capsys):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    exited = _container(
        tmp_path,
        identifier="a" * 64,
        service="runtime",
        state="exited",
        exit_code=9,
    )
    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
        inspect_queue=[[runtime, backend], [exited, backend]],
        client_results=[_status_client()],
    )

    code = mock.main(_argv(tmp_path, "status"), runner=fake)

    assert code == 1
    result = _json_output(capsys)
    assert result["reason"] == "RUNTIME_EXITED"
    assert result["container"]["exit_code"] == 9
    assert result["healthy"] is False


def test_status_preserves_a_client_service_failure_reason(tmp_path, capsys):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    client = {
        "operation": "status",
        "run_id": RUN_ID,
        "success": False,
        "reason": "SERVICE_UNAVAILABLE",
        "status": None,
        "services": {"start": False, "end": False},
        "audit_verified": False,
    }
    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
        client_results=[client],
    )

    code = mock.main(_argv(tmp_path, "status"), runner=fake)

    assert code == 1
    result = _json_output(capsys)
    assert result["reason"] == "SERVICE_UNAVAILABLE"
    assert result["healthy"] is False


def test_status_reports_a_runtime_fault_without_backend_failure(tmp_path, capsys):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
        client_results=[_status_client(phase="FAULT", episode_id="episode-failed")],
    )

    code = mock.main(_argv(tmp_path, "status"), runner=fake)

    assert code == 1
    result = _json_output(capsys)
    assert result["reason"] == "RUNTIME_FAULT"
    assert result["healthy"] is False
    assert result["status"]["phase"] == "FAULT"


def test_up_partial_failure_removes_only_confirmed_containers(tmp_path, capsys):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    foreign = _container(
        tmp_path,
        identifier="c" * 64,
        service="runtime",
        run_id="other-run",
    )
    fake = _FakeDocker(
        project_results=[[], [runtime["Id"], backend["Id"], foreign["Id"]]],
        documents=[runtime, backend, foreign],
        up_result=_completed(returncode=7, stderr="image missing"),
    )

    code = mock.main(_argv(tmp_path, "up"), runner=fake)

    assert code == 1
    result = _json_output(capsys)
    assert result["reason"] == "COMPOSE_UP_FAILED"
    assert result["cleanup"]["removed"] == [runtime["Id"], backend["Id"]]
    remove = next(call[0] for call in fake.calls if call[0][1] == "rm")
    assert foreign["Id"] not in remove


def test_up_backend_only_failure_can_be_followed_by_another_up(tmp_path, capsys):
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")

    failed = _FakeDocker(
        project_results=[[], [backend["Id"]]],
        documents=[backend],
        up_result=_completed(returncode=7, stderr="backend only"),
    )
    assert mock.main(_argv(tmp_path, "up"), runner=failed) == 1
    failed_result = _json_output(capsys)
    assert failed_result["cleanup"]["removed"] == [backend["Id"]]
    assert any(call[0][1] == "rm" for call in failed.calls)

    healthy = _FakeDocker(
        project_results=[[], [runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
        client_results=[_status_client()],
    )
    assert mock.main(_argv(tmp_path, "up"), runner=healthy) == 0
    assert _json_output(capsys)["reason"] == "STACK_READY"
    assert healthy.project_results == []


def test_down_fails_when_compose_returns_nonzero_even_without_containers(tmp_path, capsys):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]], [runtime["Id"], backend["Id"]], []],
        documents=[runtime, backend],
        down_result=_completed(returncode=17, stderr="network removal failed"),
    )

    code = mock.main(_argv(tmp_path, "down"), runner=fake)

    assert code == 1
    result = _json_output(capsys)
    assert result["reason"] == "COMPOSE_DOWN_FAILED"
    assert result["compose_exit_code"] == 17
    assert result["remaining_containers"] == []


def test_down_is_idempotent_and_does_not_delete_products(tmp_path, capsys):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    (tmp_path / "raw" / RUN_ID / "episode-1").mkdir(parents=True)
    (tmp_path / "runs" / RUN_ID).mkdir(parents=True)
    fake = _FakeDocker(
        project_results=[[], [runtime["Id"], backend["Id"]], []],
        documents=[runtime, backend],
    )

    first = mock.main(_argv(tmp_path, "down"), runner=fake)
    first_result = _json_output(capsys)
    assert first == 0
    assert first_result["reason"] == "STACK_ALREADY_STOPPED"
    assert all(call[0][1] != "compose" for call in fake.calls)

    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]], [runtime["Id"], backend["Id"]], []],
        documents=[runtime, backend],
    )
    second = mock.main(_argv(tmp_path, "down"), runner=fake)
    second_result = _json_output(capsys)
    assert second == 0
    assert second_result["reason"] == "STACK_STOPPED"
    assert any(call[0][1] == "compose" and "down" in call[0] for call in fake.calls)
    assert (tmp_path / "raw" / RUN_ID / "episode-1").is_dir()
    assert (tmp_path / "runs" / RUN_ID).is_dir()


def test_start_forwards_task_wait_and_shared_timeout(tmp_path, capsys):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    client = {
        "operation": "start",
        "run_id": RUN_ID,
        "success": True,
        "reason": "COMPLETED",
        "episode_id": "episode-1",
        "ack": {"success": True, "reason": "PREPARING", "episode_id": "episode-1"},
        "status": {
            "phase": "IDLE",
            "episode_id": "episode-1",
            "reason": "",
            "record_directory": "",
        },
        "manifest": {"audit_verified": False},
        "audit_verified": False,
    }
    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
        client_results=[client],
    )

    code = mock.main(_argv(tmp_path, "start", "--task-id", "mock-script", "--wait"), runner=fake)

    assert code == 0
    result = _json_output(capsys)
    assert result["success"] is True
    assert result["episode_id"] == "episode-1"
    exec_command = next(
        call[0] for call in fake.calls if call[0][1] == "compose" and "exec" in call[0]
    )
    assert exec_command[-4:] == ["start", "--task-id", "mock-script", "--wait"]
    assert 4.0 < float(exec_command[exec_command.index("--timeout") + 1]) <= 4.5


def test_end_forwards_episode_reason_and_wait(tmp_path, capsys):
    runtime = _container(tmp_path, identifier="a" * 64, service="runtime")
    backend = _container(tmp_path, identifier="b" * 64, service="backend")
    client = {
        "operation": "end",
        "run_id": RUN_ID,
        "success": True,
        "reason": "STOPPED",
        "episode_id": "episode-1",
        "ack": {"success": True, "reason": "ENDING"},
        "status": {
            "phase": "IDLE",
            "episode_id": "episode-1",
            "reason": "",
            "record_directory": "",
        },
        "manifest": {"audit_verified": False},
        "audit_verified": False,
    }
    fake = _FakeDocker(
        project_results=[[runtime["Id"], backend["Id"]]],
        documents=[runtime, backend],
        client_results=[client],
    )

    code = mock.main(
        _argv(tmp_path, "end", "--episode-id", "episode-1", "--reason", "OPERATOR_STOP", "--wait"),
        runner=fake,
    )

    assert code == 0
    assert _json_output(capsys)["reason"] == "STOPPED"
    exec_command = next(
        call[0] for call in fake.calls if call[0][1] == "compose" and "exec" in call[0]
    )
    assert exec_command[-6:] == [
        "end",
        "--episode-id",
        "episode-1",
        "--reason",
        "OPERATOR_STOP",
        "--wait",
    ]


def test_end_rejects_an_empty_episode_id_without_docker(tmp_path, capsys):
    fake = _FakeDocker()

    code = mock.main(_argv(tmp_path, "end", "--episode-id", ""), runner=fake)

    assert code == 2
    assert _json_output(capsys)["reason"] == "INVALID_ARGUMENT"
    assert fake.calls == []


def _load_cli_without_ros(monkeypatch):
    """Load the real cli module with only its ROS import surface stubbed."""
    modules = {}

    def module(name):
        value = ModuleType(name)
        modules[name] = value
        monkeypatch.setitem(sys.modules, name, value)
        return value

    rclpy = module("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.shutdown = lambda *args, **kwargs: None
    context = module("rclpy.context")
    context.Context = type("Context", (), {})
    executors = module("rclpy.executors")
    executors.SingleThreadedExecutor = type("SingleThreadedExecutor", (), {})
    node = module("rclpy.node")
    node.Node = type("Node", (), {})
    signals = module("rclpy.signals")
    signals.SignalHandlerOptions = SimpleNamespace(NO=0)
    rclpy.context, rclpy.executors, rclpy.node, rclpy.signals = (
        context,
        executors,
        node,
        signals,
    )

    api = module("robot_api_msgs")
    api_msg = module("robot_api_msgs.msg")
    api_msg.RuntimeStatus = type("RuntimeStatus", (), {})
    api_srv = module("robot_api_msgs.srv")
    api_srv.StartEpisode = type("StartEpisode", (), {})
    api_srv.EndEpisode = type("EndEpisode", (), {})
    api.msg, api.srv = api_msg, api_srv

    support = module("robot_ros_support")
    qos = module("robot_ros_support.qos")
    qos.INFO_QOS = object()
    support.qos = qos

    path = ROOT / "components/runtime/ros/dual_arm_runtime/dual_arm_runtime/cli.py"
    spec = importlib.util.spec_from_file_location("dsh012_cli_without_ros", path)
    assert spec is not None and spec.loader is not None
    cli = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, cli)
    spec.loader.exec_module(cli)
    return cli


def test_cli_main_success_path_emits_one_json_and_exits_zero(tmp_path, capsys, monkeypatch):
    cli = _load_cli_without_ros(monkeypatch)
    monkeypatch.setattr(
        cli,
        "execute",
        lambda arguments, *, raw_root: {
            "operation": "status",
            "run_id": arguments.run_id,
            "success": True,
            "reason": "OK",
            "audit_verified": False,
        },
    )

    code = cli.main(["--run-id", "manual-001", "status"], raw_root=tmp_path)

    assert code == 0
    lines = [line for line in capsys.readouterr().out.splitlines() if line.strip()]
    assert len(lines) == 1
    assert json.loads(lines[0])["success"] is True
