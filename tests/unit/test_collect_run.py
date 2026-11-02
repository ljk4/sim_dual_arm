from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import jsonschema
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
TOOLS_RUN = ROOT / "tools/run"
if str(TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(TOOLS_RUN))

import collect_artifacts  # noqa: E402
import collect_config  # noqa: E402
import collect_run  # noqa: E402
import mock_stack  # noqa: E402

RUN_ID = "collect-test-001"
PROJECT = "collect-test"
IMAGE = "sim-dual-arm/dev-ros:test"
STATUS_SCHEMA = json.loads(
    (ROOT / "contracts/schemas/v1/run_status.schema.json").read_text(encoding="utf-8")
)
BASE_CONFIG = json.loads((ROOT / "configs/deploy/mock_collect.json").read_text(encoding="utf-8"))


def _completed(returncode: int = 0, stdout: str = "", stderr: str = "") -> SimpleNamespace:
    return SimpleNamespace(returncode=returncode, stdout=stdout, stderr=stderr)


def _write_config(tmp_path: Path, **changes) -> Path:
    document = dict(BASE_CONFIG)
    document.update(changes)
    path = tmp_path / "collect.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def _summary(run_id: str = RUN_ID) -> dict:
    return {
        "schema_version": 1,
        "run_id": run_id,
        "component": "runtime",
        "instance_id": "runtime-instance-1",
        "state": "complete",
        "reason": None,
        "attempted": 5,
        "enqueued": 5,
        "written": 5,
        "dropped_noncritical": 0,
        "files": ["events.000001.jsonl"],
    }


def _event_lines(run_id: str, sequences: list[int], instance: str = "runtime-instance-1") -> str:
    lines = []
    for sequence in sequences:
        lines.append(
            json.dumps(
                {
                    "schema_version": 1,
                    "run_id": run_id,
                    "component": "runtime",
                    "instance_id": instance,
                    "event_seq": sequence,
                    "event": "test.event",
                }
            )
        )
    return "\n".join(lines) + "\n"


def _write_events(
    directory: Path, run_id: str, sequences: list[int], name: str = "events.000001.jsonl"
) -> None:
    (directory / name).write_text(_event_lines(run_id, sequences), encoding="utf-8")


def _audit(episode_id: str = "episode-1") -> dict:
    return {
        "schema_version": 2,
        "episode_id": episode_id,
        "source": "mock",
        "test_only": True,
        "recording": {"complete": True, "failure_reason": ""},
        "runtime": {"outcome": "completed", "reason": ""},
        "task": {"outcome": "not_evaluated", "reason": ""},
        "observations": 12,
        "frames": 12,
        "commands": 3,
        "terminal_events": 3,
        "commands_without_terminal": 0,
        "max_joint_motion": 0.2,
        "message_counts": {"a": 1},
    }


def _make_run_evidence(tmp_path: Path, run_id: str = RUN_ID) -> dict[str, Path]:
    run_dir = tmp_path / "runs" / run_id
    instance = run_dir / "logs/runtime/runtime-instance-1"
    instance.mkdir(parents=True)
    _write_events(instance, run_id, [1, 2, 3, 4, 5])
    (instance / "summary.json").write_text(json.dumps(_summary(run_id)), encoding="utf-8")
    raw = tmp_path / "raw" / run_id
    episode = raw / "episode-1"
    episode.mkdir(parents=True)
    manifest = {
        "schema_version": 2,
        "run_id": run_id,
        "episode_id": "episode-1",
        "source": "mock",
        "test_only": True,
        "recording": {"complete": True, "failure_reason": ""},
        "runtime": {"outcome": "completed", "reason": ""},
        "task": {"outcome": "not_evaluated", "reason": ""},
    }
    (episode / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return {"run_dir": run_dir, "raw_dir": raw, "episode": episode}


def _container_document(
    environment: dict[str, str],
    service: str,
    *,
    state: str = "running",
    exit_code: int = 0,
    identifier: str | None = None,
) -> dict:
    run_id = environment["DEV_RUN_ID"]
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
            "--once",
        ]
        mounts = [
            {"Destination": "/workspace", "Source": environment["DEV_REPO_ROOT"]},
            {"Destination": "/opt/ros-dev", "Source": environment["DEV_BUILD_ROOT"]},
            {"Destination": "/data/raw", "Source": environment["DEV_RAW_ROOT"]},
            {"Destination": "/data/runs", "Source": environment["DEV_RUNS_ROOT"]},
        ]
    else:
        command = ["python", "-m", "mock_robot_backend.node"]
        mounts = [
            {"Destination": "/workspace", "Source": environment["DEV_REPO_ROOT"]},
            {"Destination": "/opt/ros-dev", "Source": environment["DEV_BUILD_ROOT"]},
        ]
    return {
        "Id": identifier or ("a" * 64 if service == "runtime" else "b" * 64),
        "Name": f"/{PROJECT}-{service}-1",
        "Config": {
            "Image": environment["DEV_IMAGE"],
            "Cmd": command,
            "Labels": {
                "com.docker.compose.project": environment["DEV_COMPOSE_PROJECT"],
                "com.docker.compose.service": service,
                "com.docker.compose.project.working_dir": str(ROOT),
            },
        },
        "State": {"Status": state, "ExitCode": exit_code},
        "Mounts": mounts,
    }


class FakeDocker:
    """Small state machine for the launcher's Docker boundary, never real Docker."""

    def __init__(
        self,
        *,
        project_exists: bool = False,
        up_rc: int = 0,
        up_error: str = "compose up failed",
        runtime_exit_after: int | None = 1,
        runtime_exit_code: int = 0,
        runtime_stop_exit: int = 143,
        backend_stop_exit: int = 143,
        early_runtime_exit: int | None = None,
        early_backend_exit: int | None = None,
        disappear_runtime: bool = False,
        unknown_runtime_exit: bool = False,
        audit_summary: dict | None = None,
        audit_rc: int = 0,
        audit_timeout: bool = False,
        probe_rc: int = 0,
        cleanup_down_rc: int = 0,
        cleanup_down_removes: bool = True,
        rm_timeout: bool = False,
        on_up=None,
        command_delay: float = 0.0,
        foreign_document: dict | None = None,
    ) -> None:
        self.now = 1000.0
        self.project_exists = project_exists
        self.up_rc = up_rc
        self.up_error = up_error
        self.runtime_exit_after = runtime_exit_after
        self.runtime_exit_code = runtime_exit_code
        self.runtime_stop_exit = runtime_stop_exit
        self.backend_stop_exit = backend_stop_exit
        self.early_runtime_exit = early_runtime_exit
        self.early_backend_exit = early_backend_exit
        self.disappear_runtime = disappear_runtime
        self.unknown_runtime_exit = unknown_runtime_exit
        self.audit_summary = _audit() if audit_summary is None else audit_summary
        self.audit_rc = audit_rc
        self.audit_timeout = audit_timeout
        self.probe_rc = probe_rc
        self.cleanup_down_rc = cleanup_down_rc
        self.cleanup_down_removes = cleanup_down_removes
        self.rm_timeout = rm_timeout
        self.on_up = on_up
        self.command_delay = command_delay
        self.foreign_document = foreign_document
        self.foreign_added = False
        self.environment: dict[str, str] = {}
        self.documents: dict[str, dict] = {}
        self.network: str | None = None
        self.calls: list[list[str]] = []
        self.sleeps = 0
        self.started_mono = self.now

    def clock(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.001, float(seconds))
        self.sleeps += 1
        if (
            self.disappear_runtime
            and self.runtime_exit_after is not None
            and self.sleeps >= self.runtime_exit_after
        ):
            self.documents.pop("runtime", None)
        if (
            not self.disappear_runtime
            and self.runtime_exit_after is not None
            and self.sleeps >= self.runtime_exit_after
            and "runtime" in self.documents
            and self.documents["runtime"]["State"]["Status"] == "running"
        ):
            if self.unknown_runtime_exit:
                self.documents["runtime"]["State"] = {"Status": "dead"}
            else:
                self.documents["runtime"]["State"] = {
                    "Status": "exited",
                    "ExitCode": self.runtime_exit_code,
                }
        if (
            self.foreign_document is not None
            and not self.foreign_added
            and "runtime" in self.documents
            and self.documents["runtime"]["State"]["Status"] != "running"
        ):
            self.documents["foreign"] = self.foreign_document
            self.foreign_added = True

    def _identifiers(self) -> list[str]:
        if self.project_exists and not self.documents:
            return ["existing" * 8]
        return [document["Id"] for document in self.documents.values()]

    def _create(self, service: str, *, state: str = "running", exit_code: int = 0) -> None:
        self.documents[service] = _container_document(
            self.environment,
            service,
            state=state,
            exit_code=exit_code,
        )

    def __call__(self, argv, *, env, check, capture_output, text, timeout):
        assert argv[0] == "docker"
        command = list(argv[1:])
        self.calls.append(command)
        if self.command_delay:
            self.now += self.command_delay
        assert check is False
        assert capture_output is True
        assert text is True
        assert timeout > 0
        if command[0] == "ps":
            return _completed(stdout="\n".join(self._identifiers()))
        if command[0] == "inspect" and command[1] == "--format":
            if "State" in command[2]:
                identifier = command[3]
                for document in self.documents.values():
                    if document["Id"] == identifier:
                        return _completed(stdout=json.dumps(document["State"]))
                return _completed(returncode=1, stderr="no such object")
            return _completed(stdout="sha256:image-id")
        if command[0] == "inspect":
            identifiers = command[1:]
            documents = [
                document
                for document in self.documents.values()
                if not identifiers or document["Id"] in identifiers
            ]
            return _completed(stdout=json.dumps(documents))
        if command[0] == "image":
            return _completed(stdout="sha256:image-id")
        if command[0] == "logs":
            return _completed(
                stdout='{"exit_code": 0, "process_outcome": "succeeded", "logging_closed": true}\n'
            )
        if command[0] == "stop":
            identifier = command[-1]
            document = next(
                (item for item in self.documents.values() if item["Id"] == identifier),
                None,
            )
            if document is not None:
                document["State"] = {
                    "Status": "exited",
                    "ExitCode": (
                        self.runtime_stop_exit
                        if document["Config"]["Labels"]["com.docker.compose.service"] == "runtime"
                        else self.backend_stop_exit
                    ),
                }
            return _completed()
        if command[0] == "kill":
            document = next(
                (item for item in self.documents.values() if item["Id"] == command[-1]),
                None,
            )
            if document is not None:
                document["State"] = {"Status": "exited", "ExitCode": 137}
            return _completed()
        if command[0] == "rm":
            if self.rm_timeout:
                raise TimeoutError("rm timed out")
            for name in command[1:]:
                if name.startswith("-"):
                    continue
                for key, document in list(self.documents.items()):
                    if document["Id"] == name:
                        self.documents.pop(key, None)
            return _completed()
        if command[0] == "network":
            if "ls" in command:
                return _completed(stdout="" if self.network is None else "network-id")
            if "inspect" in command:
                endpoints = {document["Id"]: {} for document in self.documents.values()}
                return _completed(stdout=json.dumps(endpoints))
            if "rm" in command:
                self.network = None
                return _completed()
        if command[0] == "compose":
            return self._compose(command, env)
        raise AssertionError(f"unexpected docker command: {command}")

    def _compose(self, command: list[str], env: dict[str, str]):
        self.environment = env
        index = 1
        while index < len(command):
            token = command[index]
            if token in {"-p", "--project-directory", "-f", "--profile"}:
                index += 2
                continue
            if token.startswith("-"):
                index += 1
                continue
            break
        subcommand = command[index]
        rest = command[index + 1 :]
        if subcommand == "up":
            services = [token for token in rest if not token.startswith("-")]
            target = services[-1] if services else "runtime"
            if self.up_rc != 0:
                return _completed(returncode=self.up_rc, stderr=self.up_error)
            self.network = "network-id"
            if target == "backend":
                if "backend" not in self.documents:
                    self._create(
                        "backend",
                        state="exited" if self.early_backend_exit is not None else "running",
                        exit_code=self.early_backend_exit or 0,
                    )
            else:
                if "backend" not in self.documents:
                    self._create(
                        "backend",
                        state="exited" if self.early_backend_exit is not None else "running",
                        exit_code=self.early_backend_exit or 0,
                    )
                if "runtime" not in self.documents:
                    self._create(
                        "runtime",
                        state="exited" if self.early_runtime_exit is not None else "running",
                        exit_code=self.early_runtime_exit or 0,
                    )
            if self.on_up is not None:
                self.on_up(target)
            return _completed()
        if subcommand == "run":
            name_index = rest.index("--name") + 1 if "--name" in rest else None
            name = rest[name_index] if name_index is not None else "one-off"
            service = next(token for token in rest if token in {"runtime", "backend"})
            service_index = rest.index(service)
            service_command = rest[service_index + 1 :]
            if service == "backend":
                return _completed(
                    returncode=self.probe_rc,
                    stdout='{"backend_api_probe": "passed"}\n',
                )
            del name
            if "dual_arm_runtime.audit" in service_command:
                if self.audit_timeout:
                    raise TimeoutError("audit timed out")
                return _completed(
                    returncode=self.audit_rc,
                    stdout=json.dumps(self.audit_summary),
                    stderr="" if self.audit_rc == 0 else "audit failed",
                )
            raise AssertionError(f"unexpected compose run: {service_command}")
        if subcommand == "down":
            if self.cleanup_down_removes:
                self.documents.clear()
                self.network = None
            return _completed(returncode=self.cleanup_down_rc)
        raise AssertionError(f"unexpected compose subcommand: {subcommand}")


class _Logger:
    def __init__(self, directory: Path, **_kwargs) -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.events: list[str] = []

    def emit(self, event: str, **_kwargs) -> bool:
        self.events.append(event)
        return True

    def close(self) -> None:
        pass


class _CloseFailingLogger(_Logger):
    def close(self) -> None:
        raise RuntimeError("close failed")


class _EmitFailingLogger(_Logger):
    def emit(self, event: str, **_kwargs) -> bool:
        raise RuntimeError("queue full")


def _main(
    tmp_path: Path,
    fake: FakeDocker,
    *,
    config: Path | None = None,
    extra: list[str] | None = None,
    logger_factory=None,
    stop_intent=None,
) -> int:
    arguments = [
        "--run-id",
        RUN_ID,
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
        "--config",
        str(config or _write_config(tmp_path)),
        *(extra or []),
    ]
    kwargs = {}
    if logger_factory is not None:
        kwargs["logger_factory"] = logger_factory
    if stop_intent is not None:
        kwargs["stop_intent"] = stop_intent
    return collect_run.main(
        arguments,
        runner=fake,
        clock=fake.clock,
        sleeper=fake.sleep,
        **kwargs,
    )


def _output(capsys) -> dict:
    lines = [line for line in capsys.readouterr().out.splitlines() if line.strip()]
    assert lines, "no JSON summary was printed"
    return json.loads(lines[-1])


def _status(tmp_path: Path) -> dict:
    path = tmp_path / "runs" / RUN_ID / "status.json"
    return json.loads(path.read_text(encoding="utf-8"))


# ---- configuration ------------------------------------------------------------


def test_config_accepts_the_repository_recipe_and_hashes_it():
    config = collect_config.load_config(ROOT / "configs/deploy/mock_collect.json")
    assert config.work_deadline_s == 60.0
    assert config.values["command_timeout_s"] <= config.values["work_deadline_s"]
    assert len(config.sha256) == 64


@pytest.mark.parametrize(
    "change",
    [
        {"unknown": 1},
        {"work_deadline_s": True},
        {"work_deadline_s": 0},
        {"work_deadline_s": -1},
        {"work_deadline_s": float("inf")},
        {"poll_interval_s": 0},
        {"audit_timeout_s": 1000.0},
        {"diagnostic_max_bytes": 10},
        {"launcher_log_max_event_bytes": 10 * 1024 * 1024},
        {"launcher_log_queue_capacity": True},
    ],
)
def test_config_rejects_unknown_bool_nonpositive_nan_and_unreasonable_budgets(change):
    document = dict(BASE_CONFIG)
    document.update(change)
    with pytest.raises(collect_config.ConfigError):
        collect_config.parse_config(document)


def test_config_requires_every_field_and_rejects_duplicate_json_keys(tmp_path):
    missing = dict(BASE_CONFIG)
    missing.pop("work_deadline_s")
    with pytest.raises(collect_config.ConfigError):
        collect_config.parse_config(missing)
    path = tmp_path / "duplicate.json"
    body = json.dumps(BASE_CONFIG)[:-1] + ', "work_deadline_s": 10.0}'
    path.write_text(body, encoding="utf-8")
    with pytest.raises(collect_config.ConfigError):
        collect_config.load_config(path)
    nan_path = tmp_path / "nan.json"
    nan_path.write_text(json.dumps(BASE_CONFIG).replace("60.0", "NaN"), encoding="utf-8")
    with pytest.raises(collect_config.ConfigError):
        collect_config.load_config(nan_path)


def test_legacy_output_maps_to_one_raw_directory_and_rejects_collision(tmp_path):
    output = tmp_path / "raw" / "legacy-run"
    arguments = collect_run._arguments(
        [
            "--run-id",
            "legacy-run",
            "--output",
            str(output),
            "--runs-root",
            str(tmp_path / "runs"),
            "--raw-root",
            str(tmp_path / "raw"),
        ]
    )
    assert arguments.raw_root == (tmp_path / "raw").resolve()
    assert arguments.run_id == "legacy-run"
    output.mkdir(parents=True)
    with pytest.raises(collect_run.UsageError):
        collect_run._arguments(["--output", str(output)])
    with pytest.raises(collect_run.UsageError):
        collect_run._arguments(
            [
                "--output",
                str(tmp_path / "runs" / "same"),
                "--runs-root",
                str(tmp_path / "runs"),
            ]
        )


# ---- audit and evidence gates -------------------------------------------------


@pytest.mark.parametrize(
    "change",
    [
        {"schema_version": 1},
        {"episode_id": "other"},
        {"source": "real"},
        {"test_only": False},
        {"recording": {"complete": False}},
        {"runtime": {"outcome": "stopped"}},
        {"task": {"outcome": "succeeded"}},
        {"observations": 0},
        {"frames": 0},
        {"commands": 2},
        {"terminal_events": 2},
        {"commands_without_terminal": 1},
        {"max_joint_motion": 0.001},
    ],
)
def test_each_missing_audit_gate_prevents_success(change):
    summary = _audit()
    summary.update(change)
    problems = collect_artifacts.audit_problems(
        summary, episode_id="episode-1", min_motion_rad=0.05
    )
    assert problems


def test_runtime_log_counts_allow_only_accounted_noncritical_drops(tmp_path):
    evidence = _make_run_evidence(tmp_path)
    summary = _summary()
    summary.update({"attempted": 6, "enqueued": 5, "written": 5, "dropped_noncritical": 1})
    instance = evidence["run_dir"] / "logs/runtime/runtime-instance-1"
    (instance / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
    _write_events(instance, RUN_ID, [1, 3, 4, 5, 6])
    _evidence, problems = collect_artifacts.runtime_log_evidence(evidence["run_dir"], RUN_ID)
    assert problems == []
    summary.update({"dropped_noncritical": 0})
    (evidence["run_dir"] / "logs/runtime/runtime-instance-1/summary.json").write_text(
        json.dumps(summary), encoding="utf-8"
    )
    _evidence, problems = collect_artifacts.runtime_log_evidence(evidence["run_dir"], RUN_ID)
    assert "RUNTIME_LOG_COUNTS_INCONSISTENT" in problems


def test_valid_audit_summary_has_no_problems():
    assert (
        collect_artifacts.audit_problems(_audit(), episode_id="episode-1", min_motion_rad=0.05)
        == []
    )


@pytest.mark.parametrize(
    ("change", "problem"),
    [
        ({"state": "running"}, "RUNTIME_LOG_NOT_DRAINED"),
        ({"written": 4}, "RUNTIME_LOG_COUNTS_INCONSISTENT"),
        ({"run_id": "other"}, "RUNTIME_LOG_RUN_MISMATCH"),
        ({"component": "launcher"}, "RUNTIME_LOG_COMPONENT_MISMATCH"),
        ({"instance_id": "other"}, "RUNTIME_LOG_INSTANCE_MISMATCH"),
        ({"files": []}, "RUNTIME_LOG_FILES_MISSING"),
        ({"files": ["missing.jsonl"]}, "RUNTIME_LOG_FILE_MISSING"),
        ({"reason": "boom"}, "RUNTIME_LOG_HAS_FAILURE"),
    ],
)
def test_runtime_log_summary_must_be_drained_and_self_consistent(tmp_path, change, problem):
    evidence = _make_run_evidence(tmp_path)
    summary = _summary()
    summary.update(change)
    (evidence["run_dir"] / "logs/runtime/runtime-instance-1/summary.json").write_text(
        json.dumps(summary), encoding="utf-8"
    )
    _evidence, problems = collect_artifacts.runtime_log_evidence(evidence["run_dir"], RUN_ID)
    assert problem in problems


@pytest.mark.parametrize(
    ("change", "problem"),
    [
        ({"schema_version": 1}, "MANIFEST_VERSION"),
        ({"episode_id": "other"}, "MANIFEST_EPISODE_MISMATCH"),
        ({"run_id": "other"}, "MANIFEST_RUN_MISMATCH"),
        ({"source": "real"}, "MANIFEST_SOURCE"),
        ({"test_only": False}, "MANIFEST_NOT_TEST_ONLY"),
    ],
)
def test_manifest_identity_is_checked_before_audit(tmp_path, change, problem):
    evidence = _make_run_evidence(tmp_path)
    manifest_path = evidence["episode"] / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest.update(change)
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    _episode, problems = collect_artifacts.select_episode(evidence["raw_dir"], RUN_ID)
    assert problem in problems


def test_episode_selection_rejects_missing_multiple_and_bad_json(tmp_path):
    raw = tmp_path / "raw" / RUN_ID
    raw.mkdir(parents=True)
    _episode, problems = collect_artifacts.select_episode(raw, RUN_ID)
    assert problems == ["RECORDING_MISSING"]
    one = raw / "episode-1"
    one.mkdir()
    (one / "manifest.json").write_text("{", encoding="utf-8")
    _episode, problems = collect_artifacts.select_episode(raw, RUN_ID)
    assert problems and problems[0].startswith("MANIFEST_INVALID")
    (raw / "episode-2").mkdir()
    _episode, problems = collect_artifacts.select_episode(raw, RUN_ID)
    assert problems == ["RECORDING_AMBIGUOUS"]


# ---- full supervisor behavior -------------------------------------------------


def test_successful_collection_writes_succeeded_status_and_validates_schema(tmp_path, capsys):
    evidence = {}

    def on_up(_target):
        evidence.update(_make_run_evidence(tmp_path))

    fake = FakeDocker(runtime_exit_after=1, on_up=on_up)

    code = _main(tmp_path, fake)

    assert code == 0
    summary = _output(capsys)
    assert summary["status"] == "succeeded"
    status = _status(tmp_path)
    jsonschema.validate(status, STATUS_SCHEMA)
    assert status["status"] == "succeeded"
    assert status["reason"] == "OK"
    assert status["components"]["runtime"]["exit_code"] == 0
    assert status["components"]["backend"]["exit_code"] == 143
    assert status["audit"]["passed"] is True
    assert status["logs"]["runtime_drained"] is True
    assert status["logs"]["launcher_closed"] is True
    assert status["cleanup"]["complete"] is True
    assert status["task"]["outcome"] == "not_evaluated"
    assert (evidence["run_dir"] / "artifacts/audit.json").is_file()
    assert not (tmp_path / "runs/.launch" / RUN_ID).exists()
    assert any("dual_arm_runtime.audit" in call for call in fake.calls if call[0] == "compose")


def test_runtime_nonzero_exit_is_failure_and_keeps_the_real_exit_code(tmp_path, capsys):
    evidence = {}

    def on_up(_target):
        evidence.update(_make_run_evidence(tmp_path))

    fake = FakeDocker(runtime_exit_after=1, runtime_exit_code=9, on_up=on_up)

    code = _main(tmp_path, fake)

    assert code == 1
    status = _status(tmp_path)
    jsonschema.validate(status, STATUS_SCHEMA)
    assert status["status"] == "failed"
    assert status["reason"] == "RUNTIME_EXIT_NONZERO"
    assert status["components"]["runtime"]["exit_code"] == 9
    assert status["audit"]["passed"] is False
    assert status["recording"]["manifest_complete"] is True
    assert status["recording"]["complete"] is False
    assert status["runtime"]["outcome"] == "completed"


def test_unknown_runtime_exit_stays_null_instead_of_zero(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(runtime_exit_after=1, disappear_runtime=True, on_up=on_up)

    code = _main(tmp_path, fake)

    assert code == 1
    status = _status(tmp_path)
    assert status["reason"] == "RUNTIME_LOST"
    assert status["components"]["runtime"]["exit_code"] is None


def test_backend_early_exit_is_failure(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(early_backend_exit=5, runtime_exit_after=None, on_up=on_up)

    code = _main(tmp_path, fake)

    assert code == 1
    status = _status(tmp_path)
    assert status["reason"] == "BACKEND_EXITED"
    assert status["components"]["backend"]["exit_code"] == 5


def test_work_deadline_is_not_reset_by_polling_and_returns_124(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    config = _write_config(
        tmp_path,
        work_deadline_s=1.0,
        command_timeout_s=0.5,
        poll_interval_s=0.05,
        audit_timeout_s=0.2,
        stop_grace_s=1.0,
        cleanup_timeout_s=1.0,
    )
    fake = FakeDocker(runtime_exit_after=None, on_up=on_up)

    code = _main(tmp_path, fake, config=config)

    assert code == 124
    status = _status(tmp_path)
    jsonschema.validate(status, STATUS_SCHEMA)
    assert status["status"] == "failed"
    assert status["reason"] == "WORK_DEADLINE_EXCEEDED"
    assert status["exit_code"] == 124
    assert fake.now - fake.started_mono < 3.0
    assert any(call[0] == "stop" for call in fake.calls)


def test_signal_intent_takes_priority_and_stops_before_start(tmp_path, capsys):
    fake = FakeDocker()
    intent = collect_run.StopIntent(reason="SIGTERM")

    code = _main(tmp_path, fake, stop_intent=intent)

    assert code == 143
    summary = _output(capsys)
    assert summary["status"] == "interrupted"
    assert not [call for call in fake.calls if call[0] == "compose" and "up" in call]
    launch_status = tmp_path / "runs/.launch" / RUN_ID / "status.json"
    document = json.loads(launch_status.read_text(encoding="utf-8"))
    jsonschema.validate(document, STATUS_SCHEMA)
    assert document["status"] == "interrupted"


def test_runtime_identified_and_stopped_before_directory_owns_final_failure_dir(tmp_path, capsys):
    fake = FakeDocker(early_runtime_exit=1, runtime_exit_after=None)

    code = _main(tmp_path, fake)

    assert code == 1
    run_dir = tmp_path / "runs" / RUN_ID
    assert run_dir.is_dir()
    status = json.loads((run_dir / "status.json").read_text(encoding="utf-8"))
    assert status["status"] == "failed"
    assert status["components"]["runtime"]["exit_code"] == 1
    assert not (tmp_path / "runs/.launch" / RUN_ID).exists()


def test_startup_failure_without_identified_runtime_keeps_launch_evidence(tmp_path, capsys):
    fake = FakeDocker(up_rc=1)

    code = _main(tmp_path, fake)

    assert code == 1
    assert not (tmp_path / "runs" / RUN_ID).exists()
    launch = tmp_path / "runs/.launch" / RUN_ID
    assert (launch / "status.json").is_file()
    status = json.loads((launch / "status.json").read_text(encoding="utf-8"))
    jsonschema.validate(status, STATUS_SCHEMA)
    assert status["status"] == "failed"
    assert status["reason"] == "COMPOSE_UP_FAILED"


def test_existing_project_is_refused_before_creating_any_run_evidence(tmp_path, capsys):
    fake = FakeDocker(project_exists=True)

    code = _main(tmp_path, fake)

    assert code == 1
    summary = _output(capsys)
    assert summary["reason"] == "PROJECT_EXISTS"
    assert [call[0] for call in fake.calls] == ["ps"]
    assert not (tmp_path / "runs").exists()


def test_existing_run_products_are_refused_without_docker(tmp_path, capsys):
    run_dir = tmp_path / "runs" / RUN_ID
    run_dir.mkdir(parents=True)
    previous_result = b'{"operator_note":"preserve previous result"}\n'
    for name in ("status.json", "launcher.json"):
        (run_dir / name).write_bytes(previous_result)
    before = {p.relative_to(run_dir): p.read_bytes() for p in run_dir.rglob("*") if p.is_file()}
    fake = FakeDocker()

    code = _main(tmp_path, fake)

    assert code == 1
    assert fake.calls == []
    assert _output(capsys)["status_path"] is None
    after = {p.relative_to(run_dir): p.read_bytes() for p in run_dir.rglob("*") if p.is_file()}
    assert after == before


@pytest.mark.parametrize(
    ("fake_kwargs", "reason"),
    [
        ({"audit_summary": {**_audit(), "recording": {"complete": False}}}, "AUDIT_REJECTED"),
        ({"audit_rc": 3}, "AUDIT_FAILED"),
        ({"audit_timeout": True}, "AUDIT_TIMEOUT"),
    ],
)
def test_bad_audit_cannot_publish_success(tmp_path, capsys, fake_kwargs, reason):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up, **fake_kwargs)

    code = _main(tmp_path, fake)

    assert code == 1
    status = _status(tmp_path)
    assert status["status"] == "failed"
    assert status["audit"]["passed"] is False
    assert status["reason"] == reason


def test_nonzero_launcher_log_close_fails_the_run(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up)

    code = _main(tmp_path, fake, logger_factory=_CloseFailingLogger)

    assert code == 1
    status = _status(tmp_path)
    assert status["status"] == "failed"
    assert status["reason"] == "LAUNCHER_LOG_CLOSE_FAILED"
    assert status["logs"]["launcher_closed"] is False


def test_launcher_log_emit_failure_fails_the_run(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up)

    code = _main(tmp_path, fake, logger_factory=_EmitFailingLogger)

    assert code == 1
    status = _status(tmp_path)
    assert status["status"] == "failed"
    assert status["reason"] == "LAUNCHER_LOG_FAILED"
    assert status["logs"]["launcher_closed"] is True


def test_bounded_cleanup_failure_is_reported(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up, rm_timeout=True)

    code = _main(tmp_path, fake)

    assert code == 1
    status = _status(tmp_path)
    assert status["cleanup"]["complete"] is False
    assert status["reason"] == "COMMAND_TIMEOUT"


def test_probe_runs_before_once_runtime(tmp_path, capsys):
    def on_up(target):
        if target == "runtime":
            _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up)

    code = _main(tmp_path, fake, extra=["--probe-backend"])

    assert code == 0
    compose_calls = [call for call in fake.calls if call[0] == "compose"]
    probe_index = next(
        index for index, call in enumerate(compose_calls) if "probe_backend.py" in " ".join(call)
    )
    runtime_up_index = next(
        index for index, call in enumerate(compose_calls) if "up" in call and call[-1] == "runtime"
    )
    assert probe_index < runtime_up_index


def test_atomic_final_status_write_failure_returns_nonzero(tmp_path, capsys, monkeypatch):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up)
    real_write = collect_artifacts.atomic_write_json
    status_writes: list[Path] = []

    def flaky_write(path: Path, document) -> None:
        if path.name == "status.json":
            status_writes.append(path)
            if len(status_writes) >= 3:
                raise OSError("disk full")
        real_write(path, document)

    monkeypatch.setattr(collect_run.collect_artifacts, "atomic_write_json", flaky_write)

    code = _main(tmp_path, fake)

    assert code == 1
    assert "cannot write final status" in capsys.readouterr().err
    status_path = tmp_path / "runs" / RUN_ID / "status.json"
    if status_path.exists():
        # The old running marker may remain; it must never be rewritten as success.
        assert json.loads(status_path.read_text(encoding="utf-8"))["status"] == "running"


def test_backend_forced_kill_exit_137_is_not_a_normal_stop(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up, backend_stop_exit=137)

    code = _main(tmp_path, fake)

    assert code == 1
    status = _status(tmp_path)
    assert status["reason"] == "BACKEND_STOP_UNEXPECTED_EXIT"
    assert status["components"]["backend"]["exit_code"] == 137


def test_runtime_completing_before_first_poll_can_still_succeed(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(early_runtime_exit=0, runtime_exit_after=None, on_up=on_up)

    code = _main(tmp_path, fake)

    assert code == 0
    status = _status(tmp_path)
    assert status["status"] == "succeeded"
    assert status["components"]["runtime"]["exit_code"] == 0


def test_unknown_exit_code_from_container_state_stays_null(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(unknown_runtime_exit=True, on_up=on_up)

    code = _main(tmp_path, fake)

    assert code == 1
    status = _status(tmp_path)
    assert status["reason"] == "RUNTIME_EXIT_NONZERO"
    assert status["components"]["runtime"]["exit_code"] is None


def _foreign_document() -> dict:
    return {
        "Id": "f" * 64,
        "Name": "/operator-helper",
        "Config": {
            "Image": IMAGE,
            "Cmd": ["sleep", "infinity"],
            "Labels": {
                "com.docker.compose.project": PROJECT,
                "com.docker.compose.service": "operator-helper",
            },
        },
        "State": {"Status": "running", "ExitCode": 0},
        "Mounts": [],
    }


def test_runtime_log_evidence_rejects_truncated_duplicate_bad_and_foreign_lines(tmp_path):
    evidence = _make_run_evidence(tmp_path)
    instance = evidence["run_dir"] / "logs/runtime/runtime-instance-1"
    _write_events(instance, RUN_ID, [2, 3, 4, 5])
    _evidence, problems = collect_artifacts.runtime_log_evidence(evidence["run_dir"], RUN_ID)
    assert "RUNTIME_LOG_LINE_COUNT_MISMATCH" in problems

    _write_events(instance, RUN_ID, [1, 2, 2, 4, 5])
    _evidence, problems = collect_artifacts.runtime_log_evidence(evidence["run_dir"], RUN_ID)
    assert "RUNTIME_LOG_EVENT_SEQUENCE" in problems

    lines = _event_lines(RUN_ID, [1, 2]).splitlines()
    lines.append("not-json")
    lines.extend(_event_lines(RUN_ID, [4, 5]).splitlines())
    (instance / "events.000001.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    _evidence, problems = collect_artifacts.runtime_log_evidence(evidence["run_dir"], RUN_ID)
    assert "RUNTIME_LOG_EVENT_INVALID" in problems

    (instance / "events.000001.jsonl").write_text(
        _event_lines("other-run", [1, 2, 3, 4, 5]), encoding="utf-8"
    )
    _evidence, problems = collect_artifacts.runtime_log_evidence(evidence["run_dir"], RUN_ID)
    assert "RUNTIME_LOG_EVENT_RUN_MISMATCH" in problems


def test_foreign_same_project_container_is_not_removed(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up, foreign_document=_foreign_document())

    code = _main(tmp_path, fake)

    assert code == 1
    status = _status(tmp_path)
    assert status["cleanup"]["complete"] is False
    assert any(item["id"] == "f" * 64 for item in status["cleanup"]["foreign"])
    assert "foreign" in fake.documents
    assert "runtime" not in fake.documents
    assert "backend" not in fake.documents
    assert fake.network == "network-id"
    assert not any("f" * 64 in call for call in fake.calls if call[0] == "rm")
    assert not any(call[0] == "compose" and "down" in call for call in fake.calls)


def test_corrupt_mcap_preserves_the_writer_claim_without_claiming_verified_integrity(
    tmp_path, capsys
):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up, audit_rc=3)

    code = _main(tmp_path, fake)

    assert code == 1
    status = _status(tmp_path)
    assert status["status"] == "failed"
    assert status["reason"] == "AUDIT_FAILED"
    assert status["recording"]["manifest_complete"] is True
    assert status["recording"]["complete"] is False
    manifest = json.loads((Path(status["recording"]["path"]) / "manifest.json").read_text())
    assert manifest["recording"]["complete"] is True
    assert status["runtime"]["outcome"] == "completed"
    assert status["task"]["outcome"] == "not_evaluated"


def test_shared_stop_and_cleanup_budgets_are_not_renewed(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    config = _write_config(
        tmp_path,
        work_deadline_s=4.0,
        command_timeout_s=2.0,
        poll_interval_s=0.05,
        audit_timeout_s=0.5,
        stop_grace_s=2.0,
        cleanup_timeout_s=2.0,
    )
    fake = FakeDocker(
        runtime_exit_after=None,
        on_up=on_up,
        command_delay=0.8,
    )

    code = _main(tmp_path, fake, config=config)

    assert code == 124
    assert fake.now - fake.started_mono <= 15.0


def test_bounded_docker_streams_and_truncates_real_output():
    docker = mock_stack.BoundedDocker(sys.executable, output_limit=1000)
    result = docker.run(
        ["-c", "import sys; sys.stdout.write('x' * 100000); sys.stderr.write('y' * 5000)"],
        time.monotonic() + 5.0,
    )
    assert result.returncode == 0
    assert result.truncated is True
    assert len(result.stdout) <= 1000
    assert result.stdout_bytes >= 100000
    assert result.stdout == "x" * 1000
    assert result.stderr.endswith("y" * 1000)


def test_bounded_docker_kills_a_timed_out_real_process_group():
    docker = mock_stack.BoundedDocker(sys.executable, output_limit=1000)
    started = time.monotonic()
    with pytest.raises(mock_stack.HostError) as caught:
        docker.run(["-c", "import time; time.sleep(30)"], time.monotonic() + 0.5)
    assert caught.value.reason == "COMMAND_TIMEOUT"
    assert time.monotonic() - started < 3.0


def test_bounded_docker_kills_real_child_on_async_base_exception(monkeypatch):
    created = []
    original_popen = mock_stack.subprocess.Popen

    def recording_popen(*args, **kwargs):
        process = original_popen(*args, **kwargs)
        created.append(process)
        return process

    def interrupted_select(_self, timeout=None):
        raise KeyboardInterrupt

    monkeypatch.setattr(mock_stack.subprocess, "Popen", recording_popen)
    monkeypatch.setattr(mock_stack.selectors.DefaultSelector, "select", interrupted_select)
    docker = mock_stack.BoundedDocker(sys.executable, output_limit=1000)

    with pytest.raises(KeyboardInterrupt):
        docker.run(
            ["-c", "import time; time.sleep(30)"],
            time.monotonic() + 10.0,
        )

    assert created
    pid = created[0].pid
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)


def test_audit_command_uses_read_only_overlay(tmp_path, capsys):
    def on_up(_target):
        _make_run_evidence(tmp_path)

    fake = FakeDocker(on_up=on_up)

    code = _main(tmp_path, fake)

    assert code == 0
    audit_call = next(
        call for call in fake.calls if call[0] == "compose" and "dual_arm_runtime.audit" in call
    )
    assert str(ROOT / "deploy/compose/mock_audit.yaml") in audit_call
    overlay = yaml.safe_load((ROOT / "deploy/compose/mock_audit.yaml").read_text())
    volumes = overlay["services"]["runtime"]["volumes"]
    assert {volume["target"] for volume in volumes if volume.get("read_only")} >= {
        "/data/raw",
        "/data/runs",
        "/workspace",
        "/opt/ros-dev",
    }


def test_config_mapping_and_document_are_immutable_snapshots():
    config = collect_config.load_config(ROOT / "configs/deploy/mock_collect.json")
    with pytest.raises(TypeError):
        config.values["work_deadline_s"] = 1.0
    document = config.document()
    document["work_deadline_s"] = 1.0
    assert config.work_deadline_s == 60.0


def test_diagnostic_tail_records_its_own_truncation(tmp_path):
    path = tmp_path / "diag.log"
    info = collect_artifacts.write_diagnostic(path, "x" * 5000, 1000)
    data = path.read_bytes()
    assert info["truncated"] is True
    assert len(data) <= 1000
    assert b"diagnostic truncated" in data


@pytest.mark.parametrize("reader", ["config", "evidence"])
def test_json_reader_rejects_fifo_without_waiting_for_a_writer(tmp_path, reader):
    path = tmp_path / "input.json"
    os.mkfifo(path)
    read = collect_config.load_config if reader == "config" else collect_artifacts.read_json_file
    with pytest.raises(ValueError, match="regular JSON file"):
        read(path)


@pytest.mark.parametrize("reader", ["config", "evidence"])
def test_json_reader_enforces_the_byte_limit(tmp_path, reader):
    path = tmp_path / "input.json"
    if reader == "config":
        limit = collect_config.MAX_JSON_BYTES
        read = collect_config.load_config
    else:
        limit = collect_artifacts.MAX_MANIFEST_BYTES
        read = collect_artifacts.read_json_file
    path.write_bytes(b" " * (limit + 1))
    with pytest.raises(ValueError, match="larger"):
        read(path)


def test_source_digest_includes_executable_mock_backend_but_not_unrelated_tests(tmp_path):
    backend = tmp_path / "tests/doubles/ros/mock_robot_backend/mock_robot_backend/node.py"
    backend.parent.mkdir(parents=True)
    backend.write_text("# first backend version\n")
    first = collect_artifacts.source_manifest(tmp_path)["sha256"]
    backend.write_text("# changed backend version\n")
    second = collect_artifacts.source_manifest(tmp_path)["sha256"]
    assert first != second
    unrelated = tmp_path / "tests/unit/test_example.py"
    unrelated.parent.mkdir(parents=True)
    unrelated.write_text("# this is not executable application code\n")
    assert collect_artifacts.source_manifest(tmp_path)["sha256"] == second
