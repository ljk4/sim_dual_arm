from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOLS_RUN = ROOT / "tools/run"
if str(TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(TOOLS_RUN))

import native_backend  # noqa: E402


def valid_arguments(tmp_path: Path, *extra: str):
    build_root = tmp_path / "ros-dev"
    build_root.mkdir(exist_ok=True)
    assets_root = tmp_path / "assets"
    assets_root.mkdir(exist_ok=True)
    summary = tmp_path / "summary.json"
    summary.write_text("{}\n")
    return [
        "--config",
        str(ROOT / "configs/simulation/backend.yaml"),
        "--assets-root",
        str(assets_root),
        "--asset-summary",
        str(summary),
        "--build-root",
        str(build_root),
        "--run-root",
        str(tmp_path / "runs"),
        "--run-id",
        "launcher-test",
        *extra,
    ]


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "-1"])
def test_timeout_must_be_finite_and_non_negative(tmp_path, value):
    with pytest.raises(native_backend.UsageError, match="finite non-negative"):
        native_backend.parse_arguments(valid_arguments(tmp_path, f"--timeout={value}"))


def test_zero_timeout_means_no_deadline(tmp_path):
    parsed, _environment = native_backend.parse_arguments(
        valid_arguments(tmp_path, "--timeout", "0")
    )
    assert parsed.timeout == 0.0
    assert parsed.project_explicit is False


def test_explicit_project_is_marked(tmp_path):
    parsed, _environment = native_backend.parse_arguments(
        valid_arguments(tmp_path, "--project", "native-explicit")
    )
    assert parsed.project == "native-explicit"
    assert parsed.project_explicit is True


def test_existing_explicit_project_is_refused_before_up(tmp_path, monkeypatch, capsys):
    parsed, environment = native_backend.parse_arguments(
        valid_arguments(tmp_path, "--project", "native-existing")
    )
    monkeypatch.setattr(
        native_backend, "parse_arguments", lambda _argv=None: (parsed, environment)
    )
    monkeypatch.setattr(
        native_backend, "_project_resources", lambda _docker, _project: ("container-id",)
    )

    def forbidden_popen(*_args, **_kwargs):
        raise AssertionError("Popen must not run for an existing explicit project")

    monkeypatch.setattr(native_backend.subprocess, "Popen", forbidden_popen)
    assert native_backend.main([]) == native_backend.EXIT_USAGE
    assert "refusing to take ownership" in capsys.readouterr().err


def test_project_query_uses_docker_label_filters(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(argv, **_kwargs):
        calls.append(list(argv))
        return subprocess.CompletedProcess(argv, 0, stdout="", stderr="")

    monkeypatch.setattr(native_backend.subprocess, "run", fake_run)
    assert native_backend._project_resources("docker", "proj") == ()
    assert calls[0][1:3] == ["ps", "-aq"]
    assert calls[1][1:3] == ["network", "ls"]
    assert all("label=com.docker.compose.project=proj" in argv for argv in calls)


def test_project_query_failure_is_explicit(monkeypatch):
    def fake_run(argv, **_kwargs):
        return subprocess.CompletedProcess(argv, 1, stdout="", stderr="daemon down")

    monkeypatch.setattr(native_backend.subprocess, "run", fake_run)
    with pytest.raises(native_backend.UsageError, match="daemon down"):
        native_backend._project_resources("docker", "proj")
