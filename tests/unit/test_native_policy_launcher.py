import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOLS_RUN = ROOT / "tools/run"
if str(TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(TOOLS_RUN))

import native_policy  # noqa: E402


def valid_arguments(tmp_path: Path, *extra: str):
    build_root = tmp_path / "ros-dev"
    build_root.mkdir(exist_ok=True)
    assets_root = tmp_path / "assets"
    assets_root.mkdir(exist_ok=True)
    sdk_root = tmp_path / "policy-sdk"
    sdk_root.mkdir(exist_ok=True)
    summary = tmp_path / "summary.json"
    summary.write_text("{}\n")
    return [
        "--config",
        str(ROOT / "configs/runtime/native_policy.yaml"),
        "--policy-config",
        str(ROOT / "configs/learning/fake_policy.yaml"),
        "--policy-sdk",
        str(sdk_root),
        "--assets-root",
        str(assets_root),
        "--asset-summary",
        str(summary),
        "--build-root",
        str(build_root),
        "--data-root",
        str(tmp_path / "data"),
        "--run-id",
        "policy-launcher-test",
        *extra,
    ]


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "0", "-1"])
def test_timeout_is_finite_and_positive(tmp_path, value):
    with pytest.raises(native_policy.UsageError, match="finite positive"):
        native_policy.parse_arguments(valid_arguments(tmp_path, f"--timeout={value}"))


def test_parse_forwards_three_compose_inputs_and_readonly_sdk(tmp_path):
    parsed, environment = native_policy.parse_arguments(
        valid_arguments(tmp_path, "--project", "policy-explicit")
    )
    assert parsed.project == "policy-explicit"
    assert parsed.project_explicit is True
    assert environment["NATIVE_RUNTIME_CONFIG_CONTAINER"] == (
        "/workspace/configs/runtime/native_policy.yaml"
    )
    assert environment["NATIVE_POLICY_CONFIG_CONTAINER"] == (
        "/workspace/configs/learning/fake_policy.yaml"
    )
    assert environment["NATIVE_POLICY_SDK_ROOT"].endswith("policy-sdk")
    assert parsed.run_root.is_dir()


def test_existing_explicit_project_is_refused_before_up(tmp_path, monkeypatch, capsys):
    parsed, environment = native_policy.parse_arguments(
        valid_arguments(tmp_path, "--project", "policy-existing")
    )
    monkeypatch.setattr(native_policy, "parse_arguments", lambda _argv=None: (parsed, environment))
    monkeypatch.setattr(
        native_policy.native_backend,
        "_project_resources",
        lambda _docker, _project: ("container-id",),
    )

    def forbidden_popen(*_args, **_kwargs):
        raise AssertionError("Popen must not run for an existing explicit project")

    monkeypatch.setattr(native_policy.subprocess, "Popen", forbidden_popen)
    assert native_policy.main([]) == native_policy.EXIT_USAGE
    assert "refusing to take ownership" in capsys.readouterr().err


def test_compose_argv_has_all_three_files_and_project(tmp_path, monkeypatch):
    calls = []

    def fake_run(argv, **_kwargs):
        calls.append(list(argv))
        return subprocess.CompletedProcess(argv, 0, stdout="", stderr="")

    monkeypatch.setattr(native_policy.subprocess, "run", fake_run)
    native_policy._compose("docker", "proj", ["config"], {})
    argv = calls[0]
    assert argv.count("-f") == 3
    assert "native_backend.yaml" in " ".join(argv)
    assert "native_runtime.yaml" in " ".join(argv)
    assert "native_policy.yaml" in " ".join(argv)
    assert argv[-2:] == ["-p", "proj"] or "-p" in argv


def test_teacher_stack_uses_task_backend_without_fake_policy(tmp_path):
    _, environment = native_policy.parse_arguments(
        valid_arguments(
            tmp_path,
            "--stack-mode",
            "teacher",
            "--backend-config",
            str(ROOT / "configs/simulation/backend_stack_teacher.yaml"),
            "--episode-id",
            "reserved-pilot-episode",
        )
    )
    files = native_policy._compose_files(environment)
    assert files[-1].name == "native_task.yaml"
    assert all(path.name != "native_policy.yaml" for path in files)
    assert environment["NATIVE_CONFIG_CONTAINER"].endswith("backend_stack_teacher.yaml")
    assert environment["NATIVE_EPISODE_ID"] == "reserved-pilot-episode"


def test_act_mode_selects_independent_overlay_and_readonly_learning_mounts(tmp_path):
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    venv = tmp_path / "venv"
    venv.mkdir()
    source = tmp_path / "lerobot"
    source.mkdir()
    cache = tmp_path / "cache"
    cache.mkdir()
    parsed, environment = native_policy.parse_arguments(
        valid_arguments(
            tmp_path,
            "--policy-mode",
            "act",
            "--bundle",
            str(bundle),
            "--learning-venv",
            str(venv),
            "--learning-source",
            str(source),
            "--learning-cache",
            str(cache),
        )
    )
    assert parsed.policy_mode == "act"
    files = native_policy._compose_files(environment)
    assert [path.name for path in files] == [
        "native_backend.yaml",
        "native_runtime.yaml",
        "native_act_policy.yaml",
    ]
    assert environment["NATIVE_ACT_BUNDLE"] == str(bundle.resolve())
    assert environment["NATIVE_ACT_BUNDLE_CONTAINER"] == "/opt/act-bundle"
    assert environment["NATIVE_LEARNING_VENV"] == str(venv.resolve())
    assert environment["NATIVE_ACT_POLICY_CONFIG_CONTAINER"].startswith("/workspace/")


def test_act_mode_without_bundle_is_refused(tmp_path):
    with pytest.raises(native_policy.UsageError, match="--bundle"):
        native_policy.parse_arguments(
            valid_arguments(tmp_path, "--policy-mode", "act")
        )
