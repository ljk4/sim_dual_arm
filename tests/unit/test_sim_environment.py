from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "components/backend_sim/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim import assets  # noqa: E402
from robot_backend_sim import probe_environment as probe  # noqa: E402


def load_tool(name: str, relative: str):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


fetch_panda = load_tool("dsh014_fetch_panda", "tools/assets/fetch_panda.py")
sim_check = load_tool("dsh014_sim_check", "tools/run/sim_check.py")

COMMIT = "a" * 40
OTHER_COMMIT = "b" * 40


def source_document(
    repository: str = "https://example.invalid/menagerie.git",
    commit: str = COMMIT,
) -> dict[str, object]:
    return {
        "schema_version": 1,
        "name": "franka_emika_panda",
        "repository": repository,
        "commit": commit,
        "subdirectory": "franka_emika_panda",
        "license": "Apache-2.0",
        "license_path": "LICENSE",
    }


def write_source(path: Path, **changes: object) -> Path:
    document = source_document()
    document.update(changes)
    path.write_text(json.dumps(document))
    return path


def make_asset_tree(root: Path, subdirectory: str = "franka_emika_panda") -> Path:
    directory = root / subdirectory
    (directory / "meshes").mkdir(parents=True)
    (directory / "scene.xml").write_text('<mujoco><include file="panda.xml"/></mujoco>\n')
    (directory / "panda.xml").write_text('<mujoco model="panda"/>\n')
    (directory / "LICENSE").write_text("Apache License\n")
    (directory / "meshes/link0.stl").write_text("solid link0\n")
    return directory


def write_summary(source_path: Path, asset_root: Path, summary: Path) -> dict[str, object]:
    source = assets.load_source(source_path)
    manifest = assets.build_manifest(asset_root, source)
    document = assets.summary_document(source, manifest)
    assets.write_summary(summary, document, asset_root)
    return document


def git(cwd: Path, *arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *arguments],
        cwd=cwd,
        check=True,
        capture_output=True,
        text=True,
    )


def make_upstream(tmp_path: Path) -> tuple[Path, str]:
    repository = tmp_path / "upstream"
    make_asset_tree(repository)
    (repository / "unrelated").mkdir()
    (repository / "unrelated/ignored.txt").write_text("not part of the sparse path\n")
    git(repository, "init", "--quiet")
    git(repository, "config", "user.email", "test@example.com")
    git(repository, "config", "user.name", "Test")
    git(repository, "add", ".")
    git(repository, "commit", "--quiet", "-m", "fixed upstream")
    commit = git(repository, "rev-parse", "HEAD").stdout.strip()
    return repository, commit


def host_arguments(tmp_path: Path, run_id: str = "sim-test") -> object:
    build_root = tmp_path / "build"
    asset_root = tmp_path / "assets"
    report_root = tmp_path / "reports"
    for directory in (build_root, asset_root, report_root):
        directory.mkdir()
    summary = tmp_path / "panda.json"
    summary.write_text("{}")
    source = tmp_path / "panda-source.json"
    source.write_text("{}")
    return sim_check.Arguments(
        image="sim-dual-arm/backend-sim:mujoco-dev",
        build_root=build_root,
        assets_root=asset_root,
        summary=summary,
        report_root=report_root,
        run_id=run_id,
        project="sim-check-test",
        source=source,
        docker="docker",
        timeout_s=30.0,
        container_name="sim-check-test-container",
        cleanup_label="token-test-001",
    )


def write_host_report(
    tmp_path: Path,
    run_id: str,
    status: str,
    exit_code: int,
    *,
    reason: str | None = None,
    artifact: str = "artifacts/panda_preview.png",
) -> Path:
    report_dir = tmp_path / "reports" / run_id
    report_dir.mkdir(parents=True)
    artifact_path = report_dir / artifact
    artifact_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_path.write_bytes(b"png")
    document = {
        "schema_version": 1,
        "run_id": run_id,
        "status": status,
        "exit_code": exit_code,
        "reason": reason,
        "artifact": artifact,
        "logs_closed": status == "succeeded",
    }
    (report_dir / "report.json").write_text(json.dumps(document))
    return report_dir / "report.json"


@pytest.mark.parametrize(
    "changes",
    [
        {"commit": "abc1234"},
        {"commit": COMMIT.upper()},
        {"subdirectory": "../outside"},
        {"subdirectory": "/absolute"},
        {"license_path": "../LICENSE"},
        {"repository": "http://example.invalid/menagerie.git"},
        {"name": ""},
        {"schema_version": 2},
    ],
)
def test_source_description_rejects_unsafe_or_incomplete_values(tmp_path, changes):
    path = write_source(tmp_path / "source.json", **changes)
    with pytest.raises(assets.AssetError):
        assets.load_source(path)


def test_manifest_and_summary_detect_changed_missing_and_extra_files(tmp_path):
    asset_root = tmp_path / "assets"
    make_asset_tree(asset_root)
    source_path = write_source(tmp_path / "source.json")
    source = assets.load_source(source_path)
    summary = tmp_path / "summary.json"
    document = write_summary(source_path, asset_root, summary)

    verified = assets.verify_assets(asset_root, source, summary)
    assert verified["file_count"] == 4
    assert verified["total_sha256"] == document["total_sha256"]

    mesh = asset_root / "franka_emika_panda/meshes/link0.stl"
    mesh.write_text("solid changed\n")
    with pytest.raises(assets.AssetError, match="do not match"):
        assets.verify_assets(asset_root, source, summary)

    mesh.write_text("solid link0\n")
    (asset_root / "franka_emika_panda/scene.xml").unlink()
    with pytest.raises(assets.AssetError, match="do not match"):
        assets.verify_assets(asset_root, source, summary)

    (asset_root / "franka_emika_panda/scene.xml").write_text(
        '<mujoco><include file="panda.xml"/></mujoco>\n'
    )
    (asset_root / "franka_emika_panda/meshes/extra.stl").write_text("solid extra\n")
    with pytest.raises(assets.AssetError, match="extra="):
        assets.verify_assets(asset_root, source, summary)


def test_manifest_rejects_symlinks_and_summary_inside_checkout(tmp_path):
    asset_root = tmp_path / "assets"
    make_asset_tree(asset_root)
    source_path = write_source(tmp_path / "source.json")
    source = assets.load_source(source_path)
    document = assets.summary_document(source, assets.build_manifest(asset_root, source))

    link = asset_root / "franka_emika_panda/meshes/link-link.stl"
    link.symlink_to(asset_root / "franka_emika_panda/meshes/link0.stl")
    with pytest.raises(assets.AssetError, match="symbolic link"):
        assets.build_manifest(asset_root, source)

    with pytest.raises(assets.AssetError, match="inside the vendor checkout"):
        assets.write_summary(asset_root / "summary.json", document, asset_root)


def test_summary_rejects_tampered_total_escape_and_commit_mismatch(tmp_path):
    asset_root = tmp_path / "assets"
    make_asset_tree(asset_root)
    source_path = write_source(tmp_path / "source.json")
    source = assets.load_source(source_path)
    summary = tmp_path / "summary.json"
    document = write_summary(source_path, asset_root, summary)

    tampered = json.loads(json.dumps(document))
    tampered["total_sha256"] = "0" * 64
    summary.write_text(json.dumps(tampered))
    with pytest.raises(assets.AssetError, match="total_sha256"):
        assets.load_summary(summary)

    escaped = json.loads(json.dumps(document))
    escaped["files"][0]["path"] = "../escape.stl"
    summary.write_text(json.dumps(escaped))
    with pytest.raises(assets.AssetError, match="path"):
        assets.load_summary(summary)

    mismatched = json.loads(json.dumps(document))
    mismatched["source"]["commit"] = OTHER_COMMIT
    summary.write_text(json.dumps(mismatched))
    with pytest.raises(assets.AssetError, match="commit"):
        assets.verify_assets(asset_root, source, summary)


def test_fetch_sparse_checkout_writes_separate_summary_and_reuses_clean_tree(tmp_path):
    repository, commit = make_upstream(tmp_path)
    source_path = write_source(
        tmp_path / "source.json", repository=repository.as_uri(), commit=commit
    )
    checkout = tmp_path / "checkout"
    summary = tmp_path / "summary.json"

    first = fetch_panda.fetch(source_path, checkout, summary)
    assert first["file_count"] == 4
    assert git(checkout, "rev-parse", "HEAD").stdout.strip() == commit
    assert not (checkout / "unrelated").exists()
    assert summary.is_file()
    document = assets.load_summary(summary)
    assert document["source"]["commit"] == commit

    second = fetch_panda.fetch(source_path, checkout, summary)
    assert second["total_sha256"] == first["total_sha256"]
    assert fetch_panda.verify(source_path, checkout, summary)["file_count"] == 4


def test_fetch_refuses_foreign_or_dirty_directories_and_preserves_files(tmp_path):
    repository, commit = make_upstream(tmp_path)
    source_path = write_source(
        tmp_path / "source.json", repository=repository.as_uri(), commit=commit
    )

    foreign = tmp_path / "foreign"
    foreign.mkdir()
    keep = foreign / "operator-notes.txt"
    keep.write_text("keep\n")
    with pytest.raises(fetch_panda.FetchError, match="not this tool"):
        fetch_panda.fetch(source_path, foreign, tmp_path / "summary.json")
    assert keep.read_text() == "keep\n"

    checkout = tmp_path / "checkout"
    fetch_panda.fetch(source_path, checkout, tmp_path / "summary.json")
    mesh = checkout / "franka_emika_panda/meshes/link0.stl"
    mesh.write_text("manual edit\n")
    with pytest.raises(fetch_panda.FetchError, match="local changes"):
        fetch_panda.fetch(source_path, checkout, tmp_path / "summary.json")
    assert mesh.read_text() == "manual edit\n"

    git(repository, "commit", "--quiet", "--allow-empty", "-m", "next")
    new_commit = git(repository, "rev-parse", "HEAD").stdout.strip()
    (tmp_path / "source.json").write_text(
        json.dumps({**source_document(repository.as_uri(), new_commit)})
    )
    with pytest.raises(fetch_panda.FetchError, match="does not match fixed commit"):
        fetch_panda.fetch(source_path, checkout, tmp_path / "summary.json")
    assert mesh.read_text() == "manual edit\n"


def test_fetch_failure_preserves_only_its_staging_directory(tmp_path):
    repository, _commit = make_upstream(tmp_path)
    source_path = write_source(
        tmp_path / "source.json", repository=repository.as_uri(), commit=OTHER_COMMIT
    )
    checkout = tmp_path / "checkout"
    with pytest.raises(fetch_panda.FetchError):
        fetch_panda.fetch(source_path, checkout, tmp_path / "summary.json")
    assert not checkout.exists()
    staging = list(tmp_path.glob(".checkout.fetch-*"))
    assert len(staging) == 1
    assert (staging[0] / ".git").exists()


def test_fetch_verify_cli_returns_nonzero_for_changed_mesh(tmp_path, capsys):
    repository, commit = make_upstream(tmp_path)
    source_path = write_source(
        tmp_path / "source.json", repository=repository.as_uri(), commit=commit
    )
    checkout = tmp_path / "checkout"
    summary = tmp_path / "summary.json"
    fetch_panda.fetch(source_path, checkout, summary)
    (checkout / "franka_emika_panda/scene.xml").unlink()

    code = fetch_panda.main(
        [
            "verify",
            "--source",
            str(source_path),
            "--checkout",
            str(checkout),
            "--summary",
            str(summary),
        ]
    )
    captured = capsys.readouterr()
    assert code == 1
    assert "error:" in captured.err


def test_compose_contract_has_read_only_mounts_gpu_and_explicit_pythonpath():
    document = yaml.safe_load((ROOT / "deploy/compose/sim_check.yaml").read_text())
    service = document["services"]["sim-check"]
    environment = service["environment"]

    assert service["gpus"] == "all"
    assert service["runtime"] == "nvidia"
    assert service["network_mode"] == "none"
    assert service["restart"] == "no"
    assert service["user"].startswith("${SIM_UID")
    assert "privileged" not in service
    assert environment["MUJOCO_GL"] == "egl"
    assert environment["PYOPENGL_PLATFORM"] == "egl"
    assert environment["NVIDIA_VISIBLE_DEVICES"] == "all"
    assert environment["NVIDIA_DRIVER_CAPABILITIES"] == "compute,utility,graphics"
    assert environment["PYTHONPATH"] == "/workspace/components/backend_sim/src"
    assert "${PYTHONPATH" not in environment["PYTHONPATH"]
    assert "DISPLAY" not in environment
    assert "DOCKER_HOST" not in environment

    volumes = {volume["target"]: volume for volume in service["volumes"]}
    for target in (
        "/workspace",
        "/opt/ros-dev",
        "/sim-assets",
        "/sim-assets-summary/panda.json",
        "/sim-configs/panda_stack.yaml",
        "/sim-configs/scene_check.yaml",
        "/sim-configs/kinematics_check.yaml",
    ):
        assert volumes[target]["read_only"] is True
    assert volumes["/sim-reports"].get("read_only", False) is False
    assert not any("docker.sock" in volume["source"] for volume in service["volumes"])
    command = service["command"]
    assert "robot_backend_sim.probe_environment" in command
    assert "/workspace/deploy/docker/sim-packages.json" in command


def test_lock_and_package_record_are_consistent_and_fully_pinned():
    lock_lines = [
        line
        for line in (ROOT / "deploy/docker/sim-requirements.lock").read_text().splitlines()
        if line and not line.startswith("#") and not line.startswith("--")
    ]
    locked: dict[str, tuple[str, str]] = {}
    for line in lock_lines:
        requirement, *hashes = line.split()
        name, version = requirement.split("==")
        assert len(hashes) == 1 and hashes[0].startswith("--hash=sha256:")
        locked[name.lower()] = (version, hashes[0].removeprefix("--hash=sha256:"))

    record = json.loads((ROOT / "deploy/docker/sim-packages.json").read_text())
    packages = {entry["name"].lower(): entry for entry in record["packages"]}
    assert len(locked) == len(packages) == 15
    assert locked.keys() == packages.keys()
    for name, (version, digest) in locked.items():
        assert packages[name]["version"] == version
        assert packages[name]["sha256"] == digest


def test_probe_rejects_unsafe_run_identity_and_report_root():
    with pytest.raises(probe.ProbeUsageError):
        probe._validate_run_id("..")
    with pytest.raises(probe.ProbeUsageError):
        probe._validate_run_id("bad/path")
    with pytest.raises(probe.ProbeUsageError):
        probe._validate_report_root(Path("/"))
    with pytest.raises(probe.ProbeUsageError):
        probe._validate_report_root(Path.home())
    assert probe._validate_run_id("sim-001_OK.1") == "sim-001_OK.1"


def test_probe_package_record_must_pin_required_versions(tmp_path):
    assert probe._load_expected_packages(None) == probe._REQUIRED_VERSIONS
    actual_record = ROOT / "deploy/docker/sim-packages.json"
    actual = probe._load_expected_packages(actual_record)
    assert actual["pillow"] == "13.0.0"
    assert actual["PyOpenGL"] == "3.1.10"
    record = tmp_path / "packages.json"
    record.write_text(json.dumps({"packages": [{"name": "mujoco", "version": "3.17.0"}]}))
    with pytest.raises(probe.ProbeFailure, match="package record"):
        probe._load_expected_packages(record)


def test_compose_environment_keeps_caller_ids_and_drops_host_pythonpath(tmp_path, monkeypatch):
    arguments = host_arguments(tmp_path)
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("CODEX_HOME", str(tmp_path / "codex"))
    monkeypatch.setenv("PYTHONPATH", "/host/unsafe")

    environment = sim_check.compose_environment(arguments)

    assert environment["SIM_UID"] == str(os.getuid())
    assert environment["SIM_GID"] == str(os.getgid())
    assert environment["SIM_REPO_ROOT"] == str(ROOT)
    assert environment["HOME"] == str(tmp_path / "home")
    assert environment["CODEX_HOME"] == str(tmp_path / "codex")
    assert "PYTHONPATH" not in environment
    argv = sim_check.compose_argv(arguments)
    assert argv[-9:] == [
        "run",
        "--rm",
        "--no-deps",
        "-T",
        "--name",
        "sim-check-test-container",
        "--label",
        "io.sim-dual-arm.sim-check.token=token-test-001",
        "sim-check",
    ]
    assert "--name" in argv and "--label" in argv


def test_parse_arguments_generates_fresh_container_identity(tmp_path):
    build_root = tmp_path / "build"
    assets_root = tmp_path / "assets"
    build_root.mkdir()
    assets_root.mkdir()
    summary = tmp_path / "summary.json"
    summary.write_text("{}")
    argv = [
        "--build-root",
        str(build_root),
        "--assets-root",
        str(assets_root),
        "--summary",
        str(summary),
        "--report-root",
        str(tmp_path / "reports"),
        "--run-id",
        "identity-test",
    ]

    first = sim_check.parse_arguments(argv)
    second = sim_check.parse_arguments(argv)
    assert first.container_name.startswith("sim-check-")
    assert second.container_name.startswith("sim-check-")
    assert first.container_name != second.container_name
    assert first.cleanup_label != second.cleanup_label


def test_execute_refuses_existing_run_before_starting_docker(tmp_path, monkeypatch):
    arguments = host_arguments(tmp_path)
    (arguments.report_root / arguments.run_id).mkdir()

    def unexpected():
        raise AssertionError("docker must not be queried for an existing run")

    monkeypatch.setattr(sim_check, "_project_has_containers", unexpected)
    with pytest.raises(sim_check.UsageError, match="already has a report directory"):
        sim_check.execute(arguments)


def test_verify_report_requires_matching_exit_code_status_and_artifact(tmp_path):
    report = write_host_report(tmp_path, "sim-a", "succeeded", 0)
    assert sim_check.verify_report(report, run_id="sim-a", process_exit=0)[:2] == (
        "succeeded",
        "",
    )
    assert sim_check.verify_report(report, run_id="sim-a", process_exit=1)[1] == (
        "report_success_but_process_failed"
    )
    unclosed = write_host_report(tmp_path, "sim-e", "succeeded", 0)
    document = json.loads(unclosed.read_text())
    document["logs_closed"] = False
    unclosed.write_text(json.dumps(document))
    assert sim_check.verify_report(unclosed, run_id="sim-e", process_exit=0)[1] == (
        "report_logs_not_closed"
    )
    missing = sim_check.verify_report(tmp_path / "missing.json", run_id="sim-a", process_exit=0)
    assert missing[1] == "report_missing"

    failed = write_host_report(tmp_path, "sim-b", "failed", 1, reason="opengl_not_nvidia")
    assert sim_check.verify_report(failed, run_id="sim-b", process_exit=1)[:2] == (
        "failed",
        "opengl_not_nvidia",
    )
    assert sim_check.verify_report(failed, run_id="sim-b", process_exit=0)[1] == (
        "report_failed_but_process_succeeded"
    )
    inconsistent = write_host_report(tmp_path, "sim-d", "failed", 0, reason="boom")
    assert sim_check.verify_report(inconsistent, run_id="sim-d", process_exit=1)[1] == (
        "report_failed_but_exit_zero"
    )

    escaped = write_host_report(tmp_path, "sim-c", "succeeded", 0, artifact="../../outside.png")
    assert sim_check.verify_report(escaped, run_id="sim-c", process_exit=0)[1] == (
        "report_artifact_outside_run"
    )


def test_verify_report_rejects_fifo_symlink_and_oversized_regular_file(tmp_path):
    fifo = tmp_path / "report.fifo"
    os.mkfifo(fifo)
    started = time.monotonic()
    status, reason, _document = sim_check.verify_report(fifo, run_id="sim-fifo", process_exit=0)
    assert status == "mismatch"
    assert reason == "report_not_regular"
    assert time.monotonic() - started < 1.0

    target = write_host_report(tmp_path, "sim-link", "succeeded", 0)
    link = tmp_path / "report-link.json"
    link.symlink_to(target)
    assert sim_check.verify_report(link, run_id="sim-link", process_exit=0)[1] == ("report_symlink")

    oversized = tmp_path / "report-large.json"
    oversized.write_bytes(b"x" * (sim_check.REPORT_JSON_LIMIT + 1))
    assert sim_check.verify_report(oversized, run_id="sim-large", process_exit=0)[1] == (
        "report_too_large"
    )


def test_execute_does_not_trust_success_report_when_process_fails(tmp_path, monkeypatch):
    arguments = host_arguments(tmp_path)
    report_dir = arguments.report_root / arguments.run_id
    report_dir.mkdir()
    artifact = report_dir / "artifacts/panda_preview.png"
    artifact.parent.mkdir()
    artifact.write_bytes(b"png")
    (report_dir / "report.json").write_text(
        json.dumps(
            {
                "run_id": arguments.run_id,
                "status": "succeeded",
                "exit_code": 0,
                "artifact": "artifacts/panda_preview.png",
            }
        )
    )
    monkeypatch.setattr(sim_check, "_project_has_containers", lambda _arguments: False)
    monkeypatch.setattr(
        sim_check,
        "_run_process",
        lambda _arguments, timeout_s: (sim_check.CommandResult(1, "", "failed"), False),
    )
    monkeypatch.setattr(sim_check, "cleanup_container", lambda _arguments: [])
    original_lexists = sim_check.os.path.lexists
    monkeypatch.setattr(
        sim_check.os.path,
        "lexists",
        lambda path: False if Path(path) == report_dir else original_lexists(path),
    )

    outcome = sim_check.execute(arguments)
    assert outcome.exit_code == 1
    assert outcome.reason == "report_success_but_process_failed"


def test_execute_timeout_cleans_up_the_single_container(tmp_path, monkeypatch):
    arguments = host_arguments(tmp_path)
    cleaned: list[object] = []
    monkeypatch.setattr(sim_check, "_project_has_containers", lambda _arguments: False)
    monkeypatch.setattr(
        sim_check,
        "_run_process",
        lambda _arguments, timeout_s: (sim_check.CommandResult(124, "", ""), True),
    )
    monkeypatch.setattr(
        sim_check,
        "cleanup_container",
        lambda value: cleaned.append(value) or [],
    )

    outcome = sim_check.execute(arguments)
    assert outcome.exit_code == 124
    assert outcome.reason == "timeout"
    assert cleaned == [arguments]


def test_execute_cleanup_failure_cannot_return_success(tmp_path, monkeypatch):
    arguments = host_arguments(tmp_path)
    monkeypatch.setattr(sim_check, "_project_has_containers", lambda _arguments: False)

    def run_success(actual, timeout_s):
        write_host_report(tmp_path, actual.run_id, "succeeded", 0)
        return sim_check.CommandResult(0, "", ""), False

    monkeypatch.setattr(sim_check, "_run_process", run_success)
    monkeypatch.setattr(sim_check, "cleanup_container", lambda _arguments: ["remove failed"])

    outcome = sim_check.execute(arguments)
    assert outcome.exit_code == 1
    assert outcome.reason == "cleanup_failed"


class RecordingDocker:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls: list[list[str]] = []

    def run(self, arguments, deadline, *, environment=None):
        self.calls.append(list(arguments))
        return self.responses.pop(0)


def test_cleanup_removes_only_matching_named_token_container(tmp_path):
    arguments = host_arguments(tmp_path)
    labels = json.dumps({sim_check.CLEANUP_LABEL_KEY: arguments.cleanup_label})
    fake = RecordingDocker(
        [
            SimpleNamespace(
                returncode=0,
                stdout=f"{'a' * 64}\n/{arguments.container_name}\n{labels}\n",
                stderr="",
            ),
            SimpleNamespace(returncode=0, stdout="", stderr=""),
        ]
    )

    assert sim_check.cleanup_container(arguments, docker=fake) == []
    assert fake.calls == [
        [
            "inspect",
            "--format",
            "{{.Id}}\n{{.Name}}\n{{json .Config.Labels}}",
            arguments.container_name,
        ],
        ["rm", "-f", "a" * 64],
    ]


def test_cleanup_preserves_absent_or_foreign_containers_in_same_project(tmp_path):
    arguments = host_arguments(tmp_path)
    absent = RecordingDocker(
        [
            SimpleNamespace(
                returncode=1,
                stdout="",
                stderr=f"Error: No such object: {arguments.container_name}",
            )
        ]
    )
    assert sim_check.cleanup_container(arguments, docker=absent) == []
    assert absent.calls[0][0] == "inspect"
    # A foreign operator helper may share the Compose project label; this
    # cleanup path never lists or removes by project, so it stays untouched.
    rendered = " ".join(" ".join(call) for call in absent.calls)
    assert "operator-helper" not in rendered
    assert " ps " not in f" {rendered} " and " down " not in f" {rendered} "

    wrong_token = RecordingDocker(
        [
            SimpleNamespace(
                returncode=0,
                stdout=(
                    f"{'b' * 64}\n/{arguments.container_name}\n"
                    f"{json.dumps({sim_check.CLEANUP_LABEL_KEY: 'another-run'})}\n"
                ),
                stderr="",
            )
        ]
    )
    diagnostics = sim_check.cleanup_container(arguments, docker=wrong_token)
    assert diagnostics and "preserved" in diagnostics[0]
    assert all(call[0] != "rm" for call in wrong_token.calls)


def test_bounded_docker_caps_each_stream_for_two_mib_real_output():
    limit = 256 * 1024
    docker = sim_check.mock_stack.BoundedDocker(sys.executable, output_limit=limit)
    result = docker.run(
        [
            "-c",
            "import sys; "
            "sys.stdout.write('x' * (2 * 1024 * 1024)); "
            "sys.stderr.write('y' * (2 * 1024 * 1024))",
        ],
        time.monotonic() + 10.0,
    )
    assert result.returncode == 0
    assert result.truncated is True
    assert len(result.stdout) <= limit
    assert len(result.stderr) <= limit
    assert result.stdout_bytes >= 2 * 1024 * 1024
    assert result.stderr_bytes >= 2 * 1024 * 1024


def test_probe_log_directory_uses_its_actual_instance_identity(tmp_path, monkeypatch):
    arguments = probe.ProbeArguments(
        tmp_path, tmp_path / "summary.json", tmp_path / "source.json", tmp_path, "identity"
    )
    monkeypatch.setattr(
        probe,
        "check_dependencies",
        lambda _path: {"python": {"version": "3.12"}, "packages": {}, "imports": {}},
    )
    monkeypatch.setattr(probe, "_verify_mounted_source", lambda *_args: None)
    monkeypatch.setattr(
        probe,
        "_check_assets",
        lambda _args: {
            "verification": {"commit": "a" * 40, "file_count": 1, "total_sha256": "b" * 64}
        },
    )
    monkeypatch.setattr(probe, "_check_model", lambda _args: {"stats": {"nq": 9, "nv": 9, "nu": 8}})
    monkeypatch.setattr(probe, "_check_qp", lambda: {"max_abs_error": 0.0})
    monkeypatch.setattr(
        probe,
        "_render_preview",
        lambda *_args: {
            "artifact": "preview.png",
            "vendor": "fixture",
            "renderer": "fixture",
            "unique_colors": 32,
        },
    )

    assert probe.run(arguments) == 0
    report = json.loads((tmp_path / "identity/report.json").read_text())
    assert Path(report["log_directory"]).name == report["instance_id"]
    events = list((tmp_path / "identity" / report["log_directory"]).glob("events*.jsonl"))
    assert events
    assert all(
        json.loads(line)["instance_id"] == report["instance_id"]
        for path in events
        for line in path.read_text().splitlines()
    )


@pytest.mark.parametrize("relative", ["operator-notes.txt", "franka_emika_panda/.DS_Store"])
def test_fetch_preserves_changes_anywhere_in_owned_checkout(tmp_path, relative):
    repository, _commit = make_upstream(tmp_path)
    (repository / ".gitignore").write_text(".DS_Store\n")
    git(repository, "add", ".gitignore")
    git(repository, "commit", "--quiet", "-m", "ignore generated metadata")
    commit = git(repository, "rev-parse", "HEAD").stdout.strip()
    source = write_source(tmp_path / "source.json", repository=repository.as_uri(), commit=commit)
    checkout = tmp_path / "checkout"
    summary = tmp_path / "summary.json"
    fetch_panda.fetch(source, checkout, summary)
    before = summary.read_bytes()
    operator_file = checkout / relative
    operator_file.write_text("operator edit\n")

    with pytest.raises(fetch_panda.FetchError) as error:
        fetch_panda.fetch(source, checkout, summary)
    assert error.value.reason == "checkout_dirty"
    assert operator_file.read_text() == "operator edit\n"
    assert summary.read_bytes() == before


def test_host_check_mode_kinematics_uses_fixed_command_and_mount(tmp_path):
    build_root = tmp_path / "build"
    assets_root = tmp_path / "assets"
    for directory in (build_root, assets_root):
        directory.mkdir()
    summary = tmp_path / "summary.json"
    summary.write_text("{}")
    base = [
        "--build-root",
        str(build_root),
        "--assets-root",
        str(assets_root),
        "--summary",
        str(summary),
        "--report-root",
        str(tmp_path / "reports"),
        "--run-id",
        "kinematics-entry",
    ]

    arguments = sim_check.parse_arguments([*base, "--check", "kinematics"])
    assert arguments.kinematics_config == ROOT / "configs/simulation/kinematics_check.yaml"
    assert sim_check.compose_environment(arguments)["SIM_KINEMATICS_CONFIG"] == str(
        arguments.kinematics_config
    )
    assert sim_check.compose_environment(arguments)["SIM_CHECK_CONFIG"] == str(
        arguments.check_config
    )
    argv = sim_check.compose_argv(arguments)
    assert argv[-1] == "kinematics-entry"
    assert "robot_backend_sim.check_kinematics" in argv
    for value in (
        "/sim-configs/panda_stack.yaml",
        "/sim-configs/scene_check.yaml",
        "/sim-configs/kinematics_check.yaml",
        "/sim-reports",
    ):
        assert value in argv
    assert "robot_backend_sim.probe_environment" not in argv
    assert argv.count("--run-id") == 1


def kinematics_host_report(tmp_path: Path, run_id: str, *, kind: str | None) -> Path:
    report_dir = tmp_path / "reports" / run_id
    artifact = report_dir / "artifacts/overview.png"
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(b"png")
    document = {
        "run_id": run_id,
        "status": "succeeded",
        "exit_code": 0,
        "logs_closed": True,
        "artifact": "artifacts/overview.png",
    }
    if kind is not None:
        document["kind"] = kind
    report = report_dir / "report.json"
    report.write_text(json.dumps(document))
    return report


def test_host_report_verification_requires_kinematics_kind(tmp_path):
    missing = kinematics_host_report(tmp_path, "kinematics-a", kind=None)
    assert (
        sim_check.verify_report(missing, run_id="kinematics-a", process_exit=0, kind="kinematics")[
            1
        ]
        == "report_kind_mismatch"
    )
    report = kinematics_host_report(tmp_path, "kinematics-b", kind="kinematics")
    assert sim_check.verify_report(
        report, run_id="kinematics-b", process_exit=0, kind="kinematics"
    )[:2] == ("succeeded", "")
