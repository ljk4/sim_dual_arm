from __future__ import annotations

import importlib.util
import json
import os
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def load_tool(name: str, relative: str):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


dev_workspace = load_tool("dsh011_dev_workspace", "infrastructure/scripts/dev_workspace.py")


def write_contract(project: Path, body: str = "int32 value\n") -> Path:
    source = project / "contracts/ros/robot_api_msgs/msg"
    source.mkdir(parents=True, exist_ok=True)
    (source / "Example.msg").write_text(body)
    (project / "contracts/ros/robot_api_msgs/package.xml").write_text(
        "<package format='3'><name>robot_api_msgs</name></package>"
    )
    return source


def create_generated_install(build_root: Path) -> None:
    install = build_root / "install"
    package = install / "robot_api_msgs/lib/python3.12/site-packages/robot_api_msgs"
    for relative in ("", "msg", "srv", "action"):
        directory = package / relative if relative else package
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "__init__.py").write_text("# generated\n")
    install.mkdir(parents=True, exist_ok=True)
    (install / "setup.bash").write_text("# generated setup\n")


def successful_runner(project: Path, build_root: Path) -> int:
    create_generated_install(build_root)
    return 0


def failed_runner(project: Path, build_root: Path) -> int:
    return 7


def test_contract_digest_is_stable_and_changes_with_content(tmp_path):
    write_contract(tmp_path)
    first, count = dev_workspace.source_digest_details(tmp_path / "contracts/ros")
    second = dev_workspace.contract_digest(tmp_path)

    assert first == second
    assert count == 2

    write_contract(tmp_path, "int32 value\n# changed\n")
    changed = dev_workspace.contract_digest(tmp_path)
    assert changed != first


def test_digest_includes_relative_paths_not_just_file_contents(tmp_path):
    write_contract(tmp_path)
    first = dev_workspace.contract_digest(tmp_path)
    source = tmp_path / "contracts/ros/robot_api_msgs/msg/Example.msg"
    target = source.with_name("Renamed.msg")
    source.rename(target)

    assert dev_workspace.contract_digest(tmp_path) != first


def test_successful_build_writes_marker_and_verify_accepts_it(tmp_path):
    write_contract(tmp_path)
    document = dev_workspace.build_workspace(
        tmp_path, tmp_path / "output", runner=successful_runner
    )

    marker = tmp_path / "output/contracts.json"
    assert marker.is_file()
    assert json.loads(marker.read_text()) == document
    assert dev_workspace.verify_workspace(tmp_path, tmp_path / "output") == document


def test_source_change_during_build_is_reported_as_stale(tmp_path):
    write_contract(tmp_path)

    def racing_runner(project, build_root):
        create_generated_install(build_root)
        write_contract(project, "int32 changed while building\n")
        return 0

    document = dev_workspace.build_workspace(tmp_path, tmp_path / "output", runner=racing_runner)

    assert document["contracts"]["sha256"] != dev_workspace.contract_digest(tmp_path)
    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        dev_workspace.verify_workspace(tmp_path, tmp_path / "output")
    assert "stale" in str(caught.value)


def test_verify_rejects_changed_contract_sources(tmp_path):
    write_contract(tmp_path)
    dev_workspace.build_workspace(tmp_path, tmp_path / "output", runner=successful_runner)
    write_contract(tmp_path, "int32 changed\n")

    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        dev_workspace.verify_workspace(tmp_path, tmp_path / "output")
    assert "stale" in str(caught.value)
    assert dev_workspace.BUILD_COMMAND in str(caught.value)


def test_verify_rejects_missing_or_damaged_generated_modules(tmp_path):
    write_contract(tmp_path)
    build_root = tmp_path / "output"
    dev_workspace.build_workspace(tmp_path, build_root, runner=successful_runner)
    module = build_root / "install/robot_api_msgs/lib/python3.12/site-packages/robot_api_msgs"
    (module / "msg/__init__.py").unlink()

    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        dev_workspace.verify_workspace(tmp_path, build_root)
    assert "robot_api_msgs.msg" in str(caught.value)

    (module / "msg/__init__.py").write_text("# generated\n")
    (build_root / "contracts.json").write_text("{not json")
    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        dev_workspace.verify_workspace(tmp_path, build_root)
    assert "unreadable" in str(caught.value)


def test_check_cli_reports_stale_contracts_and_rebuild_command(tmp_path, capsys):
    write_contract(tmp_path)
    dev_workspace.build_workspace(tmp_path, tmp_path / "output", runner=successful_runner)
    write_contract(tmp_path, "int32 changed\n")

    code = dev_workspace.main(
        ["check", "--project-root", str(tmp_path), "--build-root", str(tmp_path / "output")]
    )

    captured = capsys.readouterr()
    assert code == 1
    assert "stale" in captured.err
    assert dev_workspace.BUILD_COMMAND in captured.err


def test_verify_rejects_missing_marker(tmp_path):
    write_contract(tmp_path)
    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        dev_workspace.verify_workspace(tmp_path, tmp_path / "output")
    assert "missing" in str(caught.value)
    assert dev_workspace.BUILD_COMMAND in str(caught.value)


def test_failed_build_removes_previous_marker(tmp_path):
    write_contract(tmp_path)
    build_root = tmp_path / "output"
    dev_workspace.build_workspace(tmp_path, build_root, runner=successful_runner)
    assert (build_root / "contracts.json").is_file()

    with pytest.raises(dev_workspace.DevWorkspaceError) as caught:
        dev_workspace.build_workspace(tmp_path, build_root, runner=failed_runner)
    assert "exit code 7" in str(caught.value)
    assert not (build_root / "contracts.json").exists()
    assert (build_root / "workspace.json").is_file()
    with pytest.raises(dev_workspace.ContractCheckError):
        dev_workspace.verify_workspace(tmp_path, build_root)

    # Ownership survives a failed build, so a corrected retry is allowed.
    document = dev_workspace.build_workspace(tmp_path, build_root, runner=successful_runner)
    assert (build_root / "contracts.json").is_file()
    assert dev_workspace.verify_workspace(tmp_path, build_root) == document


def test_unowned_nonempty_output_root_is_rejected_and_preserved(tmp_path):
    write_contract(tmp_path)
    build_root = tmp_path / "output"
    note = build_root / "install/operator-notes.txt"
    note.parent.mkdir(parents=True)
    note.write_text("keep this\n")
    calls = []

    def runner(project, root):
        calls.append((project, root))
        return 0

    with pytest.raises(dev_workspace.DevWorkspaceError) as caught:
        dev_workspace.build_workspace(tmp_path, build_root, runner=runner)

    message = str(caught.value)
    assert "not empty" in message
    assert "No files were deleted" in message
    assert note.read_text() == "keep this\n"
    assert not (build_root / "workspace.json").exists()
    assert not (build_root / "contracts.json").exists()
    assert calls == []


def test_empty_output_root_is_claimed_then_can_be_rebuilt(tmp_path):
    write_contract(tmp_path)
    build_root = tmp_path / "output"
    build_root.mkdir()

    dev_workspace.build_workspace(tmp_path, build_root, runner=successful_runner)
    ownership = json.loads((build_root / "workspace.json").read_text())
    assert ownership["kind"] == dev_workspace.OWNERSHIP_KIND

    kept = build_root / "operator-notes.txt"
    kept.write_text("top-level file\n")
    dev_workspace.build_workspace(tmp_path, build_root, runner=successful_runner)
    assert (build_root / "workspace.json").is_file()
    assert (build_root / "contracts.json").is_file()
    assert kept.read_text() == "top-level file\n"


def test_invalid_ownership_marker_is_rejected_without_deletion(tmp_path):
    write_contract(tmp_path)
    build_root = tmp_path / "output"
    note = build_root / "install/operator-notes.txt"
    note.parent.mkdir(parents=True)
    note.write_text("keep this\n")
    (build_root / "workspace.json").write_text('{"kind": "someone-else"}\n')

    with pytest.raises(dev_workspace.DevWorkspaceError) as caught:
        dev_workspace.build_workspace(tmp_path, build_root, runner=successful_runner)
    assert "not recognised" in str(caught.value)
    assert note.read_text() == "keep this\n"


def test_build_rejects_concurrent_builder_before_cleaning(tmp_path):
    write_contract(tmp_path)
    build_root = tmp_path / "output"
    build_root.mkdir()
    (build_root / "install").mkdir()
    calls = []

    def runner(project, root):
        calls.append((project, root))
        return 1

    with dev_workspace.build_lock(build_root):
        with pytest.raises(dev_workspace.BuildBusyError) as caught:
            dev_workspace.build_workspace(tmp_path, build_root, runner=runner)
    assert "another builder" in str(caught.value)
    assert calls == []
    assert (build_root / "install").is_dir()


@pytest.mark.parametrize(
    "value",
    [
        "/",
        ".",
    ],
)
def test_build_root_rejects_dangerous_locations(tmp_path, value):
    write_contract(tmp_path)
    candidate = Path(value) if value == "/" else tmp_path
    with pytest.raises(dev_workspace.DevWorkspaceError):
        dev_workspace.validate_build_root(tmp_path, candidate)


def test_build_rejects_a_file_at_the_output_root(tmp_path):
    write_contract(tmp_path)
    build_root = tmp_path / "output"
    build_root.write_text("not a directory\n")

    with pytest.raises(dev_workspace.DevWorkspaceError) as caught:
        dev_workspace.build_workspace(tmp_path, build_root, runner=successful_runner)
    assert "not a directory" in str(caught.value)
    assert build_root.read_text() == "not a directory\n"


def test_build_removes_symlinked_outputs_in_an_owned_root_without_following(tmp_path):
    write_contract(tmp_path)
    build_root = tmp_path / "output"
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "keep.txt").write_text("keep\n")

    dev_workspace.build_workspace(tmp_path, build_root, runner=successful_runner)
    assert (build_root / "workspace.json").is_file()
    shutil.rmtree(build_root / "install")
    (build_root / "install").symlink_to(outside, target_is_directory=True)

    dev_workspace.build_workspace(tmp_path, build_root, runner=successful_runner)
    assert (outside / "keep.txt").read_text() == "keep\n"
    assert (build_root / "install").is_dir()
    assert not (build_root / "install").is_symlink()


def test_run_colcon_uses_explicit_directories_without_resetting_home(tmp_path, monkeypatch):
    captured = {}

    def fake_run(command, *, cwd, env, check):
        captured["command"] = command
        captured["env"] = env
        return type("Completed", (), {"returncode": 0})()

    monkeypatch.setattr(dev_workspace.subprocess, "run", fake_run)
    build_root = tmp_path / "output"
    assert dev_workspace.run_colcon(tmp_path, build_root) == 0

    environment = captured["env"]
    assert environment.get("HOME") == os.environ.get("HOME")
    assert environment["COLCON_HOME"] == str(build_root.resolve() / "colcon-home")
    assert environment["ROS_LOG_DIR"] == dev_workspace.ROS_LOG_DIRECTORY
    assert str(build_root.resolve() / "build") in captured["command"]
    assert str(build_root.resolve() / "install") in captured["command"]
